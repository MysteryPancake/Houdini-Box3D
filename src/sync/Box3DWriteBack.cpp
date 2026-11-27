// Box3DWriteBack.cpp
//
// After the step: the write-back of awake bodies to their points (and whole
// objects' Position data), `SIM_SolverBox3D::writeResultsBack`.
#include "Box3DSolverInternal.h"
#include "dop/SIM_SolverBox3D.h"

#include <GA/GA_SplittableRange.h>
#include <array>

namespace box3dsolver
{

namespace
{

// ---- writeResultsBack, step 0 (serial): the bodies that moved
// Box3D's move events (read into st.moves right after the step): every awake
// body and its transform (a body falling asleep appears once more). Each body's
// user data is its entry.
struct MovedBodies
{
	std::vector<int> wholeMoves; // move events of whole-object bodies
	int missing = 0;			 // moved bodies without a handle
};

// Fills st.writeItems with the point bodies to write. The moves are sorted into
// kinds in parallel, the items numbered by a serial pass over the kinds, and
// the items filled in parallel.
MovedBodies collectMovedBodies( Box3DSharedState& st )
{
	MovedBodies out;
	const size_t moveCount = st.moves.size();
	enum : uint8_t
	{
		kSkip,
		kMissing,
		kWhole,
		kPoint,
		kAgent
	};
	// Kept between steps (no allocation). References: a thread_local named in
	// a lambda would be each worker thread's own, empty one.
	thread_local std::vector<uint8_t> kindStore;
	thread_local std::vector<uint32_t> itemStore;
	std::vector<uint8_t>& kind = kindStore;
	std::vector<uint32_t>& itemAt = itemStore;
	kind.resize( moveCount );
	itemAt.resize( moveCount );
	box3dParallelFor(
		moveCount,
		[&]( size_t mi ) {
			const b3dop::BodyMove& mv = st.moves[mi];
			const Box3DBodyMirror* entry = static_cast<const Box3DBodyMirror*>( mv.userData );
			// Only bodies seen by this step's sync: dynamic ones, and a ragdoll
			// agent's whatever their type (animated ones are kinematic).
			if ( !entry || !st.isLive( *entry ) || ( !entry->agentBody && !entry->dynamic() ) )
				kind[mi] = kSkip;
			else if ( entry->agentBody )
				kind[mi] = kAgent; // written as arrays
			else if ( !entry->handle )
				kind[mi] = kMissing;
			else if ( entry->wholeObject )
				kind[mi] = kWhole; // written to the object's Position data
			else
				kind[mi] = kPoint;
		},
		minParallelWrite(), kCheapGrain );
	uint32_t items = 0;
	for ( size_t mi = 0; mi < moveCount; ++mi )
	{
		switch ( kind[mi] )
		{
			case kMissing:
				++out.missing;
				break;
			case kWhole:
				out.wholeMoves.push_back( static_cast<int>( mi ) );
				break;
			case kAgent:
			{
				const b3dop::BodyMove& mv = st.moves[mi];
				Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( mv.userData );
				std::copy( mv.pos, mv.pos + 3, e->pos );
				std::copy( mv.rot, mv.rot + 4, e->rot );
				std::copy( mv.lin, mv.lin + 3, e->lin );
				std::copy( mv.ang, mv.ang + 3, e->ang );
				e->awake = !mv.fellAsleep;
				break;
			}
			case kPoint:
				itemAt[mi] = items++;
				break;
			default:
				break;
		}
	}
	// Only a growing vector constructs elements (serially); the parallel pass
	// below resets each one it uses, so a step does not rewrite them all in one
	// thread.
	st.writeItems.resize( items );
	box3dParallelFor(
		moveCount,
		[&]( size_t mi ) {
			if ( kind[mi] != kPoint )
				return;
			const b3dop::BodyMove& mv = st.moves[mi];
			Box3DBodyMirror* entry = static_cast<Box3DBodyMirror*>( mv.userData );
			Box3DWriteItem& w = st.writeItems[itemAt[mi]];
			w = Box3DWriteItem();
			// The velocities of the move event (a piece of a compound: its
			// compound's, its own parked body has none).
			const float* lin = entry->inCompound ? entry->glueLin : mv.lin;
			const float* ang = entry->inCompound ? entry->glueAng : mv.ang;
			std::copy( lin, lin + 3, w.lin );
			std::copy( ang, ang + 3, w.ang );
			w.entry = entry;
			w.move = static_cast<int>( mi );
			w.objectId = entry->objectId;
			// `awake`: only changes are written (and only if the attribute
			// exists). entry->awake is updated where the write happens, so an
			// unresolved point is retried next step.
			const bool nowAwake = !mv.fellAsleep;
			if ( nowAwake != entry->awake )
				w.awakeState = nowAwake ? 1 : 0;
		},
		minParallelWrite(), kCheapGrain );
	return out;
}

// Whole-object bodies: one Position data each (serial; few objects). Returns
// how many could not be written.
int writeWholeObjects( Box3DSharedState& st, SIM_ObjectArray& objects, const std::vector<int>& wholeMoves )
{
	int notWritten = 0;
	for ( const int mi : wholeMoves )
	{
		const b3dop::BodyMove& mv = st.moves[static_cast<size_t>( mi )];
		Box3DBodyMirror* entry = static_cast<Box3DBodyMirror*>( mv.userData );
		SIM_Object* obj = box3dObjectById( objects, entry->objectId, st.objectsSorted );
		if ( !obj )
			continue;
		entry->awake = !mv.fellAsleep;
		if ( !writeWholeObject( *obj, *entry, mv.pos, mv.rot, mv.lin, mv.ang ) )
			++notWritten;
	}
	return notWritten;
}

// ---- step 2 (parallel): write one body's attributes
// The attribute handles of one object's geometry, and what it takes to write
// one item. write() touches only its own point and its own item's entry, so
// items can be written concurrently.
class BodyWriter
{
public:
	// The items [begin, end) of st.writeItems belong to `gdp`; their points are
	// resolved.
	BodyWriter( Box3DSharedState& st, GU_Detail* gdp, const PointAttrs& a, size_t begin, size_t end )
		: myMoves( st.moves )
		, myGdp( gdp )
		, myA( a )
		, myAnyPrims( gdp->getNumPrimitives() > 0 )
		, myP( gdp, GA_ATTRIB_POINT, "P" )
		, myV( gdp, GA_ATTRIB_POINT, "v" )
		, myW( gdp, GA_ATTRIB_POINT, "w" )
		, myQ( gdp, GA_ATTRIB_POINT, "orient" )
		, myAwake( gdp, GA_ATTRIB_POINT, "awake" ) // only if it exists
	{
		if ( myAwake.isValid() )
			for ( size_t k = begin; k < end && !myAnyAwakeWrite; ++k )
				myAnyAwakeWrite = st.writeItems[k].awakeState >= 0 && st.writeItems[k].off != GA_INVALID_OFFSET;
#ifdef BOX3DDOP_DEBUG
		myVerifyReadback = B3D_DEBUG_ENV_INT( "BOX3DDOP_VERIFY_READBACK", 0 ) != 0;
#endif
	}

	// Bodies whose primitive is gone (the topology changed without a new id).
	int missingPrimitives() const
	{
		return myMissingPrims.load();
	}
	bool wrotePacked() const
	{
		return myWrotePacked.load();
	}

	// The attributes write() writes (null: not written).
	std::array<GA_Attribute*, 5> writtenAttributes() const
	{
		return { myP.isValid() ? myP.getAttribute() : nullptr, myQ.isValid() ? myQ.getAttribute() : nullptr,
				 myV.isValid() ? myV.getAttribute() : nullptr, myW.isValid() ? myW.getAttribute() : nullptr,
				 myAnyAwakeWrite ? myAwake.getAttribute() : nullptr };
	}

	void write( Box3DWriteItem& w ) const
	{
		if ( w.off == GA_INVALID_OFFSET )
			return;
		const b3dop::BodyMove& mv = myMoves[static_cast<size_t>( w.move )];
		writeAttributes( w, mv );
		PackedNew pw;
		const bool packed = composePacked( w, mv, pw );
		if ( packed )
			writePrimitive( pw );
		updateMirror( w, mv, packed );
#ifdef BOX3DDOP_DEBUG
		if ( myVerifyReadback )
			verifyReadback( w, packed ? &pw : nullptr );
#endif
	}

private:
	void writeAttributes( const Box3DWriteItem& w, const b3dop::BodyMove& mv ) const
	{
		const GA_Offset off = w.off;
		if ( myP.isValid() )
			myP.set( off, box3dVec3F( mv.pos ) );
		if ( myQ.isValid() )
		{
			// Packed with a rest: the rotation since rest (Bullet's orient); the
			// packed transform below carries the absolute one.
			float q[4] = { mv.rot[0], mv.rot[1], mv.rot[2], mv.rot[3] };
			if ( w.entry->packed && w.entry->hasRest )
				box3dAbsoluteToRest( w.entry->restRot, mv.rot, q );
			myQ.set( off, UT_QuaternionF( q[0], q[1], q[2], q[3] ) );
		}
		if ( myV.isValid() )
			myV.set( off, UT_Vector3( w.lin[0], w.lin[1], w.lin[2] ) );
		if ( myW.isValid() )
			myW.set( off, UT_Vector3( w.ang[0], w.ang[1], w.ang[2] ) );
		if ( myAnyAwakeWrite && w.awakeState >= 0 )
		{
			myAwake.set( off, static_cast<int>( w.awakeState ) );
			w.entry->awake = ( w.awakeState != 0 );
		}
	}

	// A packed body's new 3x3 (the scale the sync read, the new rotation) and the
	// primitive the last sync stored for it.
	struct PackedNew
	{
		double newM3[9];
		GA_Offset primOff;
	};

	// Packed: composes the new 3x3 into `pw`. False if the body is not packed.
	bool composePacked( const Box3DWriteItem& w, const b3dop::BodyMove& mv, PackedNew& pw ) const
	{
		Box3DBodyMirror& me = *w.entry;
		if ( !( myAnyPrims && me.packed ) )
			return false;
		composePackedMatrix( mv.rot, me.rowScale, pw.newM3 );
		b3dop::keepMirror( me.mirrored, pw.newM3 ); // a mirrored piece stays mirrored
		pw.primOff = static_cast<GA_Offset>( me.primOff );
		return true;
	}

	// Each body has its own primitive, so these writes are to distinct ones.
	// setLocalTransform does not mark the primitive list dirty: the caller bumps
	// it once (wrotePacked), as the HDK leaves that to us.
	void writePrimitive( const PackedNew& pw ) const
	{
		GA_Primitive* raw = ( pw.primOff >= 0 && pw.primOff < myGdp->getNumPrimitiveOffsets() )
								? myGdp->getPrimitive( pw.primOff )
								: nullptr;
		if ( !raw || !GU_PrimPacked::isPackedPrimitive( *raw ) )
		{
			myMissingPrims.fetch_add( 1, std::memory_order_relaxed );
			return;
		}
		setLocal3( *static_cast<GU_PrimPacked*>( raw ), pw.newM3 );
		myWrotePacked.store( true, std::memory_order_relaxed );
	}

	// What the read path will see, so our own write does not look like an
	// external edit: P through float; the rotation as written (poseDiffers allows
	// for the float noise of a packed transform's round trip and for q vs -q).
	// Non-finite P: Box3D's values.
	void updateMirror( const Box3DWriteItem& w, const b3dop::BodyMove& mv, bool packed ) const
	{
		Box3DBodyPose& mirror = *w.entry;
		const bool hasRot = packed || myQ.isValid(); // else the read path sees identity
		const float identity[4] = { 0, 0, 0, 1 };
		std::copy( hasRot ? mv.rot : identity, ( hasRot ? mv.rot : identity ) + 4, mirror.rot );
		b3dop::sanitizeQuat( mirror.rot );
		for ( int i = 0; i < 3; ++i )
		{
			mirror.lin[i] = myV.isValid() ? w.lin[i] : 0.0f;
			mirror.ang[i] = myW.isValid() ? w.ang[i] : 0.0f;
		}
		if ( !b3dop::storedPosition( mv.pos, mirror.pos ) )
			std::copy( mv.pos, mv.pos + 3, mirror.pos );
	}

#ifdef BOX3DDOP_DEBUG
	// BOX3DDOP_VERIFY_READBACK=1: read back; a difference poseDiffers sees would
	// make the next scan treat our own write as an external edit.
	void verifyReadback( const Box3DWriteItem& w, const PackedNew* pw ) const
	{
		const Box3DBodyPose& mirror = *w.entry;
		Box3DBodyPose stored;
		PackedInfo pk;
		if ( pw )
		{
			pk.prim = static_cast<const GU_PrimPacked*>( myGdp->getPrimitive( pw->primOff ) );
			double sc[3];
			splitPackedMatrix( pw->newM3, pk.rot, sc );
		}
		if ( readPointState( myGdp, myA, w.off, pk, stored ) && poseDiffers( stored, mirror ) )
		{
			B3D_DEBUG_LOG( "  READBACK MISMATCH object=%d point=%lld: predicted pos=(%.9g "
						   "%.9g %.9g) rot=(%.9g %.9g %.9g %.9g) "
						   "read pos=(%.9g %.9g %.9g) rot=(%.9g %.9g %.9g %.9g)",
						   w.objectId, static_cast<long long>( w.entry->pointId ), mirror.pos[0], mirror.pos[1],
						   mirror.pos[2], mirror.rot[0], mirror.rot[1], mirror.rot[2], mirror.rot[3], stored.pos[0],
						   stored.pos[1], stored.pos[2], stored.rot[0], stored.rot[1], stored.rot[2], stored.rot[3] );
		}
	}
#endif

	const std::vector<b3dop::BodyMove>& myMoves;
	GU_Detail* myGdp;
	[[maybe_unused]] const PointAttrs& myA;
	const bool myAnyPrims;
	mutable GA_RWHandleV3 myP, myV, myW;
	mutable GA_RWHandleQ myQ;
	mutable GA_RWHandleI myAwake;
	bool myAnyAwakeWrite = false;
	// Set by writePrimitive from the parallel pass.
	mutable std::atomic<int> myMissingPrims{ 0 };
	mutable std::atomic<bool> myWrotePacked{ false };
#ifdef BOX3DDOP_DEBUG
	bool myVerifyReadback = false;
#endif
};

// One object's results, st.writeItems[begin, end), into its (locked, writable)
// geometry. Returns how many bodies could not be matched to a point or
// primitive.
int writeObjectResults( Box3DSharedState& st, GU_Detail* gdp, size_t begin, size_t end )
{
	std::vector<Box3DWriteItem>& items = st.writeItems;

	if ( ensureResultAttribs( gdp ) )
		startOrientAtIdentity( gdp );
	const PointAttrs a( gdp ); // after the attributes exist

	// ---- step 1 (serial): find each body's point in the geometry ----
	resolvePointOffsets( gdp, a, items, begin, end );
	int missing = 0;
	for ( size_t k = begin; k < end; ++k )
		if ( items[k].off == GA_INVALID_OFFSET )
			++missing;

	// ---- step 2 (parallel): write attributes and packed transforms, one task
	// per body ----
	const BodyWriter writer( st, gdp, a, begin, end );
	const std::array<GA_Attribute*, 5> written = writer.writtenAttributes();
	// Shared copy-on-write pages would race: a parallel pass hardens what it
	// writes first.
	if ( end - begin >= minParallelWrite() )
		hardenWrittenPages( gdp, items, begin, end, written );
	parallelForEach( end - begin, [&]( size_t j ) { writer.write( items[begin + j] ); }, minParallelWrite() );
	for ( GA_Attribute* attr : written )
		if ( attr )
			attr->bumpDataId();
	// setLocalTransform does not mark the primitive list dirty; the HDK leaves
	// that to us.
	if ( writer.wrotePacked() )
		gdp->getPrimitiveList().bumpDataId();
	return missing + writer.missingPrimitives();
}

} // namespace

} // namespace box3dsolver

using namespace box3dsolver;

void SIM_SolverBox3D::writeResultsBack( SIM_Box3DWorld& worldData, SIM_ObjectArray& objects )
{
	Box3DSharedState* st = worldData.state();
	// Every way out writes the ragdoll agents' arrays (a body that did not move
	// still has its creation state to write).
	struct AgentArrays
	{
		Box3DSharedState& st;
		SIM_ObjectArray& objects;
		~AgentArrays()
		{
			writeAgentArrays( st, objects );
		}
	} agentArrays{ *st, objects };
	if ( st->moves.empty() )
		return;

	MovedBodies moved = collectMovedBodies( *st );
	int missing = moved.missing;
	warnCount( objects, writeWholeObjects( *st, objects, moved.wholeMoves ),
			   "object(s) have Position data the solver cannot write (not a "
			   "Position or Motion); their results were not written back" );

	const auto reportMissing = [&]( int count ) {
		if ( !count )
			return;
		B3D_DEBUG_LOG_VERBOSE( "  writeBack: %d moved bodies had no readable state or no "
							   "point to write to",
							   count );
		warnCount( objects, count,
				   "body(ies) could not be matched to a point; their results were "
				   "not written back" );
	};

	std::vector<Box3DWriteItem>& items = st->writeItems;
	if ( items.empty() )
	{
		reportMissing( missing ); // bodies without a handle
		return;
	}

	// Grouped by object, so each geometry is locked once.
	forEachObjectRun( items, [&]( size_t begin, size_t end ) {
		SIM_Object* obj = box3dObjectById( objects, items[begin].objectId, st->objectsSorted );
		if ( !obj )
			return;

		SIM_GeometryCopy* geo = box3dWritableGeometry( *obj, SIM_GEOMETRY_DATANAME );
		if ( !geo )
		{
			B3D_DEBUG_LOG_VERBOSE( "  writeBack: object id=%d Geometry is not writable and "
								   "could not be converted",
								   obj->getObjectId() );
			addError( obj, SIM_MESSAGE,
					  "This object's Geometry is not writable and could not be "
					  "converted to a SIM_GeometryCopy; results are not written back",
					  UT_ERROR_WARNING );
			return;
		}

		{
			SIM_GeometryAutoWriteLock lock( geo );
			missing += writeObjectResults( *st, &lock.getGdp(), begin, end );
		}
		noteOwnWrite( *st, *obj );
	} );

	reportMissing( missing );
}
