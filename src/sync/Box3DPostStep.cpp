// Box3DPostStep.cpp
//
// After the step: the pieces of the post-step that are neither write-back nor
// Impacts: queued point writes, fingerprints after our own writes, merged move
// events, aging and dead point deletion.
#include "Box3DSolverInternal.h"
#include "dop/SIM_SolverBox3D.h"

#include <GA/GA_SplittableRange.h>
#include <array>

namespace box3dsolver
{

namespace
{

// Adds the float point attribute `name` with the given type info unless it
// exists. True if added.
bool addPointAttribIfMissing( GU_Detail* gdp, const char* name, int tupleSize, GA_TypeInfo typeInfo = GA_TYPE_VOID,
							  GA_Storage storage = GA_STORE_REAL32 )
{
	if ( GA_Attribute* existing = gdp->findPointAttribute( name ) )
	{
		// A 32-bit one made earlier is widened when 64-bit is asked for, so it
		// keeps what is written.
		const GA_AIFTuple* tuple = existing->getAIFTuple();
		if ( storage == GA_STORE_REAL64 && tuple && tuple->getStorage( existing ) == GA_STORE_REAL32 )
			tuple->setStorage( existing, GA_STORE_REAL64 );
		return false;
	}
	// The add returns the new attribute, so no second lookup.
	GA_Attribute* attr =
		gdp->addFloatTuple( GA_ATTRIB_POINT, name, tupleSize, GA_Defaults( 0.0 ), nullptr, nullptr, storage );
	if ( attr && typeInfo != GA_TYPE_VOID )
		attr->setTypeInfo( typeInfo );
	return true;
}

// Adds the int point attribute `name` unless it exists.
void addIntPointAttribIfMissing( GU_Detail* gdp, const char* name, int tupleSize )
{
	if ( !gdp->findPointAttribute( name ) )
		gdp->addIntTuple( GA_ATTRIB_POINT, name, tupleSize );
}

} // namespace

// A new quaternion attribute is 0,0,0,0: start every point at identity.
void startOrientAtIdentity( GU_Detail* gdp )
{
	GA_RWHandleQ qOut( gdp, GA_ATTRIB_POINT, "orient" );
	if ( !qOut.isValid() )
		return;
	for ( GA_Iterator it( gdp->getPointRange() ); !it.atEnd(); ++it )
		qOut.set( *it, UT_QuaternionF( 0.0f, 0.0f, 0.0f, 1.0f ) );
}

// Creates `orient`, `v`, `w` if missing. True if `orient` was created.
bool ensureResultAttribs( GU_Detail* gdp )
{
	addPointAttribIfMissing( gdp, "v", 3, GA_TYPE_VECTOR );
	addPointAttribIfMissing( gdp, "w", 3, GA_TYPE_VECTOR );
	return addPointAttribIfMissing( gdp, "orient", 4, GA_TYPE_QUATERNION );
}

// Harden only the offset ranges the parallel write pass touches (hardenAllPages
// takes a range). Written offsets are bucketed by 1024 and runs of buckets
// hardened; nearly all hit: one full call. `end` is the first offset after a
// run, right whether the HDK reads it as exclusive or not.
void hardenWrittenPages( const GU_Detail* gdp, const std::vector<Box3DWriteItem>& items, size_t begin, size_t end,
						 const std::array<GA_Attribute*, 5>& attrs )
{
	constexpr int64_t kBucketBits = 10; // 1024 offsets
	const int64_t numOffsets = static_cast<int64_t>( gdp->getNumPointOffsets() );
	const size_t numBuckets = static_cast<size_t>( ( numOffsets >> kBucketBits ) + 1 );
	std::vector<char> hit( numBuckets, 0 );
	size_t hitCount = 0;
	for ( size_t k = begin; k < end; ++k )
	{
		const GA_Offset off = items[k].off;
		if ( off == GA_INVALID_OFFSET || static_cast<int64_t>( off ) < 0 || static_cast<int64_t>( off ) >= numOffsets )
			continue;
		char& h = hit[static_cast<size_t>( static_cast<int64_t>( off ) >> kBucketBits )];
		hitCount += h ? 0 : 1;
		h = 1;
	}
	if ( hitCount == 0 )
		return;

	const bool nearlyAll = hitCount * 10 >= numBuckets * 9;
	for ( GA_Attribute* attr : attrs )
	{
		if ( !attr )
			continue;
		if ( nearlyAll )
		{
			attr->hardenAllPages();
			continue;
		}
		for ( size_t b = 0; b < numBuckets; )
		{
			if ( !hit[b] )
			{
				++b;
				continue;
			}
			size_t e = b;
			while ( e + 1 < numBuckets && hit[e + 1] )
				++e;
			attr->hardenAllPages( static_cast<GA_Offset>( static_cast<int64_t>( b ) << kBucketBits ),
								  static_cast<GA_Offset>( static_cast<int64_t>( e + 1 ) << kBucketBits ) );
			b = e + 1;
		}
	}
}

void noteOwnWrite( Box3DSharedState& st, const SIM_Object& obj )
{
	st.writtenObjects.push_back( obj.getObjectId() );
}

// Re-record the fingerprints of the objects our writes touched this step, once
// each, so they do not look like external edits. Called with every write lock
// released (releasing one may bump ids).
void recordFingerprintsAfterWrites( Box3DSharedState& st, SIM_ObjectArray& objects )
{
	std::vector<int>& ids = st.writtenObjects;
	std::sort( ids.begin(), ids.end() );
	ids.erase( std::unique( ids.begin(), ids.end() ), ids.end() );
	for ( const int objectId : ids )
	{
		const auto f = st.objectSync.find( objectId );
		if ( f == st.objectSync.end() || f->second.visited != st.syncStamp )
			continue;
		const SIM_Object* obj = box3dObjectById( objects, objectId, st.objectsSorted );
		const SIM_Geometry* geo = obj ? SIM_DATA_GETCONST( *obj, SIM_GEOMETRY_DATANAME, SIM_Geometry ) : nullptr;
		if ( !geo )
			continue;
		SIM_GeometryAutoReadLock rlock( geo );
		const GU_Detail* gdp = rlock.getGdp();
		if ( !gdp )
			continue;
		const PointAttrs a( gdp );
		Box3DSyncFingerprint& fp = f->second.fp;
		fp = makeFingerprint( gdp, a, fp.nodeType, fp.nodeProps, fp.addImpact );
	}
	ids.clear();
}

// Make the next sync scan the object, after a write the scan must act on
// (active, dead, deletions).
void forceScan( Box3DSharedState& st, int objectId )
{
	const auto f = st.objectSync.find( objectId );
	if ( f != st.objectSync.end() )
		f->second.canSkip = false;
}

namespace
{

// The writable Geometry of an object, and the object itself in `obj` (null
// when it is gone, or its Geometry cannot be written).
SIM_GeometryCopy* runGeometry( Box3DSharedState& st, SIM_ObjectArray& objects, int objectId, SIM_Object*& obj )
{
	obj = box3dObjectById( objects, objectId, st.objectsSorted );
	return obj ? box3dWritableGeometry( *obj, SIM_GEOMETRY_DATANAME ) : nullptr;
}

} // namespace

// The queued point writes: bullet_autofit_valid = 1, the computed `mass`, and a
// new packed body's `restxform`, `pivot` and `orient` (the rotation since rest:
// identity unless the body already moved this step). A point that cannot be
// found keeps its value (a missed bullet_autofit_valid just refits again; a
// missed restxform leaves the point on its packed rotation, which agrees).
void writePointAttrs( Box3DSharedState& st, SIM_ObjectArray& objects )
{
	std::vector<Box3DPointWrite>& items = st.pointWrites;
	if ( items.empty() )
		return;
	forEachObjectRun( items, [&]( size_t begin, size_t end ) {
		SIM_Object* obj = nullptr;
		SIM_GeometryCopy* geo = runGeometry( st, objects, items[begin].objectId, obj );
		if ( !geo )
			return; // the object is gone, or writeResultsBack warned already
		unsigned want = 0;
		for ( size_t k = begin; k < end; ++k )
			want |= items[k].flags;
		unsigned wrote = 0;
		{
			SIM_GeometryAutoWriteLock lock( geo );
			GU_Detail* gdp = &lock.getGdp();
			if ( want & Box3DPointWrite::kMass )
				addPointAttribIfMissing( gdp, "mass", 1 );
			if ( ( want & Box3DPointWrite::kCompound ) && !gdp->findPointAttribute( kCompoundIdName ) )
				gdp->addIntTuple( GA_ATTRIB_POINT, kCompoundIdName, 1, GA_Defaults( -1 ) );
			if ( want & Box3DPointWrite::kRest )
			{
				// A new restxform is all zero (unset) on the other points.
				addPointAttribIfMissing( gdp, "restxform", 16, GA_TYPE_TRANSFORM );
				// P at rest: 64-bit beside a 64-bit P (computeRestInit reads it).
				addPointAttribIfMissing( gdp, "pivot", 3, GA_TYPE_POINT,
										 box3dP64( gdp ) ? GA_STORE_REAL64 : GA_STORE_REAL32 );
				if ( addPointAttribIfMissing( gdp, "orient", 4, GA_TYPE_QUATERNION ) )
					startOrientAtIdentity( gdp );
			}
			GA_RWHandleI valid( gdp, GA_ATTRIB_POINT, "bullet_autofit_valid" );
			GA_RWHandleF mass( gdp, GA_ATTRIB_POINT, "mass" );
			GA_RWHandleI compound( gdp, GA_ATTRIB_POINT, kCompoundIdName );
			GA_RWHandleM4 restxform( gdp, GA_ATTRIB_POINT, "restxform" );
			GA_RWHandleV3D pivot( gdp, GA_ATTRIB_POINT, "pivot" );
			GA_RWHandleQ orient( gdp, GA_ATTRIB_POINT, "orient" );
			const bool restOk = restxform.isValid() && pivot.isValid() && orient.isValid();
			const PointAttrs a( gdp );
			resolvePointOffsets( gdp, a, items, begin, end );
			for ( size_t k = begin; k < end; ++k )
			{
				const Box3DPointWrite& w = items[k];
				if ( w.off == GA_INVALID_OFFSET )
					continue;
				if ( ( w.flags & Box3DPointWrite::kAutofitValid ) && valid.isValid() )
				{
					valid.set( w.off, 1 );
					wrote |= Box3DPointWrite::kAutofitValid;
				}
				if ( ( w.flags & Box3DPointWrite::kMass ) && mass.isValid() )
				{
					mass.set( w.off, w.mass );
					wrote |= Box3DPointWrite::kMass;
				}
				if ( ( w.flags & Box3DPointWrite::kCompound ) && compound.isValid() )
				{
					compound.set( w.off, w.compoundId );
					wrote |= Box3DPointWrite::kCompound;
				}
				if ( ( w.flags & Box3DPointWrite::kRest ) && restOk )
				{
					restxform.set( w.off, box3dMatrix4F( w.restxform ) );
					pivot.set( w.off, box3dVec3D( w.pivot ) );
					float q[4];
					box3dAbsoluteToRest( w.entry->restRot, w.entry->rot, q );
					orient.set( w.off, UT_QuaternionF( q[0], q[1], q[2], q[3] ) );
					wrote |= Box3DPointWrite::kRest;
				}
			}
			if ( wrote & Box3DPointWrite::kAutofitValid )
				valid.getAttribute()->bumpDataId();
			if ( wrote & Box3DPointWrite::kMass )
				mass.getAttribute()->bumpDataId();
			if ( wrote & Box3DPointWrite::kCompound )
				compound.getAttribute()->bumpDataId();
			if ( wrote & Box3DPointWrite::kRest )
			{
				restxform.getAttribute()->bumpDataId();
				pivot.getAttribute()->bumpDataId();
				orient.getAttribute()->bumpDataId();
			}
		}
		if ( wrote )
			noteOwnWrite( st, *obj );
	} );
	items.clear();
}

// Enable Aging: age += dt on every point; with a `life` attribute, dead = 1
// once age exceeds it (as RBD Bullet Solver: a life of 0 or less dies on the
// first step, no `life` attribute never dies). With Reap Dead Points and Reap
// At Frame End, points that die are queued in st.deletions, so deleteDeadPoints
// removes them in this same step; otherwise the next step's sync queues them.
void ageObjects( Box3DSharedState& st, SIM_ObjectArray& objects, float dt )
{
	for ( int i = 0; i < objects.entries(); ++i )
	{
		SIM_Object* obj = objects( i );
		if ( !SIM_DATA_GETCONST( *obj, SIM_GEOMETRY_DATANAME, SIM_Geometry ) )
			continue;
		const auto osIt = st.objectSync.find( obj->getObjectId() );
		if ( osIt != st.objectSync.end() && osIt->second.wholeObject )
			continue; // one body: its points are not particles
		SIM_GeometryCopy* geo = box3dWritableGeometry( *obj, SIM_GEOMETRY_DATANAME );
		if ( !geo )
			continue;
		bool markedDead = false;
		{
			SIM_GeometryAutoWriteLock lock( geo );
			GU_Detail* gdp = &lock.getGdp();
			addPointAttribIfMissing( gdp, "age", 1 );
			const bool hasLife = gdp->findPointAttribute( "life" ) != nullptr;
			if ( hasLife )
				addIntPointAttribIfMissing( gdp, "dead", 1 );
			GA_RWHandleF age( gdp, GA_ATTRIB_POINT, "age" );
			GA_ROHandleF life( gdp, GA_ATTRIB_POINT, "life" );
			GA_RWHandleI dead( gdp, GA_ATTRIB_POINT, "dead" );
			if ( !age.isValid() )
				continue;
			const GA_Range range( gdp->getPointRange() );
			const bool killing = life.isValid() && dead.isValid();
			// The deletion check matches points by `id`, or by point number without
			// it (as the sync queues them).
			const GA_ROHandleI pointIds( gdp, GA_ATTRIB_POINT, "id" );
			std::atomic<bool> anyDead{ false };
			std::vector<Box3DDeletion> died; // rare: a lock is cheap enough
			std::mutex diedLock;
			// Each point touches only its own `age` / `dead` slot.
			const auto agePoint = [&]( GA_Offset off ) {
				const float a = age.get( off ) + dt;
				age.set( off, a );
				if ( killing )
				{
					const float l = life.get( off );
					if ( a > l && dead.get( off ) == 0 )
					{
						dead.set( off, 1 );
						anyDead.store( true, std::memory_order_relaxed );
						if ( !st.reapDead || !st.reapAtFrameEnd )
							return;
						Box3DDeletion d;
						d.objectId = obj->getObjectId();
						d.off = off;
						d.pointId = pointIds.isValid() ? static_cast<int64_t>( pointIds.get( off ) )
													   : static_cast<int64_t>( gdp->pointIndex( off ) );
						const std::lock_guard<std::mutex> guard( diedLock );
						died.push_back( d );
					}
				}
			};
			if ( static_cast<size_t>( range.getEntries() ) < minParallelPoints() )
			{
				for ( GA_Iterator it( range ); !it.atEnd(); ++it )
					agePoint( *it );
			}
			else
			{
				// Shared copy-on-write pages would race: harden what the parallel pass
				// writes.
				age.getAttribute()->hardenAllPages();
				if ( killing )
					dead.getAttribute()->hardenAllPages();
				UTparallelFor( GA_SplittableRange( range ), [&]( const GA_SplittableRange& r ) {
					GA_Offset start, end;
					for ( GA_Iterator it( r ); it.blockAdvance( start, end ); )
						for ( GA_Offset off = start; off < end; ++off )
							agePoint( off );
				} );
			}
			markedDead = anyDead.load();
			st.deletions.insert( st.deletions.end(), died.begin(), died.end() );
			age.getAttribute()->bumpDataId();
			if ( markedDead )
				dead.getAttribute()->bumpDataId();
		}
		noteOwnWrite( st, *obj );
		if ( markedDead )
			forceScan( st, obj->getObjectId() );
	}
}

// Delete the `dead` points (packed ones with their primitive), each checked to
// still be the same point first.
void deleteDeadPoints( Box3DSharedState& st, SIM_ObjectArray& objects )
{
	std::vector<Box3DDeletion>& items = st.deletions;
	if ( items.empty() )
		return;
	forEachObjectRun( items, [&]( size_t begin, size_t end ) {
		SIM_Object* obj = nullptr;
		SIM_GeometryCopy* geo = runGeometry( st, objects, items[begin].objectId, obj );
		if ( !geo )
			return;
		bool deleted = false;
		{
			SIM_GeometryAutoWriteLock lock( geo );
			GU_Detail* gdp = &lock.getGdp();
			const GA_ROHandleI id( gdp, GA_ATTRIB_POINT, "id" );
			// Check every point first: deleting changes point numbers.
			std::vector<GA_Offset> doomed;
			for ( size_t k = begin; k < end; ++k )
			{
				if ( samePointAt( gdp, id, items[k].off, items[k].pointId ) )
					doomed.push_back( items[k].off );
			}
			for ( const GA_Offset off : doomed )
			{
				const GA_Offset vtx = gdp->pointVertex( off );
				GA_Primitive* prim = GAisValid( vtx ) ? gdp->getPrimitive( gdp->vertexPrimitive( vtx ) ) : nullptr;
				if ( prim && GU_PrimPacked::isPackedPrimitive( *prim ) )
					gdp->destroyPrimitive( *prim, true ); // and its point
				else
					gdp->destroyPointOffset( off, GA_Detail::GA_DESTROY_DEGENERATE );
				deleted = true;
			}
			if ( deleted )
				gdp->bumpAllDataIds();
		}
		if ( deleted )
		{
			noteOwnWrite( st, *obj );
			forceScan( st, obj->getObjectId() ); // point numbers changed
		}
	} );
	items.clear();
}

// Box3D clears its move events every step, so a frame's steps are merged: the
// last event of a body is its final pose (and its fell-asleep flag). A body
// with an entry keeps its slot on the entry (moveStamp / moveSlot); the others
// (a compound's body) go through `slot`.
//
// Many events: sorted in parallel into those that overwrite a slot already
// taken, those that take a new one (their slots numbered by a serial pass over
// the kinds) and those without an entry (serial, last). Each body is in the
// events once unless `mayRepeat` (a compound taken apart reports its pieces
// again), which merges serially.
void mergeMoveEvents( Box3DSharedState& st, const std::vector<b3dop::BodyMove>& events,
					  std::unordered_map<b3dop::BodyHandle, size_t>& slot, bool mayRepeat )
{
	const auto serialOne = [&]( const b3dop::BodyMove& mv ) {
		Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( mv.userData );
		if ( e )
		{
			if ( e->moveStamp == st.syncStamp )
			{
				st.moves[e->moveSlot] = mv;
			}
			else
			{
				e->moveStamp = st.syncStamp;
				e->moveSlot = st.moves.size();
				st.moves.push_back( mv );
			}
			return;
		}
		const auto ins = slot.try_emplace( mv.handle, st.moves.size() );
		if ( ins.second )
			st.moves.push_back( mv );
		else
			st.moves[ins.first->second] = mv;
	};
	const size_t n = events.size();
	const uint64_t stamp = st.syncStamp;
	// Every body notes the slot of its move (where the events were taken as the
	// moves without that).
	const auto stampPending = [&]() {
		if ( !st.moveSlotsPending )
			return;
		st.moveSlotsPending = false;
		box3dParallelFor(
			st.moves.size(),
			[&]( size_t k ) {
				Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( st.moves[k].userData );
				if ( e )
				{
					e->moveStamp = stamp;
					e->moveSlot = k;
				}
			},
			0, kCheapGrain );
	};
	if ( mayRepeat || n < minParallelWrite() )
	{
		stampPending();
		for ( const b3dop::BodyMove& mv : events )
			serialOne( mv );
		return;
	}

	// The frame's first step: every event is new (one per body), so the events
	// are the moves.
	if ( st.moves.empty() )
	{
		st.moves.assign( events.begin(), events.end() );
		st.moveSlotsPending = true;
		for ( size_t k = 0; k < n; ++k )
			if ( !events[k].userData )
				slot[events[k].handle] = k;
		return;
	}

	// The same bodies in the same order as the moves so far (usual while nothing
	// falls asleep between the steps): the events replace them, and no body is
	// touched.
	if ( n == st.moves.size() )
	{
		std::atomic<bool> same{ true };
		box3dParallelFor(
			n,
			[&]( size_t k ) {
				if ( events[k].handle != st.moves[k].handle )
					same.store( false, std::memory_order_relaxed );
			},
			0, kCheapGrain );
		if ( same.load() )
		{
			std::copy( events.begin(), events.end(), st.moves.begin() );
			return;
		}
	}
	stampPending();

	enum : uint8_t
	{
		kOverwrite,
		kNew,
		kNoEntry
	};
	// Kept between steps (no allocation). References: a thread_local named in
	// a lambda would be each worker thread's own, empty one.
	thread_local std::vector<uint8_t> kindStore;
	thread_local std::vector<size_t> slotStore, newStore;
	std::vector<uint8_t>& kind = kindStore;
	std::vector<size_t>& slotOf = slotStore;
	std::vector<size_t>& fresh = newStore;
	kind.resize( n );
	slotOf.resize( n );
	// A body already in the moves takes its new event at once; the others are
	// sorted out below.
	box3dParallelFor(
		n,
		[&]( size_t k ) {
			Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( events[k].userData );
			if ( !e )
			{
				kind[k] = kNoEntry;
			}
			else if ( e->moveStamp == stamp )
			{
				kind[k] = kOverwrite;
				st.moves[e->moveSlot] = events[k];
			}
			else
			{
				kind[k] = kNew;
			}
		},
		0, kCheapGrain );
	size_t next = st.moves.size();
	fresh.clear();
	for ( size_t k = 0; k < n; ++k )
		if ( kind[k] == kNew )
		{
			slotOf[k] = next++;
			fresh.push_back( k );
		}
	st.moves.resize( next );
	box3dParallelFor(
		fresh.size(),
		[&]( size_t f ) {
			const size_t k = fresh[f];
			Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( events[k].userData );
			e->moveStamp = stamp;
			e->moveSlot = slotOf[k];
			st.moves[e->moveSlot] = events[k];
		},
		0, kCheapGrain );
	for ( size_t k = 0; k < n; ++k )
		if ( kind[k] == kNoEntry )
			serialOne( events[k] );
}

} // namespace box3dsolver
