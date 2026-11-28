// Box3DJointBreak.cpp
//
// Constraint networks after the step: Glue's impulse model, joints that break
// (or switch to their next constraint) and the live frames and feedback
// attributes written onto the networks. Box3DJointSync.cpp reads the networks
// and makes the joints.
#include "Box3DCompoundSync.h"
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSync.h"
#include "core/Box3DCompound.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_AIFStringTuple.h>
#include <GA/GA_AIFTuple.h>
#include <GA/GA_AttributeDict.h>
#include <GA/GA_Defaults.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

// After the step, for a written network: every live joint's anchors get
// current_P / current_orient set to where its frames are now (P / orient stay
// as authored), and joints made from P / orient save
// their local frames to local_P / local_orient (relative to their pieces: exact
// on a rebuild, wherever the pieces are). A distance joint made with a negative
// rest length gets its starting distance written to `auto_restlength` (not
// `restlength`, which multiplies the node's value), so a world rebuilt
// mid-simulation keeps it.
template <class PrimOk>
void writeLiveFrames( const b3dop::Box3DWorld& world, const Box3DSharedState& st, GU_Detail* gdp,
					  const std::vector<Box3DJointEntry*>& entries, const PrimOk& primOk )
{
	if ( entries.empty() )
		return;
	const bool p64 = box3dP64( gdp );
	auto ensure = [&]( const char* name, int size, bool quat, bool& created, GA_Storage storage = GA_STORE_REAL32 ) {
		created = false;
		GA_Attribute* attr = gdp->findPointAttribute( name );
		if ( !attr )
		{
			gdp->addFloatTuple( GA_ATTRIB_POINT, name, size, GA_Defaults( 0.0 ), nullptr, nullptr, storage );
			created = true;
			if ( quat )
				if ( GA_Attribute* added = gdp->findPointAttribute( name ) )
					added->setTypeInfo( GA_TYPE_QUATERNION );
			return;
		}
		// A 32-bit one made before P became 64-bit: widened, so it keeps what is
		// written.
		const GA_AIFTuple* tuple = attr->getAIFTuple();
		if ( storage == GA_STORE_REAL64 && tuple && tuple->getStorage( attr ) == GA_STORE_REAL32 )
			tuple->setStorage( attr, GA_STORE_REAL64 );
	};
	// A new current_orient is 0,0,0,0 on every point: "not written" for the
	// points of joints that are not live (see NetworkReader::live).
	bool created = false;
	// World positions: 64-bit beside a 64-bit P.
	ensure( kCurrentP, 3, false, created, p64 ? GA_STORE_REAL64 : GA_STORE_REAL32 );
	ensure( kCurrentOrient, 4, true, created );
	GA_RWHandleV3D pH( gdp, GA_ATTRIB_POINT, kCurrentP );
	GA_RWHandleQ orientH( gdp, GA_ATTRIB_POINT, kCurrentOrient );
	if ( !pH.isValid() || !orientH.isValid() )
		return;
	bool wroteRestLength = false, wroteLocal = false;
	GA_RWHandleF restLengthH;
	GA_RWHandleV3 localPH;
	GA_RWHandleQ localQH;

	// The rare per-joint work (rest length, local frames) is serial, and only
	// looks at the joints that have some; the live frames, one write per bond
	// every step, go in parallel afterwards, each looking up its own primitive.
	size_t active = 0;
	for ( Box3DJointEntry* e : entries )
	{
		if ( e->quiet )
			continue;
		++active;
		if ( e->madeRestLength < 0.0f && !e->writeLocal )
			continue;
		if ( !primOk( e->prim ) )
			continue;
		const GA_Primitive* prim = gdp->getPrimitive( e->prim );
		if ( !prim || prim->getVertexCount() != 2 )
			continue;
		const GA_Offset pts[2] = { prim->getPointOffset( 0 ), prim->getPointOffset( 1 ) };
		if ( e->madeRestLength >= 0.0f )
		{
			if ( !restLengthH.isValid() )
			{
				if ( !gdp->findPrimitiveAttribute( kAutoRestLength ) )
					gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, kAutoRestLength, 1, GA_Defaults( -1.0f ) );
				restLengthH = GA_RWHandleF( gdp, GA_ATTRIB_PRIMITIVE, kAutoRestLength );
			}
			if ( restLengthH.isValid() )
			{
				restLengthH.set( e->prim, e->madeRestLength );
				wroteRestLength = true;
			}
			e->madeRestLength = -1.0f;
		}

		if ( e->writeLocal )
		{
			b3dop::JointFrame la, lb;
			if ( jointLocalFramesOf( world, *e, la, lb ) )
			{
				if ( !localPH.isValid() )
				{
					ensure( "local_P", 3, false, created );
					ensure( "local_orient", 4, true, created );
					localPH = GA_RWHandleV3( gdp, GA_ATTRIB_POINT, "local_P" );
					localQH = GA_RWHandleQ( gdp, GA_ATTRIB_POINT, "local_orient" );
				}
				if ( localPH.isValid() && localQH.isValid() )
				{
					const b3dop::JointFrame* frames[2] = { &la, &lb };
					const bool agentEnd[2] = { e->agentEndA, e->agentEndB };
					for ( int k = 0; k < 2; ++k )
					{
						// An agent end's local_P / local_orient are its authored place in
						// its rig transform, which the anchor is read from.
						if ( agentEnd[k] )
							continue;
						const b3dop::JointFrame& f = *frames[k];
						localPH.set( pts[k], box3dVec3F( f.pos ) );
						localQH.set( pts[k], UT_QuaternionF( f.rot[0], f.rot[1], f.rot[2], f.rot[3] ) );
					}
					wroteLocal = true;
				}
			}
			e->writeLocal = false;
		}
	}

	// Each joint reads the world and the geometry (read-only calls may run in
	// parallel) and writes its own entry and its own two points, whose pages are
	// hardened first.
	std::atomic<bool> wroteFrames{ false };
	auto writeOne = [&]( size_t i ) {
		Box3DJointEntry* e = entries[i];
		if ( e->quiet || !primOk( e->prim ) )
			return;
		const GA_Primitive* prim = gdp->getPrimitive( e->prim );
		if ( !prim || prim->getVertexCount() != 2 )
			return;
		const GA_Offset pts[2] = { prim->getPointOffset( 0 ), prim->getPointOffset( 1 ) };
		b3dop::JointFrame fa, fb;
		// A bond inside a compound joins two parked bodies, which are not kept
		// where their pieces are: the frames come from the compound's pose.
		b3dop::BodyPose poseA, poseB;
		const bool found = e->inCompound && compoundJointPoses( st, *e, poseA, poseB )
							   ? world.jointWorldFramesAt( e->handle, poseA.p, poseA.q, poseB.p, poseB.q, fa, fb )
							   : world.jointWorldFrames( e->handle, fa, fb );
		if ( found )
		{
			// The anchors, not the frames (SideFX data may offset the frames from
			// them).
			e->wroteA = asStored( anchorFrameOf( fa, e->adjA ), p64 );
			e->wroteB = asStored( anchorFrameOf( fb, e->adjB ), p64 );
			e->wrote = true;
		}
		else if ( !e->switchFrames )
			return;				 // broken, or never made
		e->switchFrames = false; // (switched: wroteA / wroteB are set already)
		const b3dop::JointFrame* frames[2] = { &e->wroteA, &e->wroteB };
		for ( int k = 0; k < 2; ++k )
		{
			const b3dop::JointFrame& f = *frames[k];
			pH.set( pts[k], box3dVec3D( f.pos ) );
			orientH.set( pts[k], UT_QuaternionF( f.rot[0], f.rot[1], f.rot[2], f.rot[3] ) );
		}
		wroteFrames.store( true, std::memory_order_relaxed );
	};
	constexpr size_t kMinParallelFrames = 256;
	if ( active >= kMinParallelFrames )
	{
		pH.getAttribute()->hardenAllPages();
		orientH.getAttribute()->hardenAllPages();
	}
	// (parallel only with the pages hardened, which is when this many are live)
	if ( active > 0 )
		box3dParallelFor( entries.size(), writeOne, active >= kMinParallelFrames ? 0 : entries.size() + 1, 64 );
	// Only what was written changes its data id (anything watching these
	// attributes sees a change only when there is one).
	if ( wroteFrames.load() )
	{
		pH.getAttribute()->bumpDataId();
		orientH.getAttribute()->bumpDataId();
	}
	if ( wroteRestLength )
		restLengthH.getAttribute()->bumpDataId();
	if ( wroteLocal )
	{
		localPH.getAttribute()->bumpDataId();
		localQH.getAttribute()->bumpDataId();
	}
}

} // namespace

namespace
{

// Joint local frames are read first (a next constraint may start from them),
// then the joint is destroyed and the entry marked broken.
void destroyBrokenJoint( b3dop::Box3DWorld& world, Box3DJointEntry& e )
{
	Box3DJointExtra& x = e.ensureExtra();
	e.brokeLocalKnown = jointLocalFramesOf( world, e, x.brokeLocalA, x.brokeLocalB );
	world.destroyJoint( e.handle );
	e.handle = 0;
	e.broken = true;
}

} // namespace

// The glue group of each node (st.glueGroup), if the bonds changed.
void refreshGlueGroups( Box3DSharedState& st )
{
	if ( st.glueGroupKnown && st.glueGroupSignature == st.glueSignature && st.glueGroupNodes == st.glueNodes.size() )
		return;
	st.glueGroup = b3dop::glueComponents( st.glueNodes.size(), glueBondList( st, false, true ) ).of;
	st.glueGroupSignature = st.glueSignature;
	st.glueGroupNodes = st.glueNodes.size();
	st.glueGroupKnown = true;
}

void collectGlueHits( Box3DSharedState& st )
{
	b3dop::Box3DWorld& world = st.world;
	++st.hitsTick;
	refreshGlueGroups( st );
	st.compoundGroup.clear();
	for ( const auto& c : st.compounds )
		st.compoundGroup[c->body] = compoundGlueGroup( st, *c );
	// A contact with a piece of its own group (or with the body of a compound of
	// it) is not a hit: pieces of one glued structure press on each other with
	// its weight (and push each other along with whatever hits it, which the hit
	// itself counts). Not bonded to each other, they do collide.
	const bool weightPossible = !st.glueNodes.empty();
	int group = -1;
	const std::function<bool( b3dop::BodyHandle )> sameGroup = [&st, &group]( b3dop::BodyHandle other ) {
		return glueIgnoresContact( st, other, group );
	};
	// The impulse each glued body's contacts carried, from the step's impulse
	// events: one pass over the contact points, where asking each of thousands
	// of bodies for its contacts is the cost. The same contacts are left out:
	// those with a glued neighbour, and with the body's own group.
	thread_local std::vector<b3dop::ContactImpulseSum> events;
	thread_local std::vector<float> eventHits;
	const bool fromEvents = st.glueHitsFromEvents;
	if ( fromEvents )
	{
		world.getContactImpulseSums( events );
		eventHits.assign( st.glueNodes.size(), 0.0f );
		for ( const b3dop::ContactImpulseSum& e : events )
		{
			const auto na = st.glueNodeOf.find( e.a );
			const auto nb = st.glueNodeOf.find( e.b );
			const bool aNode = na != st.glueNodeOf.end();
			const bool bNode = nb != st.glueNodeOf.end();
			if ( !aNode && !bNode )
				continue;
			if ( aNode && bNode )
			{
				// Two glued bodies: no hit between glued neighbours (their partner
				// lists are alike both ways) or between bodies of one group.
				const size_t ia = static_cast<size_t>( na->second );
				const size_t ib = static_cast<size_t>( nb->second );
				const std::vector<b3dop::BodyHandle>& partners = st.glueNodes[ia].partners;
				if ( std::find( partners.begin(), partners.end(), e.b ) != partners.end() ||
					 st.glueGroup[ia] == st.glueGroup[ib] )
					continue;
				eventHits[ia] += e.impulse;
				eventHits[ib] += e.impulse;
				continue;
			}
			// One glued body, and what it touches (not a glued body, but maybe the
			// body of a compound of its group).
			const size_t i = static_cast<size_t>( ( aNode ? na : nb )->second );
			if ( glueIgnoresContact( st, aNode ? e.b : e.a, st.glueGroup[i] ) )
				continue;
			eventHits[i] += e.impulse;
		}
	}
	for ( size_t i = 0; i < st.glueNodes.size(); ++i )
	{
		Box3DGlueNode& n = st.glueNodes[i];
		// A piece of a compound takes the hits on its shapes of the compound.
		float pieceHits = 0.0f;
		if ( compoundGlueHits( st, n.body, pieceHits ) )
		{
			n.hits += pieceHits;
			continue;
		}
		if ( fromEvents && !( eventHits[i] > 0.0f ) )
			continue;
		// Hits land on moving bodies; a sleeping one has none this step.
		if ( world.typeByHandle( n.body ) != b3dop::BodySettings::Type::Dynamic || !world.isAwakeByHandle( n.body ) )
			continue;
		if ( fromEvents )
		{
			n.hits += eventHits[i];
			continue;
		}
		group = st.glueGroup[i];
		// A glued neighbour is not a hit (the pair never collides anyway).
		n.hits += world.contactImpulseByHandle( n.body, n.partners.data(), n.partners.size(),
												weightPossible ? &sameGroup : nullptr );
	}
	if ( fromEvents )
		estimateGlueHits( st );
}

std::vector<b3dop::GlueBond> glueBondList( const Box3DSharedState& st, bool withParams, bool blankBroken )
{
	// One bond per iteration, each reading its own entry: the pointer chasing is
	// what takes the time with thousands of bonds.
	constexpr size_t kMinParallelBonds = 2048;
	std::vector<b3dop::GlueBond> bonds( st.glueBonds.size() );
	box3dParallelFor(
		bonds.size(),
		[&]( size_t i ) {
			const Box3DJointEntry* e = st.glueBonds[i];
			b3dop::GlueBond& g = bonds[i];
			g.a = st.glueBondNodes[i].first;
			g.b = st.glueBondNodes[i].second;
			if ( withParams )
			{
				g.params = e->extra->glueParams;
				g.impact = e->extra->glueImpact;
			}
			if ( blankBroken && e->broken )
				g.a = g.b = -1;
		},
		kMinParallelBonds, 256 );
	return bonds;
}

namespace
{
// The joints of each network, again when the joints changed (syncJoints erases
// and adds entries, so the lists must not be read before this).
void refreshNetworkEntries( Box3DSharedState& st )
{
	if ( !st.networkEntriesDirty )
		return;
	st.networkEntriesDirty = false;
	st.networkEntries.clear();
	for ( auto& je : st.joints )
		st.networkEntries[je.second.network].push_back( &je.second );
}
} // namespace

namespace
{
// Whether any bond carries an impact (or a NaN, which must not hide): stops at
// the first, so a frame that still has some pays little.
bool anyImpact( const std::vector<b3dop::GlueBond>& bonds )
{
	return std::any_of( bonds.begin(), bonds.end(),
						[]( const b3dop::GlueBond& g ) { return g.impact > 0.0f || std::isnan( g.impact ); } );
}
} // namespace

void advanceGlue( Box3DSharedState& st, float dt )
{
	st.glueEstimated.clear();
	if ( st.glueBonds.empty() )
		return;
	// The bonds as the spread reads them are copied again only when something
	// changed one (touchGlueBonds): the copy walks every bond's entry.
	if ( st.glueMirrorEpoch != st.glueEpoch || st.glueMirror.size() != st.glueBonds.size() )
	{
		st.glueMirror = glueBondList( st, true, true );
		st.glueMirrorEpoch = st.glueEpoch;
		st.glueMirrorQuiet = !anyImpact( st.glueMirror );
	}
	// Nothing hit this frame and no bond carries any impact: the decay and the
	// spread would change nothing.
	bool idle = st.glueMirrorQuiet;
	if ( idle )
		for ( const Box3DGlueNode& n : st.glueNodes )
			if ( n.hits > 0.0f || n.estimate > 0.0f )
			{
				idle = false;
				break;
			}
	if ( idle )
		return;
	std::vector<float> hits( st.glueNodes.size() ), estimates( st.glueNodes.size() );
	for ( size_t i = 0; i < hits.size(); ++i )
	{
		hits[i] = st.glueNodes[i].hits;
		estimates[i] = st.glueNodes[i].estimate;
	}
	std::vector<b3dop::GlueBond>& bonds = st.glueMirror;
	std::vector<int> broken;
	b3dop::glueAdvance( bonds, std::move( hits ), std::move( estimates ), dt, broken );
	box3dParallelFor(
		bonds.size(), [&]( size_t i ) { st.glueBonds[i]->extra->glueImpact = bonds[i].impact; }, 2048, 256 );
	st.glueMirrorQuiet = !anyImpact( bonds );
	for ( const int i : broken )
		st.glueBroken.push_back( st.glueBonds[static_cast<size_t>( i )] );
	for ( Box3DGlueNode& n : st.glueNodes )
		n.hits = n.estimate = 0.0f;
}

namespace
{

using EntriesByNetwork = std::unordered_map<uint64_t, std::vector<Box3DJointEntry*>>;

// Joints over their force / torque threshold: read their local frames (a next
// constraint may start from them), then destroy them. Read the events before
// destroying anything.
int breakJoints( Box3DSharedState& st, EntriesByNetwork& brokeIn )
{
	b3dop::Box3DWorld& world = st.world;
	std::vector<void*> events;
	if ( st.brokenCollected )
		events = st.brokenThisFrame;
	else
		world.brokenJoints( events );
	// Glue bonds over their strength (advanceGlue).
	events.insert( events.end(), st.glueBroken.begin(), st.glueBroken.end() );
	st.glueBroken.clear();
	if ( !events.empty() )
	{
		st.glueDirty = true; // the Glue graph loses a bond
		st.touchGlueBonds();
	}
	int broke = 0;
	for ( void* p : events )
	{
		Box3DJointEntry* e = static_cast<Box3DJointEntry*>( p );
		if ( !e || !e->handle )
			continue;
		destroyBrokenJoint( world, *e );
		// A bond is taken out of the glue lists; any other joint changes what is
		// tied to what, so they are made again.
		if ( inGlueGraph( *e ) && e->glueIndex >= 0 )
			st.glueRemoved.push_back( e );
		else
			st.glueFull = true;
		brokeIn[e->network].push_back( e );
		++broke;
	}
	return broke;
}

// Whether a primitive offset still names a primitive of the (written) geometry.
struct PrimExists
{
	const GU_Detail* gdp;
	bool operator()( GA_Offset off ) const
	{
		return off >= 0 && off < gdp->getNumPrimitiveOffsets() && gdp->getPrimitive( off ) != nullptr;
	}
};

// A broken joint with a next_constraint_name switches to it: its joint is
// remade as that constraint at the next step, from where the pieces are now,
// and the attribute is cleared. The anchors go where the broken joint's frames
// are now (writeLiveFrames).
void switchToNextConstraint( const b3dop::Box3DWorld& world, Box3DJointEntry& e, bool p64 )
{
	e.broken = false;
	b3dop::JointFrame fa, fb;
	if ( e.brokeLocalKnown && e.extra && world.localToWorld( e.bodyA, e.extra->brokeLocalA, fa ) &&
		 world.localToWorld( e.bodyB, e.extra->brokeLocalB, fb ) )
	{
		e.wroteA = asStored( anchorFrameOf( fa, e.adjA ), p64 );
		e.wroteB = asStored( anchorFrameOf( fb, e.adjB ), p64 );
		e.wrote = true;
		e.switchFrames = true;
	}
}

// The attributes of a switching constraint's next settings: every primitive
// attribute `next_<setting>` (but next_constraint_name) holds the value the
// primitive's `<setting>` takes when it switches, "not set" (kUnsetFloat for
// numbers, a negative integer for toggles, an empty string) meaning none.
struct NextSettings
{
	std::vector<GA_Attribute*> attribs;

	explicit NextSettings( GU_Detail* gdp )
	{
		constexpr const char* kPrefix = "next_";
		const GA_AttributeDict& dict = gdp->getAttributes().getDict( GA_ATTRIB_PRIMITIVE );
		for ( GA_AttributeDict::iterator it = dict.begin( GA_SCOPE_PUBLIC ); !it.atEnd(); ++it )
		{
			GA_Attribute* a = it.attrib();
			const char* name = a->getName().c_str();
			if ( std::strncmp( name, kPrefix, 5 ) == 0 && std::strcmp( name, kNextConstraintName ) != 0 )
				attribs.push_back( a );
		}
	}
};

// Moves the primitive's next settings onto its own: each `next_<x>` that is set
// becomes `<x>` (made, with every other primitive "not set", if the geometry
// has none) and is cleared, so it applies once. `next_constraint_type` is
// `constraint_type`. True if any moved.
bool applyNextSettings( GU_Detail* gdp, const NextSettings& next, GA_Offset prim )
{
	bool moved = false;
	for ( GA_Attribute* from : next.attribs )
	{
		const std::string base = std::string( from->getName().c_str() + 5 );
		if ( from->getAIFStringTuple() )
		{
			GA_ROHandleS value( from );
			const UT_StringHolder text = value.isValid() ? value.get( prim ) : UT_StringHolder();
			if ( !text.isstring() || text.length() == 0 )
				continue;
			if ( !gdp->findPrimitiveAttribute( base.c_str() ) )
				gdp->addStringTuple( GA_ATTRIB_PRIMITIVE, base.c_str(), 1 );
			GA_RWHandleS to( gdp, GA_ATTRIB_PRIMITIVE, base.c_str() );
			if ( !to.isValid() )
				continue;
			to.set( prim, text );
			GA_RWHandleS( from ).set( prim, UT_StringHolder() );
			to.getAttribute()->bumpDataId();
			from->bumpDataId();
			moved = true;
			continue;
		}
		const GA_AIFTuple* tuple = from->getAIFTuple();
		if ( !tuple )
			continue;
		const int n = std::min( tuple->getTupleSize( from ), 4 );
		const bool isFloat =
			tuple->getStorage( from ) == GA_STORE_REAL32 || tuple->getStorage( from ) == GA_STORE_REAL64;
		fpreal64 v[4] = { 0, 0, 0, 0 };
		tuple->get( from, prim, v, n );
		// "Not set": a number at kUnsetFloat, a toggle or integer below 0.
		if ( isFloat ? b3dop::isUnsetFloat( static_cast<float>( v[0] ) ) : v[0] < 0 )
			continue;
		GA_Attribute* to = gdp->findPrimitiveAttribute( base.c_str() );
		if ( !to )
		{
			fpreal64 unset[4] = { isFloat ? static_cast<fpreal64>( b3dop::kUnsetFloat ) : -1.0, 0, 0, 0 };
			const GA_Defaults defaults( unset, n );
			to = isFloat ? gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, base.c_str(), n, defaults )
						 : gdp->addIntTuple( GA_ATTRIB_PRIMITIVE, base.c_str(), n, defaults );
		}
		const GA_AIFTuple* target = to ? to->getAIFTuple() : nullptr;
		if ( !target )
			continue;
		target->set( to, prim, v, n );
		fpreal64 clear[4] = { isFloat ? static_cast<fpreal64>( b3dop::kUnsetFloat ) : -1.0, 0, 0, 0 };
		tuple->set( from, prim, clear, n );
		to->bumpDataId();
		from->bumpDataId();
		moved = true;
	}
	return moved;
}

// The joints that broke: a primitive with a next_constraint_name switches to
// it, with its next settings (applyNextSettings); the rest go into the `broken`
// group. True if any switched.
bool writeBrokenJoints( const b3dop::Box3DWorld& world, GU_Detail* gdp, const PrimExists& primOk,
						const std::vector<Box3DJointEntry*>& broken )
{
	GA_RWHandleS conNameH( gdp, GA_ATTRIB_PRIMITIVE, kConstraintName );
	GA_RWHandleS nextH( gdp, GA_ATTRIB_PRIMITIVE, kNextConstraintName );
	GA_PrimitiveGroup* grp = nullptr;
	bool switched = false;
	const NextSettings next( gdp );
	for ( Box3DJointEntry* e : broken )
	{
		if ( !primOk( e->prim ) )
			continue;
		UT_StringHolder nextName;
		if ( nextH.isValid() && conNameH.isValid() )
			nextName = nextH.get( e->prim );
		if ( nextName.isstring() )
		{
			conNameH.set( e->prim, nextName );
			nextH.set( e->prim, UT_StringHolder() );
			applyNextSettings( gdp, next, e->prim );
			switchToNextConstraint( world, *e, box3dP64( gdp ) );
			switched = true;
			continue;
		}
		if ( !grp )
		{
			grp = gdp->findPrimitiveGroup( kBrokenGroup );
			if ( !grp )
				grp = gdp->newPrimitiveGroup( kBrokenGroup );
		}
		if ( grp )
			grp->addOffset( e->prim );
	}
	if ( grp )
		grp->bumpDataId();
	if ( switched )
	{
		conNameH.getAttribute()->bumpDataId();
		nextH.getAttribute()->bumpDataId();
	}
	return switched;
}

// Every joint is quiet (markQuietJoints): nothing of it needs writing.
bool allQuiet( const std::vector<Box3DJointEntry*>& entries )
{
	return std::all_of( entries.begin(), entries.end(), []( const Box3DJointEntry* e ) { return e->quiet; } );
}

// The bodies' relative rotation since rest, in radians (Bullet's `angle`): 0
// while the joint holds them as they started, however they were rotated then.
// Box3D's own reading compares the bodies' absolute rotations instead.
float angleSinceRest( const b3dop::Box3DWorld& world, const Box3DJointEntry& e, float fallback )
{
	float now[2][4] = { { 0, 0, 0, 1 }, { 0, 0, 0, 1 } };
	const b3dop::BodyHandle bodies[2] = { e.bodyA, e.bodyB };
	for ( int i = 0; i < 2; ++i )
	{
		double pos[3];
		if ( bodies[i] && !world.getTransformByHandle( bodies[i], pos, now[i] ) )
			return fallback;
	}
	float sinceA[4], sinceB[4], rel[4];
	box3dAbsoluteToRest( e.restRotA, now[0], sinceA );
	box3dAbsoluteToRest( e.restRotB, now[1], sinceB );
	box3dAbsoluteToRest( sinceA, sinceB, rel );
	const float w = std::min( 1.0f, std::fabs( rel[3] ) );
	return 2.0f * std::acos( w );
}

// One joint's readings, worked out before they are written.
struct FeedbackItem
{
	Box3DJointEntry* e;
	b3dop::JointFeedback fb;
	float distance;
	float angle;
	bool ok;		// the readings were taken
	bool primValid; // the joint's primitive is in the geometry
};

// Feedback, under the Constraint Network DOP's names: force / distance for the
// linear part, torque / angle (radians) for the angular part, plus the per-type
// readings.
void writeFeedback( const b3dop::Box3DWorld& world, GU_Detail* gdp, const PrimExists& primOk,
					const std::vector<Box3DJointEntry*>& entries )
{
	if ( allQuiet( entries ) )
		return;
	// The handles by name (a handful, all string literals): a linear search with
	// no string built per lookup. A deque, so the references stay valid.
	std::deque<std::pair<const char*, GA_RWHandleF>> scalars;
	auto scalar = [&]( const char* name ) -> GA_RWHandleF& {
		for ( auto& s : scalars )
			if ( s.first == name || std::strcmp( s.first, name ) == 0 )
				return s.second;
		if ( !gdp->findPrimitiveAttribute( name ) )
			gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, name, 1 );
		scalars.emplace_back( name, GA_RWHandleF( gdp, GA_ATTRIB_PRIMITIVE, name ) );
		return scalars.back().second;
	};
	GA_RWHandleF& forceH = scalar( "force" );
	GA_RWHandleF& distH = scalar( "distance" );
	GA_RWHandleF& torqueH = scalar( "torque" );
	GA_RWHandleF& angleH = scalar( "angle" );
	// The joints that carry readings this step.
	// Kept between steps (a reference: the lambdas below run on other threads,
	// which would each find their own empty copy of a thread_local): no
	// allocation, and no fresh pages to touch for thousands of joints.
	thread_local std::vector<FeedbackItem> itemStore;
	itemStore.clear();
	std::vector<FeedbackItem>& items = itemStore;
	items.reserve( entries.size() );
	bool glueImpactNeeded = false;
	for ( Box3DJointEntry* e : entries )
	{
		if ( e->quiet )
			continue;
		if ( e->glue && e->inCompound )
		{
			// A weld inside a compound carries nothing: zeros once, then `impact`
			// when it changed.
			if ( e->extra && primOk( e->prim ) )
			{
				if ( !e->feedbackZeroed )
				{
					for ( GA_RWHandleF* h : { &forceH, &distH, &torqueH, &angleH } )
						if ( h->isValid() )
							h->set( e->prim, 0.0f );
					e->feedbackZeroed = true;
				}
				if ( e->extra->glueImpact != e->impactWritten )
				{
					GA_RWHandleF& impactH = scalar( kImpact );
					if ( impactH.isValid() )
						impactH.set( e->prim, e->extra->glueImpact );
					e->impactWritten = e->extra->glueImpact;
				}
			}
			continue;
		}
		e->feedbackZeroed = false;
		// (a Glue's impact is written with the readings below; its primitive is
		// looked up there too)
		glueImpactNeeded = glueImpactNeeded || ( e->glue && e->extra );
		items.push_back( { e, {}, 0.0f, 0.0f, false, false } );
	}
	GA_RWHandleF* impactH = glueImpactNeeded ? &scalar( kImpact ) : nullptr;

	// The readings of every joint (read-only calls on the world and the entry,
	// one item each) are worked out in parallel; the attributes the per-type
	// readings need are made serially; the values are then written in parallel,
	// each joint to its own primitive, once the pages are hardened.
	constexpr size_t kMinParallelFeedback = 256;
	const auto forEach = [&items]( const auto& f ) { box3dParallelFor( items.size(), f, kMinParallelFeedback, 64 ); };
	forEach( [&]( size_t i ) {
		FeedbackItem& it = items[i];
		const Box3DJointEntry* e = it.e;
		it.primValid = primOk( e->prim );
		if ( !it.primValid )
			return;
		it.ok = world.jointFeedback( e->handle, e->type, it.fb );
		if ( !it.ok )
			return;
		// As RBD Bullet Solver: how far apart the two anchors are now (the ones
		// written to current_P), not how far the joint is from satisfied.
		// (writeLiveFrames, just before, stored this step's anchors.)
		it.distance = it.fb.linearSeparation;
		if ( e->wrote )
		{
			const double d[3] = { e->wroteA.pos[0] - e->wroteB.pos[0], e->wroteA.pos[1] - e->wroteB.pos[1],
								  e->wroteA.pos[2] - e->wroteB.pos[2] };
			it.distance = static_cast<float>( b3dop::length3( d ) );
		}
		it.angle = angleSinceRest( world, *e, it.fb.angularSeparation );
	} );
	for ( const FeedbackItem& it : items )
		if ( it.ok )
			for ( int k = 0; k < it.fb.extraCount; ++k )
				scalar( it.fb.extraName[k] );
	if ( items.size() >= kMinParallelFeedback )
		for ( auto& sc : scalars )
			if ( sc.second.isValid() )
				sc.second.getAttribute()->hardenAllPages();
	forEach( [&]( size_t i ) {
		const FeedbackItem& it = items[i];
		if ( !it.primValid )
			return;
		const GA_Offset prim = it.e->prim;
		if ( impactH && impactH->isValid() && it.e->glue && it.e->extra )
		{
			impactH->set( prim, it.e->extra->glueImpact );
			it.e->impactWritten = it.e->extra->glueImpact;
		}
		if ( !it.ok )
			return;
		if ( forceH.isValid() )
			forceH.set( prim, b3dop::length3( it.fb.force ) );
		if ( distH.isValid() )
			distH.set( prim, it.distance );
		// As RBD Bullet Solver: the force's moment about the first piece's centre.
		if ( torqueH.isValid() )
			torqueH.set( prim, b3dop::length3( it.fb.forceMoment ) );
		if ( angleH.isValid() )
			angleH.set( prim, it.angle );
		for ( int k = 0; k < it.fb.extraCount; ++k )
		{
			// Made above: only found here.
			GA_RWHandleF& h = scalar( it.fb.extraName[k] );
			if ( h.isValid() )
				h.set( prim, it.fb.extraIsAngle[k] ? it.fb.extra[k] * b3dop::kRadToDeg : it.fb.extra[k] );
		}
	} );
	for ( auto& sc : scalars )
		if ( sc.second.isValid() )
			sc.second.getAttribute()->bumpDataId();
}

// A body that can still move: not the world, not static, and awake.
bool bodyMoves( const b3dop::Box3DWorld& world, b3dop::BodyHandle h )
{
	return h && world.typeByHandle( h ) != b3dop::BodySettings::Type::Static && world.isAwakeByHandle( h );
}

// Joints whose bodies are both at rest and were already written that way, and
// joints that broke and had their last values written: their frames and
// feedback cannot have changed, so this step does not write them.
void markQuietJoints( const b3dop::Box3DWorld& world, const Box3DSharedState& st,
					  const std::vector<Box3DJointEntry*>& entries )
{
	// One entry per iteration: the world is only read (as in writeFeedback's
	// parallel pass) and each iteration writes its own entry.
	constexpr size_t kMinParallelQuiet = 512;
	box3dParallelFor(
		entries.size(),
		[&]( size_t i ) {
			Box3DJointEntry* e = entries[i];
			// A bond inside a compound joins parked bodies (never awake) that move
			// with their compound: it is at rest when the compound's body is.
			bool compoundAwake = false;
			if ( e->inCompound )
			{
				const auto c = st.compoundOf.find( e->bodyA );
				compoundAwake = c == st.compoundOf.end() || world.isAwakeByHandle( c->second.first->body );
			}
			const bool asleep =
				e->handle && !compoundAwake && !bodyMoves( world, e->bodyA ) && !bodyMoves( world, e->bodyB );
			const bool nothingPending = !e->writeLocal && !e->switchFrames && e->madeRestLength < 0.0f &&
										// a Glue's impact decays each step until it reaches zero
										( !e->glue || ( e->extra && e->extra->glueImpact == e->impactWritten ) );
			// A joint that broke has no frames or feedback left to write, only a
			// Glue's final impact.
			const bool spent = e->broken && !e->handle && nothingPending;
			e->quiet = spent || ( asleep && e->asleepWritten && e->wrote && nothingPending );
			e->asleepWritten = asleep;
		},
		kMinParallelQuiet );
}

// Enable Aging, as RBD Bullet Solver: every constraint primitive's `age` (made
// if missing) grows by the time step.
void ageConstraints( GU_Detail* gdp, float dt )
{
	if ( !gdp->findPrimitiveAttribute( "age" ) )
		gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, "age", 1 );
	GA_RWHandleF age( gdp, GA_ATTRIB_PRIMITIVE, "age" );
	if ( !age.isValid() )
		return;
	for ( GA_Iterator it( gdp->getPrimitiveRange() ); !it.atEnd(); ++it )
		age.set( *it, age.get( *it ) + dt );
	age.getAttribute()->bumpDataId();
}

// One network's results into its (locked, writable) geometry. True if a
// constraint_name switched.
bool writeNetworkResults( const b3dop::Box3DWorld& world, const Box3DSharedState& st, GU_Detail* gdp,
						  const std::vector<Box3DJointEntry*>& entries, const std::vector<Box3DJointEntry*>* broken,
						  float ageDt )
{
	const PrimExists primOk{ gdp };
	const bool switched = broken && writeBrokenJoints( world, gdp, primOk, *broken );
	writeLiveFrames( world, st, gdp, entries, primOk );
	writeFeedback( world, gdp, primOk, entries );
	if ( ageDt > 0.0f )
		ageConstraints( gdp, ageDt );
	return switched;
}

} // namespace

int jointsAfterStep( Box3DSharedState& st, SIM_ObjectArray& objects )
{
	b3dop::Box3DWorld& world = st.world;

	EntriesByNetwork brokeIn; // network -> joints that broke
	const int broke = breakJoints( st, brokeIn );
	refreshLivePoses( st );

	// ---- write-back: broken group / next_constraint_name, live frames, feedback
	// ----
	EntriesByNetwork& entriesOf = st.networkEntries;
	for ( auto& kv : st.jointNetworks )
	{
		Box3DJointNetwork& net = kv.second;
		const auto broken = brokeIn.find( kv.first );
		const bool anyBroken = broken != brokeIn.end();
		if ( net.visited != st.syncStamp )
			continue;
		refreshNetworkEntries( st );
		const auto ents = entriesOf.find( kv.first );
		if ( ents == entriesOf.end() && !anyBroken )
			continue; // no joints to keep live, nothing broke
		static const std::vector<Box3DJointEntry*> kNone;
		const std::vector<Box3DJointEntry*>& entries = ents != entriesOf.end() ? ents->second : kNone;
		// Nothing broke, no age to add and every joint is as it was last written
		// (both bodies at rest): the geometry is left alone, with no write lock,
		// no new data ids and no fingerprint to take again.
		markQuietJoints( world, st, entries );
		if ( !anyBroken && st.constraintAgeDt <= 0.0f && allQuiet( entries ) )
			continue;
		SIM_Object* obj = box3dObjectById( objects, net.objectId, st.objectsSorted );
		NetworkSource src;
		src.inGroup = net.inGroup;
		src.relName = net.relName;
		SIM_GeometryCopy* geo = obj ? writableNetworkGeometry( *obj, src ) : nullptr;
		net.writable = geo != nullptr;
		if ( !geo )
			continue;
		bool switched = false;
		{
			SIM_GeometryAutoWriteLock lock( geo );
			GU_Detail* gdp = &lock.getGdp();
			switched = writeNetworkResults( world, st, gdp, entries, anyBroken ? &broken->second : nullptr,
											st.constraintAgeDt );
		}
		// Our own write is not an edit: re-record the fingerprint (the lock is
		// released). A switched constraint_name is ours too, but must be read:
		// force a re-read.
		if ( const SIM_Geometry* rgeo = networkGeometry( *obj, src ) )
		{
			SIM_GeometryAutoReadLock rlock( rgeo );
			if ( const GU_Detail* rgdp = rlock.getGdp() )
				net.fingerprint = networkFingerprint( rgdp );
		}
		if ( switched )
			net.fingerprint = 0;
	}
	return broke;
}
