// Box3DSourceSync.cpp
//
// Animated bodies follow their source SOP, as RBD Bullet Solver's "Updating
// Animated and Deforming Objects". An object made by the Box3D Packed Object
// (or RBD Packed Object) carries SourceObject data whose `objpath` is the SOP
// it was made from. Before the sync, each kinematic body (a point with
// `animated`, or every point of a Kinematic Box3D Data) is moved to the pose
// its piece has in that SOP at the step's end time: P and `orient` are written
// onto the Geometry the way an edit would be, and the sync turns the change
// into a kinematic target; the packed transform turns with it, so the piece is
// drawn as posed. Pieces are matched by their `name`.
#include "sync/Box3DSolverInternal.h"

#include <GA/GA_Handle.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPacked.h>
#include <OP/OP_Context.h>
#include <OP/OP_Director.h>
#include <OP/OP_Node.h>
#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace box3dsolver
{

namespace
{

constexpr const char* kSourceDataName = "SourceObject";

// The source SOP of `obj`, or null; `useTransform` is whether the SOP's object
// transform applies (the data's `usetransform`, written by the Box3D Packed
// Object).
SOP_Node* findSourceSop( const SIM_Object& obj, bool& useTransform )
{
	const SIM_Data* source = obj.getConstNamedSubData( kSourceDataName );
	if ( !source )
		return nullptr;
	UT_StringHolder path;
	if ( !box3dDataOption( *source, "objpath", path ) || !path.isstring() )
		return nullptr;
	fpreal64 use = 0.0;
	useTransform = box3dDataOption( *source, "usetransform", use ) && use != 0.0;
	OP_Node* node = OPgetDirector()->findNode( path.c_str() );
	return node ? node->castToSOPNode() : nullptr;
}

// The pose a source piece has: its position and rotation through `xf` (the
// SOP object's world transform, or identity).
bool sourcePose( const GU_Detail* src, GA_Offset off, const UT_Matrix4D& xf, double pos[3], float rot[4],
				 double scale[3] )
{
	const PackedInfo pk = getPackedInfo( src, off );
	if ( !pk.prim )
		return false;
	UT_Matrix4D m;
	for ( unsigned r = 0; r < 4; ++r )
		for ( unsigned c = 0; c < 4; ++c )
			m( r, c ) = pk.m4[r * 4 + c];
	m *= xf;
	UT_Vector3D p( pk.P[0], pk.P[1], pk.P[2] );
	p *= xf;
	pos[0] = p.x();
	pos[1] = p.y();
	pos[2] = p.z();
	double m3[9];
	for ( unsigned r = 0; r < 3; ++r )
		for ( unsigned c = 0; c < 3; ++c )
			m3[r * 3 + c] = m( r, c );
	return splitPackedMatrix( m3, rot, scale );
}

// The poses the pieces in `names` (a subset of cache.wanted) have in `sop` at
// global time `t`, by name. The source's points are walked only when its point
// count or `name` attribute changed since the cache's index was made.
void readSourcePoses( SOP_Node& sop, double t, bool useTransform, Box3DSourceCache& cache,
					  const std::unordered_map<std::string, bool>& names,
					  std::unordered_map<std::string, Box3DSourcePose>& out )
{
	OP_Context context( t );
	const GU_Detail* src = sop.getCookedGeo( context );
	if ( !src )
		return;
	UT_Matrix4D xf( 1.0 );
	if ( useTransform && sop.getCreator() )
		sop.getCreator()->getWorldTransform( xf, context );
	GA_ROHandleS srcNames( src, GA_ATTRIB_POINT, "name" );
	if ( !srcNames.isValid() )
		return;
	const uint64_t key = b3dop::mixHash(
		b3dop::mixHash( b3dop::mixHash( b3dop::kHashSeed, static_cast<uint64_t>( src->getNumPoints() ) ),
						static_cast<uint64_t>( src->getNumPointOffsets() ) ),
		static_cast<uint64_t>( srcNames.getAttribute()->getDataId() ) );
	if ( !cache.indexKnown || cache.indexKey != key )
	{
		cache.index.clear();
		GA_Offset off;
		GA_FOR_ALL_PTOFF( src, off )
		{
			const std::string name = srcNames.get( off ).c_str();
			if ( cache.wanted.find( name ) != cache.wanted.end() )
				cache.index[name] = off; // the last point of a name, as a scan finds
		}
		cache.indexKey = key;
		cache.indexKnown = true;
	}
	for ( const auto& kv : names )
	{
		const auto it = cache.index.find( kv.first );
		if ( it == cache.index.end() )
			continue;
		Box3DSourcePose pose;
		if ( sourcePose( src, it->second, xf, pose.pos, pose.rot, pose.scale ) )
			out[kv.first] = pose;
	}
}

// Row lengths that differ by more than float noise.
bool scalesDiffer( const double a[3], const double b[3] )
{
	for ( int i = 0; i < 3; ++i )
		if ( std::fabs( a[i] - b[i] ) > 1e-6 * std::fabs( b[i] ) )
			return true;
	return false;
}

bool samePose( const Box3DSourcePose& a, const Box3DSourcePose& b )
{
	for ( int i = 0; i < 3; ++i )
		if ( std::fabs( a.pos[i] - b.pos[i] ) > 1e-6 )
			return false;
	return !scalesDiffer( a.scale, b.scale ) && !b3dop::quatsDiffer( a.rot, b.rot, 1e-6 );
}

} // namespace

void followSourceSops( Box3DSharedState& st, SIM_ObjectArray& objects, double now, double dt )
{
	for ( int i = 0; i < objects.entries(); ++i )
	{
		SIM_Object& obj = *objects( i );
		bool useTransform = false;
		SOP_Node* sop = findSourceSop( obj, useTransform );
		if ( !sop )
			continue;

		// The pieces that move: with `animated`, or all of a Kinematic Box3D Data.
		const SIM_Box3DData* dataNode = SIM_DATA_GETCONST( obj, kBox3DDataName, SIM_Box3DData );
		const bool allKinematic = dataNode && static_cast<int>( dataNode->getBodyType() ) == 1;
		Box3DSourceCache& cache = st.sourceCaches[obj.getObjectId()];
		{
			const SIM_Geometry* geo = SIM_DATA_GETCONST( obj, SIM_GEOMETRY_DATANAME, SIM_Geometry );
			if ( !geo )
				continue;
			SIM_GeometryAutoReadLock lock( geo );
			const GU_Detail* gdp = lock.getGdp();
			if ( !gdp )
				continue;
			GA_ROHandleS names( gdp, GA_ATTRIB_POINT, "name" );
			GA_ROHandleI animated( gdp, GA_ATTRIB_POINT, "animated" );
			GA_ROHandleM4 restxform( gdp, GA_ATTRIB_POINT, "restxform" );
			GA_ROHandleQ orient( gdp, GA_ATTRIB_POINT, "orient" );
			if ( !names.isValid() || !restxform.isValid() || !orient.isValid() ||
				 ( !allKinematic && !animated.isValid() ) )
				continue;
			// The followers depend on these markers only, so an unchanged object is
			// not walked. Not on P, `orient` or the primitive list: they change every
			// step (the solver writes them, and this function the packed transforms).
			const auto idOf = []( const GA_ROHandleS& h ) {
				return h.isValid() ? static_cast<uint64_t>( h.getAttribute()->getDataId() ) : ~0ull;
			};
			uint64_t key = b3dop::kHashSeed;
			key = b3dop::mixHash( key, static_cast<uint64_t>( gdp->getNumPoints() ) );
			key = b3dop::mixHash( key, static_cast<uint64_t>( gdp->getNumPointOffsets() ) );
			key = b3dop::mixHash( key, idOf( names ) );
			key = b3dop::mixHash( key, static_cast<uint64_t>( restxform.getAttribute()->getDataId() ) );
			key = b3dop::mixHash(
				key, animated.isValid() ? static_cast<uint64_t>( animated.getAttribute()->getDataId() ) : ~0ull );
			key = b3dop::mixHash( key, allKinematic ? 1 : 2 );
			if ( !cache.followersKnown || cache.followersKey != key )
			{
				cache.followers.clear();
				cache.wanted.clear();
				GA_Offset off;
				GA_FOR_ALL_PTOFF( gdp, off )
				{
					if ( !allKinematic && animated.get( off ) == 0 )
						continue;
					Box3DSourceFollower f;
					f.off = off;
					f.name = names.get( off ).c_str();
					const UT_Matrix4F rx = restxform.get( off );
					if ( !restRotation( rx, f.rest ) )
						continue;
					for ( unsigned r = 0; r < 3; ++r )
						f.restScale[r] = std::sqrt( static_cast<double>( rx( r, 0 ) ) * rx( r, 0 ) +
													static_cast<double>( rx( r, 1 ) ) * rx( r, 1 ) +
													static_cast<double>( rx( r, 2 ) ) * rx( r, 2 ) );
					cache.wanted.emplace( f.name, true );
					cache.followers.push_back( std::move( f ) );
				}
				cache.followersKey = key;
				cache.followersKnown = true;
				cache.indexKnown = false; // the wanted names changed
			}
		}
		const std::vector<Box3DSourceFollower>& followers = cache.followers;
		if ( followers.empty() )
			continue;
		const std::unordered_map<std::string, bool>& wanted = cache.wanted;

		// What each piece had when last read; pieces seen for the first time take
		// the SOP a step earlier.
		const SIM_Engine& engine = obj.getEngine();
		auto& known = st.sourcePoses[obj.getObjectId()];
		std::unordered_map<std::string, bool> unseen;
		for ( const Box3DSourceFollower& f : followers )
			if ( known.find( f.name ) == known.end() )
				unseen.emplace( f.name, true );
		if ( !unseen.empty() )
		{
			std::unordered_map<std::string, Box3DSourcePose> before;
			readSourcePoses( *sop, engine.getGlobalTime( SIM_Time( now - dt ) ), useTransform, cache, unseen, before );
			for ( const auto& kv : before )
				known[kv.first] = kv.second;
		}
		std::unordered_map<std::string, Box3DSourcePose> current;
		readSourcePoses( *sop, engine.getGlobalTime( SIM_Time( now ) ), useTransform, cache, wanted, current );

		// The pieces whose pose changed, and of those the ones whose scale did. On
		// the object's first step every piece is written, even one whose source
		// is still, as Bullet's: its packed transform becomes the source's,
		// replacing a `pscale` or `orient` the packed object baked in when it made
		// the piece. Its body is made in the sync after this, already at that
		// pose, so nothing sweeps there. Only then: a state rebuilt later (a
		// reloaded checkpoint) reads every piece for the first time too, and must
		// not undo edits made since (a SOP Solver's). The creation time is the
		// object's, kept through a reload.
		const bool firstStep = now - dt <= static_cast<double>( obj.getCreationTime() ) + 0.5 * dt;
		struct Move
		{
			const Box3DSourceFollower* follower;
			const Box3DSourcePose* pose;
			bool scaled;
		};
		std::vector<Move> moves;
		for ( const Box3DSourceFollower& f : followers )
		{
			auto it = current.find( f.name );
			if ( it == current.end() )
				continue;
			auto was = known.find( f.name );
			const bool moved = was != known.end() && !samePose( was->second, it->second );
			const bool scaled = moved && scalesDiffer( was->second.scale, it->second.scale );
			known[f.name] = it->second;
			if ( moved || ( firstStep && unseen.find( f.name ) != unseen.end() ) )
				moves.push_back( { &f, &it->second, scaled } );
		}
		if ( moves.empty() )
			continue;

		SIM_GeometryCopy* geoCopy = box3dWritableGeometry( obj, SIM_GEOMETRY_DATANAME );
		if ( !geoCopy )
			continue;
		SIM_GeometryAutoWriteLock lock( geoCopy );
		GU_Detail* gdp = &lock.getGdp();
		GA_RWHandleQ orient( gdp, GA_ATTRIB_POINT, "orient" );
		if ( !orient.isValid() )
			continue;
		bool wrotePacked = false;
		// `scale` on the points, as RBD Bullet Solver writes it: the piece's scale
		// against its rest, which Transform Pieces applies to the piece (added at
		// 1 for every point the first time a piece's scale changes).
		GA_RWHandleV3 scaleAttr( gdp, GA_ATTRIB_POINT, "scale" );
		// `pscale` goes to 1 on a piece that follows, as Bullet writes it: the
		// packed object baked it into the packed transform when it made the piece,
		// and the source's transform (which has no `pscale`) replaces that now.
		GA_RWHandleF pscale( gdp, GA_ATTRIB_POINT, "pscale" );
		bool wrotePscale = false;
		for ( const Move& mv : moves )
		{
			const Box3DSourceFollower& f = *mv.follower;
			const Box3DSourcePose& pose = *mv.pose;
			float since[4];
			box3dAbsoluteToRest( f.rest, pose.rot, since );
			b3dop::sanitizeQuat( since );
			gdp->setPos3( f.off, UT_Vector3( static_cast<float>( pose.pos[0] ), static_cast<float>( pose.pos[1] ),
											 static_cast<float>( pose.pos[2] ) ) );
			orient.set( f.off, UT_QuaternionF( since[0], since[1], since[2], since[3] ) );
			if ( pscale.isValid() && pscale.get( f.off ) != 1.0f )
			{
				pscale.set( f.off, 1.0f );
				wrotePscale = true;
			}
			// The packed transform is the source's, as Bullet's: the absolute
			// rotation (which `orient` since rest gives the body), so the piece is
			// drawn as it is posed, and the source's row lengths, which the sync
			// sizes the shapes by (a convex or concave piece is rebuilt when they
			// change). On the first step they replace a `pscale` the packed object
			// baked in; the row lengths are rewritten only when they differ, so
			// float noise never rebuilds a shape.
			const PackedInfo pk = getPackedInfo( gdp, f.off );
			if ( pk.prim )
			{
				double m3[9];
				composePackedMatrix( pose.rot, scalesDiffer( pk.scale, pose.scale ) ? pose.scale : pk.scale, m3 );
				b3dop::keepMirror( pk.m3, m3 );
				setLocal3( *static_cast<GU_PrimPacked*>( gdp->getPrimitive( pk.primOff ) ), m3 );
				wrotePacked = true;
			}
			// `scale` only when the source's own scale changed.
			if ( !mv.scaled || !pk.prim )
				continue;
			if ( !scaleAttr.isValid() )
			{
				const GA_Defaults ones( 1.0 );
				scaleAttr = GA_RWHandleV3(
					gdp->addFloatTuple( GA_ATTRIB_POINT, "scale", 3, ones, nullptr, nullptr, GA_STORE_REAL32 ) );
			}
			if ( scaleAttr.isValid() )
				scaleAttr.set( f.off, UT_Vector3( static_cast<float>( pose.scale[0] / f.restScale[0] ),
												  static_cast<float>( pose.scale[1] / f.restScale[1] ),
												  static_cast<float>( pose.scale[2] / f.restScale[2] ) ) );
		}
		if ( scaleAttr.isValid() )
			scaleAttr.getAttribute()->bumpDataId();
		if ( wrotePscale )
			pscale.getAttribute()->bumpDataId();
		if ( wrotePacked )
			gdp->getPrimitiveList().bumpDataId();
		if ( GA_Attribute* p = gdp->getP() )
			p->bumpDataId();
		orient.getAttribute()->bumpDataId();
	}
}

} // namespace box3dsolver
