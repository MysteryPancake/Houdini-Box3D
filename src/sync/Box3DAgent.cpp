// Box3DAgent.cpp
//
// Crowd agents as ragdolls: one body per rig transform that has a shape in the
// agent's collision layer, as the RBD Bullet Solver makes them (docs/agents/
// todo.md, "Agent ragdolls"). An agent point is a ragdoll when its
// `active_agent` array has elements; the point itself gets no body (a ragdoll
// agent is not a piece). Bodies only: nothing is written back to the agent yet.
#include "Box3DSolverInternal.h"

#include <GA/GA_PrimitiveTypes.h>
#include <GEO/GEO_Primitive.h>
#include <GU/GU_Agent.h>
#include <GU/GU_AgentDefinition.h>
#include <GU/GU_AgentLayer.h>
#include <GU/GU_AgentRig.h>
#include <GU/GU_AgentShapeLib.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <functional>
#include <unordered_set>
#include <vector>

namespace box3dsolver
{

namespace
{

// The bodies of one agent point take ids above every point id: the point's id
// times kAgentTransforms plus the rig transform's index.
constexpr int64_t kAgentIdBase = static_cast<int64_t>( 1 ) << 40;
constexpr int64_t kAgentTransforms = 4096;

// Bullet's `mass_agent` for the Agent Collision Layer's capsule shape (a Tube
// primitive with a `tubeCapt` attribute) is this times the density times its
// volume (the Tube's `measuredvolume`, its cylinder, in the agent's units):
// measured on the mocap biped of the PinnedRagdolls and PartialRagdolls
// examples, the same for every shape of it and for capsules twice as long; not
// derived. For a sphere, box or Custom shape (a Tube in one included) it is the
// density times the volume (measured the same way).
constexpr double kAgentMassScale = 5.0 / 3.0;

// The shape's own volume is used for its mass while it is within these of its
// hull's (planBody), else the hull's.
constexpr double kMinVolumeOfHull = 0.05;
constexpr double kMaxVolumeOfHull = 1.5;

const GU_Agent* agentOf( const GU_PrimPacked* prim )
{
	return prim ? dynamic_cast<const GU_Agent*>( prim->sharedImplementation() ) : nullptr;
}

int64_t agentBodyId( int64_t pointId, int transform )
{
	return kAgentIdBase + pointId * kAgentTransforms + transform;
}

// One rig transform with a collision shape: its geometry in the transform's own
// space (a Tube primitive on one point, which is the body's centre of mass).
struct TransformShape
{
	int transform = -1;
	GU_ConstDetailHandle geometry;
};

// The shapes of the agent's first collision layer, one per transform with a
// shape (a second shape on a transform is left out). False: no collision layer.
bool collectShapes( const GU_Agent& agent, std::vector<TransformShape>& out )
{
	const UT_Array<GU_AgentLayerConstPtr>& layers = agent.getCollisionLayers();
	if ( layers.entries() == 0 || !layers( 0 ) )
		return false;
	const GU_AgentShapeLibConstPtr& lib = agent.definition().shapeLibrary();
	if ( !lib )
		return false;
	const int count = static_cast<int>( agent.transformCount() );
	std::vector<char> seen( static_cast<size_t>( std::max( count, 0 ) ), 0 );
	for ( const GU_AgentLayer::ShapeBinding& b : *layers( 0 ) )
	{
		const int t = b.transformId();
		if ( t < 0 || t >= count || seen[static_cast<size_t>( t )] || !b.shape() )
			continue;
		TransformShape s;
		s.transform = t;
		s.geometry = b.shape()->shapeGeometry( *lib );
		if ( !s.geometry.isValid() )
			continue;
		seen[static_cast<size_t>( t )] = 1;
		out.push_back( std::move( s ) );
	}
	return !out.empty();
}

// The body of one transform: its pose is the transform's world pose (the
// agent's own transform times the transform's world transform in the agent)
// with the origin at the shape's point.
struct BodyPlan
{
	Box3DBodyPose pose;
	BuiltShapes shapes;
	// The shape's own volume (a Tube's, exactly, not its sampled hull's) in the
	// body's units, and what Bullet's `mass_agent` is of the density times it.
	double volume = 0.0;
	double massScale = 1.0;
};

// Where the body's centre is in the shape's own space, as Bullet has it: the
// point of a lone Tube or Sphere primitive, else the centroid of the convex
// hull of the shape's points, packed geometry unpacked as its hull is built (a
// box's centre; Bullet samples a Custom shape's and lands near it), else, for
// a flat shape, the centre of its bounding box.
void shapeCentre( const GU_Detail& sgdp, double out[3] )
{
	if ( sgdp.getNumPrimitives() == 1 && sgdp.getNumPoints() == 1 &&
		 sgdp.countPrimitiveType( GA_PRIMTUBE ) + sgdp.countPrimitiveType( GA_PRIMSPHERE ) == 1 )
	{
		const UT_Vector3 p = sgdp.getPos3( *GA_Iterator( sgdp.getPointRange() ) );
		out[0] = p.x();
		out[1] = p.y();
		out[2] = p.z();
		return;
	}
	static const double kIdentity[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	geometryCentreOfMass( &sgdp, kIdentity, out );
}

// The Agent Collision Layer's capsule shape: a Tube with a `tubeCapt`
// attribute (a Tube in a Custom shape has none).
bool layerCapsule( const GU_Detail& sgdp )
{
	return sgdp.countPrimitiveType( GA_PRIMTUBE ) > 0 && sgdp.findPrimitiveAttribute( "tubeCapt" ) != nullptr;
}

// The body of a shape at its transform's world matrix `world4`, centred on
// `pivot` (shapeCentre's, in the shape's own space).
bool planBody( b3dop::Box3DWorld& world, const TransformShape& shape, const double world4[16], const double pivot[3],
			   BodyPlan& out )
{
	GU_DetailHandleAutoReadLock lock( shape.geometry );
	const GU_Detail* sgdp = lock.getGdp();
	if ( !sgdp || sgdp->getNumPoints() == 0 )
		return false;
	// The transform's world matrix M carries the agent's units: the body's
	// rotation R (splitPackedMatrix's) times a scale, and any shear or mirroring.
	// The shape's points are in the transform's own space, so the body-local
	// points are those times M times R transposed, which keeps all of it (but a
	// Sphere primitive, which stays round, sized by M's first row).
	double m3[9], scale[3];
	b3dop::upper3x3( world4, m3 );
	if ( !splitPackedMatrix( m3, out.pose.rot, scale ) )
		return false;
	UT_DMatrix3 rot;
	UT_QuaternionD( out.pose.rot[0], out.pose.rot[1], out.pose.rot[2], out.pose.rot[3] ).getRotationMatrix( rot );
	double toBody[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j )
			for ( int k = 0; k < 3; ++k )
				toBody[i * 4 + j] += m3[i * 3 + k] * rot( static_cast<unsigned>( j ), static_cast<unsigned>( k ) );
	double bodyPivot[3];
	b3dop::xformPoint( toBody, pivot, bodyPivot );
	out.shapes = buildWholeObjectShapes( world, sgdp, toBody, bodyPivot, b3dop::ShapeType::Convex, false );
	if ( out.shapes.pieces.empty() )
		return false;
	// Each primitive's volume about one point: a closed polygon mesh's faces sum
	// to its volume, a Tube's or Sphere's is its own. Faces that do not close a
	// volume, or close it with mixed winding, sum to something far from the
	// hull's (at most it, a sampled Tube's a little less): then the hull's.
	for ( GA_Iterator it( sgdp->getPrimitiveRange() ); !it.atEnd(); ++it )
		out.volume += sgdp->getGEOPrimitive( *it )->calcVolume( UT_Vector3( 0, 0, 0 ) );
	const double det = m3[0] * ( m3[4] * m3[8] - m3[5] * m3[7] ) - m3[1] * ( m3[3] * m3[8] - m3[5] * m3[6] ) +
					   m3[2] * ( m3[3] * m3[7] - m3[4] * m3[6] );
	out.volume = std::fabs( out.volume * det );
	const double hull = out.shapes.totalVolume;
	if ( !( out.volume > kMinVolumeOfHull * hull && out.volume < kMaxVolumeOfHull * hull ) )
		out.volume = hull;
	out.massScale = layerCapsule( *sgdp ) ? kAgentMassScale : 1.0;
	b3dop::xformPoint( world4, pivot, out.pose.pos );
	return true;
}

// The row-vector product a * b of two row-major 4x4 matrices.
void mul4( const double a[16], const double b[16], double out[16] )
{
	for ( int r = 0; r < 4; ++r )
		for ( int k = 0; k < 4; ++k )
		{
			double v = 0.0;
			for ( int j = 0; j < 4; ++j )
				v += a[r * 4 + j] * b[j * 4 + k];
			out[r * 4 + k] = v;
		}
}

// The shape's point in its transform's own space: the body's centre of mass.
bool shapePivot( const TransformShape& shape, double out[3] )
{
	GU_DetailHandleAutoReadLock lock( shape.geometry );
	const GU_Detail* sgdp = lock.getGdp();
	if ( !sgdp || sgdp->getNumPoints() == 0 )
		return false;
	shapeCentre( *sgdp, out );
	return true;
}

// The frame an anchor on a transform with this world matrix has.
Box3DAgentFrame frameOf( const double world4[16] )
{
	Box3DAgentFrame frame;
	double m3[9], sc[3];
	b3dop::upper3x3( world4, m3 );
	float q[4];
	if ( splitPackedMatrix( m3, q, sc ) )
		frame.rot = { q[0], q[1], q[2], q[3] };
	const double det = m3[0] * ( m3[4] * m3[8] - m3[5] * m3[7] ) - m3[1] * ( m3[3] * m3[8] - m3[5] * m3[6] ) +
					   m3[2] * ( m3[3] * m3[7] - m3[4] * m3[6] );
	frame.mirrored = det < 0.0;
	return frame;
}

// The agent's rig at rest, from its transforms' world matrices now (`now`).
// Its first bodies are made at rest, so rest is now, unless the point's arrays
// (pivot_agent, trans_agent, orient_agent, mass_agent, as writeAgentArrays
// writes them) describe the pose it has now: a simulation reloaded or rebuilt
// mid-way. Then each transform is moved back by the motion since rest of its
// body, or of its closest ancestor's (the inverse of writeAgentPose: axes
// times the turn's transpose, origin less the body's centre now, turned back,
// plus its centre at rest), so its bodies, arrays and pose carry on. True when
// the arrays were used: the bodies then also start at v_agent / w_agent.
bool restOfAgent( const GU_Detail* gdp, GA_Offset off, const GU_AgentRig& rig,
				  const std::vector<std::array<double, 16>>& now, const std::vector<TransformShape>& shapes,
				  Box3DAgentRest& rest )
{
	rest.world = now;
	const size_t n = now.size();
	// Each shape's centre, once for the agent (a hull for most shapes).
	rest.pivots.assign( n, { 0.0, 0.0, 0.0 } );
	rest.hasPivot.assign( n, 0 );
	for ( const TransformShape& shape : shapes )
	{
		const size_t t = static_cast<size_t>( shape.transform );
		if ( t < n && shapePivot( shape, rest.pivots[t].data() ) )
			rest.hasPivot[t] = 1;
	}
	const auto arrayOf = [&]( const char* name, UT_Fpreal32Array& out ) {
		const GA_ROHandleFA h( gdp->findFloatArray( GA_ATTRIB_POINT, name ) );
		if ( h.isValid() )
			h.get( off, out );
		return h.isValid();
	};
	UT_Fpreal32Array pivot, trans, orient, mass;
	const bool arrays = arrayOf( "pivot_agent", pivot ) && arrayOf( "trans_agent", trans ) &&
						arrayOf( "orient_agent", orient ) && arrayOf( "mass_agent", mass ) &&
						static_cast<size_t>( pivot.entries() ) >= 3 * n &&
						static_cast<size_t>( trans.entries() ) >= 3 * n &&
						static_cast<size_t>( orient.entries() ) >= 4 * n && static_cast<size_t>( mass.entries() ) >= n;
	bool any = false;
	for ( const TransformShape& shape : shapes )
	{
		const size_t t = static_cast<size_t>( shape.transform );
		double centre[3];
		if ( !arrays || t >= n || !( mass( static_cast<exint>( t ) ) > 0.0f ) || !rest.hasPivot[t] )
			continue;
		b3dop::xformPoint( now[t].data(), rest.pivots[t].data(), centre );
		double d2 = 0.0, scale = 1.0;
		for ( int k = 0; k < 3; ++k )
		{
			const double at = static_cast<double>( pivot( static_cast<exint>( 3 * t + k ) ) ) +
							  static_cast<double>( trans( static_cast<exint>( 3 * t + k ) ) );
			d2 += ( centre[k] - at ) * ( centre[k] - at );
			scale = std::max( scale, std::fabs( at ) );
		}
		if ( d2 > ( 1e-3 * scale ) * ( 1e-3 * scale ) )
			return false; // the arrays are of another pose: rest is now
		any = true;
	}
	if ( !any )
		return false;
	for ( size_t t = 0; t < n; ++t )
	{
		exint b = static_cast<exint>( t );
		while ( b >= 0 && !( mass( b ) > 0.0f ) )
			b = rig.parentIndex( b );
		if ( b < 0 )
			continue;
		const UT_QuaternionD q( orient( 4 * b ), orient( 4 * b + 1 ), orient( 4 * b + 2 ), orient( 4 * b + 3 ) );
		UT_Matrix3D turn;
		q.getRotationMatrix( turn );
		const double at[3] = { static_cast<double>( pivot( 3 * b ) + trans( 3 * b ) ),
							   static_cast<double>( pivot( 3 * b + 1 ) + trans( 3 * b + 1 ) ),
							   static_cast<double>( pivot( 3 * b + 2 ) + trans( 3 * b + 2 ) ) };
		const double from[3] = { static_cast<double>( pivot( 3 * b ) ), static_cast<double>( pivot( 3 * b + 1 ) ),
								 static_cast<double>( pivot( 3 * b + 2 ) ) };
		const std::array<double, 16>& m = now[t];
		std::array<double, 16>& out = rest.world[t];
		// A row-vector v moved back: v * turn^T, so out = m * turn^T per row.
		for ( int r = 0; r < 4; ++r )
		{
			const double v[3] = { r < 3 ? m[static_cast<size_t>( r * 4 )] : m[12] - at[0],
								  r < 3 ? m[static_cast<size_t>( r * 4 + 1 )] : m[13] - at[1],
								  r < 3 ? m[static_cast<size_t>( r * 4 + 2 )] : m[14] - at[2] };
			for ( int k = 0; k < 3; ++k )
				out[static_cast<size_t>( r * 4 + k )] =
					v[0] * turn( k, 0 ) + v[1] * turn( k, 1 ) + v[2] * turn( k, 2 ) + ( r < 3 ? 0.0 : from[k] );
		}
	}
	return true;
}

} // namespace

namespace
{

// An animated transform's kinematic body to where the animation has the
// transform now (its shape's point through the transform's matrix), reached
// over the step as Box3D drives a kinematic body to a target, so it moves with
// the animation's velocity; when the animation stops, that velocity is
// cleared. `retyped`: the body was simulated until now, so the velocity it had
// then goes too (else it would coast on it for a step).
void followAnimation( b3dop::Box3DWorld& world, const double local[3], const double now4[16], float stepDt,
					  bool retyped, Box3DBodyMirror& entry )
{
	double m3[9], sc[3];
	Box3DBodyPose target;
	b3dop::upper3x3( now4, m3 );
	if ( !splitPackedMatrix( m3, target.rot, sc ) )
		return;
	b3dop::xformPoint( now4, local, target.pos );
	double d2 = 0.0;
	for ( int k = 0; k < 3; ++k )
		d2 += ( target.pos[k] - entry.pos[k] ) * ( target.pos[k] - entry.pos[k] );
	if ( d2 > 1e-14 || b3dop::quatsDiffer( target.rot, entry.rot ) )
	{
		world.setTargetTransformByHandle( entry.handle, target.pos, target.rot, stepDt );
		entry.setPose( target );
		entry.driven = true;
	}
	else if ( entry.driven || retyped )
	{
		const float zero[3] = { 0.0f, 0.0f, 0.0f };
		world.setVelocityByHandle( entry.handle, zero, zero );
		entry.driven = false;
	}
}

} // namespace

bool hasAgentPrims( const GU_Detail* gdp )
{
	return gdp && gdp->countPrimitiveType( GU_Agent::typeId() ) > 0;
}

bool isAgentPrim( const PackedInfo& pk )
{
	return pk.prim && pk.prim->getTypeId() == GU_Agent::typeId();
}

bool isRagdollAgent( const PackedInfo& pk, const GA_ROHandleIA& active, GA_Offset off )
{
	if ( !pk.prim || !active.isValid() || !agentOf( pk.prim ) )
		return false;
	UT_Int32Array values;
	active.get( off, values );
	return values.entries() > 0;
}

void syncAgentBodies( Box3DSharedState& st, Box3DObjectSync& os, int objectId, const GU_Detail* gdp,
					  const std::vector<Box3DPointSlot>& slots, const PointAttrs& a, const b3dop::BodySettings& node,
					  SyncCounts& c )
{
	const GA_ROHandleIA active( gdp->findIntArray( GA_ATTRIB_POINT, "active_agent" ) );
	b3dop::Box3DWorld& world = st.world;
	os.agents.clear();
	os.agentByName.clear();
	os.agentRest.clear();
	os.agentWithoutBody.clear();
	os.agentsNotRagdoll.clear();
	os.agentsLeftRagdoll.clear();
	os.agentsGone.clear();
	const GA_ROHandleFA massArrays( gdp->findFloatArray( GA_ATTRIB_POINT, "mass_agent" ) );
	std::unordered_set<int64_t> seen;
	const GA_ROHandleS primName( gdp, GA_ATTRIB_PRIMITIVE, "name" );
	const GA_ROHandleS pointName( gdp, GA_ATTRIB_POINT, "name" );
	for ( const Box3DPointSlot& s : slots )
	{
		if ( !s.agent )
			continue;
		os.anyAgents = true;
		if ( !s.agentRagdoll )
		{
			// No bodies: a constraint naming this agent makes no joint.
			std::string name;
			if ( primName.isValid() )
				name = primName.get( static_cast<GA_Offset>( s.primOff ) );
			if ( name.empty() && pointName.isValid() )
				name = pointName.get( s.off );
			if ( !name.empty() )
			{
				os.agentsNotRagdoll.insert( name );
				// Bodies still listed in its arrays: it was a ragdoll (they stay as
				// they were last written when it leaves the state, as Bullet's).
				UT_Fpreal32Array mass;
				if ( massArrays.isValid() )
					massArrays.get( s.off, mass );
				for ( exint t = 0; t < mass.entries(); ++t )
					if ( mass( t ) > 0.0f )
					{
						os.agentsLeftRagdoll.insert( name );
						break;
					}
			}
			continue;
		}
		const GA_Primitive* gp = gdp->getPrimitive( static_cast<GA_Offset>( s.primOff ) );
		const GU_PrimPacked* prim =
			gp && GU_PrimPacked::isPackedPrimitive( *gp ) ? static_cast<const GU_PrimPacked*>( gp ) : nullptr;
		const GU_Agent* agent = agentOf( prim );
		if ( !agent )
			continue;
		// Still in the crowd, whether or not its bodies can be made this scan.
		seen.insert( s.bid );
		// What constraints call the agent: its primitive's `name`, else its
		// point's.
		std::string agentName;
		if ( primName.isValid() )
			agentName = primName.get( static_cast<GA_Offset>( s.primOff ) );
		if ( agentName.empty() && pointName.isValid() )
			agentName = pointName.get( s.off );
		UT_Int32Array flags;
		active.get( s.off, flags );

		std::vector<TransformShape> shapes;
		if ( !collectShapes( *agent, shapes ) )
		{
			++c.agentNoShapes;
			continue;
		}
		// The agent's transforms in the object's space: each one's world transform
		// in the agent times the agent's own (row vectors).
		GU_Agent::Matrix4ArrayConstPtr xforms;
		if ( !agent->computeWorldTransforms( xforms ) || !xforms )
		{
			++c.agentFailed;
			continue;
		}
		double full[16];
		fetchFull4( *prim, full );
		const size_t n = static_cast<size_t>( xforms->size() );
		std::vector<std::array<double, 16>> now( n );
		for ( size_t t = 0; t < n; ++t )
		{
			double local[16];
			const UT_Matrix4F& m = ( *xforms )( static_cast<exint>( t ) );
			for ( int r = 0; r < 4; ++r )
				for ( int k = 0; k < 4; ++k )
					local[r * 4 + k] = m( r, k );
			mul4( local, full, now[t].data() );
		}
		const GU_AgentRig& rig = *agent->getRig();
		auto restIt = os.agentRestWorld.find( s.bid );
		bool restored = false; // the rest came from the arrays: so do velocities
		if ( restIt == os.agentRestWorld.end() )
		{
			Box3DAgentRest made;
			made.name = agentName;
			restored = restOfAgent( gdp, s.off, rig, now, shapes, made );
			made.frames.resize( n );
			for ( size_t t = 0; t < n; ++t )
				made.frames[t] = frameOf( made.world[t].data() );
			restIt = os.agentRestWorld.emplace( s.bid, std::move( made ) ).first;
		}
		const Box3DAgentRest& rest = restIt->second;

		Box3DAgentRecord record;
		record.pointOff = static_cast<int64_t>( s.off );
		record.primOff = s.primOff;
		record.agentId = s.bid;
		record.bodies.assign( n, nullptr );
		b3dop::BodyProps props = readDynamicProps( node, a, s.off );
		props.computeCom = false;
		props.computeMass = true;

		for ( const TransformShape& shape : shapes )
		{
			const int t = shape.transform;
			if ( static_cast<size_t>( t ) >= n || rest.world.size() != n )
				continue;
			// An `active_agent` 0 transform follows the animation: a kinematic body,
			// as Bullet's, that the ragdoll's joints hang from.
			const b3dop::BodySettings::Type want =
				( t < flags.entries() && flags( t ) == 0 ) ? b3dop::BodySettings::Type::Kinematic : node.type;
			const uint64_t key = b3dop::makeBodyKey( objectId, agentBodyId( s.bid, t ) );
			Box3DBodyMirror* entry = st.mirror.find( key );
			if ( !entry )
			{
				// Made where the transform is now; its rest (pivot, rotation) is the
				// rig's at rest, which the arrays and the pose measure from.
				const size_t ti = static_cast<size_t>( t );
				BodyPlan plan;
				if ( !rest.hasPivot[ti] || !planBody( world, shape, now[ti].data(), rest.pivots[ti].data(), plan ) ||
					 ti >= rest.hasPivot.size() || !rest.hasPivot[ti] )
				{
					++c.agentFailed;
					continue;
				}
				// Mass: the density times the shape's own volume, spread over the hull
				// that stands for it.
				b3dop::BodyProps bodyProps = props;
				if ( plan.volume > 0.0 )
				{
					bodyProps.computeMass = false;
					bodyProps.mass = static_cast<float>( props.density * plan.volume * plan.massScale );
				}
				const Box3DMirrorMap::Ref res = st.mirror.tryEmplace( key );
				entry = res.entry;
				entry->props = bodyProps;
				entry->volume = plan.shapes.totalVolume;
				b3dop::BodySettings b = node;
				b.type = want;
				b.props = bodyProps;
				b.props.density = boxDensity( bodyProps, plan.shapes.totalVolume );
				entry->awake = b.startAwake;
				const b3dop::BodyHandle h =
					createBodyWithShapes( world, b, plan.pose, plan.shapes, false, bodyProps, true, *entry );
				if ( !h )
				{
					st.mirror.erase( res.index );
					++c.agentFailed;
					continue;
				}
				entry->setPose( plan.pose );
				entry->handle = h;
				entry->type = want;
				entry->agentBody = true;
				entry->agentFreshStamp = restored ? 0 : st.syncStamp;
				b3dop::xformPoint( rest.world[ti].data(), rest.pivots[ti].data(), entry->pivot );
				b3dop::copy4( entry->restRot, rest.frames[ti].rot.data() );
				if ( restored )
				{
					UT_Fpreal32Array v, w;
					const GA_ROHandleFA vh( gdp->findFloatArray( GA_ATTRIB_POINT, "v_agent" ) ),
						wh( gdp->findFloatArray( GA_ATTRIB_POINT, "w_agent" ) );
					if ( vh.isValid() && wh.isValid() )
					{
						vh.get( s.off, v );
						wh.get( s.off, w );
						if ( v.entries() >= 3 * ( t + 1 ) && w.entries() >= 3 * ( t + 1 ) )
						{
							const float lin[3] = { v( 3 * t ), v( 3 * t + 1 ), v( 3 * t + 2 ) },
										ang[3] = { w( 3 * t ), w( 3 * t + 1 ), w( 3 * t + 2 ) };
							world.setVelocityByHandle( h, lin, ang );
							std::copy( lin, lin + 3, entry->lin );
							std::copy( ang, ang + 3, entry->ang );
							// An animated body: the velocity is the animation's, cleared
							// (as one followAnimation set) if the animation holds still.
							if ( want == b3dop::BodySettings::Type::Kinematic )
								entry->driven = true;
						}
					}
				}
				++c.agentBodies;
				++c.added;
			}
			else if ( entry->handle )
			{
				// Simulated or animation-driven: retyped when the agent's flag flips
				// (turned dynamic, the body keeps the animation's velocity).
				const bool retyped = entry->type != want;
				if ( retyped )
				{
					retypeBody( world, *entry, want );
					++c.retyped;
				}
				const size_t ti = static_cast<size_t>( t );
				if ( entry->type == b3dop::BodySettings::Type::Kinematic && ti < rest.hasPivot.size() &&
					 rest.hasPivot[ti] )
					followAnimation( world, rest.pivots[ti].data(), now[ti].data(), st.stepDt, retyped, *entry );
			}
			entry->pointOff = -1;
			entry->pointId = agentBodyId( s.bid, t );
			entry->pointIndex = -1;
			entry->objectId = objectId;
			entry->stamp = st.syncStamp;
			entry->sync = &os;
			entry->packed = false;
			++st.stampedCount;
			++os.entryCount;
			os.bodies.push_back( entry );
			record.bodies[static_cast<size_t>( t )] = entry;
		}
		if ( !agentName.empty() )
		{
			// Each transform's anchor frame, and the body a constraint naming it
			// joins: its own, else (a transform with no shape) the closest
			// ancestor's that has one (the anchor still sits at its own authored
			// position).
			for ( size_t t = 0; t < n && t < rest.frames.size(); ++t )
			{
				const std::string name =
					agentName + "/" + std::string( rig.transformName( static_cast<exint>( t ) ).c_str() );
				os.agentRest[name] = { &rest, t };
				exint at = static_cast<exint>( t );
				while ( at >= 0 && !record.bodies[static_cast<size_t>( at )] )
					at = rig.parentIndex( at );
				if ( at >= 0 )
					os.agentByName.emplace( name, record.bodies[static_cast<size_t>( at )] );
				else
					os.agentWithoutBody.insert( name );
			}
		}
		os.agents.push_back( std::move( record ) );
	}
	// An agent that is no longer a ragdoll (or gone) starts again from its pose
	// if it comes back; one whose point went is noted as gone.
	for ( const auto& kv : os.agentRestWorld )
		if ( !seen.count( kv.first ) && !kv.second.name.empty() && !os.agentsNotRagdoll.count( kv.second.name ) )
			os.agentsGone.insert( kv.second.name );
	for ( auto it = os.agentRestWorld.begin(); it != os.agentRestWorld.end(); )
		it = seen.count( it->first ) ? std::next( it ) : os.agentRestWorld.erase( it );
}

// One array of the agent point: `values` for every transform, `per` floats
// each.
void setArray( GU_Detail* gdp, GA_Offset off, const char* name, const std::vector<float>& values )
{
	GA_Attribute* attr = gdp->findFloatArray( GA_ATTRIB_POINT, name );
	if ( !attr )
		attr = gdp->addFloatArray( GA_ATTRIB_POINT, name, 1 );
	if ( !attr )
		return;
	GA_RWHandleFA h( attr );
	UT_Fpreal32Array array;
	array.setSize( static_cast<exint>( values.size() ) );
	for ( size_t i = 0; i < values.size(); ++i )
		array( static_cast<exint>( i ) ) = values[i];
	h.set( off, array );
}

namespace
{

UT_Matrix4D matrixOf( const std::array<double, 16>& a )
{
	UT_Matrix4D m;
	for ( int r = 0; r < 4; ++r )
		for ( int c = 0; c < 4; ++c )
			m( r, c ) = a[static_cast<size_t>( r * 4 + c )];
	return m;
}

UT_Matrix3D rotationPart( const UT_Matrix4D& m )
{
	UT_Matrix3D r;
	for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j )
			r( i, j ) = m( i, j );
	return r;
}

// The agent's pose from its bodies' motion since they were made. A body's
// transform moves rigidly with it: its origin to the body's centre plus its
// rest offset turned by the rotation since, its axes turned by it. The root
// body (the one with no body among its ancestors) gives the agent point's
// P / orient, as the agent's own space follows it; every other body's transform
// is placed in that space, and a transform without a body keeps its local
// transform under its parent. Bullet does the same (checked against it).
void writeAgentPose( GU_Detail* gdp, const Box3DObjectSync& os, const Box3DAgentRecord& r )
{
	const auto restIt = os.agentRestWorld.find( r.agentId );
	if ( restIt == os.agentRestWorld.end() )
		return;
	GA_Primitive* gp = gdp->getPrimitive( static_cast<GA_Offset>( r.primOff ) );
	if ( !gp || !GU_PrimPacked::isPackedPrimitive( *gp ) )
		return;
	GU_PrimPacked* prim = static_cast<GU_PrimPacked*>( gp );
	GU_Agent* agent = dynamic_cast<GU_Agent*>( prim->hardenImplementation() );
	if ( !agent )
		return;
	const size_t n = static_cast<size_t>( agent->transformCount() );
	const std::vector<std::array<double, 16>>& rest = restIt->second.world;
	if ( n != r.bodies.size() || n != rest.size() )
		return;
	GU_Agent::Matrix4ArrayConstPtr localPtr, worldPtr;
	if ( !agent->computeLocalTransforms( localPtr ) || !agent->computeWorldTransforms( worldPtr ) || !localPtr ||
		 !worldPtr )
		return;
	const GU_AgentRig& rig = *agent->getRig();
	const auto toD = []( const UT_Matrix4F& m ) {
		UT_Matrix4D d;
		for ( int i = 0; i < 4; ++i )
			for ( int j = 0; j < 4; ++j )
				d( i, j ) = m( i, j );
		return d;
	};

	// Each body's transform now, in the simulation's space.
	std::vector<UT_Matrix4D> now( n );
	std::vector<char> hasBody( n, 0 );
	for ( size_t i = 0; i < n; ++i )
	{
		const Box3DBodyMirror* e = r.bodies[i];
		// Only simulated bodies pose their transforms: a static one (its
		// `active_agent` 0) leaves the animation's.
		if ( !e || !e->handle || !e->dynamic() )
			continue;
		float q[4];
		box3dAbsoluteToRest( e->restRot, e->rot, q );
		UT_Matrix3D turn;
		UT_QuaternionD( q[0], q[1], q[2], q[3] ).getRotationMatrix( turn );
		const UT_Matrix4D j0 = matrixOf( rest[i] );
		UT_Matrix3D axes = rotationPart( j0 );
		axes *= turn;
		const UT_Vector3D offset = ( UT_Vector3D( j0( 3, 0 ), j0( 3, 1 ), j0( 3, 2 ) ) -
									 UT_Vector3D( e->pivot[0], e->pivot[1], e->pivot[2] ) ) *
								   turn;
		UT_Matrix4D m( 1.0 );
		for ( int a = 0; a < 3; ++a )
			for ( int b = 0; b < 3; ++b )
				m( a, b ) = axes( a, b );
		m( 3, 0 ) = e->pos[0] + offset.x();
		m( 3, 1 ) = e->pos[1] + offset.y();
		m( 3, 2 ) = e->pos[2] + offset.z();
		now[i] = m;
		hasBody[i] = 1;
	}
	// The root body: none of its ancestors has a simulated one.
	exint root = -1;
	for ( size_t i = 0; i < n && root < 0; ++i )
	{
		if ( !hasBody[i] )
			continue;
		bool top = true;
		for ( exint a = rig.parentIndex( static_cast<exint>( i ) ); a >= 0; a = rig.parentIndex( a ) )
			top = top && !hasBody[static_cast<size_t>( a )];
		if ( top )
			root = static_cast<exint>( i );
	}
	if ( root < 0 )
		return;

	// The agent's own transform: when the root body is the rig's topmost (nothing
	// above it has a body, simulated or not), the one that puts the root's
	// transform, as it is in the agent, where its body has it. Under a body that
	// is not simulated the agent keeps the transform the crowd gave it, and so it
	// does when the Crowd Solver's bullet_updateagentxform is 0 (1 for a whole
	// ragdoll, 0 for a partial one), as RBD Bullet Solver reads it.
	bool topmost = true;
	for ( exint a = rig.parentIndex( root ); a >= 0; a = rig.parentIndex( a ) )
		topmost = topmost && !r.bodies[static_cast<size_t>( a )];
	const GA_ROHandleI follow( gdp, GA_ATTRIB_POINT, "bullet_updateagentxform" );
	if ( follow.isValid() && follow.get( static_cast<GA_Offset>( r.pointOff ) ) == 0 )
		topmost = false;
	UT_Matrix4D agentXform;
	if ( topmost )
	{
		UT_Matrix4D rootWorld = toD( ( *worldPtr )( root ) ), rootInv;
		if ( rootWorld.invert( rootInv ) != 0 )
			return;
		agentXform = rootInv * now[static_cast<size_t>( root )];
	}
	else
	{
		double full[16];
		fetchFull4( *prim, full );
		for ( int a = 0; a < 4; ++a )
			for ( int b = 0; b < 4; ++b )
				agentXform( a, b ) = full[a * 4 + b];
	}
	UT_Matrix3D rot = rotationPart( agentXform );
	for ( int i = 0; i < 3; ++i )
	{ // the agent's units are in its transforms
		UT_Vector3D row( rot( i, 0 ), rot( i, 1 ), rot( i, 2 ) );
		row.normalize();
		for ( int j = 0; j < 3; ++j )
			rot( i, j ) = row( j );
	}
	UT_QuaternionD heading;
	heading.updateFromRotationMatrix( rot );
	const GA_Offset off = static_cast<GA_Offset>( r.pointOff );
	if ( topmost )
		gdp->setPos3( off,
					  UT_Vector3( static_cast<float>( agentXform( 3, 0 ) ), static_cast<float>( agentXform( 3, 1 ) ),
								  static_cast<float>( agentXform( 3, 2 ) ) ) );
	GA_RWHandleQ orient( gdp, GA_ATTRIB_POINT, "orient" );
	if ( topmost && orient.isValid() )
		orient.set( off, UT_QuaternionF( static_cast<float>( heading.x() ), static_cast<float>( heading.y() ),
										 static_cast<float>( heading.z() ), static_cast<float>( heading.w() ) ) );
	UT_Matrix4D agentInv;
	if ( agentXform.invert( agentInv ) != 0 )
		return;

	// The transforms in the agent's space, parents first.
	std::vector<UT_Matrix4D> target( n );
	std::vector<char> done( n, 0 );
	std::function<void( size_t )> place = [&]( size_t i ) {
		if ( done[i] )
			return;
		done[i] = 1;
		if ( hasBody[i] )
		{
			target[i] = now[i] * agentInv;
			return;
		}
		const exint p = rig.parentIndex( static_cast<exint>( i ) );
		if ( p < 0 )
		{
			target[i] = toD( ( *worldPtr )( static_cast<exint>( i ) ) );
			return;
		}
		place( static_cast<size_t>( p ) );
		target[i] = toD( ( *localPtr )( static_cast<exint>( i ) ) ) * target[static_cast<size_t>( p )];
	};
	GU_Agent::Matrix4Array worlds;
	worlds.setSize( static_cast<exint>( n ) );
	for ( size_t i = 0; i < n; ++i )
	{
		place( i );
		UT_Matrix4F f;
		for ( int a = 0; a < 4; ++a )
			for ( int b = 0; b < 4; ++b )
				f( a, b ) = static_cast<float>( target[i]( a, b ) );
		worlds( static_cast<exint>( i ) ) = f;
	}
	agent->setWorldTransforms( worlds );
}

} // namespace

void writeAgentArrays( Box3DSharedState& st, SIM_ObjectArray& objects )
{
	for ( auto& kv : st.objectSync )
	{
		Box3DObjectSync& os = kv.second;
		if ( os.visited != st.syncStamp || os.agents.empty() )
			continue;
		SIM_Object* obj = box3dObjectById( objects, kv.first, st.objectsSorted );
		if ( !obj )
			continue;
		SIM_GeometryCopy* geo = box3dWritableGeometry( *obj, SIM_GEOMETRY_DATANAME );
		if ( !geo )
			continue;
		SIM_GeometryAutoWriteLock lock( geo );
		GU_Detail* gdp = &lock.getGdp();
		for ( const Box3DAgentRecord& r : os.agents )
		{
			const size_t n = r.bodies.size();
			std::vector<float> pivot( 3 * n, 0.0f ), trans( 3 * n, 0.0f ), orient( 4 * n, 0.0f ), vel( 3 * n, 0.0f ),
				ang( 3 * n, 0.0f ), mass( n, 0.0f );
			for ( size_t i = 0; i < n; ++i )
			{
				const Box3DBodyMirror* e = r.bodies[i];
				if ( !e || !e->handle )
					continue;
				for ( size_t k = 0; k < 3; ++k )
				{
					pivot[3 * i + k] = static_cast<float>( e->pivot[k] );
					trans[3 * i + k] = static_cast<float>( e->pos[k] - e->pivot[k] );
					vel[3 * i + k] = e->lin[k];
					ang[3 * i + k] = e->ang[k];
				}
				float q[4];
				box3dAbsoluteToRest( e->restRot, e->rot, q );
				std::copy( q, q + 4, orient.begin() + static_cast<long>( 4 * i ) );
				mass[i] = effectiveMass( e->props, e->volume );
			}
			const GA_Offset off = static_cast<GA_Offset>( r.pointOff );
			setArray( gdp, off, "pivot_agent", pivot );
			setArray( gdp, off, "trans_agent", trans );
			setArray( gdp, off, "orient_agent", orient );
			setArray( gdp, off, "v_agent", vel );
			setArray( gdp, off, "w_agent", ang );
			setArray( gdp, off, "mass_agent", mass );
			writeAgentPose( gdp, os, r );
		}
	}
}

} // namespace box3dsolver
