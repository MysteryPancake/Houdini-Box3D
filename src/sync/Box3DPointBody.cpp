// Box3DPointBody.cpp
//
// A point's body settings: dynamic properties, mass, type and what a change of
// settings does to an existing body, and making a body with its shapes.
#include "Box3DSolverInternal.h"

#include <functional>
#include <string_view>

namespace box3dsolver
{

// A point's changeable settings: Box3D Data's, with the point's attributes on
// top. Building and comparing both read through here, so an untouched point
// compares equal. (`awake` is not a setting: see startAwakeFor.)
b3dop::BodyProps readDynamicProps( const b3dop::BodySettings& node, const PointAttrs& a, GA_Offset off )
{
	b3dop::BodyProps p = node.props;
	if ( a.mass.isValid() )
	{
		const float m = static_cast<float>( a.mass.get( off ) );
		if ( m > 0.0f && std::isfinite( m ) )
			p.mass = m;
	}
	if ( a.friction.isValid() )
		p.friction = b3dop::clampNonNegOr( a.friction.get( off ), p.friction );
	if ( a.bounce.isValid() )
		p.restitution = b3dop::clampNonNegOr( a.bounce.get( off ), p.restitution );
	if ( a.rollingResistance.isValid() )
		p.rollingResistance = b3dop::clampNonNegOr( a.rollingResistance.get( off ), p.rollingResistance );
	if ( a.tangentVelocity.isValid() )
	{
		const UT_Vector3 t = a.tangentVelocity.get( off );
		if ( b3dop::finite3( t.data() ) )
			for ( unsigned i = 0; i < 3; ++i )
				p.tangentVelocity[i] = static_cast<float>( t( i ) );
	}
	if ( a.lineardamping.isValid() )
		p.linearDamping = b3dop::clampNonNegOr( a.lineardamping.get( off ), p.linearDamping );
	if ( a.angulardamping.isValid() )
		p.angularDamping = b3dop::clampNonNegOr( a.angulardamping.get( off ), p.angularDamping );
	if ( a.gravityscale.isValid() )
	{
		const float g = a.gravityscale.get( off );
		if ( std::isfinite( g ) )
			p.gravityScale = g; // may be negative
	}
	if ( a.enablesleep.isValid() )
		p.enableSleep = a.enablesleep.get( off ) != 0;
	if ( a.enabled.isValid() )
		p.enabled = a.enabled.get( off ) != 0;

	if ( a.computeMass.isValid() )
		p.computeMass = a.computeMass.get( off ) != 0;
	if ( a.computeCom.isValid() )
		p.computeCom = a.computeCom.get( off ) != 0;
	if ( a.density.isValid() )
	{
		const float d = a.density.get( off );
		if ( d > 0.0f && std::isfinite( d ) )
			p.density = d;
	}
	if ( a.inertiaStiffness.isValid() )
	{
		const float k = a.inertiaStiffness.get( off );
		if ( k > 0.0f && std::isfinite( k ) )
			p.inertiaScale = k;
	}
	auto threshold = [&]( const GA_ROHandleF& h ) {
		if ( !h.isValid() )
			return -1.0f;
		const float v = h.get( off );
		return ( v >= 0.0f && std::isfinite( v ) ) ? v : -1.0f;
	};
	if ( const float t = threshold( a.sleepThreshold ); t >= 0.0f )
		p.sleepThreshold = t;
	if ( const float t = threshold( a.ccdThreshold ); t >= 0.0f )
		p.ccdThreshold = t;
	if ( a.fastObject.isValid() )
		p.fastObject = a.fastObject.get( off ) != 0;
	if ( a.fastRotation.isValid() )
		p.fastRotation = a.fastRotation.get( off ) != 0;
	if ( a.contactRecycling.isValid() )
		p.contactRecycling = a.contactRecycling.get( off ) != 0;
	if ( a.collisionCategory.isValid() )
		p.filter.category = static_cast<uint64_t>( a.collisionCategory.get( off ) );
	if ( a.collisionMask.isValid() )
		p.filter.mask = static_cast<uint64_t>( a.collisionMask.get( off ) );
	if ( a.collisionGroup.isValid() )
		p.filter.group = a.collisionGroup.get( off );
	return p;
}

// Box3D density: computemass -> density; else mass / volume (density if
// unusable).
float boxDensity( const b3dop::BodyProps& p, float volume )
{
	if ( p.computeMass )
		return p.density;
	return ( p.mass > 0.0f && std::isfinite( p.mass ) && volume > 0.0f ) ? p.mass / volume : p.density;
}

// The mass Houdini reports (written to `mass` with computemass).
float effectiveMass( const b3dop::BodyProps& p, float volume )
{
	return p.computeMass ? p.density * volume : p.mass;
}

// active -> the node's type; inactive -> static, or kinematic if animated;
// shapeless -> static.
b3dop::BodySettings::Type wantedType( const b3dop::BodySettings& node, bool active, bool animated, bool shapeless )
{
	if ( shapeless )
		return b3dop::BodySettings::Type::Static;
	if ( active )
		return node.type;
	return animated ? b3dop::BodySettings::Type::Kinematic : b3dop::BodySettings::Type::Static;
}

// The body's mass after shapes are added / replaced or its type changes. With
// computecom the centre of mass stays where the shapes put it (the point, the
// body origin, orbits it); without, as in Houdini RBD's pivot, the shapes'
// centroid is moved to the point. Inertia is scaled by inertialtensorstiffness
// (a custom or bounding box inertia replaces it first). Static / kinematic
// bodies only get shape densities. setShapeDensity = false for shapes just
// created with boxDensity(p, volume).
void applyBodyMass( b3dop::Box3DWorld& world, b3dop::BodyHandle h, const b3dop::BodyProps& p, float volume,
					const float* restRot, bool setShapeDensity )
{
	float boxInverse[6];
	const float* inverse = nullptr;
	if ( p.customInertia )
		inverse = p.inverseInertia;
	else if ( p.boundsInertia &&
			  b3dop::boundsInverseInertia( p.boundsExtents, effectiveMass( p, volume ), boxInverse ) )
		inverse = boxInverse;
	else if ( p.shapeBoundsInertia )
	{
		// Bullet's convex hulls and compounds: the inertia of the box over them.
		float extents[3];
		if ( world.boundsInertiaExtentsByHandle( h, restRot, b3dop::kBulletInertiaPad, extents ) &&
			 b3dop::boundsInverseInertiaIn( extents, effectiveMass( p, volume ), restRot, boxInverse ) )
			inverse = boxInverse;
	}
	world.applyMassByHandle( h, boxDensity( p, volume ), !p.computeCom, p.inertiaScale, setShapeDensity, inverse );
}

// `awake` for a new body: the point's value, else the Box3D Data default.
// (Box3D starts a body that may not sleep awake whatever this says.)
bool startAwakeFor( const b3dop::BodySettings& node, const PointAttrs& a, GA_Offset off )
{
	return a.awake.isValid() ? a.awake.get( off ) != 0 : node.startAwake;
}

bool retypeBody( b3dop::Box3DWorld& world, Box3DBodyMirror& entry, b3dop::BodySettings::Type want )
{
	if ( entry.type == want )
		return false;
	world.setTypeByHandle( entry.handle, want );
	entry.type = want;
	entry.driven = false;
	// The mass comes back from the shapes: centre back to the point.
	if ( entry.dynamic() )
		applyBodyMass( world, entry.handle, entry.props, entry.volume, entry.restRot );
	return true;
}

// Adds the pieces without mass updates (the caller calls applyBodyMass).
// Returns the count.
int addShapes( b3dop::Box3DWorld& world, b3dop::BodyHandle h, const b3dop::BodySettings& b, const BuiltShapes& shapes )
{
	int added = 0;
	for ( const Piece& pc : shapes.pieces )
	{
		bool ok = false;
		if ( pc.mesh ) // static bodies only (Box3DWorld refuses any other)
			ok = world.createMeshShape( h, b, pc.mesh->xyz.data(), static_cast<int>( pc.mesh->xyz.size() / 3 ),
										pc.mesh->tris.data(), static_cast<int>( pc.mesh->tris.size() / 3 ) );
		else if ( pc.isHull )
			ok = world.createHullShape( h, b, pc.hull, false, pc.densityScale );
		else
			ok = world.createShape( h, b, pc.spec, false, pc.densityScale );
		added += ok ? 1 : 0;
	}
	return added;
}

// Every piece or none, so a body is never left with only some of its shapes.
// True if the body has its shapes (false with no pieces to add too).
bool addAllShapes( b3dop::Box3DWorld& world, b3dop::BodyHandle h, const b3dop::BodySettings& b,
				   const BuiltShapes& shapes )
{
	const int added = addShapes( world, h, b, shapes );
	if ( added == static_cast<int>( shapes.pieces.size() ) )
		return added > 0;
	if ( added > 0 )
		world.destroyShapesByHandle( h );
	return false;
}

// A body with settings `b` at `pose` (its user data `entry`), its `shapes` and
// mass, then its initial velocity when `giveVelocity`: after the mass is final
// (a moving centre of mass shifts the velocity), and not for a static body or
// one that starts asleep or disabled (a velocity would wake it, so `v` only
// takes effect once something wakes it). A shapeless body (`none`) gets no
// shapes. 0 on failure, leaving no body behind.
b3dop::BodyHandle createBodyWithShapes( b3dop::Box3DWorld& world, const b3dop::BodySettings& b,
										const Box3DBodyPose& pose, const BuiltShapes& shapes, bool shapeless,
										const b3dop::BodyProps& p, bool giveVelocity, Box3DBodyMirror& entry )
{
	const b3dop::BodyHandle h = world.createBody( b, pose.pos, pose.rot, &entry );
	if ( !h || shapeless )
		return h;
	if ( addShapes( world, h, b, shapes ) != static_cast<int>( shapes.pieces.size() ) )
	{
		// Never a shapeless body (except `none`) or one with only some of its
		// shapes: the caller counts it as failed.
		world.destroyBody( h );
		return 0;
	}
	applyBodyMass( world, h, p, shapes.totalVolume, entry.restRot, false );
	const bool hasVel = !std::all_of( pose.lin, pose.lin + 3, []( float v ) { return v == 0.0f; } ) ||
						!std::all_of( pose.ang, pose.ang + 3, []( float v ) { return v == 0.0f; } );
	if ( giveVelocity && hasVel && b.type != b3dop::BodySettings::Type::Static && world.isAwakeByHandle( h ) )
		world.setVelocityByHandle( h, pose.lin, pose.ang );
	return h;
}

// Push the settings that differ. After a rebuild the new shapes already carry
// material, mass and filter profile. Returns kPropsChanged / kMassChanged.
unsigned applyPropChanges( b3dop::Box3DWorld& world, Box3DBodyMirror& e, const b3dop::BodyProps& p, bool shapesRebuilt )
{
	const b3dop::BodyProps& o = e.props;
	if ( !shapesRebuilt && p == o )
		// nothing differs (the field compares below would call setters for a NaN)
		return 0;
	const b3dop::BodyHandle h = e.handle;
	unsigned changed = shapesRebuilt ? static_cast<unsigned>( kPropsChanged ) : 0u;
	// Runs `set` and records a props change when `differs`.
	auto apply = [&]( bool differs, auto&& set ) {
		if ( differs )
		{
			set();
			changed |= kPropsChanged;
		}
	};
	const bool massChanged = p.mass != o.mass || p.computeMass != o.computeMass || p.computeCom != o.computeCom ||
							 p.density != o.density || p.inertiaScale != o.inertiaScale || !p.sameInertia( o );
	if ( !shapesRebuilt )
	{
		apply( p.friction != o.friction || p.restitution != o.restitution,
			   [&] { world.setMaterialByHandle( h, p.friction, p.restitution ); } );
		if ( massChanged )
		{
			applyBodyMass( world, h, p, e.volume, e.restRot );
			changed |= kPropsChanged | kMassChanged;
		}
		apply( p.filter != o.filter, [&] { world.setFilterByHandle( h, p.filter ); } );
	}
	else if ( massChanged )
		changed |= kMassChanged;
	apply( p.linearDamping != o.linearDamping || p.angularDamping != o.angularDamping,
		   [&] { world.setDampingByHandle( h, p.linearDamping, p.angularDamping ); } );
	apply( p.gravityScale != o.gravityScale, [&] { world.setGravityScaleByHandle( h, p.gravityScale ); } );
	apply( p.ccdThreshold != o.ccdThreshold, [&] { world.setCcdThresholdByHandle( h, p.ccdThreshold ); } );
	apply( p.sleepThreshold != o.sleepThreshold, [&] { world.setSleepThresholdByHandle( h, p.sleepThreshold ); } );
	apply( p.fastRotation != o.fastRotation, [&] { world.setFastRotationByHandle( h, p.fastRotation ); } );
	apply( p.fastObject != o.fastObject, [&] { world.setBulletByHandle( h, p.fastObject ); } );
	apply( p.rollingResistance != o.rollingResistance ||
			   !std::equal( p.tangentVelocity, p.tangentVelocity + 3, o.tangentVelocity ),
		   [&] { world.setSurfaceByHandle( h, p.rollingResistance, p.tangentVelocity ); } );
	apply( p.contactRecycling != o.contactRecycling,
		   [&] { world.setContactRecyclingByHandle( h, p.contactRecycling ); } );
	apply( p.enableSleep != o.enableSleep, [&] { world.setSleepEnabledByHandle( h, p.enableSleep ); } );
	apply( p.enabled != o.enabled, [&] {
		world.setEnabledByHandle( h, p.enabled );
		if ( p.enabled )
			// Box3D enables a body awake; the awake check after this applies a 0
			e.awake = true;
	} );
	e.props = p;
	return changed;
}

} // namespace box3dsolver
