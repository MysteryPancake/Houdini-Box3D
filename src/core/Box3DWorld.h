// Box3DWorld.h
//
// Houdini-free wrapper around one Box3D world; the only interface the SIM_*
// code uses. Box3D's headers are not included here (pimpl), so Houdini code
// never sees them.
//
// Conventions: positions double[3] (works with BOX3D_DOUBLE_PRECISION too);
// quaternions float[4] x, y, z, w (the layout of UT_QuaternionF and b3Quat).
// Bodies are addressed by BodyHandle only (tests/KeyedWorld.h adds
// caller-chosen ids on top, for the tests).
#pragma once

#include "Box3DJoints.h"
#include "Box3DMath.h" // sameFloat
#include "Box3DShapes.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace b3dop
{

// A convex hull in the process-wide hull cache (its content or caller-supplied
// key). 0 is invalid.
using HullId = uint64_t;

// Box3D worker tasks per step when nothing else is asked for. The solver's
// Threads default is the machine's core count instead (defaultWorkerCount in
// Box3DCpu.h), falling back to this when it cannot be detected.
constexpr int kDefaultWorkerCount = 8;

// The most worker tasks a world can use (Box3D's B3_MAX_WORKERS; Box3DWorld.cpp
// checks they match).
constexpr int kMaxWorkerCount = 32;

// The most edges bodyShapeOutlines returns for one mesh (more are thinned).
constexpr size_t kMaxMeshOutlineEdges = 20000;

// A body, valid until it is destroyed (or the world is). No hash lookup. 0 is
// invalid.
using BodyHandle = uint64_t;

// A joint, valid until it, either of its bodies, or the world is destroyed. 0
// is invalid.
using JointHandle = uint64_t;

// One of Box3D's body move events: an awake body after the last step and its
// final transform. A body that fell asleep this step appears once more with
// fellAsleep set. Bodies moved by the caller (setTransform...) do not appear.
struct BodyMove
{
	void* userData = nullptr; // whatever createBody stored
	BodyHandle handle = 0;
	double pos[3] = { 0, 0, 0 };
	float rot[4] = { 0, 0, 0, 1 };
	float lin[3] = { 0, 0, 0 }; // velocities at the end of the step (0: asleep)
	float ang[3] = { 0, 0, 0 };
	bool fellAsleep = false;
};

// What the external-force pass needs of one body, read in one call
// (bodyStateByHandle).
struct BodyState
{
	bool awake = false;
	float mass = 0.0f;
	double com[3] = { 0, 0, 0 }; // world centre of mass
	float lin[3] = { 0, 0, 0 };	 // velocities (zero for a body that is not awake)
	float ang[3] = { 0, 0, 0 };
};

// Lets the host supply the threads Box3D's parallel work runs on, so worlds do
// not each start an OS thread pool of their own (Box3D's built-in scheduler
// keeps workerCount - 1 threads alive for the life of every world).
// Houdini-free: the plugin implements it on Houdini's TBB pool
// (Box3DHoudiniTasks.h). Without one, Box3DWorld uses Box3D's own pool (the
// tests do).
//
// A runner belongs to one world and is used from one stepping thread at a time.
class TaskRunner
{
public:
	virtual ~TaskRunner() = default;

	// Run fn(ctx) exactly once. Returns true if it was queued, in which case
	// wait() must be called before the data it uses goes away; false if it
	// already ran before returning.
	virtual bool run( void ( *fn )( void* ), void* ctx ) = 0;

	// Block until every task queued by run() has finished. The calling thread may
	// run tasks.
	virtual void wait() = 0;

	// Run fn(ctx) around a whole step (a place to isolate task stealing).
	// Default: just call it.
	virtual void isolate( void ( *fn )( void* ), void* ctx )
	{
		fn( ctx );
	}
};

// Everything but workerCount can change on a live world (applyTuning). Defaults
// are Box3D's, except gravity (standard gravity instead of 10).
struct WorldSettings
{
	// How the friction / restitution of two touching materials is combined. The
	// enumerators are in the order of the solver's menus (SIM_SolverBox3D), so a
	// menu index casts straight to them.
	enum class FrictionMixing : int
	{
		GeometricMean,
		Minimum,
		Maximum,
		Average,
		Multiply
	}; // GeometricMean is Box3D's default
	enum class RestitutionMixing : int
	{
		Maximum,
		Minimum,
		Average,
		Multiply,
		GeometricMean
	}; // Maximum is Box3D's default

	float gravity[3] = { 0.0f, -9.80665f, 0.0f };
	int subSteps = 4; // Box3D recommends ~4 per step
	// Parallel tasks per step, fixed at creation (1 to kMaxWorkerCount). With a
	// TaskRunner these run on the host's pool; without one, on threads Box3D
	// starts for this world.
	int workerCount = kDefaultWorkerCount;
	bool enableSleep = true;
	// continuous collision vs static bodies (fast objects: vs moving ones too)
	bool enableContinuous = true;
	float contactStiffness = 30.0f;	   // contact stiffness in Hz (cycles per second)
	float contactDampingRatio = 10.0f; // contact damping (non-dimensional)
	float contactPushSpeed = 3.0f;	   // max speed overlapping shapes are pushed apart (m/s)
	float restitutionThreshold = 1.0f; // slower collisions do not bounce (m/s)
	float maxLinearSpeed = 400.0f;	   // m/s
	// Contact recycling: a touching pair that has moved less than this since its
	// contact points were last computed (and turned less than ~10 degrees) keeps
	// them instead of recomputing, which saves time but is slightly less exact.
	// World-wide; 0 turns recycling off for every body. Box3D's default is 10 *
	// linear slop = 0.05 at its default length units (1 unit = 1 metre). Per
	// body: BodySettings::contactRecycling.
	float contactRecycleDistance = 0.05f;

	int restitutionIterations = 2;		 // extra solver passes for bounce (0-63)
	bool restitutionPropagation = false; // bounce passes on through stacks
	// 0-1; near-spherical bodies lose gyroscopic torque. New bodies / mass
	// changes only.
	float isotropicInertiaTolerance = 0.2f;
	// Off only for comparing against the reference solver.
	bool warmStarting = true;
	// Box3D hit event threshold (m/s): Impacts only for pairs that hit this
	// fast. 0 = every contact.
	float impactMinSpeed = 0.0f;
	// The size of every ground (a Ground Plane DOP, a `plane` piece), read when a
	// ground is built: a new size rebuilds them.
	GroundSize ground;
	FrictionMixing frictionMixing = FrictionMixing::GeometricMean;
	RestitutionMixing restitutionMixing = RestitutionMixing::Maximum;
};

// One contact point of a body after a step (getContactsByHandle).
struct ContactPoint
{
	BodyHandle other = 0;
	double pos[3] = { 0, 0, 0 };   // world space
	float normal[3] = { 0, 1, 0 }; // from this body into the other
	float impulse = 0.0f;		   // total normal impulse over the step (N*s), > 0
};

// One manifold point that carried impulse in the last step, with both of its
// bodies (getContactImpulseEvents).
struct ContactImpulse
{
	BodyHandle a = 0, b = 0;
	// The shapes of the two bodies the point is between, as shapeKeysByHandle
	// lists them.
	uint64_t shapeA = 0, shapeB = 0;
	void *userDataA = nullptr, *userDataB = nullptr; // of the bodies
	double pos[3] = { 0, 0, 0 };					 // world space
	float normal[3] = { 0, 1, 0 };					 // from a into b
	float impulse = 0.0f;							 // total normal impulse over the step (N*s), > 0
	bool awakeA = false, awakeB = false;			 // in the awake set when it was made
};

// A contact of the last step whose shapes came together faster than the hit
// threshold (getHitContacts): the speed they approached at when the step began.
struct HitContact
{
	BodyHandle a = 0, b = 0;
	uint64_t shapeA = 0, shapeB = 0; // as ContactImpulse names them
	float approachSpeed = 0.0f;		 // m/s, > 0
};

// The normal impulse the last step's contact points carried between two bodies
// (getContactImpulseSums).
struct ContactImpulseSum
{
	BodyHandle a = 0, b = 0;
	float impulse = 0.0f; // N*s, > 0
};

// Box3D's b3Filter, checked by Box3D in its broad phase. Two shapes with the
// same non-zero group: positive always collide, negative never. Otherwise they
// collide only when each one's mask has a bit of the other's category. The
// defaults (every bit, group 0) collide with all.
struct CollisionFilter
{
	uint64_t category = ~static_cast<uint64_t>( 0 ); // B3_DEFAULT_CATEGORY_BITS
	uint64_t mask = ~static_cast<uint64_t>( 0 );	 // B3_DEFAULT_MASK_BITS
	int group = 0;

	bool operator==( const CollisionFilter& o ) const
	{
		return category == o.category && mask == o.mask && group == o.group;
	}
	bool operator!=( const CollisionFilter& o ) const
	{
		return !( *this == o );
	}
};

// The settings of a body that can change while it lives: the solver compares
// two of these and calls only the setters of Box3DWorld for what differs. Built
// from a point's attributes, else Box3D Data's values. operator== compares
// every field exactly, except that two NaNs count as equal (so a NaN setting
// that has not changed is not seen as a change).
struct BodyProps
{
	float mass = 1.0f; // `mass`; solver only: density = mass / shape volume
	float density =
		// `density`: the shape density Box3DWorld uses (the solver passes mass /
		// volume instead, unless computeMass)
		1.0f;
	bool computeMass = false; // `computemass`; solver only: mass = density * volume instead
	// `computecom`; solver only: the centre of mass is where the shapes put it
	// (the body origin, the point, orbits it), not moved to the origin
	bool computeCom = false;
	// `inertialtensorstiffness`; solver only: scales the inertia of the mass
	float inertiaScale = 1.0f;
	// A custom inertia tensor: the INVERSE inertia about the body origin, in
	// body space, symmetric as xx yy zz xy xz yz. Replaces the inertia computed
	// from the shapes (the mass is still computed, and inertiaScale still scales
	// it). An unusable tensor (see invertSymmetric3) leaves the computed inertia.
	bool customInertia = false;
	float inverseInertia[6] = { 0, 0, 0, 0, 0, 0 };
	// Whole objects only (RBD State's Use Bounding Box): the inertia of a solid
	// box with these full side lengths and the body's mass, in place of the
	// computed one. Ignored when customInertia is set.
	bool boundsInertia = false;
	float boundsExtents[3] = { 0, 0, 0 };
	// The solver's Bounding Box Inertia, for packed pieces (not whole
	// objects, which have their own inertia choice): a body whose shapes are
	// hulls (or several shapes) gets the inertia of the solid box over them, as
	// Bullet's convex hulls and compounds do (boundsInertiaExtentsByHandle); a
	// single box, sphere or capsule keeps its own. Ignored when customInertia or
	// boundsInertia is set.
	bool shapeBoundsInertia = false;
	float friction = 0.6f;
	float restitution = 0.0f; // `bounce`
	// `rollingresistance`: 0 .. 1, slows rolling (see b3SurfaceMaterial)
	float rollingResistance = 0.0f;
	float tangentVelocity[3] = { 0, 0, 0 }; // `tangentvelocity`: conveyor belt speed, in the body's local space
	float linearDamping = 0.0f;
	float angularDamping = 0.0f;
	float gravityScale = 1.0f;
	// `contactrecycling`: this body's contacts may be recycled (see
	// WorldSettings)
	bool contactRecycling = true;
	bool enableSleep = true; // `enablesleep`: this body may fall asleep
	// `enabled`: a disabled body does not move or collide, and is never awake
	bool enabled = true;
	// `fastobject`: Box3D "bullet", continuous collision vs moving bodies too
	bool fastObject = false;
	// `fastrotation`: no angular speed cap (B3_MAX_ROTATION per step); round
	// bodies only
	bool fastRotation = false;
	float sleepThreshold = 0.05f; // m/s
	float ccdThreshold =
		// `ccdthreshold` (Box3D safetyFactor): continuous collision once a step's
		// motion exceeds this times the body's thinnest extent. Box3D recommends
		// 0.01 .. 0.5
		0.5f;
	CollisionFilter filter; // `collision_categorybits` / `collision_maskbits` / `collision_groupindex`

	// The custom inertia matches: both off, or both on with the same tensor.
	bool sameInertia( const BodyProps& o ) const
	{
		if ( shapeBoundsInertia != o.shapeBoundsInertia )
			return false;
		if ( boundsInertia != o.boundsInertia )
			return false;
		if ( boundsInertia )
			for ( int i = 0; i < 3; ++i )
				if ( !sameFloat( boundsExtents[i], o.boundsExtents[i] ) )
					return false;
		if ( customInertia != o.customInertia )
			return false;
		if ( !customInertia )
			return true;
		for ( int i = 0; i < 6; ++i )
			if ( !sameFloat( inverseInertia[i], o.inverseInertia[i] ) )
				return false;
		return true;
	}

	bool operator==( const BodyProps& o ) const
	{
		return sameFloat( mass, o.mass ) && sameFloat( density, o.density ) && computeMass == o.computeMass &&
			   computeCom == o.computeCom && sameFloat( inertiaScale, o.inertiaScale ) && sameInertia( o ) &&
			   sameFloat( friction, o.friction ) && sameFloat( restitution, o.restitution ) &&
			   sameFloat( rollingResistance, o.rollingResistance ) &&
			   sameFloat( tangentVelocity[0], o.tangentVelocity[0] ) &&
			   sameFloat( tangentVelocity[1], o.tangentVelocity[1] ) &&
			   sameFloat( tangentVelocity[2], o.tangentVelocity[2] ) && sameFloat( linearDamping, o.linearDamping ) &&
			   sameFloat( angularDamping, o.angularDamping ) && sameFloat( gravityScale, o.gravityScale ) &&
			   contactRecycling == o.contactRecycling && enableSleep == o.enableSleep && enabled == o.enabled &&
			   fastObject == o.fastObject && fastRotation == o.fastRotation &&
			   sameFloat( sleepThreshold, o.sleepThreshold ) && sameFloat( ccdThreshold, o.ccdThreshold ) &&
			   filter == o.filter;
	}
	bool operator!=( const BodyProps& o ) const
	{
		return !( *this == o );
	}

	// Values from a parameter (a script or an expression can set one past its
	// range) made safe for Box3D, which asserts on them. A non-finite value
	// takes the default. A negative friction, bounce, rolling resistance or
	// damping becomes 0, as from a point attribute, and a negative mass or
	// density takes the default.
	void sanitize()
	{
		const BodyProps d;
		mass = nonNegOr( mass, d.mass );
		density = nonNegOr( density, d.density );
		friction = clampNonNegOr( friction, d.friction );
		restitution = clampNonNegOr( restitution, d.restitution );
		rollingResistance = clampNonNegOr( rollingResistance, d.rollingResistance );
		linearDamping = clampNonNegOr( linearDamping, d.linearDamping );
		angularDamping = clampNonNegOr( angularDamping, d.angularDamping );
		if ( !std::isfinite( gravityScale ) )
			gravityScale = d.gravityScale;
	}
};

struct BodySettings
{
	enum class Type
	{
		Static,
		Kinematic,
		Dynamic
	};

	Type type = Type::Dynamic;
	BodyProps props;		// what can change later
	bool startAwake = true; // `awake` at creation. A body that cannot sleep always starts awake

	// Solver only:
	// When `bullet_georep` is missing or unknown: a hull, as RBD Packed Object's
	// own default (points without packed geometry keep the pscale box).
	ShapeType defaultShape = ShapeType::Convex;
	bool autoFit = true;			  // box / sphere / capsule fit the packed geometry's bounds
	bool hullPerConnectedSet = false; // convex / concave: one hull per set of connected primitives
	// The hand-set shape (`bullet_primS` ...), used with Auto Fit off. A value at
	// its default is not set: pscale and scale then size the shape.
	PrimOverrides prim;
};

// The Box3D Data node's defaults, which objects without one get too (the
// solver's fallback, and the base RBD Object values are read over). Bullet's
// RBD Object defaults where Box3D has the setting (Houdini 22.0): mass from
// density 1000, friction 1, bounce 0.5. The solver's mixing defaults (multiply
// friction and bounce) are Bullet's too, measured in Houdini (test
// bullet_mixing). BodySettings() itself keeps the core's values
// (mass 1, friction 0.6, bounce 0) for direct use of Box3DWorld.
inline BodySettings nodeBodyDefaults()
{
	BodySettings s;
	s.props.computeMass = true;
	s.props.computeCom = true;
	s.props.density = 1000.0f;
	s.props.friction = 1.0f;
	s.props.restitution = 0.5f;
	return s;
}

class Box3DWorld
{
public:
	Box3DWorld();
	~Box3DWorld();

	Box3DWorld( const Box3DWorld& ) = delete;
	Box3DWorld& operator=( const Box3DWorld& ) = delete;

	// ---- lifetime ----------------------------------------------------
	// Creates the b3World. `tasks` (optional) is kept for the life of the world
	// and used for every step; with workerCount <= 1 it is not used.
	bool create( const WorldSettings& settings, std::shared_ptr<TaskRunner> tasks = nullptr );
	void destroy(); // safe to call repeatedly
	bool isValid() const;
	const WorldSettings& settings() const;

	// ---- stepping ----------------------------------------------------
	void step( float timeStep );

	// ---- bodies and shapes ---------------------------------------------------
	// No hash maps: createBody returns the handle and shape calls take the
	// caller's BodySettings. Hulls are tracked per handle, so destroyBody
	// releases them. A shape add recomputes the mass unless updateMass is false;
	// after adding with false, call applyMassFromShapes or applyMassByHandle
	// before the next step (the solver always does it that way).
	BodyHandle createBody( const BodySettings& body, const double position[3], const float rotationXYZW[4],
						   void* userData = nullptr ); // 0 on failure
	void destroyBody( BodyHandle h );
	// densityScale: this shape's density is the body's times it (a piece's
	// `densityscale`). The shape keeps it, so applyMassByHandle and
	// copyShapesByHandle carry it; non-finite or negative is 1.
	bool createShape( BodyHandle h, const BodySettings& body, const ShapeSpec& shape, bool updateMass = true,
					  float densityScale = 1.0f );
	bool createHullShape( BodyHandle h, const BodySettings& body, HullId hull, bool updateMass = true,
						  float densityScale = 1.0f );
	// Destroys the body's shapes at these positions in creation order, as
	// [first, first + count) pairs (a compound's pieces), leaving the others in
	// order. The mass is not updated (call applyMassFromShapes). Returns how
	// many were destroyed.
	int destroyShapeRangesByHandle( BodyHandle h, const std::vector<std::pair<int, int>>& ranges );
	// Copies every shape of `src` onto `dst` (same density, material and filter)
	// moved by `center` / `rotXYZW` in dst's space, in creation order; the mass
	// is not updated (call applyMassFromShapes). Hulls, spheres and capsules
	// only. Returns how many were added, or -1 if there is a shape of another
	// kind or a body or a shape could not be made (some may be added by then:
	// destroyShapesByHandle(dst) clears them).
	int copyShapesByHandle( BodyHandle src, BodyHandle dst, const float center[3], const float rotXYZW[4] );
	// A triangle mesh (xyz: body-local vertices, triangles: 3 indices each,
	// CCW). Static or kinematic bodies only; false for a dynamic body, fewer than
	// 3 vertices or 1 triangle, non-finite vertices or out-of-range indices.
	// weldTolerance > 0 welds nearby vertices. The mesh belongs to the body: it
	// is freed with the body or by destroyShapesByHandle. Triangle meshes are
	// single-sided and have no mass. `scale` (optional, per axis, positive)
	// scales the vertices in body space without rebuilding the mesh;
	// setMeshScaleByHandle changes it later.
	bool createMeshShape( BodyHandle h, const BodySettings& body, const float* xyz, int vertexCount,
						  const int32_t* triangles, int triangleCount, float weldTolerance = 0.0f,
						  const float scale[3] = nullptr );
	// A static height field: countX x countZ grid points (both >= 2), heights
	// [row * countX + column]. Body-local vertex = (column * scale[0], height *
	// scale[1], row * scale[2]): the grid corner sits at the body origin, columns
	// run along x, rows along z, heights up y, and the triangles face +y.
	// `holes` (optional, (countX-1) * (countZ-1) bytes, cell [row * (countX-1) +
	// column]) leaves the cells where it is non-zero open. Box3D keeps heights as
	// 16-bit steps between the lowest and the highest height. Static bodies
	// only; false for a non-static body, a scale component <= 0, non-finite
	// heights or too few points. The data belongs to the body, like a mesh.
	bool createHeightFieldShape( BodyHandle h, const BodySettings& body, const float* heights, int countX, int countZ,
								 const float scale[3], const uint8_t* holes = nullptr );
	void setMeshScaleByHandle( BodyHandle h,
							   const float scale[3] ); // every mesh shape of the body
	void applyMassFromShapes( BodyHandle h );
	bool setTypeByHandle( BodyHandle h,
						  BodySettings::Type type );	   // static / kinematic / dynamic
	BodySettings::Type typeByHandle( BodyHandle h ) const; // Dynamic for a null handle

	// ---- changes to existing bodies (handle API)
	// Take effect from the next step. Only Box3D's own rules wake a body
	// (disabling sleep does).
	void setMaterialByHandle( BodyHandle h, float friction,
							  float restitution ); // every shape; contacts follow
	// Rolling resistance (negative / non-finite -> 0) and conveyor velocity
	// (non-finite -> 0), every shape. Box3D skips the material of a recycled
	// contact and a sleeping body does not update its contacts, so on a change an
	// enabled body's contacts are dropped and rebuilt (disable + enable: touching
	// bodies wake, velocity kept), except a sleeping dynamic body, which takes
	// the change when it wakes.
	void setSurfaceByHandle( BodyHandle h, float rollingResistance, const float tangentVelocity[3] );
	void setDampingByHandle( BodyHandle h, float linear, float angular );
	void setGravityScaleByHandle( BodyHandle h, float gravityScale );
	void setSleepEnabledByHandle( BodyHandle h,
								  bool enableSleep ); // the world switch applies on top
	void setBulletByHandle( BodyHandle h, bool flag );
	void setFastRotationByHandle( BodyHandle h, bool flag );
	// Box3D copies a body's recycling flag into each contact when the contact is
	// made, so on its own a change only reaches new contacts. For an awake
	// dynamic body the existing ones are dropped too (disable + enable, velocity
	// kept) so they are rebuilt with the new setting. A sleeping, static or
	// kinematic body only affects contacts made from now on.
	void setContactRecyclingByHandle( BodyHandle h, bool flag );
	void setEnabledByHandle( BodyHandle h,
							 bool enabled ); // disabled: no motion, no collisions
	// Remove every shape (releasing its hulls, dropping its contacts) so new ones
	// can be added with updateMass = false, followed by one mass update.
	void destroyShapesByHandle( BodyHandle h );
	void wakeByHandle( BodyHandle h );
	// Put a body to sleep, and with it every body touching it (an island). Does
	// nothing for a static or disabled body, a body that may not sleep, or when
	// the world's sleeping is off. Box3D can refuse (an island waiting to be
	// split), so check isAwakeByHandle if it matters.
	void sleepByHandle( BodyHandle h );
	// ---- external forces (handle API) -----------------------------------------
	// The size (x, y, z) of the body's bounding box in world axes, from its
	// shapes. False for a null handle or a body with no shapes (zero size).
	bool extentByHandle( BodyHandle h, float out[3] ) const;
	// A force (N) through the centre of mass and a torque (N*m) about it, in
	// world space, for the NEXT step only: Box3D adds both to its integration in
	// every substep and clears them when the step ends, so call this once per
	// step before step(). Non-finite components count as 0. Only dynamic bodies
	// take them. A sleeping body ignores them unless `wake`, which wakes it
	// first; a constant force with wake = true therefore keeps every body awake.
	void applyForceByHandle( BodyHandle h, const float force[3], const float torque[3], bool wake );
	// isAwake, mass, world centre of mass and both velocities in one call: what a
	// force evaluation reads per body. Same values as isAwakeByHandle /
	// massByHandle / getVelocityByHandle (the first two are in the test-only
	// section). False for no body (out untouched).
	bool bodyStateByHandle( BodyHandle h, BodyState& out ) const;
	// The body's inertia tensor about its centre of mass in world axes, as xx yy
	// zz xy xz yz. False for no body.
	bool inertiaWorldByHandle( BodyHandle h, float out[6] ) const;
	// The full side lengths, plus `pad`, of the box over the body's shapes in the
	// frame the body is at rest in (`restRot`: x, y, z, w, taking body space into
	// it; null: the body's axes), for the inertia of that box (Bullet's for a
	// convex hull or a compound). False, `extents` untouched, for a body with one
	// box (found as a hull of six perpendicular faces), sphere or capsule, whose
	// own inertia is exact, for a mesh, and for no shape.
	bool boundsInertiaExtentsByHandle( BodyHandle h, const float* restRot, float pad, float extents[3] ) const;

	// Set every shape's density, times its own scale (createShape's
	// densityScale; skipped with setShapeDensity = false, for shapes just created
	// with it) and apply the mass from the shapes; then, for dynamic
	// bodies, comAtOrigin moves the centre of mass to the body origin (parallel
	// axis theorem) and inertiaScale scales the inertia. setTypeByHandle
	// recomputes from the shapes, so re-apply after a type change.
	// `inverseInertia` (null: none; xx yy zz xy xz yz) replaces the computed
	// inertia of a dynamic body with the inverse of that tensor, about the
	// centre of mass (which moves to the origin, so pass comAtOrigin), before
	// inertiaScale. False if it was given but unusable (not positive definite,
	// not finite) and the computed inertia stayed; true otherwise.
	bool applyMassByHandle( BodyHandle h, float density, bool comAtOrigin, float inertiaScale,
							bool setShapeDensity = true, const float* inverseInertia = nullptr );
	// inertiaOff (optional): the off-diagonal terms xy, xz, yz.
	bool getMassDataByHandle( BodyHandle h, float& mass, float center[3], float inertiaDiag[3],
							  float* inertiaOff = nullptr ) const;

	// Box3D's single sleep threshold on |v| + |w| * extent. Negative = Box3D's
	// default.
	void setSleepThresholdByHandle( BodyHandle h, float threshold );
	void setCcdThresholdByHandle( BodyHandle h, float threshold ); // negative / non-finite: Box3D's default

	// Kinematic body: reach this pose in timeStep (Box3D sets the velocity, so it
	// pushes what it touches). The velocity remains afterwards; zero it when the
	// target stops.
	void setTargetTransformByHandle( BodyHandle h, const double pos[3], const float rotXYZW[4], float timeStep );

	// Report the contact points that carry impulse after every step
	// (getContactImpulseEvents), for the contacts in the awake set. Off by
	// default.
	void enableContactImpulseEvents( bool on );

	// Bodies created between these calls get their broad-phase proxies in one
	// tree build at the end instead of one insertion each (Box3D's
	// b3World_BeginBulkCreate). For making many bodies at once: nothing may query
	// the world meanwhile.
	void beginBulkCreate();
	void endBulkCreate();

	// Every such point of the last step, one record each (what
	// getContactsByHandle gives for all awake bodies at once, seen once per
	// contact). `out` is cleared first. Valid until the next step.
	int getContactImpulseEvents( std::vector<ContactImpulse>& out ) const;

	// The same events as one record per body pair: the points of one contact
	// follow each other, so runs of the same two bodies are added up. Fewer
	// records and fields to read than getContactImpulseEvents, for a caller that
	// only adds impulse up per body. `out` is cleared first. Valid until the next
	// step.
	int getContactImpulseSums( std::vector<ContactImpulseSum>& out ) const;

	// The dynamic bodies each of `movers` reaches in the next step of `dt`
	// seconds: those whose bounds overlap the mover's bounds swept along its
	// velocity. Movers that are not awake, dynamic or kinematic and valid, that
	// move slower than `minSpeed` or (a dynamic one) whose own momentum is below
	// `minMomentum` are skipped (a kinematic mover has mass 0 and no momentum
	// limit), and so are bodies the mover moves with: the pair's relative speed
	// (see `relLin`) has to reach `minSpeed` too. One record per (mover, body)
	// pair, added to `out` (not cleared). Contacts are not read: a fast body has
	// none before it hits.
	struct Approach
	{
		BodyHandle mover = 0, target = 0;
		float moverMass = 0.0f;
		double moverCom[3] = { 0, 0, 0 };
		float moverLin[3] = { 0, 0, 0 };
		// The mover's velocity relative to the target's motion at the mover's
		// centre (the target's velocity plus its spin about its own centre).
		float relLin[3] = { 0, 0, 0 };
		// The largest half extent of the mover's own bounds.
		float moverHalf = 0.0f;
	};
	void approachingBodies( const BodyHandle* movers, size_t count, float dt, float minSpeed, float minMomentum,
							std::vector<Approach>& out ) const;
	// A body's bounds (its shapes' boxes together). False for no shapes.
	bool boundsByHandle( BodyHandle h, double lo[3], double hi[3] ) const;

	// The total normal impulse (N*s) of the body's contacts in the last step,
	// leaving out those with the `skipCount` bodies at `skip` (a body's glued
	// neighbours): what getContactsByHandle's records add up to, without
	// building them. `ignore` (optional) is asked about the other body of each
	// contact that is not skipped: true leaves that contact out too.
	float contactImpulseByHandle( BodyHandle h, const BodyHandle* skip, size_t skipCount,
								  const std::function<bool( BodyHandle )>* ignore = nullptr ) const;

	// The total normal impulse (N*s) of the last step's contacts on each shape of
	// the body, in the order the shapes were created (a compound's pieces). `out`
	// is resized to the shape count; returns it. A contact between two shapes of
	// the same body cannot exist, so nothing is skipped. `ignore` (optional) is
	// asked about the other body of each contact: true leaves that contact out.
	int contactImpulsesByShape( BodyHandle h, std::vector<float>& out,
								const std::function<bool( BodyHandle )>* ignore = nullptr ) const;

	// The body's shapes in the order they were created (the order of
	// contactImpulsesByShape), each as the key a ContactImpulse names it by.
	// `out` is resized to the shape count; returns it.
	int shapeKeysByHandle( BodyHandle h, std::vector<uint64_t>& out ) const;

	// The contacts of the last step that approached faster than the hit
	// threshold (impactMinSpeed, or Box3D's default when that is 0), with the
	// speed. Box3D clears them every step.
	int getHitContacts( std::vector<HitContact>& out ) const;

	// Hit events of the last step (pairs that approached faster than
	// impactMinSpeed).
	// append: keep the pairs already collected (Box3D clears its hit events every
	// step, so a frame of several steps appends after each one).
	void updateHitEvents( bool append = false );
	bool hadHitEvent( BodyHandle a, BodyHandle b ) const;

	// ---- joints
	// frameA / frameB are world-space frames now (anchor P + orient); the joint
	// keeps each relative to its body. A null BodyHandle is the world (a
	// shapeless static body made on first use). Settings are clamped to what
	// Box3D accepts. 0 on failure (both ends the world, the same body twice, or
	// an invalid body). With framesAreLocal the frames are already relative to
	// their bodies (what jointLocalFrames returns), so a joint can be remade
	// exactly, without going through world positions.
	JointHandle createJoint( JointType type, BodyHandle a, BodyHandle b, const JointFrame& frameA,
							 const JointFrame& frameB, const JointSettings& in, void* userData = nullptr,
							 bool framesAreLocal = false );
	void destroyJoint( JointHandle h ); // harmless on a joint already gone with its body
	bool isJointValid( JointHandle h ) const;
	// Push every setting of the type into a live joint (frames and bodies stay);
	// wakes its bodies.
	void updateJoint( JointHandle h, JointType type, const JointSettings& in );
	// A spherical joint's motor alone (on, velocity in joint frame A in rad/s,
	// max torque), set every step by a driven motor. False if `h` is no live
	// spherical joint.
	bool setSphericalMotor( JointHandle h, bool enable, const float velocity[3], float maxTorque );
	bool jointFeedback( JointHandle h, JointType type, JointFeedback& out ) const;
	// Where the joint's two frames are now, in world space (each body's pose
	// times its local frame; the world end does not move). Making a joint from
	// these on bodies at the same poses gives the same joint, so a world rebuilt
	// mid-simulation can remake its joints from them.
	bool jointWorldFrames( JointHandle h, JointFrame& a, JointFrame& b ) const;
	// The same with the two bodies' poses given (the joint's own local frames
	// are all it reads), so a joint on bodies whose transform is not kept
	// current (setLazyPose) can be asked without making it current.
	bool jointWorldFramesAt( JointHandle h, const double posA[3], const float rotA[4], const double posB[3],
							 const float rotB[4], JointFrame& a, JointFrame& b ) const;
	// The joint's frames relative to its bodies, exactly as Box3D keeps them.
	bool jointLocalFrames( JointHandle h, JointFrame& a, JointFrame& b ) const;
	// Aims a joint's frame A again: it is its anchor shifted by `shift` (in the
	// anchor's axes, which are the frame's), and the shift becomes `restLength`
	// along the direction from the anchor to frame B now (none when they are
	// closer than `minDistance`). `shift` is updated; true if the frame moved.
	bool reaimJointFrameA( JointHandle h, float restLength, float minDistance, double shift[3] );
	// A body-relative frame in world space with the body's current pose (null
	// handle: the world).
	bool localToWorld( BodyHandle h, const JointFrame& local, JointFrame& out ) const;
	// userData of the joints whose force or torque went over their threshold in
	// the last step.
	int brokenJoints( std::vector<void*>& userData ) const;
	int jointCount() const;
	// Box3D's defaults for a type (b3Default*JointDef), in JointSettings terms.
	static JointSettings defaultJointSettings( JointType type );

	// ---- collision filtering
	// Set every shape of the body to `f` (Box3D re-filters the shape's contacts).
	void setFilterByHandle( BodyHandle h, const CollisionFilter& f );

	// Push changed settings into the live world. workerCount is kept (it needs a
	// new world).
	void applyTuning( const WorldSettings& s );

	// Box3D's own timing of the most recent step, in milliseconds (all 0 if no
	// world). Some stages sit inside others: `step` is the whole step, and the
	// solver's setup, constraints, transforms, events and refit are part of it,
	// and the constraint parts are inside `constraints`.
	struct StepProfile
	{
		float step = 0.0f;
		float pairs = 0.0f;
		float collide = 0.0f;
		float solverSetup = 0.0f;
		float constraints = 0.0f;
		// The parts of the constraint solve, summed over the sub-steps.
		float prepareConstraints = 0.0f;
		float integrateVelocities = 0.0f;
		float warmStart = 0.0f;
		float solveImpulses = 0.0f;
		float integratePositions = 0.0f;
		float relaxImpulses = 0.0f;
		float restitution = 0.0f;
		float storeImpulses = 0.0f;
		float splitIslands = 0.0f; // runs beside the solve, not inside it
		float transforms = 0.0f;
		float events = 0.0f; // joint and hit events
		float refit = 0.0f;
		float bullets = 0.0f; // continuous collision of fast bodies
		float sensorHits = 0.0f;
		float sleepIslands = 0.0f;
		float sensors = 0.0f;
	};
	StepProfile stepProfile() const;

	int recycledContactCount() const; // contacts recycled in the most recent step (0 if no world)
	bool isAwakeByHandle( BodyHandle h ) const;

	// ---- test-only inspection ---------------------------------------------
	// Only the tests call these; the plugin does not.
	// Touching contact points that carried impulse in the last step. `out` is
	// cleared first.
	int getContactsByHandle( BodyHandle h, std::vector<ContactPoint>& out ) const;
	// Replaces the user data createBody stored (returned in BodyMove /
	// ContactPoint).
	void setBodyUserData( BodyHandle h, void* userData );
	// Read back what was set. Material is the first shape's; 0 / false if
	// unknown.
	bool getMaterialByHandle( BodyHandle h, float& friction, float& restitution ) const;
	bool getSurfaceByHandle( BodyHandle h, float& rollingResistance, float tangentVelocity[3] ) const;
	bool getDampingByHandle( BodyHandle h, float& linear, float& angular ) const;
	float gravityScaleByHandle( BodyHandle h ) const;
	bool contactRecyclingByHandle( BodyHandle h ) const;
	bool isSleepEnabledByHandle( BodyHandle h ) const;
	bool isEnabledByHandle( BodyHandle h ) const;
	bool fastRotationByHandle( BodyHandle h ) const;
	float sleepThresholdByHandle( BodyHandle h ) const;
	float ccdThresholdByHandle( BodyHandle h ) const;
	float massByHandle( BodyHandle h ) const;
	int shapeCountByHandle( BodyHandle h ) const;
	// Hulls this world holds, hull builds / attempts (b3CreateHull calls) and the
	// shared cache's bytes.
	int hullCacheSize() const;
	// The distinct triangle meshes this world holds (bodies with the same mesh
	// share one).
	int meshCount() const;
	static size_t sharedHullBytes();
	static int hullBuildAttempts();

	int bodyCount() const; // all live bodies

	// ---- convex hulls ----------------------------------------------------
	// Two steps, so the volume is known (mass -> density) before a shape exists:
	//   1. createHull(body-local xyz) -> HullId + volume. Cached by content.
	//   Returns 0 for
	//      < 4 usable points, non-finite or flat input, or a hull too big even
	//      after reducing the vertex budget (the caller falls back to a box).
	//   2. createHullShape once per piece. Several hulls on one body make a
	//   dynamic compound;
	//      overlapping pieces count their overlap twice in the mass.
	// Hulls live in one cache shared by every world in the process, so a rebuilt
	// world finds the hulls its predecessor built. Box3D adds a hull to its
	// world's database when a shape is made (identical hulls share one clone), so
	// the cache is free to evict; a world holds
	// the hulls it found or built (findHull, createHull*) until pruneHulls(), so
	// a returned HullId stays usable for createHullShape until then. findHull,
	// createHull and createHullWithKey may be called from several threads at
	// once.
	HullId createHull( const float* xyz, int pointCount, float* outVolume = nullptr );

	// Caller-supplied key (packed instances of one source): findHull first, build
	// only on a miss. One key must always mean the same points; 0 is not allowed.
	// `budgetGroup` (optional, e.g. the source's hash) remembers which vertex
	// budget worked, so the next hull of the group skips the attempts that are
	// bound to fail.
	HullId findHull( HullId key, float* outVolume = nullptr ) const;
	HullId createHullWithKey( HullId key, const float* xyz, int pointCount, float* outVolume = nullptr,
							  uint64_t budgetGroup = 0 );
	// lets go of the hulls this world holds (the shared cache keeps them)
	void pruneHulls();

	// A body's collision shapes in body space, for drawing (the solver's guide
	// geometry). Hulls (boxes too) give their vertices and each face as a loop
	// of vertex indices; spheres a centre (c1) and radius; capsules both centres
	// and the radius; meshes their scaled vertices and the edges worth drawing
	// (boundary and creased ones, at most kMaxMeshOutlineEdges). Other kinds are
	// skipped. False for an invalid handle.
	struct ShapeOutline
	{
		enum Kind
		{
			Hull,
			Sphere,
			Capsule,
			Mesh
		} kind = Hull;
		std::vector<float> verts; // hull, mesh: xyz per vertex
		std::vector<int> loops;	  // hull: per face, its vertex count then indices;
								  // mesh: vertex index pairs, one per drawn edge
		float c1[3] = { 0, 0, 0 }, c2[3] = { 0, 0, 0 };
		float radius = 0.0f;
	};
	bool bodyShapeOutlines( BodyHandle h, std::vector<ShapeOutline>& out ) const;

	// AutoFit: a box / sphere / capsule fitted to body-local points along the
	// frame that gives the smallest box (fitShapeInFrame in Box3DShapes.h). The
	// candidates are the body axes and, from a convex hull of the points, each
	// face normal paired with each of that face's edges, so a box, prism or
	// fractured piece turned inside its packed geometry gets a shape turned
	// with it. A turned frame wins only when its box is clearly smaller, so
	// geometry along the body axes keeps the body axes exactly. Fewer than 4
	// points or no hull: the body axes. Thread-safe. Results are cached
	// process-wide by the points' content, so identical pieces fit once.
	// alongSymmetryAxis: a capsule or cylinder runs along the points' axis of
	// symmetry (symmetryAxisFrame), not the axis nearest Y: a piece's own shape
	// (Box3DShapeBuild's `bullet_georep`).
	static bool fitShapeOriented( ShapeType type, const float* xyz, int count, ShapeSpec& out,
								  bool alongSymmetryAxis = false );
	// The same cache under a caller's key (packedFitKey), for fits of packed
	// instances of one source that need not build their points to look it up.
	// storeFit keeps what fitShapeOriented returned for that source; findFit is
	// false on a miss. 0 is not a key. Thread-safe.
	static bool findFit( uint64_t key, ShapeSpec& out );
	static void storeFit( uint64_t key, const ShapeSpec& spec );
	// The centroid of the convex hull of the points (the centre of its volume,
	// as Bullet's RBD Object takes its pivot), in the points' own space. False
	// for fewer than 4 points, non-finite or flat input. Thread-safe, nothing
	// cached.
	static bool hullCentroid( const float* xyz, int count, double out[3] );

	// A rough size of this world in memory, in bytes: Box3D's bodies, shapes,
	// contacts and joints at measured average sizes, plus the hulls, meshes and
	// height fields it holds. For cache accounting, not exact. Call between
	// steps, from the thread that steps the world.
	size_t estimatedBytes() const;

	// The shared cache: the byte budget past which the least recently used
	// entries are evicted (default 256 MB). Builds count b3CreateHull calls
	// (tests).
	static void setSharedHullBudget( size_t bytes );
	static int hullBuildCount();

	// ---- pose and velocity (never pass a stale handle) -----------------------
	bool getTransformByHandle( BodyHandle h, double outPos[3], float outRotXYZW[4] ) const;
	bool getVelocityByHandle( BodyHandle h, float outLinear[3], float outAngular[3] ) const;
	bool setVelocityByHandle( BodyHandle h, const float linear[3], const float angular[3] );
	bool setTransformByHandle( BodyHandle h, const double pos[3], const float rotXYZW[4] );

	// ---- bodies whose transform follows another body -------------------------
	// A body parked in a compound (a disabled copy of a piece) is placed by its
	// compound, so setting every one's transform each step costs a Box3D call
	// each for something almost nothing reads. A lazy body's transform is only
	// brought up to date when something reads it through this API (poses,
	// states, joint frames, making a joint, enabling it): `refresh(h)` is then
	// called, once per invalidateLazyPoses(), and should set the body's
	// transform (setTransformByHandle). Reads may come from several threads:
	// refreshes are serialized. A body is made lazy at its current pose.
	void setLazyPose( BodyHandle h, bool lazy );
	// Every lazy body's transform may have changed (O(1)).
	void invalidateLazyPoses();
	void setPoseRefresher( std::function<void( BodyHandle )> refresh );
	bool hasPoseRefresher() const;

	// ---- move events (valid until the next step or a body is removed) --------
	int getMoveEvents( std::vector<BodyMove>& out ) const; // resizes `out`; returns the count
	// What createBody stored (null for a body made without any).
	void* bodyUserDataByHandle( BodyHandle h ) const;

	// ---- misc --------------------------------------------------------------
	static void version( int& major, int& minor, int& revision );
	static int liveWorldCount(); // Box3D worlds alive in this process (all Box3DWorlds)
	// Box3D assertions that failed so far in this process (always 0 when Box3D is
	// built without asserts, e.g. Release). They are logged to stderr and ignored
	// rather than breaking.
	static int assertCount();
	// B3_MAX_WORLDS: create() fails once liveWorldCount() reaches it
	static int maxWorldCount();

private:
	// Brings a lazy body's transform up to date (see setLazyPose).
	void freshenPose( BodyHandle h ) const;

	// The shape builders behind createShape.
	bool boxShape( BodyHandle h, const BodySettings& b, const float halfExtents[3], const float center[3],
				   const float rotationXYZW[4], bool updateMass, float densityScale = 1.0f );
	bool sphereShape( BodyHandle h, const BodySettings& b, float radius, const float center[3], bool updateMass,
					  float densityScale = 1.0f );
	bool capsuleShape( BodyHandle h, const BodySettings& b, float radius, float halfLength, const float center[3],
					   const float rotationXYZW[4], bool updateMass, float densityScale = 1.0f );
	// A kCylinderSides-sided prism hull from b3CreateCylinder, turned and moved
	// like a capsule; cached by its size and pose.
	bool cylinderShape( BodyHandle h, const BodySettings& b, float radius, float halfLength, const float center[3],
						const float rotationXYZW[4], bool updateMass, float densityScale = 1.0f );

	struct Impl;
	std::unique_ptr<Impl> myImpl;
};

} // namespace b3dop
