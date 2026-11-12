// Box3DWorld.h
//
// Houdini-free wrapper around one Box3D world; the only interface the SIM_* code uses.
// Box3D's headers are not included here (pimpl), so Houdini code never sees them.
//
// Conventions: positions double[3] (works with BOX3D_DOUBLE_PRECISION too); quaternions
// float[4] x, y, z, w (the layout of UT_QuaternionF and b3Quat). Bodies are addressed by
// BodyHandle only (tests/KeyedWorld.h adds caller-chosen ids on top, for the tests).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "Box3DJoints.h"
#include "Box3DShapes.h"

namespace b3dop {

// A convex hull in the process-wide hull cache (its content or caller-supplied key). 0 is invalid.
using HullId = uint64_t;

// Box3D worker tasks per step when nothing else is asked for (the solver's Threads default too).
constexpr int kDefaultWorkerCount = 8;

// A body, valid until it is destroyed (or the world is). No hash lookup. 0 is invalid.
using BodyHandle = uint64_t;

// A joint, valid until it, either of its bodies, or the world is destroyed. 0 is invalid.
using JointHandle = uint64_t;

// One of Box3D's body move events: an awake body after the last step and its final
// transform. A body that fell asleep this step appears once more with fellAsleep set.
// Bodies moved by the caller (setTransform...) do not appear.
struct BodyMove
{
    void      *userData = nullptr;      // whatever setBodyUserData stored
    BodyHandle handle   = 0;
    double     pos[3]   = { 0, 0, 0 };
    float      rot[4]   = { 0, 0, 0, 1 };
    bool       fellAsleep = false;
};

// Lets the host supply the threads Box3D's parallel work runs on, so worlds do not each start an
// OS thread pool of their own (Box3D's built-in scheduler keeps workerCount - 1 threads alive for
// the life of every world). Houdini-free: the plugin implements it on Houdini's TBB pool
// (Box3DHoudiniTasks.h). Without one, Box3DWorld uses Box3D's own pool (the tests do).
//
// A runner belongs to one world and is used from one stepping thread at a time.
class TaskRunner
{
public:
    virtual ~TaskRunner() = default;

    // Run fn(ctx) exactly once. Returns true if it was queued, in which case wait() must be called
    // before the data it uses goes away; false if it already ran before returning.
    virtual bool run(void (*fn)(void *), void *ctx) = 0;

    // Block until every task queued by run() has finished. The calling thread may run tasks.
    virtual void wait() = 0;

    // Run fn(ctx) around a whole step (a place to isolate task stealing). Default: just call it.
    virtual void isolate(void (*fn)(void *), void *ctx) { fn(ctx); }
};

// Everything but workerCount can change on a live world (applyTuning). Defaults are Box3D's,
// except gravity (standard gravity instead of 10).
struct WorldSettings
{
    float gravity[3]    = { 0.0f, -9.80665f, 0.0f };
    int   subSteps      = 4;      // Box3D recommends ~4 per step
    int   workerCount   = kDefaultWorkerCount;   // parallel tasks per step, fixed at creation (max 32). With a TaskRunner these
                                  // run on the host's pool; without one, on threads Box3D starts for this world
    bool  enableSleep   = true;
    bool  enableContinuous = true;    // continuous collision vs static bodies (fast objects: vs moving ones too)
    bool  enableSpeculative = true;   // off only to reduce ghost collisions
    float contactStiffness  = 30.0f;  // contact stiffness in Hz (cycles per second)
    float contactDampingRatio = 10.0f;    // contact damping (non-dimensional)
    float contactPushSpeed  = 3.0f;   // max speed overlapping shapes are pushed apart (m/s)
    float restitutionThreshold = 1.0f;    // slower collisions do not bounce (m/s)
    float maxLinearSpeed    = 400.0f; // m/s
    // Contact recycling: a touching pair that has moved less than this since its contact points were
    // last computed (and turned less than ~10 degrees) keeps them instead of recomputing, which
    // saves time but is slightly less exact. World-wide; 0 turns recycling off for every body.
    // Box3D's default is 10 * linear slop = 0.05 at its default length units (1 unit = 1 metre).
    // Per body: BodySettings::contactRecycling.
    float contactRecycleDistance = 0.05f;

    int   restitutionIterations = 2;          // extra solver passes for bounce (0-63)
    bool  restitutionPropagation = false;     // bounce passes on through stacks
    float isotropicInertiaTolerance = 0.2f;   // 0-1; near-spherical bodies lose gyroscopic torque. New bodies / mass changes only
    bool  warmStarting = true;                // off only for comparing against the reference solver
    float impactMinSpeed = 0.0f;              // Box3D hit event threshold (m/s): Impacts only for pairs that hit this fast. 0 = every contact
    int   frictionMixing = 0;                 // 0 geometric mean (Box3D default), 1 min, 2 max, 3 average, 4 multiply
    int   restitutionMixing = 0;              // 0 max (Box3D default), 1 min, 2 average, 3 multiply, 4 geometric mean
};

// One contact point of a body after a step (getContactsByHandle).
struct ContactPoint
{
    BodyHandle other         = 0;
    void      *otherUserData = nullptr;
    double     pos[3]        = { 0, 0, 0 }; // world space
    float      normal[3]     = { 0, 1, 0 }; // from this body into the other
    float      impulse       = 0.0f;        // total normal impulse over the step (N*s), > 0
};

// Box3D's b3Filter, checked by Box3D in its broad phase. Two shapes with the same non-zero
// group: positive always collide, negative never. Otherwise they collide only when each one's
// mask has a bit of the other's category. The defaults (every bit, group 0) collide with all.
struct CollisionFilter
{
    uint64_t category = ~uint64_t(0);   // B3_DEFAULT_CATEGORY_BITS
    uint64_t mask     = ~uint64_t(0);   // B3_DEFAULT_MASK_BITS
    int      group    = 0;

    bool operator==(const CollisionFilter &o) const
    {
        return category == o.category && mask == o.mask && group == o.group;
    }
    bool operator!=(const CollisionFilter &o) const { return !(*this == o); }
};

struct BodySettings
{
    enum class Type { Static, Kinematic, Dynamic };

    Type  type           = Type::Dynamic;
    float mass           = 1.0f;    // solver only: density = mass / shape volume
    float density        = 1.0f;    // what Box3DWorld uses
    bool  computeMass    = false;   // solver only: mass = density * volume instead
    float friction       = 0.6f;
    float restitution    = 0.0f;
    float linearDamping  = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale   = 1.0f;
    bool  contactRecycling = true;  // `contactrecycling`: this body's contacts may be recycled (see WorldSettings)
    bool  enableSleep    = true;
    bool  isBullet       = false;   // Box3D "bullet": continuous collision vs moving bodies too (`fastobject`)
    float sleepThreshold = 0.05f;   // m/s
    CollisionFilter filter;         // `collision_category` / `collision_mask` / `collision_group`

    // Solver only:
    ShapeType defaultShape = ShapeType::Box;     // when `bullet_georep` is missing or unknown
    bool      autoFit = true;                    // box / sphere / capsule fit the packed geometry's bounds
    bool      hullPerConnectedSet = false;       // convex: one hull per set of connected primitives
    bool      flipMeshWinding = false;           // affector (Static Object) meshes: use the opposite triangle winding
};

class Box3DWorld
{
public:
    Box3DWorld();
    ~Box3DWorld();

    Box3DWorld(const Box3DWorld &)            = delete;
    Box3DWorld &operator=(const Box3DWorld &) = delete;

    // ---- lifetime ----------------------------------------------------
    // Creates the b3World. `tasks` (optional) is kept for the life of the world and used for every
    // step; with workerCount <= 1 it is not used.
    bool create(const WorldSettings &settings, std::shared_ptr<TaskRunner> tasks = nullptr);
    void destroy();                                // safe to call repeatedly
    bool isValid() const;
    const WorldSettings &settings() const;

    // ---- stepping ----------------------------------------------------
    void step(float timeStep);

    // ---- bodies and shapes ---------------------------------------------------
    // No hash maps: createBody returns the handle and shape calls take the caller's
    // BodySettings. Hulls are tracked per handle, so destroyBody releases them.
    // A shape add recomputes the mass unless updateMass is false; after adding with false,
    // call applyMassFromShapes or applyMassByHandle before the next step (the solver always
    // does it that way).
    BodyHandle createBody(const BodySettings &body, const double position[3],
                          const float rotationXYZW[4], void *userData = nullptr);   // 0 on failure
    void       destroyBody(BodyHandle h);
    bool       createShape(BodyHandle h, const BodySettings &body, const ShapeSpec &shape,
                           bool updateMass = true);
    bool       createHullShape(BodyHandle h, const BodySettings &body, HullId hull,
                               bool updateMass = true);
    // A static triangle mesh (xyz: body-local vertices, triangles: 3 indices each, CCW). Static bodies
    // only; false for a non-static body, fewer than 3 vertices or 1 triangle, non-finite vertices or
    // out-of-range indices. weldTolerance > 0 welds nearby vertices. The mesh belongs to the body: it is
    // freed with the body or by destroyShapesByHandle. Triangle meshes are single-sided and have no mass.
    // `scale` (optional, per axis, positive) scales the vertices in body space without rebuilding the
    // mesh; setMeshScaleByHandle changes it later.
    bool       createMeshShape(BodyHandle h, const BodySettings &body, const float *xyz, int vertexCount,
                               const int32_t *triangles, int triangleCount, float weldTolerance = 0.0f,
                               const float scale[3] = nullptr);
    void       setMeshScaleByHandle(BodyHandle h, const float scale[3]);     // every mesh shape of the body
    void       applyMassFromShapes(BodyHandle h);
    bool       setTypeByHandle(BodyHandle h, BodySettings::Type type);   // static / kinematic / dynamic
    BodySettings::Type typeByHandle(BodyHandle h) const;                 // Dynamic for a null handle

    // ---- changes to existing bodies (handle API) --------------------------------
    // Take effect from the next step. Only Box3D's own rules wake a body (disabling sleep does).
    void  setMaterialByHandle(BodyHandle h, float friction, float restitution);   // every shape; contacts follow
    void  setDampingByHandle(BodyHandle h, float linear, float angular);
    void  setGravityScaleByHandle(BodyHandle h, float gravityScale);
    void  setSleepEnabledByHandle(BodyHandle h, bool enableSleep);   // the world switch applies on top
    void  setBulletByHandle(BodyHandle h, bool flag);
    // Box3D copies a body's recycling flag into each contact when the contact is made, so on its own
    // a change only reaches new contacts. For an awake dynamic body the existing ones are dropped
    // too (disable + enable, velocity kept) so they are rebuilt with the new setting. A sleeping,
    // static or kinematic body only affects contacts made from now on.
    void  setContactRecyclingByHandle(BodyHandle h, bool flag);
    void  setDensityByHandle(BodyHandle h, float density);           // every shape, then mass from shapes
    void  setEnabledByHandle(BodyHandle h, bool enabled);            // disabled: no motion, no collisions
    bool  isEnabledByHandle(BodyHandle h) const;
    // Remove every shape (releasing its hulls, dropping its contacts) so new ones can be added
    // with updateMass = false, followed by one mass update.
    void  destroyShapesByHandle(BodyHandle h);
    void  wakeByHandle(BodyHandle h);
    // ---- external forces (handle API) -----------------------------------------
    // World position of the body's centre of mass (where forces act). False for no body.
    bool  getCenterOfMassByHandle(BodyHandle h, double outPos[3]) const;
    // A force (N) through the centre of mass and a torque (N*m) about it, in world space, for the
    // NEXT step only: Box3D adds both to its integration in every substep and clears them when the
    // step ends, so call this once per step before step(). Non-finite components count as 0.
    // Only dynamic bodies take them. A sleeping body ignores them unless `wake`, which wakes it
    // first; a constant force with wake = true therefore keeps every body awake.
    void  applyForceByHandle(BodyHandle h, const float force[3], const float torque[3], bool wake);


    // Set every shape's density (skipped with setShapeDensity = false, for shapes just created with
    // it) and apply the mass from the shapes; then, for dynamic bodies, comAtOrigin moves the centre
    // of mass to the body origin (parallel axis theorem) and inertiaScale scales the inertia.
    // setTypeByHandle recomputes from the shapes, so re-apply after a type change.
    void  applyMassByHandle(BodyHandle h, float density, bool comAtOrigin, float inertiaScale,
                            bool setShapeDensity = true);
    bool  getMassDataByHandle(BodyHandle h, float &mass, float center[3], float inertiaDiag[3]) const;

    // Box3D's single sleep threshold on |v| + |w| * extent. Negative = Box3D's default.
    void  setSleepThresholdByHandle(BodyHandle h, float threshold);
    float sleepThresholdByHandle(BodyHandle h) const;

    // Kinematic body: reach this pose in timeStep (Box3D sets the velocity, so it pushes what it
    // touches). The velocity remains afterwards; zero it when the target stops.
    void  setTargetTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4], float timeStep);

    // Touching contact points that carried impulse in the last step. `out` is cleared first.
    int   getContactsByHandle(BodyHandle h, std::vector<ContactPoint> &out) const;

    // Hit events of the last step (pairs that approached faster than impactMinSpeed).
    void  updateHitEvents();
    bool  hadHitEvent(BodyHandle a, BodyHandle b) const;

    // ---- joints ------------------------------------------------------------------
    // frameA / frameB are world-space frames now (anchor P + orient); the joint keeps each relative
    // to its body. A null BodyHandle is the world (a shapeless static body made on first use).
    // Settings are clamped to what Box3D accepts. 0 on failure (both ends the world, the same body
    // twice, or an invalid body).
    // With framesAreLocal the frames are already relative to their bodies (what jointLocalFrames
    // returns), so a joint can be remade exactly, without going through world positions.
    JointHandle createJoint(JointType type, BodyHandle a, BodyHandle b, const JointFrame &frameA,
                            const JointFrame &frameB, const JointSettings &s, void *userData = nullptr,
                            bool framesAreLocal = false);
    void  destroyJoint(JointHandle h);                  // harmless on a joint already gone with its body
    bool  isJointValid(JointHandle h) const;
    // Push every setting of the type into a live joint (frames and bodies stay); wakes its bodies.
    void  updateJoint(JointHandle h, JointType type, const JointSettings &s);
    bool  jointFeedback(JointHandle h, JointType type, JointFeedback &out) const;
    // Where the joint's two frames are now, in world space (each body's pose times its local frame;
    // the world end does not move). Making a joint from these on bodies at the same poses gives the
    // same joint, so a world rebuilt mid-simulation can remake its joints from them.
    bool  jointWorldFrames(JointHandle h, JointFrame &a, JointFrame &b) const;
    // The joint's frames relative to its bodies, exactly as Box3D keeps them.
    bool  jointLocalFrames(JointHandle h, JointFrame &a, JointFrame &b) const;
    // A body-relative frame in world space with the body's current pose (null handle: the world).
    bool  localToWorld(BodyHandle h, const JointFrame &local, JointFrame &out) const;
    // userData of the joints whose force or torque went over their threshold in the last step.
    int   brokenJoints(std::vector<void *> &userData) const;
    int   jointCount() const;
    // Box3D's defaults for a type (b3Default*JointDef), in JointSettings terms.
    static JointSettings defaultJointSettings(JointType type);

    // ---- collision filtering ---------------------------------------------------
    // Set every shape of the body to `f` (Box3D re-filters the shape's contacts).
    void  setFilterByHandle(BodyHandle h, const CollisionFilter &f);

    // Push changed settings into the live world. workerCount is kept (it needs a new world).
    void  applyTuning(const WorldSettings &s);

    // Read back (tests, debugging). Material is the first shape's; 0 / false if unknown.
    bool  getMaterialByHandle(BodyHandle h, float &friction, float &restitution) const;
    bool  getDampingByHandle(BodyHandle h, float &linear, float &angular) const;
    float gravityScaleByHandle(BodyHandle h) const;
    bool  contactRecyclingByHandle(BodyHandle h) const;
    int   recycledContactCount() const;            // contacts recycled in the most recent step (0 if no world)
    bool  isSleepEnabledByHandle(BodyHandle h) const;
    bool  isAwakeByHandle(BodyHandle h) const;
    float massByHandle(BodyHandle h) const;
    int   shapeCountByHandle(BodyHandle h) const;

    int  bodyCount() const;                      // all live bodies

    // ---- convex hulls ----------------------------------------------------
    // Two steps, so the volume is known (mass -> density) before a shape exists:
    //   1. createHull(body-local xyz) -> HullId + volume. Cached by content. Returns 0 for
    //      < 4 usable points, non-finite or flat input, or a hull too big even after
    //      reducing the vertex budget (the caller falls back to a box).
    //   2. createHullShape once per piece. Several hulls on one body make a dynamic compound;
    //      overlapping pieces count their overlap twice in the mass.
    // Hulls live in one cache shared by every world in the process, so a rebuilt world finds the
    // hulls its predecessor built. Box3D copies a hull into its world when a shape is made, so the
    // cache is free to evict; a world holds the hulls it found or built (findHull, createHull*)
    // until pruneHulls(), so a returned HullId stays usable for createHullShape until then.
    // findHull, createHull and createHullWithKey may be called from several threads at once.
    HullId createHull(const float *xyz, int pointCount, float *outVolume = nullptr);

    // Caller-supplied key (packed instances of one source): findHull first, build only on a
    // miss. One key must always mean the same points; 0 is not allowed. `budgetGroup` (optional,
    // e.g. the source's hash) remembers which vertex budget worked, so the next hull of the
    // group skips the attempts that are bound to fail.
    HullId findHull(HullId key, float *outVolume = nullptr) const;
    HullId createHullWithKey(HullId key, const float *xyz, int pointCount, float *outVolume = nullptr,
                             uint64_t budgetGroup = 0);
    void   pruneHulls();                // lets go of the hulls this world holds (the shared cache keeps them)
    int    hullCacheSize() const;       // hulls this world holds

    // The shared cache: entries, bytes, and the byte budget past which the least recently used
    // entries are evicted (default 256 MB). Builds / attempts count b3CreateHull calls (tests).
    static int    sharedHullCount();
    static size_t sharedHullBytes();
    static void   setSharedHullBudget(size_t bytes);
    static int    hullBuildCount();
    static int    hullBuildAttempts();

    // ---- pose and velocity (never pass a stale handle) -------------------------
    bool getTransformByHandle(BodyHandle h, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocityByHandle(BodyHandle h, float outLinear[3], float outAngular[3]) const;
    bool setVelocityByHandle(BodyHandle h, const float linear[3], const float angular[3]);
    bool setTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4]);

    // ---- move events (valid until the next step or a body is removed) ----------
    int  moveEventCount() const;
    bool getMoveEvent(int index, BodyMove &out) const;
    int  getMoveEvents(std::vector<BodyMove> &out) const;   // resizes `out`; returns the count
    void setBodyUserData(BodyHandle h, void *userData);     // returned in BodyMove / ContactPoint

    // ---- misc --------------------------------------------------------------
    static void version(int &major, int &minor, int &revision);
    static int  liveWorldCount();   // Box3D worlds alive in this process (all Box3DWorlds)
    // Box3D assertions that failed so far in this process (always 0 when Box3D is built without
    // asserts, e.g. Release). They are logged to stderr and ignored rather than breaking.
    static int  assertCount();
    static int  maxWorldCount();    // B3_MAX_WORLDS: create() fails once liveWorldCount() reaches it

private:
    // The shape builders behind createShape.
    bool boxShape(BodyHandle h, const BodySettings &b, const float halfExtents[3], const float center[3],
                  const float rotationXYZW[4], bool updateMass);
    bool sphereShape(BodyHandle h, const BodySettings &b, float radius, const float center[3], bool updateMass);
    bool capsuleShape(BodyHandle h, const BodySettings &b, float radius, float halfLength,
                      const float center[3], const float rotationXYZW[4], bool updateMass);

    struct Impl;
    std::unique_ptr<Impl> myImpl;
};

} // namespace b3dop
