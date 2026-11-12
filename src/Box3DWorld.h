// Box3DWorld.h
//
// Houdini-free wrapper around one Box3D world; the only interface the SIM_* code uses.
// Box3D's headers are not included here (pimpl), so Houdini code never sees them.
//
// Conventions: positions double[3] (works with BOX3D_DOUBLE_PRECISION too); quaternions
// float[4] x, y, z, w (the layout of UT_QuaternionF and b3Quat); keyed-API ids are
// caller-supplied uint64_t.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "Box3DShapes.h"

namespace b3dop {

// Handle to a convex hull owned by the world's hull cache. 0 is invalid.
using HullId = uint64_t;

// A body, valid until it is destroyed (or the world is). No hash lookup. 0 is invalid.
using BodyHandle = uint64_t;

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
    int   workerCount   = 1;      // parallel tasks per step, fixed at creation (max 32). With a TaskRunner these
                                  // run on the host's pool; without one, on threads Box3D starts for this world
    bool  enableSleep   = true;
    bool  enableContinuous = true;    // continuous collision vs static bodies (fast objects: vs moving ones too)
    bool  enableSpeculative = true;   // off only to reduce ghost collisions
    float contactHertz      = 30.0f;  // contact stiffness (cycles per second)
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
    int   filterProfile  = -1;      // collision filter profile (filterProfile()), -1 = none

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

    // ---- handle API (what the solver uses) -------------------------------------
    // No hash maps: createBody returns the handle and shape calls take the caller's
    // BodySettings. Hulls are tracked per handle, so destroyBody releases them.
    // A shape add recomputes the mass unless updateMass is false; after adding with false,
    // call applyMassFromShapes or applyMassByHandle before the next step (the solver always
    // does it that way). The keyed API further down is a convenience layer for the tests.
    void       reserve(size_t bodyCount);          // hint: sizes the hull bookkeeping
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
    bool       createMeshShape(BodyHandle h, const BodySettings &body, const float *xyz, int vertexCount,
                               const int32_t *triangles, int triangleCount, float weldTolerance = 0.0f);
    void       applyMassFromShapes(BodyHandle h);
    bool       setTypeByHandle(BodyHandle h, BodySettings::Type type);   // static / kinematic / dynamic

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


    // Set every shape's density and apply the mass from the shapes; then, for dynamic bodies,
    // comAtOrigin moves the centre of mass to the body origin (parallel axis theorem) and
    // inertiaScale scales the inertia. setTypeByHandle recomputes from the shapes, so re-apply
    // after a type change.
    void  applyMassByHandle(BodyHandle h, float density, bool comAtOrigin, float inertiaScale);
    bool  getMassDataByHandle(BodyHandle h, float &mass, float center[3], float inertiaDiag[3]) const;

    // Box3D's single sleep threshold on |v| + |w| * extent. Negative = Box3D's default.
    void  setSleepThresholdByHandle(BodyHandle h, float threshold);
    float sleepThresholdByHandle(BodyHandle h) const;

    // Kinematic body: reach this pose in timeStep (Box3D sets the velocity, so it pushes what it
    // touches). The velocity remains afterwards; zero it when the target stops.
    void  setTargetTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4], float timeStep);

    // Touching contact points that carried impulse in the last step. `out` is cleared first.
    int   getContactsByHandle(BodyHandle h, std::vector<ContactPoint> &out) const;

    // ---- collision filtering ---------------------------------------------------
    // A profile names a body (group, object name, object id) and its ignore patterns
    // (CollisionFilterTable). Shapes store their body's profile; a Box3D custom filter asks the
    // table. Call finishFilterProfiles() after adding profiles, before stepping.
    int   filterProfile(const FilterProfile &p);
    void  finishFilterProfiles();
    bool  shouldCollide(int profileA, int profileB) const;
    const FilterProfile *filterProfileData(int profile) const;
    void  setFilterProfileByHandle(BodyHandle h, int profile);       // also re-filters its contacts

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

    // ---- bodies --------------------------------------------------------
    bool addBody(uint64_t id, const BodySettings &body,
                 const double position[3], const float rotationXYZW[4]);
    void removeBody(uint64_t id);
    bool hasBody(uint64_t id) const;
    int  bodyCount() const;                      // all live bodies, keyed or handle-made

    bool setBodyType(uint64_t id, BodySettings::Type type);
    // Frees the settings kept for adding shapes; shapes added afterwards use defaults.
    void releaseBuildState(uint64_t id);
    bool getBodyType(uint64_t id, BodySettings::Type &out) const;
    bool setBodyDensity(uint64_t id, float density);     // for shapes added after the call

    // ---- shapes --------------------------------------------------------
    // `center`: offset from the body origin. `rotationXYZW` (optional) turns the shape about
    // its centre; a rotated box is built as a hull.
    bool addBoxShape(uint64_t bodyId, const float halfExtents[3], const float center[3] = nullptr,
                     const float rotationXYZW[4] = nullptr);
    bool addSphereShape(uint64_t bodyId, float radius, const float center[3] = nullptr);
    // Along Y before rotation. halfLength: half the straight section.
    bool addCapsuleShape(uint64_t bodyId, float radius, float halfLength, const float center[3] = nullptr,
                         const float rotationXYZW[4] = nullptr);
    bool addShape(uint64_t bodyId, const ShapeSpec &shape);   // box, sphere or capsule, incl. shape.center / shape.rot

    // ---- convex hulls ----------------------------------------------------
    // Two steps, so the volume is known (mass -> density) before a shape exists:
    //   1. createHull(body-local xyz) -> HullId + volume. Cached by content. Returns 0 for
    //      < 4 usable points, non-finite or flat input, or a hull too big even after
    //      reducing the vertex budget (the caller falls back to a box).
    //   2. addHullShape / createHullShape once per piece. Several hulls on one body make a
    //      dynamic compound; overlapping pieces count their overlap twice in the mass.
    // Box3D clones the hull on shape creation; pruneHulls() frees entries no shape uses.
    HullId createHull(const float *xyz, int pointCount, float *outVolume = nullptr);

    // Caller-supplied key (packed instances of one source): findHull first, build only on a
    // miss. One key must always mean the same points; 0 is not allowed.
    HullId findHull(HullId key, float *outVolume = nullptr) const;
    HullId createHullWithKey(HullId key, const float *xyz, int pointCount, float *outVolume = nullptr);
    bool   addHullShape(uint64_t bodyId, HullId hull);
    void   pruneHulls();                // frees cache entries no body uses
    int    hullCacheSize() const;
    int    shapeCount(uint64_t bodyId) const;
    float  bodyMass(uint64_t bodyId) const;      // 0 if unknown

    // ---- handle access (never pass a stale handle) ---------------------------
    BodyHandle bodyHandle(uint64_t id) const;     // 0 if there is no such body
    bool getTransformByHandle(BodyHandle h, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocityByHandle(BodyHandle h, float outLinear[3], float outAngular[3]) const;
    bool setVelocityByHandle(BodyHandle h, const float linear[3], const float angular[3]);
    bool setTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4]);

    // ---- move events (valid until the next step or a body is removed) ----------
    int  moveEventCount() const;
    bool getMoveEvent(int index, BodyMove &out) const;
    int  getMoveEvents(std::vector<BodyMove> &out) const;   // resizes `out`; returns the count
    void setBodyUserData(BodyHandle h, void *userData);     // returned in BodyMove / ContactPoint

    // ---- keyed read back / push in ---------------------------------------
    bool getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocity(uint64_t id, float outLinear[3], float outAngular[3]) const;
    bool setVelocity(uint64_t id, const float linear[3], const float angular[3]);
    bool setTransform(uint64_t id, const double pos[3], const float rotXYZW[4]);

    // ---- misc --------------------------------------------------------------
    static void version(int &major, int &minor, int &revision);
    static int  liveWorldCount();   // Box3D worlds alive in this process (all Box3DWorlds)
    // Box3D assertions that failed so far in this process (always 0 when Box3D is built without
    // asserts, e.g. Release). They are logged to stderr and ignored rather than breaking.
    static int  assertCount();
    static int  maxWorldCount();    // B3_MAX_WORLDS: create() fails once liveWorldCount() reaches it

private:
    // The shape builders behind both APIs.
    bool boxShape(BodyHandle h, const BodySettings &b, const float halfExtents[3], const float center[3],
                  const float rotationXYZW[4], bool updateMass);
    bool sphereShape(BodyHandle h, const BodySettings &b, float radius, const float center[3], bool updateMass);
    bool capsuleShape(BodyHandle h, const BodySettings &b, float radius, float halfLength,
                      const float center[3], const float rotationXYZW[4], bool updateMass);

    struct Impl;
    std::unique_ptr<Impl> myImpl;
};

} // namespace b3dop
