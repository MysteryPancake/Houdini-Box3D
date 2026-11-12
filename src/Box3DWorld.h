// Box3DWorld.h
//
// Houdini-free wrapper around a single Box3D world. The SIM_* classes own one
// of these and talk to Box3D only through it.
//
// Box3D's headers are deliberately NOT included here (pimpl), so Houdini
// translation units never see them.
//
// Conventions used at this boundary:
//   * positions  : double[3]   (Box3D is single precision by default, but this
//                               keeps the door open for BOX3D_DOUBLE_PRECISION)
//   * quaternions: float[4] in x, y, z, w order (same as UT_QuaternionF).
//                  Box3D's b3Quat is {v.x, v.y, v.z, s}, i.e. the same layout.
//   * ids        : caller-supplied uint64_t. The DOP solver uses one body per
//                  POINT, keyed by makeBodyKey(objectId, pointId).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "Box3DShapes.h"

namespace b3dop {

// Handle to a convex hull owned by the world's hull cache. 0 is invalid.
using HullId = uint64_t;

// Opaque handle to a body, valid until that body is removed or the world is
// destroyed. Looking a body up by key costs a hash lookup; a handle does not,
// so callers that touch a body every step fetch it once (bodyHandle) and keep it.
// 0 is invalid.
using BodyHandle = uint64_t;

// One entry of Box3D's body move events: a body that was simulated (awake) in the
// last step, with its final transform. The events are Box3D's own contiguous array,
// built during the step, so reading them needs no per-body lookup. Only awake bodies
// appear; a body that fell asleep this step appears once more with fellAsleep set.
// Bodies moved by the caller (setTransform...) do not appear.
struct BodyMove
{
    void      *userData = nullptr;      // whatever setBodyUserData stored
    BodyHandle handle   = 0;
    double     pos[3]   = { 0, 0, 0 };
    float      rot[4]   = { 0, 0, 0, 1 };
    bool       fellAsleep = false;
};

struct WorldSettings
{
    float gravity[3]    = { 0.0f, -9.80665f, 0.0f };
    int   subSteps      = 4;      // Box3D recommends ~4 per step
    int   workerCount   = 1;      // > 1: Box3D's own thread pool, sized at creation (max 32)
    bool  enableSleep   = true;
    bool  enableContinuous = true;
};

struct BodySettings
{
    enum class Type { Static, Kinematic, Dynamic };

    Type  type           = Type::Dynamic;
    // `mass` is what the Houdini solver reads (node default, overridden by the
    // `mass` point attribute): it sets density = mass / total shape volume.
    // Box3DWorld itself only reads `density`.
    float mass           = 1.0f;
    float density        = 1.0f;
    float friction       = 0.6f;
    float restitution    = 0.0f;
    float linearDamping  = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale   = 1.0f;
    bool  enableSleep    = true;
    bool  isBullet       = false;

    // Used when a point has no (or an unknown) `bullet_georep` attribute.
    ShapeType defaultShape = ShapeType::Box;

    // Box / sphere / capsule on a packed point: fit the shape to the packed
    // geometry's bounds (true), or build it from the explicit size attributes
    // (bullet_primS, bullet_radius, ...) (false). The `bullet_autofit` attribute
    // overrides this per point. Read only by the Houdini solver.
    bool      autoFit = true;

    // Convex shapes only: make one hull per set of connected primitives (a
    // compound) instead of one hull of everything.
    bool      hullPerConnectedSet = false;
};

class Box3DWorld
{
public:
    Box3DWorld();
    ~Box3DWorld();

    Box3DWorld(const Box3DWorld &)            = delete;
    Box3DWorld &operator=(const Box3DWorld &) = delete;

    // ---- lifetime ----------------------------------------------------
    bool create(const WorldSettings &settings);   // creates the b3World
    void destroy();                                // safe to call repeatedly
    bool isValid() const;
    const WorldSettings &settings() const;

    // ---- stepping ----------------------------------------------------
    void step(float timeStep);

    // ---- handle-based building (the fast path the solver uses) ----------------
    // Creating a body this way touches no hash map: createBody returns the handle,
    // and the shape calls take that handle plus the caller's BodySettings (which stay a
    // local of the caller, so nothing per body is remembered here). Hulls a body uses
    // are tracked per handle, so destroyBody releases them.
    //
    // Mass: every shape add recomputes the body's mass unless `updateMass` is false.
    // For a body with several shapes pass false for all of them and call
    // applyMassFromShapes ONCE afterwards (it is required before the next step).
    // A single-shape body just leaves `updateMass` true.
    //
    // The keyed API below (addBody, addShape, removeBody ...) is a convenience layer
    // over this one: it keeps key -> handle and key -> BodySettings maps, which the
    // solver does not need.
    void       reserve(size_t bodyCount);          // hint: about that many bodies are coming (sizes the hull bookkeeping)
    BodyHandle createBody(const BodySettings &body, const double position[3],
                          const float rotationXYZW[4], void *userData = nullptr);   // 0 on failure
    void       destroyBody(BodyHandle h);
    bool       createShape(BodyHandle h, const BodySettings &body, const ShapeSpec &shape,
                           bool updateMass = true);
    bool       createHullShape(BodyHandle h, const BodySettings &body, HullId hull,
                               bool updateMass = true);
    void       applyMassFromShapes(BodyHandle h);
    bool       setTypeByHandle(BodyHandle h, BodySettings::Type type);   // static / kinematic / dynamic

    // ---- mid-simulation changes (handle API) --------------------------------
    // What the solver uses when a point attribute changes on an existing body. Each takes
    // effect from the next step; none of them wakes a sleeping body unless it says so
    // (Box3D's own rules: enabling "never sleep" wakes it).
    //
    // Friction / restitution go to every shape of the body. Box3D re-mixes a contact's
    // friction and restitution from its shapes every step, so touching contacts follow too.
    void  setMaterialByHandle(BodyHandle h, float friction, float restitution);
    void  setDampingByHandle(BodyHandle h, float linear, float angular);
    void  setGravityScaleByHandle(BodyHandle h, float gravityScale);
    // May the body sleep? ANDed with the world's enableSleep, like createBody does.
    void  setSleepEnabledByHandle(BodyHandle h, bool enableSleep);
    // Density of every shape, then the mass is applied once. The solver keeps mass constant
    // by passing mass / total shape volume, so this is "change the mass" and also what a
    // fresh build with the same attributes would produce (inertia follows the shapes).
    void  setDensityByHandle(BodyHandle h, float density);
    // Take a body out of the simulation (no collisions, no motion) or put it back. A
    // disabled body keeps its shapes and settings.
    void  setEnabledByHandle(BodyHandle h, bool enabled);
    bool  isEnabledByHandle(BodyHandle h) const;
    // Remove every shape of the body (and release the hulls it used) so new ones can be
    // added: destroyShapesByHandle, then createShape / createHullShape with updateMass =
    // false, then applyMassFromShapes once. Contacts of the old shapes are dropped.
    void  destroyShapesByHandle(BodyHandle h);
    void  wakeByHandle(BodyHandle h);

    // Read back (tests, debugging). Material is the first shape's; 0 / false if unknown.
    bool  getMaterialByHandle(BodyHandle h, float &friction, float &restitution) const;
    bool  getDampingByHandle(BodyHandle h, float &linear, float &angular) const;
    float gravityScaleByHandle(BodyHandle h) const;
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

    // Change an existing body between static / kinematic / dynamic.
    bool setBodyType(uint64_t id, BodySettings::Type type);
    // Keyed API only: frees the settings kept for building the body's shapes (the shapes,
    // density and type live in Box3D). Call once the body's shapes are all added;
    // getBodyType and setBodyType keep working, but shapes added afterwards use default
    // settings. (Bodies made with createBody never have any, so the solver does not call this.)
    void releaseBuildState(uint64_t id);
    bool getBodyType(uint64_t id, BodySettings::Type &out) const;

    // Density used by shapes added to this body AFTER the call. The solver sets
    // it once it knows the total shape volume (mass attribute -> density).
    bool setBodyDensity(uint64_t id, float density);

    // ---- shapes --------------------------------------------------------
    // `center` is the shape's offset from the body origin (default: centred).
    // `rotationXYZW` (optional, unit quaternion) turns the shape about its own
    // centre; null or identity means unrotated. A rotated box is built as a hull.
    bool addBoxShape(uint64_t bodyId, const float halfExtents[3], const float center[3] = nullptr,
                     const float rotationXYZW[4] = nullptr);
    bool addSphereShape(uint64_t bodyId, float radius, const float center[3] = nullptr);
    // The capsule stands along Y (before `rotationXYZW`). halfLength: half the straight section.
    bool addCapsuleShape(uint64_t bodyId, float radius, float halfLength, const float center[3] = nullptr,
                         const float rotationXYZW[4] = nullptr);
    bool addShape(uint64_t bodyId, const ShapeSpec &shape);   // box, sphere or capsule, incl. shape.center / shape.rot

    // ---- convex hulls ----------------------------------------------------
    // Hulls are built in two steps so the caller can learn the volume (to turn
    // a mass into a density) before any shape exists:
    //
    //   1. createHull(points) -> HullId, plus the hull volume. Points are
    //      body-local xyz triples. Hulls are cached by CONTENT, so a thousand
    //      packed prims instancing one source cost one hull build. Fails (0) if
    //      there are fewer than 4 usable points, a point is not finite, the
    //      points are flat or collinear, or the hull is too big even after
    //      reducing the vertex budget; the caller then falls back to a box.
    //   2. addHullShape(bodyId, hullId), once per piece. Several hulls on one
    //      body is a runtime compound: the right way to get a DYNAMIC concave
    //      body in Box3D. Overlapping pieces count their overlap twice in the
    //      mass.
    //
    // Box3D clones the hull into its own database when the shape is created,
    // so the cache entry is only needed until then. Entries that no shape uses
    // are released by pruneHulls().
    HullId createHull(const float *xyz, int pointCount, float *outVolume = nullptr);

    // Same, but the caller supplies the cache key. For packed instances that
    // Houdini says share one source: look the key up first (findHull) and only
    // build the points on a miss. The caller promises that one key always means
    // the same points. A 0 key is not allowed.
    HullId findHull(HullId key, float *outVolume = nullptr) const;
    HullId createHullWithKey(HullId key, const float *xyz, int pointCount, float *outVolume = nullptr);
    bool   addHullShape(uint64_t bodyId, HullId hull);
    void   pruneHulls();                // frees cache entries no body uses
    int    hullCacheSize() const;
    int    shapeCount(uint64_t bodyId) const;
    float  bodyMass(uint64_t bodyId) const;      // Box3D's computed mass, 0 if unknown
    // bool addMeshShape(...);   // static colliders only, probably

    // ---- handle access -----------------------------------------------------
    // The same operations as the keyed ones below, minus the hash lookup. A
    // stale handle (body removed, world destroyed) must not be used.
    BodyHandle bodyHandle(uint64_t id) const;     // 0 if there is no such body
    bool getTransformByHandle(BodyHandle h, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocityByHandle(BodyHandle h, float outLinear[3], float outAngular[3]) const;
    bool setVelocityByHandle(BodyHandle h, const float linear[3], const float angular[3]);
    bool setTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4]);

    // ---- move events (valid until the next step or until a body is removed) ----
    // Cleared at the start of every step, so after a step that did not run they are empty.
    int  moveEventCount() const;
    bool getMoveEvent(int index, BodyMove &out) const;
    // All of them in one call: `out` is resized to the event count (its capacity is
    // kept, so a reused vector allocates nothing in a steady state). Returns the count.
    int  getMoveEvents(std::vector<BodyMove> &out) const;
    // Stored in the Box3D body and returned in BodyMove::userData. The caller owns
    // what it points at; it must stay valid while the body exists.
    void setBodyUserData(BodyHandle h, void *userData);

    // ---- read back / push in -------------------------------------------
    bool getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocity(uint64_t id, float outLinear[3], float outAngular[3]) const;
    bool setVelocity(uint64_t id, const float linear[3], const float angular[3]);
    bool setTransform(uint64_t id, const double pos[3], const float rotXYZW[4]);

    // ---- misc --------------------------------------------------------------
    static void version(int &major, int &minor, int &revision);

private:
    // The shape builders behind both APIs (handle + the body's settings + whether this add
    // recomputes the body's mass).
    bool boxShape(BodyHandle h, const BodySettings &b, const float halfExtents[3], const float center[3],
                  const float rotationXYZW[4], bool updateMass);
    bool sphereShape(BodyHandle h, const BodySettings &b, float radius, const float center[3], bool updateMass);
    bool capsuleShape(BodyHandle h, const BodySettings &b, float radius, float halfLength,
                      const float center[3], const float rotationXYZW[4], bool updateMass);

    struct Impl;
    std::unique_ptr<Impl> myImpl;
};

} // namespace b3dop
