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

#include "Box3DShapes.h"

namespace b3dop {

// Handle to a convex hull owned by the world's hull cache. 0 is invalid.
using HullId = uint64_t;

// Opaque handle to a body, valid until that body is removed or the world is
// destroyed. Looking a body up by key costs a hash lookup; a handle does not,
// so callers that touch a body every step fetch it once (bodyHandle) and keep it.
// 0 is invalid.
using BodyHandle = uint64_t;

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
    float density        = 1.0f;
    float friction       = 0.6f;
    float restitution    = 0.0f;
    float linearDamping  = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale   = 1.0f;
    bool  enableSleep    = true;
    bool  isBullet       = false;

    // Used when a point has no (or an unknown) `shape` attribute.
    ShapeType defaultShape = ShapeType::Box;
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

    // ---- bodies --------------------------------------------------------
    bool addBody(uint64_t id, const BodySettings &body,
                 const double position[3], const float rotationXYZW[4]);
    void removeBody(uint64_t id);
    bool hasBody(uint64_t id) const;
    int  bodyCount() const;

    // Change an existing body between static / kinematic / dynamic.
    bool setBodyType(uint64_t id, BodySettings::Type type);
    bool getBodyType(uint64_t id, BodySettings::Type &out) const;

    // Density used by shapes added to this body AFTER the call. The solver sets
    // it once it knows the total shape volume (mass attribute -> density).
    bool setBodyDensity(uint64_t id, float density);

    // ---- shapes --------------------------------------------------------
    // `center` is the shape's offset from the body origin (default: centred).
    bool addBoxShape(uint64_t bodyId, const float halfExtents[3], const float center[3] = nullptr);
    bool addSphereShape(uint64_t bodyId, float radius, const float center[3] = nullptr);
    bool addShape(uint64_t bodyId, const ShapeSpec &shape);   // box or sphere, incl. shape.center

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

    // ---- read back / push in -------------------------------------------
    bool getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const;
    bool getVelocity(uint64_t id, float outLinear[3], float outAngular[3]) const;
    bool setVelocity(uint64_t id, const float linear[3], const float angular[3]);
    bool setTransform(uint64_t id, const double pos[3], const float rotXYZW[4]);

    // ---- misc --------------------------------------------------------------
    static void version(int &major, int &minor, int &revision);

private:
    struct Impl;
    std::unique_ptr<Impl> myImpl;
};

} // namespace b3dop
