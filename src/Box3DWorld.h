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
//   * ids        : caller-supplied uint64_t. For DOPs this will be the
//                  SIM_Object id (or an index into a packed-primitive set).
#pragma once

#include <cstdint>
#include <memory>

namespace b3dop {

struct WorldSettings
{
    float gravity[3]    = { 0.0f, -9.80665f, 0.0f };
    int   subSteps      = 4;      // Box3D recommends ~4 per step
    int   workerCount   = 1;      // keep 1 until threading is hooked up to Houdini
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

    // ---- shapes --------------------------------------------------------
    bool addBoxShape(uint64_t bodyId, const float halfExtents[3]);
    bool addSphereShape(uint64_t bodyId, float radius);
    // bool addHullShape(uint64_t bodyId, const float *points, int pointCount);
    // bool addMeshShape(...);   // static colliders only, probably

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
