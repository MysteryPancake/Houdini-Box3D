// Box3DWorld.cpp
#include "Box3DWorld.h"

#include <box3d/box3d.h>

#include <algorithm>
#include <unordered_map>

namespace b3dop {

struct Box3DWorld::Impl
{
    b3WorldId                             worldId = b3_nullWorldId;
    WorldSettings                         settings;
    std::unordered_map<uint64_t, b3BodyId> bodies;
    // Shape material/density comes from the body settings, remembered here
    // because shapes are added after the body exists.
    std::unordered_map<uint64_t, BodySettings> bodySettings;
};

namespace {

// b3Pos is float x/y/z normally and double x/y/z when Box3D is built with
// BOX3D_DOUBLE_PRECISION. These helpers keep the double[3] boundary working in
// both modes (this file is the only one that sees Box3D headers).
b3Pos
makePos(const double p[3])
{
    b3Pos r;
    r.x = static_cast<decltype(r.x)>(p[0]);
    r.y = static_cast<decltype(r.y)>(p[1]);
    r.z = static_cast<decltype(r.z)>(p[2]);
    return r;
}

b3Quat
makeQuat(const float q[4])   // x, y, z, w
{
    b3Quat r;
    r.v.x = q[0];
    r.v.y = q[1];
    r.v.z = q[2];
    r.s   = q[3];
    return r;
}

b3Vec3
makeVec(const float v[3])
{
    b3Vec3 r;
    r.x = v[0];
    r.y = v[1];
    r.z = v[2];
    return r;
}

b3ShapeDef
shapeDefFor(const BodySettings &b)
{
    b3ShapeDef def = b3DefaultShapeDef();
    def.density                 = b.density;
    def.baseMaterial.friction    = b.friction;
    def.baseMaterial.restitution = b.restitution;
    return def;
}

} // namespace

Box3DWorld::Box3DWorld() : myImpl(new Impl) {}

Box3DWorld::~Box3DWorld() { destroy(); }

bool
Box3DWorld::create(const WorldSettings &s)
{
    destroy();

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity.x = s.gravity[0];
    def.gravity.y = s.gravity[1];
    def.gravity.z = s.gravity[2];
    def.enableSleep      = s.enableSleep;
    def.enableContinuous = s.enableContinuous;
    // Box3D spins up its own threads when workerCount > 1 and no task
    // callbacks are given. Leave at 1 until enqueueTask/finishTask are routed
    // to Houdini's task system.
    def.workerCount = static_cast<uint32_t>(std::max(1, s.workerCount));

    myImpl->settings = s;
    myImpl->worldId  = b3CreateWorld(&def);
    return b3World_IsValid(myImpl->worldId);
}

void
Box3DWorld::destroy()
{
    if (b3World_IsValid(myImpl->worldId))
        b3DestroyWorld(myImpl->worldId);   // destroys all bodies/shapes/joints

    myImpl->worldId = b3_nullWorldId;
    myImpl->bodies.clear();
    myImpl->bodySettings.clear();
}

bool
Box3DWorld::isValid() const { return b3World_IsValid(myImpl->worldId); }

const WorldSettings &
Box3DWorld::settings() const { return myImpl->settings; }

// ---------------------------------------------------------------------------
// Stepping, bodies, shapes
// ---------------------------------------------------------------------------

void
Box3DWorld::step(float timeStep)
{
    if (!isValid())
        return;
    b3World_Step(myImpl->worldId, timeStep, myImpl->settings.subSteps);
}

bool
Box3DWorld::addBody(uint64_t id, const BodySettings &body,
                    const double position[3], const float rotationXYZW[4])
{
    if (!isValid() || hasBody(id))
        return false;

    b3BodyDef def = b3DefaultBodyDef();
    switch (body.type)
    {
        case BodySettings::Type::Static:    def.type = b3_staticBody;    break;
        case BodySettings::Type::Kinematic: def.type = b3_kinematicBody; break;
        default:                            def.type = b3_dynamicBody;   break;
    }
    def.position       = makePos(position);
    def.rotation       = makeQuat(rotationXYZW);
    def.linearDamping  = body.linearDamping;
    def.angularDamping = body.angularDamping;
    def.gravityScale   = body.gravityScale;
    def.enableSleep    = body.enableSleep && myImpl->settings.enableSleep;
    def.isBullet       = body.isBullet;

    const b3BodyId bodyId = b3CreateBody(myImpl->worldId, &def);
    if (B3_IS_NULL(bodyId))
        return false;

    myImpl->bodies[id]       = bodyId;
    myImpl->bodySettings[id] = body;
    return true;
}

void
Box3DWorld::removeBody(uint64_t id)
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return;
    if (b3Body_IsValid(it->second))
        b3DestroyBody(it->second);
    myImpl->bodies.erase(it);
    myImpl->bodySettings.erase(id);
}

bool
Box3DWorld::hasBody(uint64_t id) const
{
    return myImpl->bodies.find(id) != myImpl->bodies.end();
}

int
Box3DWorld::bodyCount() const { return static_cast<int>(myImpl->bodies.size()); }

bool
Box3DWorld::addBoxShape(uint64_t bodyId, const float halfExtents[3])
{
    auto it = myImpl->bodies.find(bodyId);
    if (it == myImpl->bodies.end())
        return false;

    // The hull and shape def are cloned by Box3D, so the temporary is fine.
    const b3BoxHull hull = b3MakeBoxHull(halfExtents[0], halfExtents[1], halfExtents[2]);
    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateHullShape(it->second, &def, &hull.base);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::addSphereShape(uint64_t bodyId, float radius)
{
    auto it = myImpl->bodies.find(bodyId);
    if (it == myImpl->bodies.end())
        return false;

    b3Sphere sphere;
    sphere.center.x = sphere.center.y = sphere.center.z = 0.0f;
    sphere.radius = radius;
    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateSphereShape(it->second, &def, &sphere);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    const b3Pos  p = b3Body_GetPosition(it->second);     // body origin
    const b3Quat q = b3Body_GetRotation(it->second);
    outPos[0] = static_cast<double>(p.x);
    outPos[1] = static_cast<double>(p.y);
    outPos[2] = static_cast<double>(p.z);
    outRotXYZW[0] = q.v.x;
    outRotXYZW[1] = q.v.y;
    outRotXYZW[2] = q.v.z;
    outRotXYZW[3] = q.s;
    return true;
}

bool
Box3DWorld::getVelocity(uint64_t id, float outLinear[3], float outAngular[3]) const
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    const b3Vec3 v = b3Body_GetLinearVelocity(it->second);
    const b3Vec3 w = b3Body_GetAngularVelocity(it->second);
    outLinear[0]  = v.x;  outLinear[1]  = v.y;  outLinear[2]  = v.z;
    outAngular[0] = w.x;  outAngular[1] = w.y;  outAngular[2] = w.z;
    return true;
}

bool
Box3DWorld::setVelocity(uint64_t id, const float linear[3], const float angular[3])
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    b3Body_SetLinearVelocity(it->second, makeVec(linear));
    b3Body_SetAngularVelocity(it->second, makeVec(angular));
    return true;
}

bool
Box3DWorld::setTransform(uint64_t id, const double pos[3], const float rotXYZW[4])
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    b3Body_SetTransform(it->second, makePos(pos), makeQuat(rotXYZW));
    return true;
}

void
Box3DWorld::version(int &major, int &minor, int &revision)
{
    const b3Version v = b3GetVersion();
    major    = v.major;
    minor    = v.minor;
    revision = v.revision;
}

} // namespace b3dop
