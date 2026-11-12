// Box3DWorld.cpp
#include "Box3DWorld.h"

#include <box3d/box3d.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace b3dop {

struct Box3DWorld::Impl
{
    b3WorldId                             worldId = b3_nullWorldId;
    WorldSettings                         settings;
    std::unordered_map<uint64_t, b3BodyId> bodies;
    // Shape material/density comes from the body settings, remembered here
    // because shapes are added after the body exists.
    std::unordered_map<uint64_t, BodySettings> bodySettings;

    // Convex hull cache, keyed by a hash of the point data. We own these
    // b3HullData (b3CreateHull allocates, b3DestroyHull frees); Box3D clones
    // them into its own world database when a shape is made.
    struct HullEntry
    {
        b3HullData *hull   = nullptr;
        float       volume = 0.0f;
        int         refs   = 0;          // shapes created from it, across all bodies
    };
    std::unordered_map<HullId, HullEntry>              hulls;
    std::unordered_map<uint64_t, std::vector<HullId>>  bodyHulls;   // hulls each body's shapes use

    void releaseHull(HullId h)
    {
        auto it = hulls.find(h);
        if (it != hulls.end() && it->second.refs > 0)
            --it->second.refs;
    }
    void destroyAllHulls()
    {
        for (auto &kv : hulls)
            if (kv.second.hull)
                b3DestroyHull(kv.second.hull);
        hulls.clear();
        bodyHulls.clear();
    }
};

namespace {

// A b3BodyId packed into a BodyHandle. b3BodyId is {int32 index1, uint16
// world0, uint16 generation}; index1 is >= 1 for a live body, so a packed
// valid id is never 0.
BodyHandle
packBody(b3BodyId id)
{
    return (uint64_t(uint32_t(id.index1)) << 32) | (uint64_t(id.world0) << 16) | uint64_t(id.generation);
}

b3BodyId
unpackBody(BodyHandle h)
{
    b3BodyId id;
    id.index1     = int32_t(uint32_t(h >> 32));
    id.world0     = uint16_t((h >> 16) & 0xffffu);
    id.generation = uint16_t(h & 0xffffu);
    return id;
}

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
makeVec3(const float *v)       // null -> origin
{
    b3Vec3 r;
    r.x = v ? v[0] : 0.0f;
    r.y = v ? v[1] : 0.0f;
    r.z = v ? v[2] : 0.0f;
    return r;
}

// FNV-1a over the raw bytes of the points, with the count mixed in. Never 0.
HullId
hashPoints(const float *xyz, int count)
{
    uint64_t h = 1469598103934665603ull ^ uint64_t(uint32_t(count));
    const unsigned char *bytes = reinterpret_cast<const unsigned char *>(xyz);
    const size_t n = size_t(count) * 3 * sizeof(float);
    for (size_t i = 0; i < n; ++i)
    {
        h ^= bytes[i];
        h *= 1099511628211ull;
    }
    return h ? h : 1ull;
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
    // Box3D spins up its own thread pool when workerCount > 1 and no task
    // callbacks are given (it does not use Houdini's task system). The pool is
    // sized here; to change the count, create the world again.
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
    myImpl->destroyAllHulls();      // after the world: Box3D holds its own clones
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

    auto bh = myImpl->bodyHulls.find(id);
    if (bh != myImpl->bodyHulls.end())
    {
        for (HullId h : bh->second)
            myImpl->releaseHull(h);
        myImpl->bodyHulls.erase(bh);
    }
}

bool
Box3DWorld::hasBody(uint64_t id) const
{
    return myImpl->bodies.find(id) != myImpl->bodies.end();
}

int
Box3DWorld::bodyCount() const { return static_cast<int>(myImpl->bodies.size()); }

bool
Box3DWorld::setBodyDensity(uint64_t id, float density)
{
    auto it = myImpl->bodySettings.find(id);
    if (it == myImpl->bodySettings.end())
        return false;
    it->second.density = density;
    return true;
}

bool
Box3DWorld::addBoxShape(uint64_t bodyId, const float halfExtents[3], const float center[3])
{
    auto it = myImpl->bodies.find(bodyId);
    if (it == myImpl->bodies.end())
        return false;

    // The hull and shape def are cloned by Box3D, so the temporary is fine.
    const bool offset = center && (center[0] != 0.0f || center[1] != 0.0f || center[2] != 0.0f);
    const b3BoxHull hull = offset
        ? b3MakeOffsetBoxHull(halfExtents[0], halfExtents[1], halfExtents[2], makeVec3(center))
        : b3MakeBoxHull(halfExtents[0], halfExtents[1], halfExtents[2]);
    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateHullShape(it->second, &def, &hull.base);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::addSphereShape(uint64_t bodyId, float radius, const float center[3])
{
    auto it = myImpl->bodies.find(bodyId);
    if (it == myImpl->bodies.end())
        return false;

    b3Sphere sphere;
    sphere.center = makeVec3(center);
    sphere.radius = radius;
    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateSphereShape(it->second, &def, &sphere);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::addCapsuleShape(uint64_t bodyId, float radius, float halfLength, const float center[3])
{
    auto it = myImpl->bodies.find(bodyId);
    if (it == myImpl->bodies.end())
        return false;

    const float c[3] = { center ? center[0] : 0.0f, center ? center[1] : 0.0f, center ? center[2] : 0.0f };
    b3Capsule capsule;
    capsule.center1.x = c[0];  capsule.center1.y = c[1] - halfLength;  capsule.center1.z = c[2];
    capsule.center2.x = c[0];  capsule.center2.y = c[1] + halfLength;  capsule.center2.z = c[2];
    capsule.radius    = radius;
    // Box3D turns a capsule that is too short into a sphere by itself.
    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateCapsuleShape(it->second, &def, &capsule);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::addShape(uint64_t bodyId, const ShapeSpec &shape)
{
    switch (shape.type)
    {
        case ShapeType::Sphere:   return addSphereShape(bodyId, shape.radius, shape.center);
        case ShapeType::Capsule:  return addCapsuleShape(bodyId, shape.radius, shape.halfLength, shape.center);
        default:                  return addBoxShape(bodyId, shape.halfExtents, shape.center);
    }
}

// ---------------------------------------------------------------------------
// Convex hulls
// ---------------------------------------------------------------------------

HullId
Box3DWorld::createHull(const float *xyz, int pointCount, float *outVolume)
{
    if (outVolume) *outVolume = 0.0f;
    if (!xyz || pointCount < 4)
        return 0;

    for (int i = 0; i < pointCount * 3; ++i)
        if (!std::isfinite(xyz[i]))
            return 0;

    return createHullWithKey(hashPoints(xyz, pointCount), xyz, pointCount, outVolume);
}

HullId
Box3DWorld::findHull(HullId key, float *outVolume) const
{
    if (outVolume) *outVolume = 0.0f;
    auto cached = myImpl->hulls.find(key);
    if (key == 0 || cached == myImpl->hulls.end())
        return 0;
    if (outVolume) *outVolume = cached->second.volume;
    return key;
}

HullId
Box3DWorld::createHullWithKey(HullId key, const float *xyz, int pointCount, float *outVolume)
{
    if (outVolume) *outVolume = 0.0f;
    if (key == 0 || !xyz || pointCount < 4)
        return 0;
    if (const HullId hit = findHull(key, outVolume))
        return hit;
    for (int i = 0; i < pointCount * 3; ++i)
        if (!std::isfinite(xyz[i]))
            return 0;

    std::vector<b3Vec3> pts(static_cast<size_t>(pointCount));
    for (int i = 0; i < pointCount; ++i)
    {
        pts[size_t(i)].x = xyz[3 * i];
        pts[size_t(i)].y = xyz[3 * i + 1];
        pts[size_t(i)].z = xyz[3 * i + 2];
    }

    // b3CreateHull returns NULL when the hull is degenerate (flat, collinear) or
    // when the result would exceed B3_MAX_HULL_VERTICES / FACES / EDGES. A smaller
    // vertex budget makes the builder keep fewer vertices, which is how a very
    // detailed point cloud still ends up as a valid (slightly smaller) hull. A
    // fully triangulated hull hits the 128-edge limit at about 44 vertices, hence
    // the 44 rung.
    b3HullData *hull = nullptr;
    for (int budget : { B3_MAX_HULL_VERTICES, 64, 44, 32, 16, 8 })
    {
        hull = b3CreateHull(pts.data(), pointCount, budget);
        if (hull)
            break;
    }
    if (!hull)
        return 0;

    const float volume = hull->volume;
    if (!(volume > 1e-9f) || !std::isfinite(volume))
    {
        b3DestroyHull(hull);        // flat: no mass, no stable contacts
        return 0;
    }

    Impl::HullEntry e;
    e.hull   = hull;
    e.volume = volume;
    myImpl->hulls[key] = e;
    if (outVolume) *outVolume = volume;
    return key;
}

bool
Box3DWorld::addHullShape(uint64_t bodyId, HullId hullId)
{
    auto bit = myImpl->bodies.find(bodyId);
    auto hit = myImpl->hulls.find(hullId);
    if (bit == myImpl->bodies.end() || hit == myImpl->hulls.end())
        return false;

    const b3ShapeDef def = shapeDefFor(myImpl->bodySettings[bodyId]);
    const b3ShapeId shape = b3CreateHullShape(bit->second, &def, hit->second.hull);
    if (B3_IS_NULL(shape))
        return false;

    ++hit->second.refs;
    myImpl->bodyHulls[bodyId].push_back(hullId);
    return true;
}

void
Box3DWorld::pruneHulls()
{
    for (auto it = myImpl->hulls.begin(); it != myImpl->hulls.end();)
    {
        if (it->second.refs <= 0)
        {
            if (it->second.hull)
                b3DestroyHull(it->second.hull);
            it = myImpl->hulls.erase(it);
        }
        else
            ++it;
    }
}

int
Box3DWorld::hullCacheSize() const { return static_cast<int>(myImpl->hulls.size()); }

int
Box3DWorld::shapeCount(uint64_t bodyId) const
{
    auto it = myImpl->bodies.find(bodyId);
    return it == myImpl->bodies.end() ? 0 : b3Body_GetShapeCount(it->second);
}

float
Box3DWorld::bodyMass(uint64_t bodyId) const
{
    auto it = myImpl->bodies.find(bodyId);
    return it == myImpl->bodies.end() ? 0.0f : b3Body_GetMass(it->second);
}

bool
Box3DWorld::setBodyType(uint64_t id, BodySettings::Type type)
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    b3BodyType t = b3_dynamicBody;
    if (type == BodySettings::Type::Static)         t = b3_staticBody;
    else if (type == BodySettings::Type::Kinematic) t = b3_kinematicBody;
    b3Body_SetType(it->second, t);
    myImpl->bodySettings[id].type = type;
    return true;
}

bool
Box3DWorld::getBodyType(uint64_t id, BodySettings::Type &out) const
{
    auto it = myImpl->bodySettings.find(id);
    if (it == myImpl->bodySettings.end())
        return false;
    out = it->second.type;
    return true;
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

    b3Body_SetLinearVelocity(it->second, makeVec3(linear));
    b3Body_SetAngularVelocity(it->second, makeVec3(angular));
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

BodyHandle
Box3DWorld::bodyHandle(uint64_t id) const
{
    auto it = myImpl->bodies.find(id);
    return it == myImpl->bodies.end() ? 0 : packBody(it->second);
}

bool
Box3DWorld::getTransformByHandle(BodyHandle h, double outPos[3], float outRotXYZW[4]) const
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    const b3Pos    p  = b3Body_GetPosition(id);
    const b3Quat   q  = b3Body_GetRotation(id);
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
Box3DWorld::getVelocityByHandle(BodyHandle h, float outLinear[3], float outAngular[3]) const
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    const b3Vec3   v  = b3Body_GetLinearVelocity(id);
    const b3Vec3   w  = b3Body_GetAngularVelocity(id);
    outLinear[0]  = v.x;  outLinear[1]  = v.y;  outLinear[2]  = v.z;
    outAngular[0] = w.x;  outAngular[1] = w.y;  outAngular[2] = w.z;
    return true;
}

bool
Box3DWorld::setVelocityByHandle(BodyHandle h, const float linear[3], const float angular[3])
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    b3Body_SetLinearVelocity(id, makeVec3(linear));
    b3Body_SetAngularVelocity(id, makeVec3(angular));
    return true;
}

bool
Box3DWorld::setTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4])
{
    if (!h)
        return false;
    b3Body_SetTransform(unpackBody(h), makePos(pos), makeQuat(rotXYZW));
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
