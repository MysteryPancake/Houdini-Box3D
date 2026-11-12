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
    int                                   liveBodies = 0;     // every live body, keyed or handle-made

    // Keyed API only (addBody / addShape / removeBody ...); the solver's handle path never
    // touches these. Shape material/density comes from the body settings, remembered here
    // because the keyed shape calls do not take them.
    std::unordered_map<uint64_t, BodyHandle>   bodies;
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
    // Hulls each body's shapes use, keyed by body handle. Only bodies with hull shapes
    // appear (a plain box / sphere / capsule body adds nothing here).
    std::unordered_map<BodyHandle, std::vector<HullId>> bodyHulls;
    size_t                                             bodyHullsReserve = 0;   // applied lazily, see noteHullUse

    void releaseHull(HullId h)
    {
        auto it = hulls.find(h);
        if (it != hulls.end() && it->second.refs > 0)
            --it->second.refs;
    }
    void noteHullUse(BodyHandle body, HullId hull)
    {
        if (bodyHullsReserve)           // first hull body of a reserve() hint: size the map now,
        {                               // so a scene without hulls never pays for the buckets
            bodyHulls.reserve(bodyHullsReserve);
            bodyHullsReserve = 0;
        }
        bodyHulls[body].push_back(hull);
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

// Null, or a rotation within float noise of identity (qx, qy, qz ~ 0, w ~ +-1).
bool
isIdentityRotation(const float q[4])
{
    return !q || (std::fabs(q[0]) < 1e-6f && std::fabs(q[1]) < 1e-6f && std::fabs(q[2]) < 1e-6f);
}

b3ShapeDef
shapeDefFor(const BodySettings &b, bool updateMass)
{
    b3ShapeDef def = b3DefaultShapeDef();
    def.updateBodyMass          = updateMass;     // false: the caller applies the mass once at the end
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
    myImpl->liveBodies = 0;
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

void
Box3DWorld::reserve(size_t bodyCount)
{
    // Only the hull bookkeeping is sized, and lazily (see Impl::noteHullUse). The keyed maps
    // belong to the convenience API; the handle path never fills them.
    myImpl->bodyHullsReserve = bodyCount;
}

BodyHandle
Box3DWorld::createBody(const BodySettings &body, const double position[3], const float rotationXYZW[4],
                       void *userData)
{
    if (!isValid())
        return 0;

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
    def.userData       = userData;

    const b3BodyId bodyId = b3CreateBody(myImpl->worldId, &def);
    if (B3_IS_NULL(bodyId))
        return 0;
    ++myImpl->liveBodies;
    return packBody(bodyId);
}

void
Box3DWorld::destroyBody(BodyHandle h)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    if (b3Body_IsValid(id))
    {
        b3DestroyBody(id);
        --myImpl->liveBodies;
    }

    auto bh = myImpl->bodyHulls.find(h);
    if (bh != myImpl->bodyHulls.end())
    {
        for (HullId hull : bh->second)
            myImpl->releaseHull(hull);
        myImpl->bodyHulls.erase(bh);
    }
}

bool
Box3DWorld::createShape(BodyHandle h, const BodySettings &body, const ShapeSpec &shape, bool updateMass)
{
    switch (shape.type)
    {
        case ShapeType::Sphere:   return sphereShape(h, body, shape.radius, shape.center, updateMass);
        case ShapeType::Capsule:  return capsuleShape(h, body, shape.radius, shape.halfLength, shape.center,
                                                      shape.rot, updateMass);
        default:                  return boxShape(h, body, shape.halfExtents, shape.center, shape.rot, updateMass);
    }
}

bool
Box3DWorld::createHullShape(BodyHandle h, const BodySettings &body, HullId hullId, bool updateMass)
{
    auto hit = myImpl->hulls.find(hullId);
    if (!h || hit == myImpl->hulls.end())
        return false;

    const b3ShapeDef def = shapeDefFor(body, updateMass);
    const b3ShapeId shape = b3CreateHullShape(unpackBody(h), &def, hit->second.hull);
    if (B3_IS_NULL(shape))
        return false;

    ++hit->second.refs;
    myImpl->noteHullUse(h, hullId);
    return true;
}

void
Box3DWorld::applyMassFromShapes(BodyHandle h)
{
    if (h)
        b3Body_ApplyMassFromShapes(unpackBody(h));
}

bool
Box3DWorld::boxShape(BodyHandle h, const BodySettings &b, const float halfExtents[3], const float center[3],
                     const float rotationXYZW[4], bool updateMass)
{
    if (!h)
        return false;

    // A rotated box is the hull of its 8 rotated corners (Box3D has no rotated
    // box helper). The hull cache keys it by content, so identical boxes share one.
    if (!isIdentityRotation(rotationXYZW))
    {
        float pts[24];
        int   n = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    const float corner[3] = { sx * halfExtents[0], sy * halfExtents[1], sz * halfExtents[2] };
                    float r[3];
                    quatRotate(rotationXYZW, corner, r);
                    for (int j = 0; j < 3; ++j)
                        pts[n++] = r[j] + (center ? center[j] : 0.0f);
                }
        const HullId hull = createHull(pts, 8, nullptr);
        return hull && createHullShape(h, b, hull, updateMass);
    }

    // The hull and shape def are cloned by Box3D, so the temporary is fine.
    const bool offset = center && (center[0] != 0.0f || center[1] != 0.0f || center[2] != 0.0f);
    const b3BoxHull hull = offset
        ? b3MakeOffsetBoxHull(halfExtents[0], halfExtents[1], halfExtents[2], makeVec3(center))
        : b3MakeBoxHull(halfExtents[0], halfExtents[1], halfExtents[2]);
    const b3ShapeDef def = shapeDefFor(b, updateMass);
    const b3ShapeId shape = b3CreateHullShape(unpackBody(h), &def, &hull.base);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::sphereShape(BodyHandle h, const BodySettings &b, float radius, const float center[3], bool updateMass)
{
    if (!h)
        return false;

    b3Sphere sphere;
    sphere.center = makeVec3(center);
    sphere.radius = radius;
    const b3ShapeDef def = shapeDefFor(b, updateMass);
    const b3ShapeId shape = b3CreateSphereShape(unpackBody(h), &def, &sphere);
    return !B3_IS_NULL(shape);
}

bool
Box3DWorld::capsuleShape(BodyHandle h, const BodySettings &b, float radius, float halfLength,
                         const float center[3], const float rotationXYZW[4], bool updateMass)
{
    if (!h)
        return false;

    const float c[3] = { center ? center[0] : 0.0f, center ? center[1] : 0.0f, center ? center[2] : 0.0f };
    // The axis is the shape's Y, turned by the shape rotation.
    float axis[3] = { 0.0f, 1.0f, 0.0f };
    if (!isIdentityRotation(rotationXYZW))
    {
        const float up[3] = { 0.0f, 1.0f, 0.0f };
        quatRotate(rotationXYZW, up, axis);
    }
    b3Capsule capsule;
    capsule.center1.x = c[0] - halfLength * axis[0];
    capsule.center1.y = c[1] - halfLength * axis[1];
    capsule.center1.z = c[2] - halfLength * axis[2];
    capsule.center2.x = c[0] + halfLength * axis[0];
    capsule.center2.y = c[1] + halfLength * axis[1];
    capsule.center2.z = c[2] + halfLength * axis[2];
    capsule.radius    = radius;
    // Box3D turns a capsule that is too short into a sphere by itself.
    const b3ShapeDef def = shapeDefFor(b, updateMass);
    const b3ShapeId shape = b3CreateCapsuleShape(unpackBody(h), &def, &capsule);
    return !B3_IS_NULL(shape);
}

// ---- keyed API: a convenience layer over the handle API ----------------------

bool
Box3DWorld::addBody(uint64_t id, const BodySettings &body, const double position[3], const float rotationXYZW[4])
{
    if (!isValid() || hasBody(id))
        return false;
    const BodyHandle h = createBody(body, position, rotationXYZW);
    if (!h)
        return false;
    myImpl->bodies[id]       = h;
    myImpl->bodySettings[id] = body;
    return true;
}

void
Box3DWorld::removeBody(uint64_t id)
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return;
    destroyBody(it->second);
    myImpl->bodies.erase(it);
    myImpl->bodySettings.erase(id);
}

bool
Box3DWorld::hasBody(uint64_t id) const
{
    return myImpl->bodies.find(id) != myImpl->bodies.end();
}

int
Box3DWorld::bodyCount() const { return myImpl->liveBodies; }

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
Box3DWorld::addBoxShape(uint64_t bodyId, const float halfExtents[3], const float center[3],
                        const float rotationXYZW[4])
{
    auto it = myImpl->bodies.find(bodyId);
    return it != myImpl->bodies.end() &&
           boxShape(it->second, myImpl->bodySettings[bodyId], halfExtents, center, rotationXYZW, true);
}

bool
Box3DWorld::addSphereShape(uint64_t bodyId, float radius, const float center[3])
{
    auto it = myImpl->bodies.find(bodyId);
    return it != myImpl->bodies.end() &&
           sphereShape(it->second, myImpl->bodySettings[bodyId], radius, center, true);
}

bool
Box3DWorld::addCapsuleShape(uint64_t bodyId, float radius, float halfLength, const float center[3],
                            const float rotationXYZW[4])
{
    auto it = myImpl->bodies.find(bodyId);
    return it != myImpl->bodies.end() &&
           capsuleShape(it->second, myImpl->bodySettings[bodyId], radius, halfLength, center, rotationXYZW, true);
}

bool
Box3DWorld::addShape(uint64_t bodyId, const ShapeSpec &shape)
{
    auto it = myImpl->bodies.find(bodyId);
    return it != myImpl->bodies.end() &&
           createShape(it->second, myImpl->bodySettings[bodyId], shape, true);
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
    return bit != myImpl->bodies.end() &&
           createHullShape(bit->second, myImpl->bodySettings[bodyId], hullId, true);
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
    return it == myImpl->bodies.end() ? 0 : b3Body_GetShapeCount(unpackBody(it->second));
}

float
Box3DWorld::bodyMass(uint64_t bodyId) const
{
    auto it = myImpl->bodies.find(bodyId);
    return it == myImpl->bodies.end() ? 0.0f : b3Body_GetMass(unpackBody(it->second));
}

bool
Box3DWorld::setTypeByHandle(BodyHandle h, BodySettings::Type type)
{
    if (!h)
        return false;
    b3BodyType t = b3_dynamicBody;
    if (type == BodySettings::Type::Static)         t = b3_staticBody;
    else if (type == BodySettings::Type::Kinematic) t = b3_kinematicBody;
    b3Body_SetType(unpackBody(h), t);
    return true;
}

// ---- mid-simulation changes ---------------------------------------------------

namespace {

// Every shape of a body, in a scratch vector the caller keeps (no allocation once warm).
int
bodyShapes(b3BodyId id, std::vector<b3ShapeId> &out)
{
    const int n = b3Body_GetShapeCount(id);
    out.resize(size_t(n > 0 ? n : 0));
    return n > 0 ? b3Body_GetShapes(id, out.data(), n) : 0;
}

thread_local std::vector<b3ShapeId> tShapes;

} // namespace

void
Box3DWorld::setMaterialByHandle(BodyHandle h, float friction, float restitution)
{
    if (!h)
        return;
    const int n = bodyShapes(unpackBody(h), tShapes);
    for (int i = 0; i < n; ++i)
    {
        if (b3Shape_GetType(tShapes[size_t(i)]) == b3_compoundShape)
            continue;                   // never made by this wrapper; Box3D asserts on it
        b3Shape_SetFriction(tShapes[size_t(i)], friction);
        b3Shape_SetRestitution(tShapes[size_t(i)], restitution);
    }
}

void
Box3DWorld::setDampingByHandle(BodyHandle h, float linear, float angular)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    b3Body_SetLinearDamping(id, linear);
    b3Body_SetAngularDamping(id, angular);
}

void
Box3DWorld::setGravityScaleByHandle(BodyHandle h, float gravityScale)
{
    if (h)
        b3Body_SetGravityScale(unpackBody(h), gravityScale);
}

void
Box3DWorld::setSleepEnabledByHandle(BodyHandle h, bool enableSleep)
{
    if (h)
        b3Body_EnableSleep(unpackBody(h), enableSleep && myImpl->settings.enableSleep);
}

void
Box3DWorld::setDensityByHandle(BodyHandle h, float density)
{
    if (!h || !(density >= 0.0f) || !std::isfinite(density))
        return;
    const b3BodyId id = unpackBody(h);
    const int n = bodyShapes(id, tShapes);
    for (int i = 0; i < n; ++i)
        b3Shape_SetDensity(tShapes[size_t(i)], density, false);
    if (n > 0)
        b3Body_ApplyMassFromShapes(id);
}

void
Box3DWorld::setEnabledByHandle(BodyHandle h, bool enabled)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    if (b3Body_IsEnabled(id) == enabled)
        return;
    if (enabled)
        b3Body_Enable(id);
    else
        b3Body_Disable(id);
}

bool
Box3DWorld::isEnabledByHandle(BodyHandle h) const
{
    return h && b3Body_IsEnabled(unpackBody(h));
}

void
Box3DWorld::destroyShapesByHandle(BodyHandle h)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    const int n = bodyShapes(id, tShapes);
    for (int i = 0; i < n; ++i)
        b3DestroyShape(tShapes[size_t(i)], false);      // the caller applies the mass after re-adding

    auto bh = myImpl->bodyHulls.find(h);
    if (bh != myImpl->bodyHulls.end())
    {
        for (HullId hull : bh->second)
            myImpl->releaseHull(hull);
        myImpl->bodyHulls.erase(bh);
    }
}

void
Box3DWorld::wakeByHandle(BodyHandle h)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    if (b3Body_GetType(id) != b3_staticBody)
        b3Body_SetAwake(id, true);
}

bool
Box3DWorld::getMaterialByHandle(BodyHandle h, float &friction, float &restitution) const
{
    friction = restitution = 0.0f;
    if (!h)
        return false;
    if (bodyShapes(unpackBody(h), tShapes) <= 0)
        return false;
    friction    = b3Shape_GetFriction(tShapes[0]);
    restitution = b3Shape_GetRestitution(tShapes[0]);
    return true;
}

bool
Box3DWorld::getDampingByHandle(BodyHandle h, float &linear, float &angular) const
{
    linear = angular = 0.0f;
    if (!h)
        return false;
    linear  = b3Body_GetLinearDamping(unpackBody(h));
    angular = b3Body_GetAngularDamping(unpackBody(h));
    return true;
}

float
Box3DWorld::gravityScaleByHandle(BodyHandle h) const
{
    return h ? b3Body_GetGravityScale(unpackBody(h)) : 0.0f;
}

bool
Box3DWorld::isSleepEnabledByHandle(BodyHandle h) const
{
    return h && b3Body_IsSleepEnabled(unpackBody(h));
}

bool
Box3DWorld::isAwakeByHandle(BodyHandle h) const
{
    return h && b3Body_IsAwake(unpackBody(h));
}

float
Box3DWorld::massByHandle(BodyHandle h) const
{
    return h ? b3Body_GetMass(unpackBody(h)) : 0.0f;
}

int
Box3DWorld::shapeCountByHandle(BodyHandle h) const
{
    return h ? b3Body_GetShapeCount(unpackBody(h)) : 0;
}

bool
Box3DWorld::setBodyType(uint64_t id, BodySettings::Type type)
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end() || !setTypeByHandle(it->second, type))
        return false;
    auto bs = myImpl->bodySettings.find(id);     // gone after releaseBuildState
    if (bs != myImpl->bodySettings.end())
        bs->second.type = type;
    return true;
}

bool
Box3DWorld::getBodyType(uint64_t id, BodySettings::Type &out) const
{
    auto it = myImpl->bodySettings.find(id);
    if (it != myImpl->bodySettings.end())
    {
        out = it->second.type;
        return true;
    }
    auto b = myImpl->bodies.find(id);           // settings released: ask Box3D
    if (b == myImpl->bodies.end())
        return false;
    switch (b3Body_GetType(unpackBody(b->second)))
    {
        case b3_staticBody:    out = BodySettings::Type::Static;    break;
        case b3_kinematicBody: out = BodySettings::Type::Kinematic; break;
        default:               out = BodySettings::Type::Dynamic;   break;
    }
    return true;
}

void
Box3DWorld::releaseBuildState(uint64_t id)
{
    myImpl->bodySettings.erase(id);
}

bool
Box3DWorld::getTransform(uint64_t id, double outPos[3], float outRotXYZW[4]) const
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    const b3Pos  p = b3Body_GetPosition(unpackBody(it->second));     // body origin
    const b3Quat q = b3Body_GetRotation(unpackBody(it->second));
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

    const b3Vec3 v = b3Body_GetLinearVelocity(unpackBody(it->second));
    const b3Vec3 w = b3Body_GetAngularVelocity(unpackBody(it->second));
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

    b3Body_SetLinearVelocity(unpackBody(it->second), makeVec3(linear));
    b3Body_SetAngularVelocity(unpackBody(it->second), makeVec3(angular));
    return true;
}

bool
Box3DWorld::setTransform(uint64_t id, const double pos[3], const float rotXYZW[4])
{
    auto it = myImpl->bodies.find(id);
    if (it == myImpl->bodies.end())
        return false;

    b3Body_SetTransform(unpackBody(it->second), makePos(pos), makeQuat(rotXYZW));
    return true;
}

BodyHandle
Box3DWorld::bodyHandle(uint64_t id) const
{
    auto it = myImpl->bodies.find(id);
    return it == myImpl->bodies.end() ? 0 : it->second;
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

int
Box3DWorld::moveEventCount() const
{
    if (!isValid())
        return 0;
    return b3World_GetBodyEvents(myImpl->worldId).moveCount;
}

bool
Box3DWorld::getMoveEvent(int index, BodyMove &out) const
{
    if (!isValid())
        return false;
    const b3BodyEvents ev = b3World_GetBodyEvents(myImpl->worldId);
    if (index < 0 || index >= ev.moveCount)
        return false;
    const b3BodyMoveEvent &e = ev.moveEvents[index];
    out.userData = e.userData;
    out.handle   = packBody(e.bodyId);
    out.pos[0] = static_cast<double>(e.transform.p.x);
    out.pos[1] = static_cast<double>(e.transform.p.y);
    out.pos[2] = static_cast<double>(e.transform.p.z);
    out.rot[0] = e.transform.q.v.x;
    out.rot[1] = e.transform.q.v.y;
    out.rot[2] = e.transform.q.v.z;
    out.rot[3] = e.transform.q.s;
    out.fellAsleep = e.fellAsleep;
    return true;
}

int
Box3DWorld::getMoveEvents(std::vector<BodyMove> &out) const
{
    if (!isValid())
    {
        out.clear();
        return 0;
    }
    const b3BodyEvents ev = b3World_GetBodyEvents(myImpl->worldId);
    // Every element is overwritten below, so only a size change needs the vector to
    // construct anything (clear() + resize() would zero the whole array every step).
    if (out.size() != size_t(ev.moveCount))
        out.resize(size_t(ev.moveCount));
    for (int i = 0; i < ev.moveCount; ++i)
    {
        const b3BodyMoveEvent &e = ev.moveEvents[i];
        BodyMove &o = out[size_t(i)];
        o.userData = e.userData;
        o.handle   = packBody(e.bodyId);
        o.pos[0] = static_cast<double>(e.transform.p.x);
        o.pos[1] = static_cast<double>(e.transform.p.y);
        o.pos[2] = static_cast<double>(e.transform.p.z);
        o.rot[0] = e.transform.q.v.x;
        o.rot[1] = e.transform.q.v.y;
        o.rot[2] = e.transform.q.v.z;
        o.rot[3] = e.transform.q.s;
        o.fellAsleep = e.fellAsleep;
    }
    return ev.moveCount;
}

void
Box3DWorld::setBodyUserData(BodyHandle h, void *userData)
{
    if (h)
        b3Body_SetUserData(unpackBody(h), userData);
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
