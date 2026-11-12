// Box3DWorld.cpp
#include "Box3DWorld.h"
#include "Box3DDebug.h"

#include <box3d/box3d.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace b3dop {

struct Box3DWorld::Impl
{
    b3WorldId                             worldId = b3_nullWorldId;
    WorldSettings                         settings;
    std::shared_ptr<TaskRunner>           tasks;              // host threads (null: Box3D's own pool); outlives worldId
    CollisionFilterTable                  filters;
    int                                   liveBodies = 0;     // every live body
    std::set<std::pair<BodyHandle, BodyHandle>> hitPairs;    // updateHitEvents()

    // Hull cache. We own the b3HullData; Box3D clones it when a shape is made.
    struct HullEntry
    {
        b3HullData *hull   = nullptr;
        float       volume = 0.0f;
        int         refs   = 0;          // shapes created from it, across all bodies
    };
    std::unordered_map<HullId, HullEntry>              hulls;
    // Hulls each body uses (only bodies with hull shapes appear).
    std::unordered_map<BodyHandle, std::vector<HullId>> bodyHulls;

    // Triangle meshes for static colliders. Box3D keeps a pointer to the b3MeshData (it does not
    // clone it), so we own it: freed after its body is destroyed, or after the world.
    std::unordered_map<BodyHandle, std::vector<b3MeshData *>> bodyMeshes;

    // What the world keeps for a body's shapes (its meshes, its hull references). Call after the
    // body or its shapes are destroyed.
    void releaseBodyResources(BodyHandle body)
    {
        auto mit = bodyMeshes.find(body);
        if (mit != bodyMeshes.end())
        {
            for (b3MeshData *m : mit->second)
                b3DestroyMesh(m);
            bodyMeshes.erase(mit);
        }
        auto hit = bodyHulls.find(body);
        if (hit != bodyHulls.end())
        {
            for (HullId h : hit->second)
            {
                auto it = hulls.find(h);
                if (it != hulls.end() && it->second.refs > 0)
                    --it->second.refs;
            }
            bodyHulls.erase(hit);
        }
    }
    void destroyAllMeshes()
    {
        for (auto &kv : bodyMeshes)
            for (b3MeshData *m : kv.second)
                b3DestroyMesh(m);
        bodyMeshes.clear();
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

// ---- Box3D assertion and log handlers ---------------------------------------
// Box3D's default assert handler breaks to the debugger, which kills the host (Houdini) when none
// is attached, and its default log handler printf()s. Asserts are compiled in only with NDEBUG
// unset or B3_ENABLE_ASSERT (RelWithDebInfo). Ours count the assert and carry on (no break), and
// both print only in BOX3DDOP_DEBUG builds (stderr, Houdini's console; see Box3DDebug.h), so a
// normal build never writes to the console, which on Windows opens a console window.
std::atomic<int> gAssertCount{0};
constexpr int    kMaxAssertLogs = 20;

int
assertHandler(const char *condition, const char *fileName, int lineNumber)
{
    const int n = gAssertCount.fetch_add(1) + 1;
    if (n <= kMaxAssertLogs)
        B3D_LOG("Box3D assertion failed (ignored, simulation continues): %s, %s, line %d",
                condition ? condition : "?", fileName ? fileName : "?", lineNumber);
    if (n == kMaxAssertLogs)
        B3D_LOG("Box3D: further assertion messages are not printed");
    return 0;
}

void
logHandler(const char *message)
{
    B3D_LOG("Box3D: %s", message ? message : "?");
}

void
installAssertHandler()
{
    static std::once_flag once;
    std::call_once(once, [] {
        b3SetAssertFcn(&assertHandler);
        b3SetLogFcn(&logHandler);
    });
}

// Non-finite components become 0 (Box3D asserts on them, and a NaN velocity spreads to the
// body's position and its broad-phase bounds).
b3Vec3
finiteVec3(const float v[3])
{
    b3Vec3 out;
    out.x = std::isfinite(v[0]) ? v[0] : 0.0f;
    out.y = std::isfinite(v[1]) ? v[1] : 0.0f;
    out.z = std::isfinite(v[2]) ? v[2] : 0.0f;
    return out;
}

// b3BodyId {int32 index1, uint16 world0, uint16 generation} packed into a BodyHandle.
// index1 >= 1 for a live body, so a valid handle is never 0.
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

// b3Pos is float, or double with BOX3D_DOUBLE_PRECISION; these work either way.
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

void
storePos(const b3Pos &p, double out[3])
{
    out[0] = static_cast<double>(p.x);
    out[1] = static_cast<double>(p.y);
    out[2] = static_cast<double>(p.z);
}

void
storeQuat(const b3Quat &q, float out[4])
{
    out[0] = q.v.x;  out[1] = q.v.y;  out[2] = q.v.z;  out[3] = q.s;
}

void
storeVec3(const b3Vec3 &v, float out[3])
{
    out[0] = v.x;  out[1] = v.y;  out[2] = v.z;
}

void
toBodyMove(const b3BodyMoveEvent &e, BodyMove &out)
{
    out.userData   = e.userData;
    out.handle     = packBody(e.bodyId);
    storePos(e.transform.p, out.pos);
    storeQuat(e.transform.q, out.rot);
    out.fellAsleep = e.fellAsleep;
}

b3BodyType
toB3Type(BodySettings::Type t)
{
    switch (t)
    {
        case BodySettings::Type::Static:    return b3_staticBody;
        case BodySettings::Type::Kinematic: return b3_kinematicBody;
        default:                            return b3_dynamicBody;
    }
}

// A content key for body-local points. Never 0.
HullId
hashPoints(const float *xyz, int count)
{
    const uint64_t h = hashFloats(xyz, size_t(count) * 3);
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
    def.updateBodyMass          = updateMass;     // false: the caller applies the mass afterwards
    def.density                 = b.density;
    def.baseMaterial.friction    = b.friction;
    def.baseMaterial.restitution = b.restitution;
    // Custom filtering on for every shape (Box3D cannot turn it on later), with the body's
    // profile + 1 in the user data (0 = none).
    def.enableCustomFiltering   = true;
    def.enableHitEvents         = true;     // for the Impacts speed filter; costs nothing without a threshold
    def.userData                = reinterpret_cast<void *>(intptr_t(b.filterProfile + 1));
    return def;
}

// Mixing rules for two touching materials. The callbacks get no context, so each rule is its own function.
float frictionMin(float a, uint64_t, float b, uint64_t) { return std::min(a, b); }
float frictionMax(float a, uint64_t, float b, uint64_t) { return std::max(a, b); }
float frictionAvg(float a, uint64_t, float b, uint64_t) { return 0.5f * (a + b); }
float frictionMul(float a, uint64_t, float b, uint64_t) { return a * b; }
float restitutionMin(float a, uint64_t, float b, uint64_t) { return std::min(a, b); }
float restitutionAvg(float a, uint64_t, float b, uint64_t) { return 0.5f * (a + b); }
float restitutionMul(float a, uint64_t, float b, uint64_t) { return a * b; }
float restitutionGeo(float a, uint64_t, float b, uint64_t) { return std::sqrt(std::max(0.0f, a * b)); }

// nullptr is Box3D's own rule (geometric mean for friction, maximum for restitution).
b3FrictionCallback *
frictionRule(int mode)
{
    switch (mode)
    {
        case 1: return &frictionMin;
        case 2: return &frictionMax;
        case 3: return &frictionAvg;
        case 4: return &frictionMul;
        default: return nullptr;
    }
}

b3RestitutionCallback *
restitutionRule(int mode)
{
    switch (mode)
    {
        case 1: return &restitutionMin;
        case 2: return &restitutionAvg;
        case 3: return &restitutionMul;
        case 4: return &restitutionGeo;
        default: return nullptr;
    }
}

int
shapeProfile(b3ShapeId id)
{
    return int(reinterpret_cast<intptr_t>(b3Shape_GetUserData(id))) - 1;
}

// Box3D's custom filter. Runs during pair finding, possibly on several threads: read only.
bool
filterCallback(b3ShapeId a, b3ShapeId b, void *context)
{
    const CollisionFilterTable *t = static_cast<const CollisionFilterTable *>(context);
    return t->shouldCollide(shapeProfile(a), shapeProfile(b));
}

bool
sameBody(b3BodyId a, b3BodyId b)
{
    return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

} // namespace

namespace {

// Box3D's task hooks, forwarding to the world's TaskRunner (userContext).
void *
enqueueTask(b3TaskCallback *task, void *taskContext, void *userContext, const char * /*name*/)
{
    TaskRunner *runner = static_cast<TaskRunner *>(userContext);
    // A non-null handle means "queued"; null tells Box3D the task already ran inline.
    return runner->run(task, taskContext) ? userContext : nullptr;
}

void
finishTask(void *userTask, void *userContext)
{
    if (userTask)
        static_cast<TaskRunner *>(userContext)->wait();
}

} // namespace

Box3DWorld::Box3DWorld() : myImpl(new Impl) {}

Box3DWorld::~Box3DWorld() { destroy(); }

bool
Box3DWorld::create(const WorldSettings &s, std::shared_ptr<TaskRunner> tasks)
{
    destroy();
    installAssertHandler();

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = finiteVec3(s.gravity);       // non-finite components: 0
    def.enableSleep      = s.enableSleep;
    def.enableContinuous = s.enableContinuous;
    def.contactHertz         = std::max(0.0f, s.contactHertz);
    def.contactDampingRatio  = std::max(0.0f, s.contactDampingRatio);
    def.contactSpeed         = std::max(0.0f, s.contactPushSpeed);
    def.restitutionThreshold = std::max(0.0f, s.restitutionThreshold);
    def.maximumLinearSpeed   = std::max(0.0f, s.maxLinearSpeed);
    def.restitutionIterations = std::clamp(s.restitutionIterations, 0, B3_MAX_RESTITUTION_ITERATIONS);
    def.enableRestitutionPropagation = s.restitutionPropagation;
    def.isotropicInertiaTolerance = std::clamp(s.isotropicInertiaTolerance, 0.0f, 1.0f);
    if (s.impactMinSpeed > 0.0f)
        def.hitEventThreshold = s.impactMinSpeed;
    def.frictionCallback    = frictionRule(s.frictionMixing);
    def.restitutionCallback = restitutionRule(s.restitutionMixing);
    def.workerCount = static_cast<uint32_t>(std::max(1, s.workerCount));
    if (tasks && def.workerCount > 1)
    {
        // The host's threads: Box3D starts none. The runner must outlive the b3World (see destroy).
        def.enqueueTask     = &enqueueTask;
        def.finishTask      = &finishTask;
        def.userTaskContext = tasks.get();
    }
    else
    {
        // No runner (or one worker): Box3D's own pool for workerCount > 1, serial otherwise.
        tasks.reset();
    }

    // At the limit b3CreateWorld asserts (a crash in builds with Box3D asserts on, such as
    // RelWithDebInfo) before failing, so fail here first.
    if (b3GetWorldCount() >= B3_MAX_WORLDS)
        return false;

    myImpl->settings = s;
    myImpl->settings.gravity[0] = def.gravity.x;    // as applied (finite)
    myImpl->settings.gravity[1] = def.gravity.y;
    myImpl->settings.gravity[2] = def.gravity.z;
    myImpl->filters  = CollisionFilterTable();
    myImpl->tasks    = std::move(tasks);
    myImpl->worldId  = b3CreateWorld(&def);
    if (!b3World_IsValid(myImpl->worldId))
    {
        myImpl->tasks.reset();
        return false;
    }
    // Not part of b3WorldDef. A non-finite value falls back to the default.
    if (std::isfinite(s.contactRecycleDistance))
        b3World_SetContactRecycleDistance(myImpl->worldId, std::max(0.0f, s.contactRecycleDistance));
    else
        myImpl->settings.contactRecycleDistance = WorldSettings().contactRecycleDistance;
    b3World_SetCustomFilterCallback(myImpl->worldId, &filterCallback, &myImpl->filters);
    b3World_EnableSpeculative(myImpl->worldId, s.enableSpeculative);
    b3World_EnableWarmStarting(myImpl->worldId, s.warmStarting);
    myImpl->hitPairs.clear();
    return true;
}

void
Box3DWorld::destroy()
{
    if (b3World_IsValid(myImpl->worldId))
        b3DestroyWorld(myImpl->worldId);   // destroys all bodies/shapes/joints

    myImpl->tasks.reset();          // after the world: Box3D may call the runner until it is gone
    myImpl->worldId = b3_nullWorldId;
    myImpl->liveBodies = 0;
    myImpl->destroyAllHulls();      // after the world: Box3D holds its own clones
    myImpl->destroyAllMeshes();     // after the world: its shapes pointed at these
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
    if (myImpl->tasks)
    {
        struct Args { b3WorldId id; float dt; int subSteps; } args{ myImpl->worldId, timeStep, myImpl->settings.subSteps };
        myImpl->tasks->isolate([](void *p) {
            const Args *a = static_cast<const Args *>(p);
            b3World_Step(a->id, a->dt, a->subSteps);
        }, &args);
        return;
    }
    b3World_Step(myImpl->worldId, timeStep, myImpl->settings.subSteps);
}

BodyHandle
Box3DWorld::createBody(const BodySettings &body, const double position[3], const float rotationXYZW[4],
                       void *userData)
{
    if (!isValid())
        return 0;

    b3BodyDef def = b3DefaultBodyDef();
    def.type           = toB3Type(body.type);
    def.position       = makePos(position);
    def.rotation       = makeQuat(rotationXYZW);
    def.linearDamping  = body.linearDamping;
    def.angularDamping = body.angularDamping;
    def.gravityScale   = body.gravityScale;
    def.enableSleep    = body.enableSleep;      // Box3D applies the world switch on top
    def.isBullet       = body.isBullet;
    def.enableContactRecycling = body.contactRecycling;
    if (body.sleepThreshold >= 0.0f && std::isfinite(body.sleepThreshold))
        def.sleepThreshold = body.sleepThreshold;
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
    myImpl->releaseBodyResources(h);    // after the body: its shapes pointed at the meshes
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
    myImpl->bodyHulls[h].push_back(hullId);
    return true;
}

bool
Box3DWorld::createMeshShape(BodyHandle h, const BodySettings &body, const float *xyz, int vertexCount,
                            const int32_t *triangles, int triangleCount, float weldTolerance)
{
    if (!h || !xyz || !triangles || vertexCount < 3 || triangleCount < 1)
        return false;
    // Static bodies only (a triangle mesh has no volume or mass): refuse anything else.
    if (b3Body_GetType(unpackBody(h)) != b3_staticBody)
        return false;

    // Reject bad input here: Box3D asserts on non-finite vertices or out-of-range indices.
    std::vector<b3Vec3> verts;
    verts.resize(size_t(vertexCount));
    for (int i = 0; i < vertexCount; ++i)
    {
        if (!finite3(xyz + 3 * i))
            return false;
        verts[size_t(i)] = makeVec3(xyz + 3 * i);
    }
    std::vector<int32_t> idx;
    idx.assign(triangles, triangles + 3 * size_t(triangleCount));
    for (int32_t v : idx)
        if (v < 0 || v >= vertexCount)
            return false;

    b3MeshDef def = {};
    def.vertices      = verts.data();
    def.stride        = 0;
    def.indices       = idx.data();
    def.materialIndices = nullptr;
    def.vertexCount   = vertexCount;
    def.triangleCount = triangleCount;
    def.weldVertices  = weldTolerance > 0.0f;
    def.weldTolerance = weldTolerance > 0.0f ? weldTolerance : 0.0f;
    def.identifyEdges = true;           // smooth rolling over triangle edges (less ghost collisions)

    b3MeshData *mesh = b3CreateMesh(&def, nullptr, 0);
    if (!mesh)
        return false;

    const b3ShapeDef sdef = shapeDefFor(body, false);
    const b3ShapeId shape = b3CreateMeshShape(unpackBody(h), &sdef, mesh, b3Vec3_one);
    if (B3_IS_NULL(shape))
    {
        b3DestroyMesh(mesh);
        return false;
    }
    myImpl->bodyMeshes[h].push_back(mesh);
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

    // A rotated box is the hull of its 8 rotated corners (cached by content).
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

int
Box3DWorld::bodyCount() const { return myImpl->liveBodies; }

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

    // b3CreateHull fails for degenerate input or when the result would exceed Box3D's
    // vertex / face / edge limits; a smaller vertex budget keeps fewer vertices, so dense
    // input still gets a (slightly smaller) hull. A triangulated hull hits the 128-edge
    // limit at about 44 vertices, hence that rung.
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

bool
Box3DWorld::setTypeByHandle(BodyHandle h, BodySettings::Type type)
{
    if (!h)
        return false;
    b3Body_SetType(unpackBody(h), toB3Type(type));
    return true;
}

BodySettings::Type
Box3DWorld::typeByHandle(BodyHandle h) const
{
    switch (h ? b3Body_GetType(unpackBody(h)) : b3_dynamicBody)
    {
        case b3_staticBody:    return BodySettings::Type::Static;
        case b3_kinematicBody: return BodySettings::Type::Kinematic;
        default:               return BodySettings::Type::Dynamic;
    }
}

// ---- changes to existing bodies -------------------------------------------------

namespace {

// Every shape of a body, into a reused scratch vector.
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
            continue;                   // never made here; Box3D asserts on it
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
Box3DWorld::setContactRecyclingByHandle(BodyHandle h, bool flag)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    if (b3Body_IsContactRecyclingEnabled(id) == flag)
        return;
    b3Body_EnableContactRecycling(id, flag);
    if (b3Body_GetType(id) != b3_dynamicBody || !b3Body_IsEnabled(id) || !b3Body_IsAwake(id))
        return;
    // Existing contacts keep the flag they were made with. Disabling a body destroys its contacts
    // (they are rebuilt, with the new flag, on the next step) but also zeroes its velocity: restore it.
    const b3Vec3 linear  = b3Body_GetLinearVelocity(id);
    const b3Vec3 angular = b3Body_GetAngularVelocity(id);
    b3Body_Disable(id);
    b3Body_Enable(id);
    b3Body_SetLinearVelocity(id, linear);
    b3Body_SetAngularVelocity(id, angular);
}

void
Box3DWorld::setSleepEnabledByHandle(BodyHandle h, bool enableSleep)
{
    if (h)
        b3Body_EnableSleep(unpackBody(h), enableSleep);
}

void
Box3DWorld::setBulletByHandle(BodyHandle h, bool flag)
{
    if (h)
        b3Body_SetBullet(unpackBody(h), flag);
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
        b3DestroyShape(tShapes[size_t(i)], false);      // the caller applies the mass afterwards
    myImpl->releaseBodyResources(h);
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
Box3DWorld::getCenterOfMassByHandle(BodyHandle h, double outPos[3]) const
{
    if (!h)
        return false;
    storePos(b3Body_GetWorldCenter(unpackBody(h)), outPos);
    return true;
}

void
Box3DWorld::applyForceByHandle(BodyHandle h, const float force[3], const float torque[3], bool wake)
{
    if (!h)
        return;
    const b3BodyId id = unpackBody(h);
    if (b3Body_GetType(id) != b3_dynamicBody)
        return;
    // Box3D asserts on non-finite input and a NaN would spread to the body's pose.
    const b3Vec3 f = finiteVec3(force);
    const b3Vec3 t = finiteVec3(torque);
    if (f.x != 0.0f || f.y != 0.0f || f.z != 0.0f)
        b3Body_ApplyForceToCenter(id, f, wake);
    if (t.x != 0.0f || t.y != 0.0f || t.z != 0.0f)
        b3Body_ApplyTorque(id, t, wake);
}

void
Box3DWorld::applyMassByHandle(BodyHandle h, float density, bool comAtOrigin, float inertiaScale)
{
    if (!h || !(density >= 0.0f) || !std::isfinite(density))
        return;
    const b3BodyId id = unpackBody(h);
    const int n = bodyShapes(id, tShapes);
    for (int i = 0; i < n; ++i)
        b3Shape_SetDensity(tShapes[size_t(i)], density, false);
    b3Body_ApplyMassFromShapes(id);

    const bool scale = inertiaScale > 0.0f && std::isfinite(inertiaScale) && inertiaScale != 1.0f;
    if (b3Body_GetType(id) != b3_dynamicBody || n == 0 || (!comAtOrigin && !scale))
        return;
    b3MassData md = b3Body_GetMassData(id);
    const bool moveCenter = comAtOrigin &&
        (md.center.x != 0.0f || md.center.y != 0.0f || md.center.z != 0.0f);
    if (!moveCenter && !scale)
        return;                         // already centred
    if (moveCenter)
    {
        // Parallel axis: I_o = I_c + m (|d|^2 E - d d^T). b3Matrix3 is three columns.
        const float d[3] = { md.center.x, md.center.y, md.center.z };
        const float dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        b3Vec3 *col[3] = { &md.inertia.cx, &md.inertia.cy, &md.inertia.cz };
        for (int c = 0; c < 3; ++c)
        {
            auto shift = [&](int r) { return md.mass * ((r == c ? dd : 0.0f) - d[r] * d[c]); };
            col[c]->x += shift(0);
            col[c]->y += shift(1);
            col[c]->z += shift(2);
        }
        md.center = b3Vec3_zero;
    }
    if (scale)
    {
        md.inertia.cx = b3MulSV(inertiaScale, md.inertia.cx);
        md.inertia.cy = b3MulSV(inertiaScale, md.inertia.cy);
        md.inertia.cz = b3MulSV(inertiaScale, md.inertia.cz);
    }
    b3Body_SetMassData(id, md);
}

bool
Box3DWorld::getMassDataByHandle(BodyHandle h, float &mass, float center[3], float inertiaDiag[3]) const
{
    if (!h)
        return false;
    const b3MassData md = b3Body_GetMassData(unpackBody(h));
    mass = md.mass;
    storeVec3(md.center, center);
    inertiaDiag[0] = md.inertia.cx.x;  inertiaDiag[1] = md.inertia.cy.y;  inertiaDiag[2] = md.inertia.cz.z;
    return true;
}

void
Box3DWorld::setSleepThresholdByHandle(BodyHandle h, float threshold)
{
    if (!h)
        return;
    const float t = (threshold >= 0.0f && std::isfinite(threshold)) ? threshold : b3DefaultBodyDef().sleepThreshold;
    b3Body_SetSleepThreshold(unpackBody(h), t);
}

float
Box3DWorld::sleepThresholdByHandle(BodyHandle h) const
{
    return h ? b3Body_GetSleepThreshold(unpackBody(h)) : 0.0f;
}

void
Box3DWorld::setTargetTransformByHandle(BodyHandle h, const double pos[3], const float rotXYZW[4], float timeStep)
{
    if (!h || !(timeStep > 0.0f))
        return;
    b3WorldTransform t;
    t.p = makePos(pos);
    t.q = makeQuat(rotXYZW);
    b3Body_SetTargetTransform(unpackBody(h), t, timeStep, true);
}

int
Box3DWorld::getContactsByHandle(BodyHandle h, std::vector<ContactPoint> &out) const
{
    out.clear();
    if (!h)
        return 0;
    const b3BodyId id = unpackBody(h);
    const int cap = b3Body_GetContactCapacity(id);
    if (cap <= 0)
        return 0;
    thread_local std::vector<b3ContactData> data;
    data.resize(size_t(cap));
    const int n = b3Body_GetContactData(id, data.data(), cap);
    for (int i = 0; i < n; ++i)
    {
        const b3ContactData &cd = data[size_t(i)];
        const b3BodyId a = b3Shape_GetBody(cd.shapeIdA);
        const b3BodyId b = b3Shape_GetBody(cd.shapeIdB);
        const bool thisIsA = sameBody(a, id);
        const b3BodyId other = thisIsA ? b : a;
        const b3Pos centerA = b3Body_GetWorldCenter(a);     // anchors are relative to A's centre of mass
        for (int m = 0; m < cd.manifoldCount; ++m)
        {
            const b3Manifold &man = cd.manifolds[m];
            for (int k = 0; k < man.pointCount; ++k)
            {
                const b3ManifoldPoint &mp = man.points[k];
                if (!(mp.totalNormalImpulse > 0.0f))
                    continue;
                ContactPoint cp;
                cp.other         = packBody(other);
                cp.otherUserData = b3Body_GetUserData(other);
                cp.pos[0] = double(centerA.x) + double(mp.anchorA.x);
                cp.pos[1] = double(centerA.y) + double(mp.anchorA.y);
                cp.pos[2] = double(centerA.z) + double(mp.anchorA.z);
                const float sgn = thisIsA ? 1.0f : -1.0f;     // the manifold normal points from A to B
                cp.normal[0] = sgn * man.normal.x;
                cp.normal[1] = sgn * man.normal.y;
                cp.normal[2] = sgn * man.normal.z;
                cp.impulse = mp.totalNormalImpulse;
                out.push_back(cp);
            }
        }
    }
    return int(out.size());
}

int
Box3DWorld::filterProfile(const FilterProfile &p)
{
    return myImpl->filters.profile(p);
}

void
Box3DWorld::finishFilterProfiles()
{
    myImpl->filters.finish();
}

bool
Box3DWorld::shouldCollide(int a, int b) const
{
    return myImpl->filters.shouldCollide(a, b);
}

const FilterProfile *
Box3DWorld::filterProfileData(int profile) const
{
    return myImpl->filters.get(profile);
}

void
Box3DWorld::setFilterProfileByHandle(BodyHandle h, int profile)
{
    if (!h)
        return;
    const int n = bodyShapes(unpackBody(h), tShapes);
    for (int i = 0; i < n; ++i)
    {
        const b3ShapeId sid = tShapes[size_t(i)];
        if (shapeProfile(sid) == profile)
            continue;
        b3Shape_SetUserData(sid, reinterpret_cast<void *>(intptr_t(profile + 1)));
        // The filter only runs on new pairs, so drop the shape's contacts. SetFilter(invoke)
        // does that but returns early on an unchanged filter: change it, then set it back.
        const b3Filter f = b3Shape_GetFilter(sid);
        b3Filter tmp = f;
        tmp.groupIndex = f.groupIndex + 1;
        b3Shape_SetFilter(sid, tmp, false);
        b3Shape_SetFilter(sid, f, true);
    }
}

void
Box3DWorld::applyTuning(const WorldSettings &s)
{
    if (!isValid())
        return;
    WorldSettings &cur = myImpl->settings;
    const b3WorldId w = myImpl->worldId;

    // What will be in force: non-finite gravity or recycle distance keep the previous value, and
    // the worker count is fixed for the life of the world.
    WorldSettings next = s;
    next.workerCount = cur.workerCount;
    if (!finite3(next.gravity))
        std::copy(cur.gravity, cur.gravity + 3, next.gravity);
    if (!std::isfinite(next.contactRecycleDistance))
        next.contactRecycleDistance = cur.contactRecycleDistance;

    if (!std::equal(next.gravity, next.gravity + 3, cur.gravity))
        b3World_SetGravity(w, makeVec3(next.gravity));
    if (next.enableSleep != cur.enableSleep)
        b3World_EnableSleeping(w, next.enableSleep);
    if (next.enableContinuous != cur.enableContinuous)
        b3World_EnableContinuous(w, next.enableContinuous);
    if (next.enableSpeculative != cur.enableSpeculative)
        b3World_EnableSpeculative(w, next.enableSpeculative);
    if (next.contactHertz != cur.contactHertz || next.contactDampingRatio != cur.contactDampingRatio ||
        next.contactPushSpeed != cur.contactPushSpeed)
        b3World_SetContactTuning(w, std::max(0.0f, next.contactHertz), std::max(0.0f, next.contactDampingRatio),
                                 std::max(0.0f, next.contactPushSpeed));
    if (next.restitutionThreshold != cur.restitutionThreshold)
        b3World_SetRestitutionThreshold(w, std::max(0.0f, next.restitutionThreshold));
    if (next.maxLinearSpeed != cur.maxLinearSpeed)
        b3World_SetMaximumLinearSpeed(w, std::max(0.0f, next.maxLinearSpeed));
    if (next.restitutionIterations != cur.restitutionIterations)
        b3World_SetRestitutionIterations(w, std::clamp(next.restitutionIterations, 0, B3_MAX_RESTITUTION_ITERATIONS));
    if (next.restitutionPropagation != cur.restitutionPropagation)
        b3World_EnableRestitutionPropagation(w, next.restitutionPropagation);
    if (next.isotropicInertiaTolerance != cur.isotropicInertiaTolerance)
        b3World_SetIsotropicInertiaTolerance(w, std::clamp(next.isotropicInertiaTolerance, 0.0f, 1.0f));
    if (next.warmStarting != cur.warmStarting)
        b3World_EnableWarmStarting(w, next.warmStarting);
    if (next.impactMinSpeed != cur.impactMinSpeed)
        b3World_SetHitEventThreshold(w, next.impactMinSpeed > 0.0f ? next.impactMinSpeed
                                                                   : b3DefaultWorldDef().hitEventThreshold);
    if (next.frictionMixing != cur.frictionMixing)
        b3World_SetFrictionCallback(w, frictionRule(next.frictionMixing));
    if (next.restitutionMixing != cur.restitutionMixing)
        b3World_SetRestitutionCallback(w, restitutionRule(next.restitutionMixing));
    if (next.contactRecycleDistance != cur.contactRecycleDistance)
        b3World_SetContactRecycleDistance(w, std::max(0.0f, next.contactRecycleDistance));
    cur = next;
}

void
Box3DWorld::updateHitEvents()
{
    myImpl->hitPairs.clear();
    if (!isValid())
        return;
    const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
    for (int i = 0; i < ev.hitCount; ++i)
    {
        const BodyHandle a = packBody(b3Shape_GetBody(ev.hitEvents[i].shapeIdA));
        const BodyHandle b = packBody(b3Shape_GetBody(ev.hitEvents[i].shapeIdB));
        myImpl->hitPairs.insert({ std::min(a, b), std::max(a, b) });
    }
}

bool
Box3DWorld::hadHitEvent(BodyHandle a, BodyHandle b) const
{
    return myImpl->hitPairs.count({ std::min(a, b), std::max(a, b) }) > 0;
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
Box3DWorld::contactRecyclingByHandle(BodyHandle h) const
{
    return h && b3Body_IsContactRecyclingEnabled(unpackBody(h));
}

int
Box3DWorld::recycledContactCount() const
{
    return isValid() ? b3World_GetCounters(myImpl->worldId).recycledContactCount : 0;
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
Box3DWorld::getTransformByHandle(BodyHandle h, double outPos[3], float outRotXYZW[4]) const
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    storePos(b3Body_GetPosition(id), outPos);       // body origin
    storeQuat(b3Body_GetRotation(id), outRotXYZW);
    return true;
}

bool
Box3DWorld::getVelocityByHandle(BodyHandle h, float outLinear[3], float outAngular[3]) const
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    storeVec3(b3Body_GetLinearVelocity(id), outLinear);
    storeVec3(b3Body_GetAngularVelocity(id), outAngular);
    return true;
}

bool
Box3DWorld::setVelocityByHandle(BodyHandle h, const float linear[3], const float angular[3])
{
    if (!h)
        return false;
    const b3BodyId id = unpackBody(h);
    b3Body_SetLinearVelocity(id, finiteVec3(linear));
    b3Body_SetAngularVelocity(id, finiteVec3(angular));
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
    toBodyMove(ev.moveEvents[index], out);
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
    // Every element is overwritten, so only resize (clear + resize would zero it every step).
    if (out.size() != size_t(ev.moveCount))
        out.resize(size_t(ev.moveCount));
    for (int i = 0; i < ev.moveCount; ++i)
        toBodyMove(ev.moveEvents[i], out[size_t(i)]);
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

int
Box3DWorld::assertCount()
{
    return gAssertCount.load();
}

int
Box3DWorld::liveWorldCount()
{
    return b3GetWorldCount();
}

int
Box3DWorld::maxWorldCount()
{
    return B3_MAX_WORLDS;
}

} // namespace b3dop
