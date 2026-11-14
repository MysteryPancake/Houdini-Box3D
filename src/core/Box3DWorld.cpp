// Box3DWorld.cpp
#include "Box3DWorld.h"
#include "Box3DDebug.h"

#include <box3d/box3d.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace b3dop {

static_assert(
    kMaxWorkerCount == B3_MAX_WORKERS,
    "kMaxWorkerCount (Box3DWorld.h) must match Box3D's B3_MAX_WORKERS");

namespace {

// ---- the process-wide hull cache
// -------------------------------------------------------------- Box3D copies a
// hull into the world's own database when a shape is made, so entries here are
// only needed until then: they can be shared by every world (a rebuilt world
// finds what its predecessor built) and evicted at any time. Worlds hold the
// entries they use in a step (Box3DWorld::Impl::held) so eviction never frees
// one between findHull and createHullShape.
struct SharedHull {
  b3HullData *hull = nullptr;
  float volume = 0.0f;
  size_t bytes = 0;
  ~SharedHull() {
    if (hull)
      b3DestroyHull(hull);
  }
};
using HullRef = std::shared_ptr<const SharedHull>;

size_t hullBytes(const b3HullData *h) {
  // The hull's whole allocation: header, vertex / point / edge / plane / face
  // arrays, the SoA vertex and normal arrays and the edge cosines.
  return static_cast<size_t>(h->byteCount);
}

class HullLibrary {
public:
  static HullLibrary &instance() {
    static HullLibrary lib;
    return lib;
  }

  HullRef find(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    auto it = myEntries.find(key);
    if (it == myEntries.end())
      return nullptr;
    it->second.lastUse = ++myTick;
    return it->second.ref;
  }

  // Takes `hull`; returns the cached entry (an earlier insert of the same key
  // wins).
  HullRef insert(HullId key, b3HullData *hull) {
    auto fresh = std::make_shared<SharedHull>();
    fresh->hull = hull;
    fresh->volume = hull->volume;
    fresh->bytes = hullBytes(hull);
    std::lock_guard<std::mutex> lock(myMutex);
    auto ins = myEntries.try_emplace(key);
    if (ins.second) {
      ins.first->second.ref = fresh;
      myBytes += fresh->bytes;
    }
    ins.first->second.lastUse = ++myTick;
    HullRef out = ins.first->second.ref;
    if (myBytes > myBudget)
      evictLocked();
    return out;
  }

  // The vertex budget that last worked for a group (0: none yet).
  int budgetHint(uint64_t group) {
    if (!group)
      return 0;
    std::lock_guard<std::mutex> lock(myMutex);
    auto it = myHints.find(group);
    return it == myHints.end() ? 0 : it->second;
  }
  void setBudgetHint(uint64_t group, int budget) {
    if (!group)
      return;
    std::lock_guard<std::mutex> lock(myMutex);
    if (myHints.size() > 65536)
      myHints.clear();
    myHints[group] = budget;
  }

  int count() {
    std::lock_guard<std::mutex> lock(myMutex);
    return static_cast<int>(myEntries.size());
  }
  size_t bytes() {
    std::lock_guard<std::mutex> lock(myMutex);
    return myBytes;
  }
  void setBudget(size_t b) {
    std::lock_guard<std::mutex> lock(myMutex);
    myBudget = b;
    if (myBytes > myBudget)
      evictLocked();
  }

  std::atomic<int> builds{0}, attempts{0};

private:
  struct Entry {
    HullRef ref;
    uint64_t lastUse = 0;
  };

  // Least recently used first, down to 3/4 of the budget. Held entries stay
  // alive through their HullRef; they just stop being found.
  void evictLocked() {
    std::vector<std::pair<uint64_t, HullId>> order;
    order.reserve(myEntries.size());
    for (const auto &kv : myEntries)
      order.emplace_back(kv.second.lastUse, kv.first);
    std::sort(order.begin(), order.end());
    const size_t target = myBudget / 4 * 3;
    for (const auto &o : order) {
      if (myBytes <= target)
        break;
      auto it = myEntries.find(o.second);
      myBytes -= it->second.ref->bytes;
      myEntries.erase(it);
    }
  }

  std::mutex myMutex;
  std::unordered_map<HullId, Entry> myEntries;
  std::unordered_map<uint64_t, int> myHints;
  size_t myBytes = 0;
  size_t myBudget = static_cast<size_t>(256) << 20;
  uint64_t myTick = 0;
};

} // namespace

struct Box3DWorld::Impl {
  b3WorldId worldId = b3_nullWorldId;
  WorldSettings settings;
  std::shared_ptr<TaskRunner>
      tasks;          // host threads (null: Box3D's own pool); outlives worldId
  int liveBodies = 0; // every live body
  std::vector<std::pair<BodyHandle, BodyHandle>>
      hitPairs; // updateHitEvents(): (lower, higher) handle, sorted, unique

  // Shared hulls this world found or built since the last pruneHulls()
  // (findHull may run on several threads, hence the lock).
  std::mutex held_mutex;
  std::unordered_map<HullId, HullRef> held;

  HullRef heldHull(HullId id) {
    std::lock_guard<std::mutex> lock(held_mutex);
    auto it = held.find(id);
    return it == held.end() ? nullptr : it->second;
  }
  void hold(HullId id, const HullRef &ref) {
    std::lock_guard<std::mutex> lock(held_mutex);
    held.emplace(id, ref);
  }

  // The world end of joints with a null BodyHandle: a shapeless static body at
  // the origin, made on first use and not counted in liveBodies.
  b3BodyId anchor = b3_nullBodyId;

  // Triangle meshes and height fields for static colliders. Box3D keeps a
  // pointer to the b3MeshData / b3HeightFieldData (it does not clone them), so
  // we own them: freed after their body is destroyed, or after the world.
  struct OwnedGeometry {
    std::vector<b3MeshData *> meshes;
    std::vector<b3HeightFieldData *> heightFields;
    void release() {
      for (b3MeshData *m : meshes)
        b3DestroyMesh(m);
      for (b3HeightFieldData *hf : heightFields)
        b3DestroyHeightField(hf);
      meshes.clear();
      heightFields.clear();
    }
  };
  std::unordered_map<BodyHandle, OwnedGeometry> bodyGeometry;

  // The meshes and height fields a body's shapes used. Call after the body or
  // its shapes are destroyed.
  void releaseBodyResources(BodyHandle body) {
    auto it = bodyGeometry.find(body);
    if (it == bodyGeometry.end())
      return;
    it->second.release();
    bodyGeometry.erase(it);
  }
  void destroyAllMeshes() {
    for (auto &kv : bodyGeometry)
      kv.second.release();
    bodyGeometry.clear();
  }
};

namespace {

// ---- Box3D assertion and log handlers ---------------------------------------
// Box3D's default assert handler breaks to the debugger, which kills the host
// (Houdini) when none is attached, and its default log handler printf()s.
// Asserts are compiled in only with NDEBUG unset or B3_ENABLE_ASSERT
// (RelWithDebInfo). Ours count the assert and carry on (no break), and both
// print only in BOX3DDOP_DEBUG builds (stderr, Houdini's console; see
// Box3DDebug.h), so a normal build never writes to the console, which on
// Windows opens a console window.
std::atomic<int> gAssertCount{0};
constexpr int kMaxAssertLogs = 20;

int assertHandler(const char *condition, const char *fileName, int lineNumber) {
  const int n = gAssertCount.fetch_add(1) + 1;
  if (n <= kMaxAssertLogs)
    B3D_LOG("Box3D assertion failed (ignored, simulation continues): %s, %s, "
            "line %d",
            condition ? condition : "?", fileName ? fileName : "?", lineNumber);
  if (n == kMaxAssertLogs)
    B3D_LOG("Box3D: further assertion messages are not printed");
  return 0;
}

void logHandler(const char *message) {
  B3D_LOG("Box3D: %s", message ? message : "?");
}

void installAssertHandler() {
  static std::once_flag once;
  std::call_once(once, [] {
    b3SetAssertFcn(&assertHandler);
    b3SetLogFcn(&logHandler);
  });
}

// Non-finite components become 0 (Box3D asserts on them, and a NaN velocity
// spreads to the body's position and its broad-phase bounds).
b3Vec3 finiteVec3(const float v[3]) {
  b3Vec3 out;
  out.x = std::isfinite(v[0]) ? v[0] : 0.0f;
  out.y = std::isfinite(v[1]) ? v[1] : 0.0f;
  out.z = std::isfinite(v[2]) ? v[2] : 0.0f;
  return out;
}

// b3Pos is float, or double with BOX3D_DOUBLE_PRECISION; these work either way.
b3Pos makePos(const double p[3]) {
  b3Pos r;
  r.x = static_cast<decltype(r.x)>(p[0]);
  r.y = static_cast<decltype(r.y)>(p[1]);
  r.z = static_cast<decltype(r.z)>(p[2]);
  return r;
}

b3Quat makeQuat(const float q[4]) // x, y, z, w
{
  b3Quat r;
  r.v.x = q[0];
  r.v.y = q[1];
  r.v.z = q[2];
  r.s = q[3];
  return r;
}

b3Vec3 makeVec3(const float *v) // null -> origin
{
  b3Vec3 r;
  r.x = v ? v[0] : 0.0f;
  r.y = v ? v[1] : 0.0f;
  r.z = v ? v[2] : 0.0f;
  return r;
}

void storePos(const b3Pos &p, double out[3]) {
  out[0] = static_cast<double>(p.x);
  out[1] = static_cast<double>(p.y);
  out[2] = static_cast<double>(p.z);
}

void storeQuat(const b3Quat &q, float out[4]) {
  out[0] = q.v.x;
  out[1] = q.v.y;
  out[2] = q.v.z;
  out[3] = q.s;
}

void storeVec3(const b3Vec3 &v, float out[3]) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}

void toBodyMove(const b3BodyMoveEvent &e, BodyMove &out) {
  out.userData = e.userData;
  out.handle = b3StoreBodyId(e.bodyId);
  storePos(e.transform.p, out.pos);
  storeQuat(e.transform.q, out.rot);
  out.fellAsleep = e.fellAsleep;
}

b3BodyType toB3Type(BodySettings::Type t) {
  switch (t) {
  case BodySettings::Type::Static:
    return b3_staticBody;
  case BodySettings::Type::Kinematic:
    return b3_kinematicBody;
  default:
    return b3_dynamicBody;
  }
}

// A content key for body-local points. Never 0.
HullId hashPoints(const float *xyz, int count) {
  const uint64_t h = hashFloats(xyz, static_cast<size_t>(count) * 3);
  return h ? h : 1ull;
}

// Null, or a rotation within float noise of identity (qx, qy, qz ~ 0, w ~ +-1).
// fitShapeOriented: a turned frame must make the box at least this much
// smaller (relative volume) to replace the body axes.
constexpr double kTurnedFitGain = 1e-3;

bool isIdentityRotation(const float q[4]) {
  return !q || (std::fabs(q[0]) < 1e-6f && std::fabs(q[1]) < 1e-6f &&
                std::fabs(q[2]) < 1e-6f);
}

b3MotionLocks toB3Locks(const MotionLocks &l) {
  b3MotionLocks out = {};
  out.linearX = l.linear[0];
  out.linearY = l.linear[1];
  out.linearZ = l.linear[2];
  out.angularX = l.angular[0];
  out.angularY = l.angular[1];
  out.angularZ = l.angular[2];
  return out;
}

b3Filter toB3Filter(const CollisionFilter &f) {
  b3Filter out = b3DefaultFilter();
  out.categoryBits = f.category;
  out.maskBits = f.mask;
  out.groupIndex = f.group;
  return out;
}

// Box3D asserts on a negative or non-finite rolling resistance and tangent
// velocity: clean them.
float cleanRolling(float v) {
  return (std::isfinite(v) && v > 0.0f) ? v : 0.0f;
}

void cleanTangent(const float in[3], float out[3]) {
  const bool ok =
      std::isfinite(in[0]) && std::isfinite(in[1]) && std::isfinite(in[2]);
  for (int i = 0; i < 3; ++i)
    out[i] = ok ? in[i] : 0.0f;
}

b3ShapeDef shapeDefFor(const BodySettings &b, bool updateMass) {
  const BodyProps &p = b.props;
  float tangent[3];
  b3ShapeDef def = b3DefaultShapeDef();
  def.updateBodyMass =
      updateMass; // false: the caller applies the mass afterwards
  def.density = p.density;
  def.baseMaterial.friction = p.friction;
  def.baseMaterial.restitution = p.restitution;
  def.baseMaterial.rollingResistance = cleanRolling(p.rollingResistance);
  cleanTangent(p.tangentVelocity, tangent);
  def.baseMaterial.tangentVelocity = makeVec3(tangent);
  def.filter = toB3Filter(p.filter);
  def.enableHitEvents =
      true; // for the Impacts speed filter; costs nothing without a threshold
  return def;
}

// Mixing rules for two touching materials. The callbacks get no context, so
// each rule is its own function.
float frictionMin(float a, uint64_t, float b, uint64_t) {
  return std::min(a, b);
}
float frictionMax(float a, uint64_t, float b, uint64_t) {
  return std::max(a, b);
}
float frictionAvg(float a, uint64_t, float b, uint64_t) {
  return 0.5f * (a + b);
}
float frictionMul(float a, uint64_t, float b, uint64_t) { return a * b; }
float restitutionMin(float a, uint64_t, float b, uint64_t) {
  return std::min(a, b);
}
float restitutionAvg(float a, uint64_t, float b, uint64_t) {
  return 0.5f * (a + b);
}
float restitutionMul(float a, uint64_t, float b, uint64_t) { return a * b; }
float restitutionGeo(float a, uint64_t, float b, uint64_t) {
  return std::sqrt(std::max(0.0f, a * b));
}

// nullptr is Box3D's own rule (geometric mean for friction, maximum for
// restitution).
b3FrictionCallback *frictionRule(WorldSettings::FrictionMixing mode) {
  using M = WorldSettings::FrictionMixing;
  switch (mode) {
  case M::Minimum:
    return &frictionMin;
  case M::Maximum:
    return &frictionMax;
  case M::Average:
    return &frictionAvg;
  case M::Multiply:
    return &frictionMul;
  case M::GeometricMean:
    break;
  }
  return nullptr;
}

b3RestitutionCallback *restitutionRule(WorldSettings::RestitutionMixing mode) {
  using M = WorldSettings::RestitutionMixing;
  switch (mode) {
  case M::Minimum:
    return &restitutionMin;
  case M::Average:
    return &restitutionAvg;
  case M::Multiply:
    return &restitutionMul;
  case M::GeometricMean:
    return &restitutionGeo;
  case M::Maximum:
    break;
  }
  return nullptr;
}

bool sameBody(b3BodyId a, b3BodyId b) {
  return a.index1 == b.index1 && a.world0 == b.world0 &&
         a.generation == b.generation;
}

} // namespace

namespace {

// Box3D's task hooks, forwarding to the world's TaskRunner (userContext).
void *enqueueTask(b3TaskCallback *task, void *taskContext, void *userContext,
                  const char * /*name*/) {
  TaskRunner *runner = static_cast<TaskRunner *>(userContext);
  // A non-null handle means "queued"; null tells Box3D the task already ran
  // inline.
  return runner->run(task, taskContext) ? userContext : nullptr;
}

void finishTask(void *userTask, void *userContext) {
  if (userTask)
    static_cast<TaskRunner *>(userContext)->wait();
}

} // namespace

Box3DWorld::Box3DWorld() : myImpl(new Impl) {}

Box3DWorld::~Box3DWorld() { destroy(); }

bool Box3DWorld::create(const WorldSettings &s,
                        std::shared_ptr<TaskRunner> tasks) {
  destroy();
  installAssertHandler();

  b3WorldDef def = b3DefaultWorldDef();
  def.gravity = finiteVec3(s.gravity); // non-finite components: 0
  def.enableSleep = s.enableSleep;
  def.enableContinuous = s.enableContinuous;
  def.contactHertz = std::max(0.0f, s.contactStiffness);
  def.contactDampingRatio = std::max(0.0f, s.contactDampingRatio);
  def.contactSpeed = std::max(0.0f, s.contactPushSpeed);
  def.restitutionThreshold = std::max(0.0f, s.restitutionThreshold);
  def.maximumLinearSpeed = std::max(0.0f, s.maxLinearSpeed);
  def.restitutionIterations =
      std::clamp(s.restitutionIterations, 0, B3_MAX_RESTITUTION_ITERATIONS);
  def.enableRestitutionPropagation = s.restitutionPropagation;
  def.isotropicInertiaTolerance =
      std::clamp(s.isotropicInertiaTolerance, 0.0f, 1.0f);
  if (s.impactMinSpeed > 0.0f)
    def.hitEventThreshold = s.impactMinSpeed;
  def.frictionCallback = frictionRule(s.frictionMixing);
  def.restitutionCallback = restitutionRule(s.restitutionMixing);
  def.workerCount = static_cast<uint32_t>(
      std::min(std::max(1, s.workerCount), kMaxWorkerCount));
  if (tasks && def.workerCount > 1) {
    // The host's threads: Box3D starts none. The runner must outlive the
    // b3World (see destroy).
    def.enqueueTask = &enqueueTask;
    def.finishTask = &finishTask;
    def.userTaskContext = tasks.get();
  } else {
    // No runner (or one worker): Box3D's own pool for workerCount > 1, serial
    // otherwise.
    tasks.reset();
  }

  // At the limit b3CreateWorld asserts (a crash in builds with Box3D asserts
  // on, such as RelWithDebInfo) before failing, so fail here first.
  if (b3GetWorldCount() >= B3_MAX_WORLDS)
    return false;

  myImpl->settings = s;
  myImpl->settings.gravity[0] = def.gravity.x; // as applied (finite)
  myImpl->settings.gravity[1] = def.gravity.y;
  myImpl->settings.gravity[2] = def.gravity.z;
  myImpl->tasks = std::move(tasks);
  myImpl->worldId = b3CreateWorld(&def);
  if (!b3World_IsValid(myImpl->worldId)) {
    myImpl->tasks.reset();
    return false;
  }
  // Not part of b3WorldDef. A non-finite value falls back to the default.
  if (std::isfinite(s.contactRecycleDistance))
    b3World_SetContactRecycleDistance(myImpl->worldId,
                                      std::max(0.0f, s.contactRecycleDistance));
  else
    myImpl->settings.contactRecycleDistance =
        WorldSettings().contactRecycleDistance;
  b3World_EnableWarmStarting(myImpl->worldId, s.warmStarting);
  myImpl->hitPairs.clear();
  return true;
}

void Box3DWorld::destroy() {
  if (b3World_IsValid(myImpl->worldId))
    b3DestroyWorld(myImpl->worldId); // destroys all bodies/shapes/joints

  myImpl->tasks
      .reset(); // after the world: Box3D may call the runner until it is gone
  myImpl->worldId = b3_nullWorldId;
  myImpl->anchor = b3_nullBodyId;
  myImpl->liveBodies = 0;
  myImpl->held.clear();       // the shared cache keeps the hulls
  myImpl->destroyAllMeshes(); // after the world: its shapes pointed at these
}

bool Box3DWorld::isValid() const { return b3World_IsValid(myImpl->worldId); }

const WorldSettings &Box3DWorld::settings() const { return myImpl->settings; }

// ---------------------------------------------------------------------------
// Stepping, bodies, shapes
// ---------------------------------------------------------------------------

void Box3DWorld::step(float timeStep) {
  if (!isValid())
    return;
  if (myImpl->tasks) {
    struct Args {
      b3WorldId id;
      float dt;
      int subSteps;
    } args{myImpl->worldId, timeStep, myImpl->settings.subSteps};
    myImpl->tasks->isolate(
        [](void *p) {
          const Args *a = static_cast<const Args *>(p);
          b3World_Step(a->id, a->dt, a->subSteps);
        },
        &args);
    return;
  }
  b3World_Step(myImpl->worldId, timeStep, myImpl->settings.subSteps);
}

BodyHandle Box3DWorld::createBody(const BodySettings &body,
                                  const double position[3],
                                  const float rotationXYZW[4], void *userData) {
  if (!isValid())
    return 0;

  const BodyProps &p = body.props;
  b3BodyDef def = b3DefaultBodyDef();
  def.type = toB3Type(body.type);
  def.position = makePos(position);
  def.rotation = makeQuat(rotationXYZW);
  def.linearDamping = p.linearDamping;
  def.angularDamping = p.angularDamping;
  def.gravityScale = p.gravityScale;
  def.enableSleep = p.enableSleep; // Box3D applies the world switch on top
  def.isAwake =
      body.startAwake; // Box3D: a body that cannot sleep starts awake anyway
  def.isEnabled = p.enabled;
  def.isBullet = p.fastObject;
  def.allowFastRotation = p.fastRotation;
  def.enableContactRecycling = p.contactRecycling;
  def.motionLocks = toB3Locks(p.locks);
  if (p.sleepThreshold >= 0.0f && std::isfinite(p.sleepThreshold))
    def.sleepThreshold = p.sleepThreshold;
  if (p.ccdThreshold >= 0.0f &&
      std::isfinite(p.ccdThreshold)) // Box3D asserts on negatives
    def.safetyFactor = p.ccdThreshold;
  def.userData = userData;

  const b3BodyId bodyId = b3CreateBody(myImpl->worldId, &def);
  if (B3_IS_NULL(bodyId))
    return 0;
  ++myImpl->liveBodies;
  return b3StoreBodyId(bodyId);
}

void Box3DWorld::destroyBody(BodyHandle h) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_IsValid(id)) {
    b3DestroyBody(id);
    --myImpl->liveBodies;
  }
  myImpl->releaseBodyResources(
      h); // after the body: its shapes pointed at the meshes
}

bool Box3DWorld::createShape(BodyHandle h, const BodySettings &body,
                             const ShapeSpec &shape, bool updateMass) {
  switch (shape.type) {
  case ShapeType::Sphere:
    return sphereShape(h, body, shape.radius, shape.center, updateMass);
  case ShapeType::Capsule:
    return capsuleShape(h, body, shape.radius, shape.halfLength, shape.center,
                        shape.rot, updateMass);
  default:
    return boxShape(h, body, shape.halfExtents, shape.center, shape.rot,
                    updateMass);
  }
}

bool Box3DWorld::createHullShape(BodyHandle h, const BodySettings &body,
                                 HullId hullId, bool updateMass) {
  const HullRef hull = h ? myImpl->heldHull(hullId) : nullptr;
  if (!hull)
    return false;
  const b3ShapeDef def = shapeDefFor(body, updateMass);
  return !B3_IS_NULL(
      b3CreateHullShape(b3LoadBodyId(h), &def, hull->hull)); // Box3D copies it
}

bool Box3DWorld::createMeshShape(BodyHandle h, const BodySettings &body,
                                 const float *xyz, int vertexCount,
                                 const int32_t *triangles, int triangleCount,
                                 float weldTolerance, const float scale[3]) {
  if (!h || !xyz || !triangles || vertexCount < 3 || triangleCount < 1)
    return false;
  // Static bodies only (a triangle mesh has no volume or mass): refuse anything
  // else.
  if (b3Body_GetType(b3LoadBodyId(h)) != b3_staticBody)
    return false;

  // Reject bad input here: Box3D asserts on non-finite vertices or out-of-range
  // indices.
  std::vector<b3Vec3> verts;
  verts.resize(static_cast<size_t>(vertexCount));
  for (int i = 0; i < vertexCount; ++i) {
    if (!finite3(xyz + 3 * i))
      return false;
    verts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);
  }
  std::vector<int32_t> idx;
  idx.assign(triangles, triangles + 3 * static_cast<size_t>(triangleCount));
  for (int32_t v : idx)
    if (v < 0 || v >= vertexCount)
      return false;

  b3MeshDef def = {};
  def.vertices = verts.data();
  def.stride = 0;
  def.indices = idx.data();
  def.materialIndices = nullptr;
  def.vertexCount = vertexCount;
  def.triangleCount = triangleCount;
  def.weldVertices = weldTolerance > 0.0f;
  def.weldTolerance = weldTolerance > 0.0f ? weldTolerance : 0.0f;
  def.identifyEdges =
      true; // smooth rolling over triangle edges (less ghost collisions)

  b3MeshData *mesh = b3CreateMesh(&def, nullptr, 0);
  if (!mesh)
    return false;

  const b3ShapeDef sdef = shapeDefFor(body, false);
  const b3ShapeId shape = b3CreateMeshShape(
      b3LoadBodyId(h), &sdef, mesh, scale ? makeVec3(scale) : b3Vec3_one);
  if (B3_IS_NULL(shape)) {
    b3DestroyMesh(mesh);
    return false;
  }
  myImpl->bodyGeometry[h].meshes.push_back(mesh);
  return true;
}

bool Box3DWorld::createHeightFieldShape(BodyHandle h, const BodySettings &body,
                                        const float *heights, int countX,
                                        int countZ, const float scale[3],
                                        const uint8_t *holes) {
  if (!h || !heights || !scale || countX < 2 || countZ < 2)
    return false;
  // Box3D indexes with int and asserts on these: refuse instead.
  if (static_cast<int64_t>(countX) * countZ > (int64_t{1} << 28))
    return false;
  if (!finite3(scale) || !(scale[0] > 0.0f) || !(scale[1] > 0.0f) ||
      !(scale[2] > 0.0f))
    return false;
  if (b3Body_GetType(b3LoadBodyId(h)) != b3_staticBody)
    return false;

  const size_t count =
      static_cast<size_t>(countX) * static_cast<size_t>(countZ);
  float lo = heights[0], hi = heights[0];
  for (size_t i = 0; i < count; ++i) {
    const float v = heights[i];
    if (!std::isfinite(v))
      return false;
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }

  const size_t cells =
      static_cast<size_t>(countX - 1) * static_cast<size_t>(countZ - 1);
  std::vector<uint8_t> material(cells, 0);
  if (holes)
    for (size_t i = 0; i < cells; ++i)
      if (holes[i])
        material[i] = B3_HEIGHT_FIELD_HOLE;

  std::vector<float> hs(heights, heights + count); // Box3D takes float*
  b3HeightFieldDef def = {};
  def.heights = hs.data();
  def.materialIndices = material.data();
  def.scale = makeVec3(scale);
  def.countX = countX;
  def.countZ = countZ;
  // Quantised to 16 bits between these (unscaled): the field's own range.
  def.globalMinimumHeight = lo;
  def.globalMaximumHeight = hi;
  def.clockwiseWinding = false; // triangles face +y

  b3HeightFieldData *hf = b3CreateHeightField(&def);
  if (!hf)
    return false;
  const b3ShapeDef sdef = shapeDefFor(body, false);
  const b3ShapeId shape = b3CreateHeightFieldShape(b3LoadBodyId(h), &sdef, hf);
  if (B3_IS_NULL(shape)) {
    b3DestroyHeightField(hf);
    return false;
  }
  myImpl->bodyGeometry[h].heightFields.push_back(hf);
  return true;
}

void Box3DWorld::applyMassFromShapes(BodyHandle h) {
  if (h)
    b3Body_ApplyMassFromShapes(b3LoadBodyId(h));
}

bool Box3DWorld::boxShape(BodyHandle h, const BodySettings &b,
                          const float halfExtents[3], const float center[3],
                          const float rotationXYZW[4], bool updateMass) {
  if (!h)
    return false;

  // A rotated box is the hull of its 8 rotated corners (cached by content).
  if (!isIdentityRotation(rotationXYZW)) {
    float pts[24];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          const float corner[3] = {static_cast<float>(sx) * halfExtents[0],
                                   static_cast<float>(sy) * halfExtents[1],
                                   static_cast<float>(sz) * halfExtents[2]};
          float r[3];
          quatRotate(rotationXYZW, corner, r);
          for (int j = 0; j < 3; ++j)
            pts[n++] = r[j] + (center ? center[j] : 0.0f);
        }
    const HullId hull = createHull(pts, 8, nullptr);
    return hull && createHullShape(h, b, hull, updateMass);
  }

  const bool offset =
      center && (center[0] != 0.0f || center[1] != 0.0f || center[2] != 0.0f);
  const b3BoxHull hull =
      offset ? b3MakeOffsetBoxHull(halfExtents[0], halfExtents[1],
                                   halfExtents[2], makeVec3(center))
             : b3MakeBoxHull(halfExtents[0], halfExtents[1], halfExtents[2]);
  const b3ShapeDef def = shapeDefFor(b, updateMass);
  const b3ShapeId shape = b3CreateHullShape(b3LoadBodyId(h), &def, &hull.base);
  return !B3_IS_NULL(shape);
}

bool Box3DWorld::sphereShape(BodyHandle h, const BodySettings &b, float radius,
                             const float center[3], bool updateMass) {
  if (!h)
    return false;

  b3Sphere sphere;
  sphere.center = makeVec3(center);
  sphere.radius = radius;
  const b3ShapeDef def = shapeDefFor(b, updateMass);
  const b3ShapeId shape = b3CreateSphereShape(b3LoadBodyId(h), &def, &sphere);
  return !B3_IS_NULL(shape);
}

bool Box3DWorld::capsuleShape(BodyHandle h, const BodySettings &b, float radius,
                              float halfLength, const float center[3],
                              const float rotationXYZW[4], bool updateMass) {
  if (!h)
    return false;

  const float c[3] = {center ? center[0] : 0.0f, center ? center[1] : 0.0f,
                      center ? center[2] : 0.0f};
  // The axis is the shape's Y, turned by the shape rotation.
  float axis[3] = {0.0f, 1.0f, 0.0f};
  if (!isIdentityRotation(rotationXYZW)) {
    const float up[3] = {0.0f, 1.0f, 0.0f};
    quatRotate(rotationXYZW, up, axis);
  }
  b3Capsule capsule;
  capsule.center1.x = c[0] - halfLength * axis[0];
  capsule.center1.y = c[1] - halfLength * axis[1];
  capsule.center1.z = c[2] - halfLength * axis[2];
  capsule.center2.x = c[0] + halfLength * axis[0];
  capsule.center2.y = c[1] + halfLength * axis[1];
  capsule.center2.z = c[2] + halfLength * axis[2];
  capsule.radius = radius;
  // Box3D turns a capsule that is too short into a sphere by itself.
  const b3ShapeDef def = shapeDefFor(b, updateMass);
  const b3ShapeId shape = b3CreateCapsuleShape(b3LoadBodyId(h), &def, &capsule);
  return !B3_IS_NULL(shape);
}

int Box3DWorld::bodyCount() const { return myImpl->liveBodies; }

// ---------------------------------------------------------------------------
// Convex hulls
// ---------------------------------------------------------------------------

HullId Box3DWorld::createHull(const float *xyz, int pointCount,
                              float *outVolume) {
  if (outVolume)
    *outVolume = 0.0f;
  if (!xyz || pointCount < 4)
    return 0;
  for (int i = 0; i < pointCount * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return 0;
  return createHullWithKey(hashPoints(xyz, pointCount), xyz, pointCount,
                           outVolume);
}

HullId Box3DWorld::findHull(HullId key, float *outVolume) const {
  if (outVolume)
    *outVolume = 0.0f;
  if (key == 0)
    return 0;
  HullRef ref = myImpl->heldHull(key);
  if (!ref) {
    ref = HullLibrary::instance().find(key);
    if (!ref)
      return 0;
    myImpl->hold(key, ref);
  }
  if (outVolume)
    *outVolume = ref->volume;
  return key;
}

HullId Box3DWorld::createHullWithKey(HullId key, const float *xyz,
                                     int pointCount, float *outVolume,
                                     uint64_t budgetGroup) {
  if (outVolume)
    *outVolume = 0.0f;
  if (key == 0 || !xyz || pointCount < 4)
    return 0;
  if (const HullId hit = findHull(key, outVolume))
    return hit;
  for (int i = 0; i < pointCount * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return 0;

  std::vector<b3Vec3> pts(static_cast<size_t>(pointCount));
  for (int i = 0; i < pointCount; ++i)
    pts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);

  // b3CreateHull fails for degenerate input or when the result would exceed
  // Box3D's vertex / face / edge limits; a smaller vertex budget keeps fewer
  // vertices, so dense input still gets a (slightly smaller) hull. A
  // triangulated hull hits the 256 half-edge limit at about 44 vertices, which
  // is why there is no rung between the maximum and 44: a 64-vertex budget
  // fails whenever the maximum did. The group's last working budget is tried
  // first.
  HullLibrary &lib = HullLibrary::instance();
  const int hint = lib.budgetHint(budgetGroup);
  b3HullData *hull = nullptr;
  for (int budget : {B3_MAX_HULL_VERTICES, 44, 32, 16, 8}) {
    if (hint && budget > hint)
      continue;
    lib.attempts.fetch_add(1, std::memory_order_relaxed);
    hull = b3CreateHull(pts.data(), pointCount, budget);
    if (hull) {
      if (budget != hint)
        lib.setBudgetHint(budgetGroup, budget);
      break;
    }
  }
  if (!hull)
    return 0;
  lib.builds.fetch_add(1, std::memory_order_relaxed);

  if (!(hull->volume > 1e-9f) || !std::isfinite(hull->volume)) {
    b3DestroyHull(hull); // flat: no mass, no stable contacts
    return 0;
  }
  const HullRef ref = lib.insert(key, hull);
  myImpl->hold(key, ref);
  if (outVolume)
    *outVolume = ref->volume;
  return key;
}

bool Box3DWorld::bodyShapeOutlines(BodyHandle h,
                                   std::vector<ShapeOutline> &out) const {
  out.clear();
  if (!h)
    return false;
  const b3BodyId body = b3LoadBodyId(h);
  if (!b3Body_IsValid(body))
    return false;
  const int n = b3Body_GetShapeCount(body);
  std::vector<b3ShapeId> shapes(static_cast<size_t>(std::max(n, 0)));
  const int got = n > 0 ? b3Body_GetShapes(body, shapes.data(), n) : 0;
  auto put3 = [](float *dst, b3Vec3 v) {
    dst[0] = static_cast<float>(v.x);
    dst[1] = static_cast<float>(v.y);
    dst[2] = static_cast<float>(v.z);
  };
  for (int i = 0; i < got; ++i) {
    ShapeOutline o;
    switch (b3Shape_GetType(shapes[static_cast<size_t>(i)])) {
    case b3_sphereShape: {
      const b3Sphere sp = b3Shape_GetSphere(shapes[static_cast<size_t>(i)]);
      o.kind = ShapeOutline::Sphere;
      put3(o.c1, sp.center);
      o.radius = sp.radius;
      break;
    }
    case b3_capsuleShape: {
      const b3Capsule c = b3Shape_GetCapsule(shapes[static_cast<size_t>(i)]);
      o.kind = ShapeOutline::Capsule;
      put3(o.c1, c.center1);
      put3(o.c2, c.center2);
      o.radius = c.radius;
      break;
    }
    case b3_hullShape: {
      const b3HullData *hull = b3Shape_GetHull(shapes[static_cast<size_t>(i)]);
      const b3Vec3 *pts = hull ? b3GetHullPoints(hull) : nullptr;
      const b3HullFace *faces = hull ? b3GetHullFaces(hull) : nullptr;
      const b3HullHalfEdge *edges = hull ? b3GetHullEdges(hull) : nullptr;
      if (!pts || !faces || !edges)
        continue;
      o.kind = ShapeOutline::Hull;
      o.verts.resize(static_cast<size_t>(hull->vertexCount) * 3);
      for (int v = 0; v < hull->vertexCount; ++v)
        put3(&o.verts[static_cast<size_t>(v) * 3], pts[v]);
      for (int f = 0; f < hull->faceCount; ++f) {
        const size_t countAt = o.loops.size();
        o.loops.push_back(0);
        const int first = faces[f].edge;
        int e = first;
        for (int guard = 0; guard < hull->edgeCount; ++guard) {
          o.loops.push_back(edges[e].origin);
          ++o.loops[countAt];
          e = edges[e].next;
          if (e == first)
            break;
        }
      }
      break;
    }
    default:
      continue; // meshes and compounds: not made for bodies here
    }
    out.push_back(std::move(o));
  }
  return true;
}

bool Box3DWorld::fitShapeOriented(ShapeType type, const float *xyz, int count,
                                  ShapeSpec &out) {
  if (!xyz || count < 4)
    return fitShapeToPoints(type, xyz, count, out);
  for (int i = 0; i < count * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return false;

  std::vector<b3Vec3> pts(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
    pts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);
  // Only the hull's directions are used, so a reduced budget is fine (the
  // extents are measured over every point).
  b3HullData *hull = nullptr;
  for (int budget : {B3_MAX_HULL_VERTICES, 44, 32, 16, 8}) {
    hull = b3CreateHull(pts.data(), count, budget);
    if (hull)
      break;
  }
  if (!hull)
    return fitShapeToPoints(type, xyz, count, out);

  const b3Vec3 *hp = b3GetHullPoints(hull);
  const b3Plane *planes = b3GetHullPlanes(hull);
  const b3HullFace *faces = b3GetHullFaces(hull);
  const b3HullHalfEdge *edges = b3GetHullEdges(hull);
  const int nv = hull->vertexCount;

  // The box volume along `axes`, over the hull's vertices.
  auto volumeAlong = [&](const float axes[3][3]) {
    double v = 1.0;
    for (int k = 0; k < 3; ++k) {
      float lo = 0.0f, hi = 0.0f;
      for (int i = 0; i < nv; ++i) {
        const float d =
            hp[i].x * axes[k][0] + hp[i].y * axes[k][1] + hp[i].z * axes[k][2];
        if (i == 0 || d < lo)
          lo = d;
        if (i == 0 || d > hi)
          hi = d;
      }
      v *= static_cast<double>(hi - lo);
    }
    return v;
  };

  float best[3][3];
  std::memcpy(best, identityFrame(), sizeof best);
  const double bodyAxesVolume = volumeAlong(best);
  double bestVolume = bodyAxesVolume;
  if (hp && planes && faces && edges)
    for (int f = 0; f < hull->faceCount; ++f) {
      const b3Vec3 n = planes[f].normal;
      const int first = faces[f].edge;
      int e = first;
      for (int guard = 0; guard < hull->edgeCount; ++guard) {
        const b3Vec3 a = hp[edges[e].origin];
        const b3Vec3 b = hp[edges[edges[e].next].origin];
        // The edge direction made perpendicular to the normal.
        float u[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
        const float dn = u[0] * n.x + u[1] * n.y + u[2] * n.z;
        u[0] -= dn * n.x;
        u[1] -= dn * n.y;
        u[2] -= dn * n.z;
        const float len = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        if (len > 1e-6f) {
          float axes[3][3] = {
              {u[0] / len, u[1] / len, u[2] / len}, {n.x, n.y, n.z}, {0, 0, 0}};
          axes[2][0] = axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1];
          axes[2][1] = axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2];
          axes[2][2] = axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0];
          const double v = volumeAlong(axes);
          if (v < bestVolume) {
            bestVolume = v;
            std::memcpy(best, axes, sizeof best);
          }
        }
        e = edges[e].next;
        if (e == first)
          break;
      }
    }
  b3DestroyHull(hull);

  // Turn only for a clearly smaller box, so float noise never turns geometry
  // that lies along the body axes.
  if (!(bestVolume < bodyAxesVolume * (1.0 - kTurnedFitGain)))
    std::memcpy(best, identityFrame(), sizeof best);
  canonicalFrame(best);
  return fitShapeInFrame(type, xyz, count, best, out);
}

void Box3DWorld::pruneHulls() {
  std::lock_guard<std::mutex> lock(myImpl->held_mutex);
  myImpl->held.clear();
}

int Box3DWorld::hullCacheSize() const {
  std::lock_guard<std::mutex> lock(myImpl->held_mutex);
  return static_cast<int>(myImpl->held.size());
}

int Box3DWorld::sharedHullCount() { return HullLibrary::instance().count(); }
size_t Box3DWorld::sharedHullBytes() { return HullLibrary::instance().bytes(); }
void Box3DWorld::setSharedHullBudget(size_t b) {
  HullLibrary::instance().setBudget(b);
}
int Box3DWorld::hullBuildCount() {
  return HullLibrary::instance().builds.load();
}
int Box3DWorld::hullBuildAttempts() {
  return HullLibrary::instance().attempts.load();
}

bool Box3DWorld::setTypeByHandle(BodyHandle h, BodySettings::Type type) {
  if (!h)
    return false;
  b3Body_SetType(b3LoadBodyId(h), toB3Type(type));
  return true;
}

BodySettings::Type Box3DWorld::typeByHandle(BodyHandle h) const {
  switch (h ? b3Body_GetType(b3LoadBodyId(h)) : b3_dynamicBody) {
  case b3_staticBody:
    return BodySettings::Type::Static;
  case b3_kinematicBody:
    return BodySettings::Type::Kinematic;
  default:
    return BodySettings::Type::Dynamic;
  }
}

// ---- changes to existing bodies
// -------------------------------------------------

namespace {

// Every shape of a body, into a reused scratch vector.
int bodyShapes(b3BodyId id, std::vector<b3ShapeId> &out) {
  const int n = b3Body_GetShapeCount(id);
  out.resize(static_cast<size_t>(n > 0 ? n : 0));
  return n > 0 ? b3Body_GetShapes(id, out.data(), n) : 0;
}

thread_local std::vector<b3ShapeId> tShapes;

} // namespace

void Box3DWorld::setMeshScaleByHandle(BodyHandle h, const float scale[3]) {
  if (!h || !scale || !finite3(scale))
    return;
  const b3Vec3 sc = makeVec3(scale);
  const int n = bodyShapes(b3LoadBodyId(h), tShapes);
  for (int i = 0; i < n; ++i) {
    const b3ShapeId sid = tShapes[static_cast<size_t>(i)];
    if (b3Shape_GetType(sid) != b3_meshShape)
      continue;
    const b3Mesh mesh = b3Shape_GetMesh(sid);
    if (mesh.scale.x != sc.x || mesh.scale.y != sc.y || mesh.scale.z != sc.z)
      b3Shape_SetMesh(sid, mesh.data, sc);
  }
}

void Box3DWorld::setMaterialByHandle(BodyHandle h, float friction,
                                     float restitution) {
  if (!h)
    return;
  const int n = bodyShapes(b3LoadBodyId(h), tShapes);
  for (int i = 0; i < n; ++i) {
    if (b3Shape_GetType(tShapes[static_cast<size_t>(i)]) == b3_compoundShape)
      continue; // never made here; Box3D asserts on it
    b3Shape_SetFriction(tShapes[static_cast<size_t>(i)], friction);
    b3Shape_SetRestitution(tShapes[static_cast<size_t>(i)], restitution);
  }
}

void Box3DWorld::setDampingByHandle(BodyHandle h, float linear, float angular) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  b3Body_SetLinearDamping(id, linear);
  b3Body_SetAngularDamping(id, angular);
}

void Box3DWorld::setGravityScaleByHandle(BodyHandle h, float gravityScale) {
  if (h)
    b3Body_SetGravityScale(b3LoadBodyId(h), gravityScale);
}

void Box3DWorld::setMotionLocksByHandle(BodyHandle h,
                                        const MotionLocks &locks) {
  if (h)
    b3Body_SetMotionLocks(b3LoadBodyId(h),
                          toB3Locks(locks)); // no-op when unchanged
}

MotionLocks Box3DWorld::motionLocksByHandle(BodyHandle h) const {
  MotionLocks out;
  if (!h)
    return out;
  const b3MotionLocks l = b3Body_GetMotionLocks(b3LoadBodyId(h));
  out.linear[0] = l.linearX;
  out.linear[1] = l.linearY;
  out.linear[2] = l.linearZ;
  out.angular[0] = l.angularX;
  out.angular[1] = l.angularY;
  out.angular[2] = l.angularZ;
  return out;
}

namespace {

// Drops a body's contacts so the next step rebuilds them (disable + enable).
// Disabling wakes the bodies touching it and zeroes the body's velocity, which
// is restored. Static bodies have none.
void rebuildContacts(b3BodyId id) {
  const bool isStatic = b3Body_GetType(id) == b3_staticBody;
  const b3Vec3 linear = isStatic ? b3Vec3_zero : b3Body_GetLinearVelocity(id);
  const b3Vec3 angular = isStatic ? b3Vec3_zero : b3Body_GetAngularVelocity(id);
  b3Body_Disable(id);
  b3Body_Enable(id);
  if (!isStatic) {
    b3Body_SetLinearVelocity(id, linear);
    b3Body_SetAngularVelocity(id, angular);
  }
}

} // namespace

void Box3DWorld::setContactRecyclingByHandle(BodyHandle h, bool flag) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_IsContactRecyclingEnabled(id) == flag)
    return;
  b3Body_EnableContactRecycling(id, flag);
  if (b3Body_GetType(id) != b3_dynamicBody || !b3Body_IsEnabled(id) ||
      !b3Body_IsAwake(id))
    return;
  // Existing contacts keep the flag they were made with: rebuild them with the
  // new one.
  rebuildContacts(id);
}

void Box3DWorld::setSurfaceByHandle(BodyHandle h, float rollingResistance,
                                    const float tangentVelocity[3]) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  const float rolling = cleanRolling(rollingResistance);
  float tangent[3];
  cleanTangent(tangentVelocity, tangent);
  const b3Vec3 tv = makeVec3(tangent);

  bool changed = false;
  const int n = bodyShapes(id, tShapes);
  for (int i = 0; i < n; ++i) {
    const b3ShapeId s = tShapes[static_cast<size_t>(i)];
    if (b3Shape_GetType(s) == b3_compoundShape)
      continue; // never made here; Box3D asserts on it
    b3SurfaceMaterial m = b3Shape_GetSurfaceMaterial(s);
    if (m.rollingResistance == rolling && m.tangentVelocity.x == tv.x &&
        m.tangentVelocity.y == tv.y && m.tangentVelocity.z == tv.z)
      continue;
    m.rollingResistance = rolling;
    m.tangentVelocity = tv;
    b3Shape_SetSurfaceMaterial(s, m);
    changed = true;
  }
  if (!changed || !b3Body_IsEnabled(id))
    return;
  if (b3Body_GetType(id) == b3_dynamicBody && !b3Body_IsAwake(id))
    return; // a cycle would wake it
  rebuildContacts(id);
}

void Box3DWorld::setSleepEnabledByHandle(BodyHandle h, bool enableSleep) {
  if (h)
    b3Body_EnableSleep(b3LoadBodyId(h), enableSleep);
}

void Box3DWorld::setFastRotationByHandle(BodyHandle h, bool flag) {
  if (h)
    b3Body_AllowFastRotation(b3LoadBodyId(h), flag); // no-op when unchanged
}

bool Box3DWorld::fastRotationByHandle(BodyHandle h) const {
  return h && b3Body_IsFastRotationAllowed(b3LoadBodyId(h));
}

void Box3DWorld::setBulletByHandle(BodyHandle h, bool flag) {
  if (h)
    b3Body_SetBullet(b3LoadBodyId(h), flag);
}

void Box3DWorld::setDensityByHandle(BodyHandle h, float density) {
  if (!h || !(density >= 0.0f) || !std::isfinite(density))
    return;
  const b3BodyId id = b3LoadBodyId(h);
  const int n = bodyShapes(id, tShapes);
  for (int i = 0; i < n; ++i)
    b3Shape_SetDensity(tShapes[static_cast<size_t>(i)], density, false);
  if (n > 0)
    b3Body_ApplyMassFromShapes(id);
}

void Box3DWorld::setEnabledByHandle(BodyHandle h, bool enabled) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_IsEnabled(id) == enabled)
    return;
  if (enabled)
    b3Body_Enable(id);
  else
    b3Body_Disable(id);
}

bool Box3DWorld::isEnabledByHandle(BodyHandle h) const {
  return h && b3Body_IsEnabled(b3LoadBodyId(h));
}

void Box3DWorld::destroyShapesByHandle(BodyHandle h) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  const int n = bodyShapes(id, tShapes);
  for (int i = 0; i < n; ++i)
    b3DestroyShape(tShapes[static_cast<size_t>(i)],
                   false); // the caller applies the mass afterwards
  myImpl->releaseBodyResources(h);
}

void Box3DWorld::wakeByHandle(BodyHandle h) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_GetType(id) != b3_staticBody)
    b3Body_SetAwake(id, true);
}

void Box3DWorld::sleepByHandle(BodyHandle h) {
  if (!h || !myImpl->settings.enableSleep)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_GetType(id) == b3_staticBody || !b3Body_IsEnabled(id) ||
      !b3Body_IsSleepEnabled(id))
    return;
  b3Body_SetAwake(id, false);
}

bool Box3DWorld::getCenterOfMassByHandle(BodyHandle h, double outPos[3]) const {
  if (!h)
    return false;
  storePos(b3Body_GetWorldCenter(b3LoadBodyId(h)), outPos);
  return true;
}

bool Box3DWorld::bodyStateByHandle(BodyHandle h, BodyState &out) const {
  if (!h)
    return false;
  const b3BodyId id = b3LoadBodyId(h);
  out.awake = b3Body_IsAwake(id);
  out.mass = b3Body_GetMass(id);
  storePos(b3Body_GetWorldCenter(id), out.com);
  storeVec3(b3Body_GetLinearVelocity(id), out.lin);
  storeVec3(b3Body_GetAngularVelocity(id), out.ang);
  return true;
}

void Box3DWorld::applyForceByHandle(BodyHandle h, const float force[3],
                                    const float torque[3], bool wake) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_GetType(id) != b3_dynamicBody)
    return;
  // Box3D asserts on non-finite input and a NaN would spread to the body's
  // pose.
  const b3Vec3 f = finiteVec3(force);
  const b3Vec3 t = finiteVec3(torque);
  if (f.x != 0.0f || f.y != 0.0f || f.z != 0.0f)
    b3Body_ApplyForceToCenter(id, f, wake);
  if (t.x != 0.0f || t.y != 0.0f || t.z != 0.0f)
    b3Body_ApplyTorque(id, t, wake);
}

void Box3DWorld::applyMassByHandle(BodyHandle h, float density,
                                   bool comAtOrigin, float inertiaScale,
                                   bool setShapeDensity) {
  if (!h || !(density >= 0.0f) || !std::isfinite(density))
    return;
  const b3BodyId id = b3LoadBodyId(h);
  const int n =
      setShapeDensity ? bodyShapes(id, tShapes) : b3Body_GetShapeCount(id);
  if (setShapeDensity)
    for (int i = 0; i < n; ++i)
      b3Shape_SetDensity(tShapes[static_cast<size_t>(i)], density, false);
  b3Body_ApplyMassFromShapes(id);

  const bool scale = inertiaScale > 0.0f && std::isfinite(inertiaScale) &&
                     inertiaScale != 1.0f;
  if (b3Body_GetType(id) != b3_dynamicBody || n == 0 ||
      (!comAtOrigin && !scale))
    return;
  b3MassData md = b3Body_GetMassData(id);
  const bool moveCenter =
      comAtOrigin &&
      (md.center.x != 0.0f || md.center.y != 0.0f || md.center.z != 0.0f);
  if (!moveCenter && !scale)
    return; // already centred
  if (moveCenter) {
    // Parallel axis: I_o = I_c + m (|d|^2 E - d d^T). b3Matrix3 is three
    // columns.
    const float d[3] = {md.center.x, md.center.y, md.center.z};
    const float dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    b3Vec3 *col[3] = {&md.inertia.cx, &md.inertia.cy, &md.inertia.cz};
    for (int c = 0; c < 3; ++c) {
      auto shift = [&](int r) {
        return md.mass * ((r == c ? dd : 0.0f) - d[r] * d[c]);
      };
      col[c]->x += shift(0);
      col[c]->y += shift(1);
      col[c]->z += shift(2);
    }
    md.center = b3Vec3_zero;
  }
  if (scale) {
    md.inertia.cx = b3MulSV(inertiaScale, md.inertia.cx);
    md.inertia.cy = b3MulSV(inertiaScale, md.inertia.cy);
    md.inertia.cz = b3MulSV(inertiaScale, md.inertia.cz);
  }
  b3Body_SetMassData(id, md);
}

bool Box3DWorld::getMassDataByHandle(BodyHandle h, float &mass, float center[3],
                                     float inertiaDiag[3]) const {
  if (!h)
    return false;
  const b3MassData md = b3Body_GetMassData(b3LoadBodyId(h));
  mass = md.mass;
  storeVec3(md.center, center);
  inertiaDiag[0] = md.inertia.cx.x;
  inertiaDiag[1] = md.inertia.cy.y;
  inertiaDiag[2] = md.inertia.cz.z;
  return true;
}

void Box3DWorld::setCcdThresholdByHandle(BodyHandle h, float threshold) {
  if (!h)
    return;
  const float t = (threshold >= 0.0f && std::isfinite(threshold))
                      ? threshold
                      : b3DefaultBodyDef().safetyFactor;
  b3Body_SetSafetyFactor(b3LoadBodyId(h), t);
}

float Box3DWorld::ccdThresholdByHandle(BodyHandle h) const {
  return h ? b3Body_GetSafetyFactor(b3LoadBodyId(h)) : 0.0f;
}

void Box3DWorld::setSleepThresholdByHandle(BodyHandle h, float threshold) {
  if (!h)
    return;
  const float t = (threshold >= 0.0f && std::isfinite(threshold))
                      ? threshold
                      : b3DefaultBodyDef().sleepThreshold;
  b3Body_SetSleepThreshold(b3LoadBodyId(h), t);
}

float Box3DWorld::sleepThresholdByHandle(BodyHandle h) const {
  return h ? b3Body_GetSleepThreshold(b3LoadBodyId(h)) : 0.0f;
}

void Box3DWorld::setTargetTransformByHandle(BodyHandle h, const double pos[3],
                                            const float rotXYZW[4],
                                            float timeStep) {
  if (!h || !(timeStep > 0.0f))
    return;
  b3WorldTransform t;
  t.p = makePos(pos);
  t.q = makeQuat(rotXYZW);
  b3Body_SetTargetTransform(b3LoadBodyId(h), t, timeStep, true);
}

int Box3DWorld::getContactsByHandle(BodyHandle h,
                                    std::vector<ContactPoint> &out) const {
  out.clear();
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  const int cap = b3Body_GetContactCapacity(id);
  if (cap <= 0)
    return 0;
  thread_local std::vector<b3ContactData> data;
  data.resize(static_cast<size_t>(cap));
  const int n = b3Body_GetContactData(id, data.data(), cap);
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    const b3BodyId a = b3Shape_GetBody(cd.shapeIdA);
    const b3BodyId b = b3Shape_GetBody(cd.shapeIdB);
    const bool thisIsA = sameBody(a, id);
    const b3BodyId other = thisIsA ? b : a;
    const b3Pos centerA =
        b3Body_GetWorldCenter(a); // anchors are relative to A's centre of mass
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k) {
        const b3ManifoldPoint &mp = man.points[k];
        if (!(mp.totalNormalImpulse > 0.0f))
          continue;
        ContactPoint cp;
        cp.other = b3StoreBodyId(other);
        cp.otherUserData = b3Body_GetUserData(other);
        cp.pos[0] =
            static_cast<double>(centerA.x) + static_cast<double>(mp.anchorA.x);
        cp.pos[1] =
            static_cast<double>(centerA.y) + static_cast<double>(mp.anchorA.y);
        cp.pos[2] =
            static_cast<double>(centerA.z) + static_cast<double>(mp.anchorA.z);
        const float sgn =
            thisIsA ? 1.0f : -1.0f; // the manifold normal points from A to B
        cp.normal[0] = sgn * man.normal.x;
        cp.normal[1] = sgn * man.normal.y;
        cp.normal[2] = sgn * man.normal.z;
        cp.impulse = mp.totalNormalImpulse;
        out.push_back(cp);
      }
    }
  }
  return static_cast<int>(out.size());
}

void Box3DWorld::setFilterByHandle(BodyHandle h, const CollisionFilter &f) {
  if (!h)
    return;
  const b3Filter bf = toB3Filter(f);
  const int n = bodyShapes(b3LoadBodyId(h), tShapes);
  for (int i = 0; i < n; ++i)
    b3Shape_SetFilter(tShapes[static_cast<size_t>(i)], bf,
                      true); // no-op when unchanged
}

void Box3DWorld::applyTuning(const WorldSettings &s) {
  if (!isValid())
    return;
  WorldSettings &cur = myImpl->settings;
  const b3WorldId w = myImpl->worldId;

  // What will be in force: non-finite gravity or recycle distance keep the
  // previous value, and the worker count is fixed for the life of the world.
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
  if (next.contactStiffness != cur.contactStiffness ||
      next.contactDampingRatio != cur.contactDampingRatio ||
      next.contactPushSpeed != cur.contactPushSpeed)
    b3World_SetContactTuning(w, std::max(0.0f, next.contactStiffness),
                             std::max(0.0f, next.contactDampingRatio),
                             std::max(0.0f, next.contactPushSpeed));
  if (next.restitutionThreshold != cur.restitutionThreshold)
    b3World_SetRestitutionThreshold(w,
                                    std::max(0.0f, next.restitutionThreshold));
  if (next.maxLinearSpeed != cur.maxLinearSpeed)
    b3World_SetMaximumLinearSpeed(w, std::max(0.0f, next.maxLinearSpeed));
  if (next.restitutionIterations != cur.restitutionIterations)
    b3World_SetRestitutionIterations(w,
                                     std::clamp(next.restitutionIterations, 0,
                                                B3_MAX_RESTITUTION_ITERATIONS));
  if (next.restitutionPropagation != cur.restitutionPropagation)
    b3World_EnableRestitutionPropagation(w, next.restitutionPropagation);
  if (next.isotropicInertiaTolerance != cur.isotropicInertiaTolerance)
    b3World_SetIsotropicInertiaTolerance(
        w, std::clamp(next.isotropicInertiaTolerance, 0.0f, 1.0f));
  if (next.warmStarting != cur.warmStarting)
    b3World_EnableWarmStarting(w, next.warmStarting);
  if (next.impactMinSpeed != cur.impactMinSpeed)
    b3World_SetHitEventThreshold(
        w, next.impactMinSpeed > 0.0f ? next.impactMinSpeed
                                      : b3DefaultWorldDef().hitEventThreshold);
  if (next.frictionMixing != cur.frictionMixing)
    b3World_SetFrictionCallback(w, frictionRule(next.frictionMixing));
  if (next.restitutionMixing != cur.restitutionMixing)
    b3World_SetRestitutionCallback(w, restitutionRule(next.restitutionMixing));
  if (next.contactRecycleDistance != cur.contactRecycleDistance)
    b3World_SetContactRecycleDistance(
        w, std::max(0.0f, next.contactRecycleDistance));
  cur = next;
}

void Box3DWorld::updateHitEvents() {
  myImpl->hitPairs.clear();
  if (!isValid())
    return;
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  for (int i = 0; i < ev.hitCount; ++i) {
    const BodyHandle a =
        b3StoreBodyId(b3Shape_GetBody(ev.hitEvents[i].shapeIdA));
    const BodyHandle b =
        b3StoreBodyId(b3Shape_GetBody(ev.hitEvents[i].shapeIdB));
    myImpl->hitPairs.emplace_back(std::min(a, b), std::max(a, b));
  }
  // Sorted once per step (the vector keeps its capacity), so a lookup is a
  // binary search.
  std::sort(myImpl->hitPairs.begin(), myImpl->hitPairs.end());
  myImpl->hitPairs.erase(
      std::unique(myImpl->hitPairs.begin(), myImpl->hitPairs.end()),
      myImpl->hitPairs.end());
}

bool Box3DWorld::hadHitEvent(BodyHandle a, BodyHandle b) const {
  return std::binary_search(myImpl->hitPairs.begin(), myImpl->hitPairs.end(),
                            std::make_pair(std::min(a, b), std::max(a, b)));
}

bool Box3DWorld::getMaterialByHandle(BodyHandle h, float &friction,
                                     float &restitution) const {
  friction = restitution = 0.0f;
  if (!h)
    return false;
  if (bodyShapes(b3LoadBodyId(h), tShapes) <= 0)
    return false;
  friction = b3Shape_GetFriction(tShapes[0]);
  restitution = b3Shape_GetRestitution(tShapes[0]);
  return true;
}

bool Box3DWorld::getSurfaceByHandle(BodyHandle h, float &rollingResistance,
                                    float tangentVelocity[3]) const {
  rollingResistance = 0.0f;
  tangentVelocity[0] = tangentVelocity[1] = tangentVelocity[2] = 0.0f;
  if (!h)
    return false;
  if (bodyShapes(b3LoadBodyId(h), tShapes) <= 0)
    return false;
  const b3SurfaceMaterial m = b3Shape_GetSurfaceMaterial(tShapes[0]);
  rollingResistance = m.rollingResistance;
  tangentVelocity[0] = m.tangentVelocity.x;
  tangentVelocity[1] = m.tangentVelocity.y;
  tangentVelocity[2] = m.tangentVelocity.z;
  return true;
}

bool Box3DWorld::getDampingByHandle(BodyHandle h, float &linear,
                                    float &angular) const {
  linear = angular = 0.0f;
  if (!h)
    return false;
  linear = b3Body_GetLinearDamping(b3LoadBodyId(h));
  angular = b3Body_GetAngularDamping(b3LoadBodyId(h));
  return true;
}

float Box3DWorld::gravityScaleByHandle(BodyHandle h) const {
  return h ? b3Body_GetGravityScale(b3LoadBodyId(h)) : 0.0f;
}

bool Box3DWorld::contactRecyclingByHandle(BodyHandle h) const {
  return h && b3Body_IsContactRecyclingEnabled(b3LoadBodyId(h));
}

int Box3DWorld::recycledContactCount() const {
  return isValid() ? b3World_GetCounters(myImpl->worldId).recycledContactCount
                   : 0;
}

bool Box3DWorld::isSleepEnabledByHandle(BodyHandle h) const {
  return h && b3Body_IsSleepEnabled(b3LoadBodyId(h));
}

bool Box3DWorld::isAwakeByHandle(BodyHandle h) const {
  return h && b3Body_IsAwake(b3LoadBodyId(h));
}

float Box3DWorld::massByHandle(BodyHandle h) const {
  return h ? b3Body_GetMass(b3LoadBodyId(h)) : 0.0f;
}

int Box3DWorld::shapeCountByHandle(BodyHandle h) const {
  return h ? b3Body_GetShapeCount(b3LoadBodyId(h)) : 0;
}

bool Box3DWorld::getTransformByHandle(BodyHandle h, double outPos[3],
                                      float outRotXYZW[4]) const {
  if (!h)
    return false;
  const b3BodyId id = b3LoadBodyId(h);
  storePos(b3Body_GetPosition(id), outPos); // body origin
  storeQuat(b3Body_GetRotation(id), outRotXYZW);
  return true;
}

bool Box3DWorld::getVelocityByHandle(BodyHandle h, float outLinear[3],
                                     float outAngular[3]) const {
  if (!h)
    return false;
  const b3BodyId id = b3LoadBodyId(h);
  storeVec3(b3Body_GetLinearVelocity(id), outLinear);
  storeVec3(b3Body_GetAngularVelocity(id), outAngular);
  return true;
}

bool Box3DWorld::setVelocityByHandle(BodyHandle h, const float linear[3],
                                     const float angular[3]) {
  if (!h)
    return false;
  const b3BodyId id = b3LoadBodyId(h);
  b3Body_SetLinearVelocity(id, finiteVec3(linear));
  b3Body_SetAngularVelocity(id, finiteVec3(angular));
  return true;
}

int Box3DWorld::moveEventCount() const {
  if (!isValid())
    return 0;
  return b3World_GetBodyEvents(myImpl->worldId).moveCount;
}

bool Box3DWorld::getMoveEvent(int index, BodyMove &out) const {
  if (!isValid())
    return false;
  const b3BodyEvents ev = b3World_GetBodyEvents(myImpl->worldId);
  if (index < 0 || index >= ev.moveCount)
    return false;
  toBodyMove(ev.moveEvents[index], out);
  return true;
}

int Box3DWorld::getMoveEvents(std::vector<BodyMove> &out) const {
  if (!isValid()) {
    out.clear();
    return 0;
  }
  const b3BodyEvents ev = b3World_GetBodyEvents(myImpl->worldId);
  // Every element is overwritten, so only resize (clear + resize would zero it
  // every step).
  if (out.size() != static_cast<size_t>(ev.moveCount))
    out.resize(static_cast<size_t>(ev.moveCount));
  for (int i = 0; i < ev.moveCount; ++i)
    toBodyMove(ev.moveEvents[i], out[static_cast<size_t>(i)]);
  return ev.moveCount;
}

void Box3DWorld::setBodyUserData(BodyHandle h, void *userData) {
  if (h)
    b3Body_SetUserData(b3LoadBodyId(h), userData);
}

bool Box3DWorld::setTransformByHandle(BodyHandle h, const double pos[3],
                                      const float rotXYZW[4]) {
  if (!h)
    return false;
  b3Body_SetTransform(b3LoadBodyId(h), makePos(pos), makeQuat(rotXYZW));
  return true;
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

namespace {

// Finite and >= 0, else `fallback` (a negative value is replaced, not clamped).
float nonNegOr(float v, float fallback = 0.0f) {
  return (std::isfinite(v) && v >= 0.0f) ? v : fallback;
}

// A cap: < 0 (or non-finite) is unlimited.
float cap(float v) { return (std::isfinite(v) && v >= 0.0f) ? v : FLT_MAX; }

float finiteOr(float v, float fallback = 0.0f) {
  return std::isfinite(v) ? v : fallback;
}

// A distance joint's spring force limits and maximum length: a negative setting
// means unlimited.
float distanceLowerSpringForce(const JointSettings &s) {
  return s.maxTensionForce >= 0.0f ? -s.maxTensionForce : -FLT_MAX;
}

float distanceUpperSpringForce(const JointSettings &s) {
  return s.maxCompressionForce >= 0.0f ? s.maxCompressionForce : FLT_MAX;
}

float distanceMaxLength(const JointSettings &s) {
  return s.maxLength >= 0.0f ? s.maxLength : B3_HUGE;
}

// A local frame carried by a body at `bodyXf`, as a world-space JointFrame.
void frameToWorld(const b3WorldTransform &bodyXf, const b3Transform &local,
                  JointFrame &out) {
  const b3Vec3 d = b3RotateVector(bodyXf.q, local.p);
  out.pos[0] = static_cast<double>(bodyXf.p.x) + static_cast<double>(d.x);
  out.pos[1] = static_cast<double>(bodyXf.p.y) + static_cast<double>(d.y);
  out.pos[2] = static_cast<double>(bodyXf.p.z) + static_cast<double>(d.z);
  storeQuat(b3NormalizeQuat(b3MulQuat(bodyXf.q, local.q)), out.rot);
}

// lower <= upper, both finite.
void orderedPair(float &lo, float &hi) {
  lo = finiteOr(lo);
  hi = finiteOr(hi);
  if (lo > hi)
    std::swap(lo, hi);
}

// Every value as Box3D accepts it (its asserts would otherwise fire).
JointSettings sanitized(const JointSettings &in) {
  JointSettings s = in;
  s.constraintStiffness = nonNegOr(s.constraintStiffness, 60.0f);
  s.constraintDampingRatio = nonNegOr(s.constraintDampingRatio, 2.0f);
  s.stiffness = nonNegOr(s.stiffness);
  s.dampingRatio = nonNegOr(s.dampingRatio);
  s.linearStiffness = nonNegOr(s.linearStiffness);
  s.linearDampingRatio = nonNegOr(s.linearDampingRatio);
  s.angularStiffness = nonNegOr(s.angularStiffness);
  s.angularDampingRatio = nonNegOr(s.angularDampingRatio);
  s.suspensionStiffness = nonNegOr(s.suspensionStiffness);
  s.suspensionDampingRatio = nonNegOr(s.suspensionDampingRatio);
  s.steeringStiffness = nonNegOr(s.steeringStiffness);
  s.steeringDampingRatio = nonNegOr(s.steeringDampingRatio);
  s.maxMotorTorque = nonNegOr(s.maxMotorTorque);
  s.maxMotorForce = nonNegOr(s.maxMotorForce);
  s.maxVelocityForce = nonNegOr(s.maxVelocityForce);
  s.maxVelocityTorque = nonNegOr(s.maxVelocityTorque);
  s.maxSpringForce = nonNegOr(s.maxSpringForce);
  s.maxSpringTorque = nonNegOr(s.maxSpringTorque);
  s.maxSpinTorque = nonNegOr(s.maxSpinTorque);
  s.maxSteeringTorque = nonNegOr(s.maxSteeringTorque);
  s.motorSpeed = finiteOr(s.motorSpeed);
  s.spinSpeed = finiteOr(s.spinSpeed);
  s.targetTranslation = finiteOr(s.targetTranslation);
  s.targetSteeringAngle = finiteOr(s.targetSteeringAngle);
  s.targetAngle = std::clamp(finiteOr(s.targetAngle), -B3_PI, B3_PI);
  s.coneAngle = std::clamp(finiteOr(s.coneAngle), 0.0f, 0.5f * B3_PI);
  orderedPair(s.lowerAngle, s.upperAngle);
  orderedPair(s.lowerTranslation, s.upperTranslation);
  orderedPair(s.lowerTwistAngle, s.upperTwistAngle);
  orderedPair(s.lowerSuspensionLimit, s.upperSuspensionLimit);
  orderedPair(s.lowerSteeringLimit, s.upperSteeringLimit);
  s.minLength = nonNegOr(s.minLength);
  if (s.maxLength >= 0.0f && std::isfinite(s.maxLength) &&
      s.maxLength < s.minLength)
    s.maxLength = s.minLength;
  sanitizeQuat(s.targetRotation);
  return s;
}

// The base definition every joint shares.
void fillBase(b3JointDef &base, const JointSettings &s, b3BodyId a, b3BodyId b,
              const b3Transform &fa, const b3Transform &fb, void *userData) {
  base.bodyIdA = a;
  base.bodyIdB = b;
  base.localFrameA = fa;
  base.localFrameB = fb;
  base.forceThreshold = cap(s.forceThreshold);
  base.torqueThreshold = cap(s.torqueThreshold);
  base.constraintHertz = s.constraintStiffness;
  base.constraintDampingRatio = s.constraintDampingRatio;
  base.collideConnected = s.collideConnected;
  base.userData = userData;
}

// A world frame relative to a body's current transform.
b3Transform toLocal(b3BodyId body, const JointFrame &f) {
  float q[4] = {f.rot[0], f.rot[1], f.rot[2], f.rot[3]};
  sanitizeQuat(q);
  const b3WorldTransform t = b3Body_GetTransform(body);
  b3Vec3 d;
  d.x = static_cast<float>(f.pos[0] - static_cast<double>(t.p.x));
  d.y = static_cast<float>(f.pos[1] - static_cast<double>(t.p.y));
  d.z = static_cast<float>(f.pos[2] - static_cast<double>(t.p.z));
  b3Transform out;
  out.p = b3InvRotateVector(t.q, d);
  out.q = b3NormalizeQuat(b3InvMulQuat(t.q, makeQuat(q)));
  return out;
}

// A frame already relative to its body. Kept bit for bit when already unit
// length (a stored local frame must come back exact).
b3Transform asLocalFrame(const JointFrame &f) {
  float q[4] = {f.rot[0], f.rot[1], f.rot[2], f.rot[3]};
  const double len2 =
      static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
      static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3];
  if (!(std::fabs(len2 - 1.0) < 1e-6))
    sanitizeQuat(q);
  b3Transform t;
  t.p.x = static_cast<float>(f.pos[0]);
  t.p.y = static_cast<float>(f.pos[1]);
  t.p.z = static_cast<float>(f.pos[2]);
  t.q = makeQuat(q);
  return t;
}

// What every per-type create function needs: the world, the sanitized settings,
// both bodies with their local frames, and (distance joints) the rest length to
// make the joint with.
struct JointBuild {
  b3WorldId world;
  const JointSettings &s;
  b3BodyId a, b;
  b3Transform fa, fb;
  void *userData;
  float distanceLength = 0.0f;

  void fillBase(b3JointDef &base) const {
    ::b3dop::fillBase(base, s, a, b, fa, fb, userData);
  }
};

b3JointId createWeldJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3WeldJointDef d = b3DefaultWeldJointDef();
  j.fillBase(d.base);
  d.linearHertz = s.enableLinearSpring ? s.linearStiffness : 0.0f; // 0: rigid
  d.linearDampingRatio = s.linearDampingRatio;
  d.angularHertz = s.enableAngularSpring ? s.angularStiffness : 0.0f;
  d.angularDampingRatio = s.angularDampingRatio;
  return b3CreateWeldJoint(j.world, &d);
}

b3JointId createSphericalJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3SphericalJointDef d = b3DefaultSphericalJointDef();
  j.fillBase(d.base);
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.targetRotation = makeQuat(s.targetRotation);
  d.enableConeLimit = s.enableConeLimit;
  d.coneAngle = s.coneAngle;
  d.enableTwistLimit = s.enableTwistLimit;
  d.lowerTwistAngle = s.lowerTwistAngle;
  d.upperTwistAngle = s.upperTwistAngle;
  d.enableMotor = s.enableMotor;
  d.maxMotorTorque = s.maxMotorTorque;
  d.motorVelocity = finiteVec3(s.motorVelocity);
  return b3CreateSphericalJoint(j.world, &d);
}

b3JointId createRevoluteJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
  j.fillBase(d.base);
  d.targetAngle = s.targetAngle;
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.enableLimit = s.enableLimit;
  d.lowerAngle = s.lowerAngle;
  d.upperAngle = s.upperAngle;
  d.enableMotor = s.enableMotor;
  d.maxMotorTorque = s.maxMotorTorque;
  d.motorSpeed = s.motorSpeed;
  return b3CreateRevoluteJoint(j.world, &d);
}

b3JointId createPrismaticJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3PrismaticJointDef d = b3DefaultPrismaticJointDef();
  j.fillBase(d.base);
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.targetTranslation = s.targetTranslation;
  d.enableLimit = s.enableLimit;
  d.lowerTranslation = s.lowerTranslation;
  d.upperTranslation = s.upperTranslation;
  d.enableMotor = s.enableMotor;
  d.maxMotorForce = s.maxMotorForce;
  d.motorSpeed = s.motorSpeed;
  return b3CreatePrismaticJoint(j.world, &d);
}

b3JointId createDistanceJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3DistanceJointDef d = b3DefaultDistanceJointDef();
  j.fillBase(d.base);
  d.length = std::max(j.distanceLength,
                      1e-4f); // Box3D wants > 0 (and clamps to its slop)
  d.enableSpring = s.enableSpring;
  d.lowerSpringForce = distanceLowerSpringForce(s);
  d.upperSpringForce = distanceUpperSpringForce(s);
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.enableLimit = s.enableLimit;
  d.minLength = s.minLength;
  d.maxLength = distanceMaxLength(s);
  d.enableMotor = s.enableMotor;
  d.maxMotorForce = s.maxMotorForce;
  d.motorSpeed = s.motorSpeed;
  return b3CreateDistanceJoint(j.world, &d);
}

b3JointId createMotorJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3MotorJointDef d = b3DefaultMotorJointDef();
  j.fillBase(d.base);
  d.linearVelocity = finiteVec3(s.linearVelocity);
  d.maxVelocityForce = s.maxVelocityForce;
  d.angularVelocity = finiteVec3(s.angularVelocity);
  d.maxVelocityTorque = s.maxVelocityTorque;
  d.linearHertz = s.linearStiffness;
  d.linearDampingRatio = s.linearDampingRatio;
  d.maxSpringForce = s.maxSpringForce;
  d.angularHertz = s.angularStiffness;
  d.angularDampingRatio = s.angularDampingRatio;
  d.maxSpringTorque = s.maxSpringTorque;
  return b3CreateMotorJoint(j.world, &d);
}

b3JointId createParallelJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3ParallelJointDef d = b3DefaultParallelJointDef();
  j.fillBase(d.base);
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.maxTorque = cap(s.maxTorque);
  return b3CreateParallelJoint(j.world, &d);
}

b3JointId createWheelJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3WheelJointDef d = b3DefaultWheelJointDef();
  j.fillBase(d.base);
  d.enableSuspensionSpring = s.enableSuspensionSpring;
  d.suspensionHertz = s.suspensionStiffness;
  d.suspensionDampingRatio = s.suspensionDampingRatio;
  d.enableSuspensionLimit = s.enableSuspensionLimit;
  d.lowerSuspensionLimit = s.lowerSuspensionLimit;
  d.upperSuspensionLimit = s.upperSuspensionLimit;
  d.enableSpinMotor = s.enableSpinMotor;
  d.maxSpinTorque = s.maxSpinTorque;
  d.spinSpeed = s.spinSpeed;
  d.enableSteering = s.enableSteering;
  d.steeringHertz = s.steeringStiffness;
  d.steeringDampingRatio = s.steeringDampingRatio;
  d.targetSteeringAngle = s.targetSteeringAngle;
  d.maxSteeringTorque = s.maxSteeringTorque;
  d.enableSteeringLimit = s.enableSteeringLimit;
  d.lowerSteeringLimit = s.lowerSteeringLimit;
  d.upperSteeringLimit = s.upperSteeringLimit;
  return b3CreateWheelJoint(j.world, &d);
}

b3JointId createFilterJoint(const JointBuild &j) {
  b3FilterJointDef d = b3DefaultFilterJointDef();
  j.fillBase(d.base);
  d.base.collideConnected = false; // its whole purpose
  return b3CreateFilterJoint(j.world, &d);
}

b3JointId createJointOfType(JointType type, const JointBuild &j) {
  switch (type) {
  case JointType::Weld:
    return createWeldJoint(j);
  case JointType::Spherical:
    return createSphericalJoint(j);
  case JointType::Revolute:
    return createRevoluteJoint(j);
  case JointType::Prismatic:
    return createPrismaticJoint(j);
  case JointType::Distance:
    return createDistanceJoint(j);
  case JointType::Motor:
    return createMotorJoint(j);
  case JointType::Parallel:
    return createParallelJoint(j);
  case JointType::Wheel:
    return createWheelJoint(j);
  case JointType::Filter:
    return createFilterJoint(j);
  }
  return b3_nullJointId;
}

} // namespace

JointSettings Box3DWorld::defaultJointSettings(JointType type) {
  JointSettings s;
  const b3JointDef *base = nullptr;
  // Each b3Default*JointDef, copied field by field. Box3D's FLT_MAX caps are
  // our -1 (unlimited).
  auto unlimited = [](float v) { return v >= 0.5f * FLT_MAX ? -1.0f : v; };
  switch (type) {
  case JointType::Weld: {
    static const b3WeldJointDef d = b3DefaultWeldJointDef();
    base = &d.base;
    s.enableLinearSpring = d.linearHertz > 0.0f;
    s.enableAngularSpring = d.angularHertz > 0.0f;
    s.linearStiffness = d.linearHertz;
    s.linearDampingRatio = d.linearDampingRatio;
    s.angularStiffness = d.angularHertz;
    s.angularDampingRatio = d.angularDampingRatio;
    break;
  }
  case JointType::Spherical: {
    static const b3SphericalJointDef d = b3DefaultSphericalJointDef();
    base = &d.base;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.targetRotation[0] = d.targetRotation.v.x;
    s.targetRotation[1] = d.targetRotation.v.y;
    s.targetRotation[2] = d.targetRotation.v.z;
    s.targetRotation[3] = d.targetRotation.s;
    s.enableConeLimit = d.enableConeLimit;
    s.coneAngle = d.coneAngle;
    s.enableTwistLimit = d.enableTwistLimit;
    s.lowerTwistAngle = d.lowerTwistAngle;
    s.upperTwistAngle = d.upperTwistAngle;
    s.enableMotor = d.enableMotor;
    s.maxMotorTorque = d.maxMotorTorque;
    storeVec3(d.motorVelocity, s.motorVelocity);
    break;
  }
  case JointType::Revolute: {
    static const b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
    base = &d.base;
    s.targetAngle = d.targetAngle;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.enableLimit = d.enableLimit;
    s.lowerAngle = d.lowerAngle;
    s.upperAngle = d.upperAngle;
    s.enableMotor = d.enableMotor;
    s.maxMotorTorque = d.maxMotorTorque;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Prismatic: {
    static const b3PrismaticJointDef d = b3DefaultPrismaticJointDef();
    base = &d.base;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.targetTranslation = d.targetTranslation;
    s.enableLimit = d.enableLimit;
    s.lowerTranslation = d.lowerTranslation;
    s.upperTranslation = d.upperTranslation;
    s.enableMotor = d.enableMotor;
    s.maxMotorForce = d.maxMotorForce;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Distance: {
    static const b3DistanceJointDef d = b3DefaultDistanceJointDef();
    base = &d.base;
    s.restLength = -1.0f; // ours: the anchors' distance at creation
    s.enableSpring = d.enableSpring;
    s.maxTensionForce = unlimited(-d.lowerSpringForce);
    s.maxCompressionForce = unlimited(d.upperSpringForce);
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.enableLimit = d.enableLimit;
    s.minLength = d.minLength;
    s.maxLength = d.maxLength >= 0.5f * B3_HUGE ? -1.0f : d.maxLength;
    s.enableMotor = d.enableMotor;
    s.maxMotorForce = d.maxMotorForce;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Motor: {
    static const b3MotorJointDef d = b3DefaultMotorJointDef();
    base = &d.base;
    storeVec3(d.linearVelocity, s.linearVelocity);
    s.maxVelocityForce = d.maxVelocityForce;
    storeVec3(d.angularVelocity, s.angularVelocity);
    s.maxVelocityTorque = d.maxVelocityTorque;
    s.linearStiffness = d.linearHertz;
    s.linearDampingRatio = d.linearDampingRatio;
    s.maxSpringForce = d.maxSpringForce;
    s.angularStiffness = d.angularHertz;
    s.angularDampingRatio = d.angularDampingRatio;
    s.maxSpringTorque = d.maxSpringTorque;
    break;
  }
  case JointType::Parallel: {
    static const b3ParallelJointDef d = b3DefaultParallelJointDef();
    base = &d.base;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.maxTorque = unlimited(d.maxTorque);
    break;
  }
  case JointType::Wheel: {
    static const b3WheelJointDef d = b3DefaultWheelJointDef();
    base = &d.base;
    s.enableSuspensionSpring = d.enableSuspensionSpring;
    s.suspensionStiffness = d.suspensionHertz;
    s.suspensionDampingRatio = d.suspensionDampingRatio;
    s.enableSuspensionLimit = d.enableSuspensionLimit;
    s.lowerSuspensionLimit = d.lowerSuspensionLimit;
    s.upperSuspensionLimit = d.upperSuspensionLimit;
    s.enableSpinMotor = d.enableSpinMotor;
    s.maxSpinTorque = d.maxSpinTorque;
    s.spinSpeed = d.spinSpeed;
    s.enableSteering = d.enableSteering;
    s.steeringStiffness = d.steeringHertz;
    s.steeringDampingRatio = d.steeringDampingRatio;
    s.targetSteeringAngle = d.targetSteeringAngle;
    s.maxSteeringTorque = d.maxSteeringTorque;
    s.enableSteeringLimit = d.enableSteeringLimit;
    s.lowerSteeringLimit = d.lowerSteeringLimit;
    s.upperSteeringLimit = d.upperSteeringLimit;
    break;
  }
  case JointType::Filter: {
    static const b3FilterJointDef d = b3DefaultFilterJointDef();
    base = &d.base;
    break;
  }
  }
  if (base) {
    s.collideConnected = base->collideConnected;
    s.forceThreshold = unlimited(base->forceThreshold);
    s.torqueThreshold = unlimited(base->torqueThreshold);
    s.constraintStiffness = base->constraintHertz;
    s.constraintDampingRatio = base->constraintDampingRatio;
  }
  return s;
}

JointHandle Box3DWorld::createJoint(JointType type, BodyHandle a, BodyHandle b,
                                    const JointFrame &frameA,
                                    const JointFrame &frameB,
                                    const JointSettings &in, void *userData,
                                    bool framesAreLocal) {
  if (!isValid() || (!a && !b) || a == b)
    return 0;
  if (!std::isfinite(frameA.pos[0]) || !std::isfinite(frameA.pos[1]) ||
      !std::isfinite(frameA.pos[2]) || !std::isfinite(frameB.pos[0]) ||
      !std::isfinite(frameB.pos[1]) || !std::isfinite(frameB.pos[2]))
    return 0;
  auto bodyOf = [&](BodyHandle h) -> b3BodyId {
    if (h)
      return b3LoadBodyId(h);
    if (B3_IS_NULL(myImpl->anchor)) {
      b3BodyDef def = b3DefaultBodyDef();
      def.type = b3_staticBody;
      myImpl->anchor = b3CreateBody(myImpl->worldId, &def);
    }
    return myImpl->anchor;
  };
  const b3BodyId ida = bodyOf(a), idb = bodyOf(b);
  if (!b3Body_IsValid(ida) || !b3Body_IsValid(idb))
    return 0;

  const JointSettings s = sanitized(in);
  const b3Transform fa =
      framesAreLocal ? asLocalFrame(frameA) : toLocal(ida, frameA);
  const b3Transform fb =
      framesAreLocal ? asLocalFrame(frameB) : toLocal(idb, frameB);
  if (!b3IsValidVec3(fa.p) || !b3IsValidVec3(fb.p))
    return 0;

  JointBuild build{myImpl->worldId, s, ida, idb, fa, fb, userData};
  if (type == JointType::Distance) {
    float len = s.restLength;
    if (!(len >= 0.0f) || !std::isfinite(len)) {
      // The anchors' distance now (from the local frames: through the bodies'
      // poses).
      JointFrame wa = frameA, wb = frameB;
      if (framesAreLocal) {
        localToWorld(a, frameA, wa);
        localToWorld(b, frameB, wb);
      }
      const double dx = wb.pos[0] - wa.pos[0], dy = wb.pos[1] - wa.pos[1],
                   dz = wb.pos[2] - wa.pos[2];
      len = static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    build.distanceLength = len;
  }
  const b3JointId id = createJointOfType(type, build);
  return B3_IS_NULL(id) ? 0 : b3StoreJointId(id);
}

void Box3DWorld::destroyJoint(JointHandle h) {
  if (isJointValid(h))
    b3DestroyJoint(b3LoadJointId(h), true);
}

bool Box3DWorld::isJointValid(JointHandle h) const {
  return h && isValid() && b3Joint_IsValid(b3LoadJointId(h));
}

void Box3DWorld::updateJoint(JointHandle h, JointType type,
                             const JointSettings &in) {
  if (!isJointValid(h))
    return;
  const b3JointId id = b3LoadJointId(h);
  const JointSettings s = sanitized(in);
  b3Joint_SetCollideConnected(
      id, type == JointType::Filter ? false : s.collideConnected);
  b3Joint_SetForceThreshold(id, cap(s.forceThreshold));
  b3Joint_SetTorqueThreshold(id, cap(s.torqueThreshold));
  b3Joint_SetConstraintTuning(id, s.constraintStiffness,
                              s.constraintDampingRatio);
  switch (type) {
  case JointType::Weld:
    b3WeldJoint_SetLinearHertz(id,
                               s.enableLinearSpring ? s.linearStiffness : 0.0f);
    b3WeldJoint_SetLinearDampingRatio(id, s.linearDampingRatio);
    b3WeldJoint_SetAngularHertz(id, s.enableAngularSpring ? s.angularStiffness
                                                          : 0.0f);
    b3WeldJoint_SetAngularDampingRatio(id, s.angularDampingRatio);
    break;
  case JointType::Spherical:
    b3SphericalJoint_EnableSpring(id, s.enableSpring);
    b3SphericalJoint_SetSpringHertz(id, s.stiffness);
    b3SphericalJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3SphericalJoint_SetTargetRotation(id, makeQuat(s.targetRotation));
    b3SphericalJoint_EnableConeLimit(id, s.enableConeLimit);
    b3SphericalJoint_SetConeLimit(id, s.coneAngle);
    b3SphericalJoint_EnableTwistLimit(id, s.enableTwistLimit);
    b3SphericalJoint_SetTwistLimits(id, s.lowerTwistAngle, s.upperTwistAngle);
    b3SphericalJoint_EnableMotor(id, s.enableMotor);
    b3SphericalJoint_SetMotorVelocity(id, finiteVec3(s.motorVelocity));
    b3SphericalJoint_SetMaxMotorTorque(id, s.maxMotorTorque);
    break;
  case JointType::Revolute:
    b3RevoluteJoint_EnableSpring(id, s.enableSpring);
    b3RevoluteJoint_SetSpringHertz(id, s.stiffness);
    b3RevoluteJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3RevoluteJoint_SetTargetAngle(id, s.targetAngle);
    b3RevoluteJoint_EnableLimit(id, s.enableLimit);
    b3RevoluteJoint_SetLimits(id, s.lowerAngle, s.upperAngle);
    b3RevoluteJoint_EnableMotor(id, s.enableMotor);
    b3RevoluteJoint_SetMotorSpeed(id, s.motorSpeed);
    b3RevoluteJoint_SetMaxMotorTorque(id, s.maxMotorTorque);
    break;
  case JointType::Prismatic:
    b3PrismaticJoint_EnableSpring(id, s.enableSpring);
    b3PrismaticJoint_SetSpringHertz(id, s.stiffness);
    b3PrismaticJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3PrismaticJoint_SetTargetTranslation(id, s.targetTranslation);
    b3PrismaticJoint_EnableLimit(id, s.enableLimit);
    b3PrismaticJoint_SetLimits(id, s.lowerTranslation, s.upperTranslation);
    b3PrismaticJoint_EnableMotor(id, s.enableMotor);
    b3PrismaticJoint_SetMotorSpeed(id, s.motorSpeed);
    b3PrismaticJoint_SetMaxMotorForce(id, s.maxMotorForce);
    break;
  case JointType::Distance:
    if (s.restLength >= 0.0f &&
        std::isfinite(s.restLength)) // else keep the length it was made with
      b3DistanceJoint_SetLength(id, std::max(s.restLength, 1e-4f));
    b3DistanceJoint_EnableSpring(id, s.enableSpring);
    b3DistanceJoint_SetSpringForceRange(id, distanceLowerSpringForce(s),
                                        distanceUpperSpringForce(s));
    b3DistanceJoint_SetSpringHertz(id, s.stiffness);
    b3DistanceJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3DistanceJoint_EnableLimit(id, s.enableLimit);
    b3DistanceJoint_SetLengthRange(id, s.minLength, distanceMaxLength(s));
    b3DistanceJoint_EnableMotor(id, s.enableMotor);
    b3DistanceJoint_SetMotorSpeed(id, s.motorSpeed);
    b3DistanceJoint_SetMaxMotorForce(id, s.maxMotorForce);
    break;
  case JointType::Motor:
    b3MotorJoint_SetLinearVelocity(id, finiteVec3(s.linearVelocity));
    b3MotorJoint_SetMaxVelocityForce(id, s.maxVelocityForce);
    b3MotorJoint_SetAngularVelocity(id, finiteVec3(s.angularVelocity));
    b3MotorJoint_SetMaxVelocityTorque(id, s.maxVelocityTorque);
    b3MotorJoint_SetLinearHertz(id, s.linearStiffness);
    b3MotorJoint_SetLinearDampingRatio(id, s.linearDampingRatio);
    b3MotorJoint_SetMaxSpringForce(id, s.maxSpringForce);
    b3MotorJoint_SetAngularHertz(id, s.angularStiffness);
    b3MotorJoint_SetAngularDampingRatio(id, s.angularDampingRatio);
    b3MotorJoint_SetMaxSpringTorque(id, s.maxSpringTorque);
    break;
  case JointType::Parallel:
    b3ParallelJoint_SetSpringHertz(id, s.stiffness);
    b3ParallelJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3ParallelJoint_SetMaxTorque(id, cap(s.maxTorque));
    break;
  case JointType::Wheel:
    b3WheelJoint_EnableSuspension(id, s.enableSuspensionSpring);
    b3WheelJoint_SetSuspensionHertz(id, s.suspensionStiffness);
    b3WheelJoint_SetSuspensionDampingRatio(id, s.suspensionDampingRatio);
    b3WheelJoint_EnableSuspensionLimit(id, s.enableSuspensionLimit);
    b3WheelJoint_SetSuspensionLimits(id, s.lowerSuspensionLimit,
                                     s.upperSuspensionLimit);
    b3WheelJoint_EnableSpinMotor(id, s.enableSpinMotor);
    b3WheelJoint_SetSpinMotorSpeed(id, s.spinSpeed);
    b3WheelJoint_SetMaxSpinTorque(id, s.maxSpinTorque);
    b3WheelJoint_EnableSteering(id, s.enableSteering);
    b3WheelJoint_SetSteeringHertz(id, s.steeringStiffness);
    b3WheelJoint_SetSteeringDampingRatio(id, s.steeringDampingRatio);
    b3WheelJoint_SetTargetSteeringAngle(id, s.targetSteeringAngle);
    b3WheelJoint_SetMaxSteeringTorque(id, s.maxSteeringTorque);
    b3WheelJoint_EnableSteeringLimit(id, s.enableSteeringLimit);
    b3WheelJoint_SetSteeringLimits(id, s.lowerSteeringLimit,
                                   s.upperSteeringLimit);
    break;
  case JointType::Filter:
    break;
  }
  b3Joint_WakeBodies(id);
}

bool Box3DWorld::jointFeedback(JointHandle h, JointType type,
                               JointFeedback &out) const {
  out = JointFeedback();
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  storeVec3(b3Joint_GetConstraintForce(id), out.force);
  storeVec3(b3Joint_GetConstraintTorque(id), out.torque);
  out.linearSeparation = b3Joint_GetLinearSeparation(id);
  // Box3D has not implemented the wheel's angular separation yet (a todo that
  // asserts): 0.
  out.angularSeparation =
      type == JointType::Wheel ? 0.0f : b3Joint_GetAngularSeparation(id);
  auto extra = [&](const char *name, float v, bool angle) {
    if (out.extraCount >= JointFeedback::kMaxExtra)
      return;
    out.extraName[out.extraCount] = name;
    out.extra[out.extraCount] = v;
    out.extraIsAngle[out.extraCount] = angle;
    ++out.extraCount;
  };
  switch (type) {
  case JointType::Spherical:
    extra("current_cone_angle", b3SphericalJoint_GetConeAngle(id), true);
    extra("current_twist_angle", b3SphericalJoint_GetTwistAngle(id), true);
    extra("motor_torque", b3Length(b3SphericalJoint_GetMotorTorque(id)),
          false); // a vector: its size
    break;
  case JointType::Revolute:
    extra("current_angle", b3RevoluteJoint_GetAngle(id), true);
    extra("motor_torque", b3RevoluteJoint_GetMotorTorque(id), false);
    break;
  case JointType::Prismatic:
    extra("current_translation", b3PrismaticJoint_GetTranslation(id), false);
    extra("current_speed", b3PrismaticJoint_GetSpeed(id), false);
    extra("motor_force", b3PrismaticJoint_GetMotorForce(id), false);
    break;
  case JointType::Distance:
    extra("current_length", b3DistanceJoint_GetCurrentLength(id), false);
    extra("motor_force", b3DistanceJoint_GetMotorForce(id), false);
    break;
  case JointType::Wheel:
    extra("current_spin_speed", b3WheelJoint_GetSpinSpeed(id), true);
    extra("current_steering_angle", b3WheelJoint_GetSteeringAngle(id), true);
    extra("spin_torque", b3WheelJoint_GetSpinTorque(id), false);
    break;
  default:
    break;
  }
  return true;
}

bool Box3DWorld::jointWorldFrames(JointHandle h, JointFrame &a,
                                  JointFrame &b) const {
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  frameToWorld(b3Body_GetTransform(b3Joint_GetBodyA(id)),
               b3Joint_GetLocalFrameA(id), a);
  frameToWorld(b3Body_GetTransform(b3Joint_GetBodyB(id)),
               b3Joint_GetLocalFrameB(id), b);
  return true;
}

bool Box3DWorld::jointLocalFrames(JointHandle h, JointFrame &a,
                                  JointFrame &b) const {
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  auto store = [](const b3Transform &t, JointFrame &out) {
    out.pos[0] = t.p.x;
    out.pos[1] = t.p.y;
    out.pos[2] = t.p.z;
    storeQuat(t.q, out.rot);
  };
  store(b3Joint_GetLocalFrameA(id), a);
  store(b3Joint_GetLocalFrameB(id), b);
  return true;
}

bool Box3DWorld::localToWorld(BodyHandle h, const JointFrame &local,
                              JointFrame &out) const {
  if (!isValid())
    return false;
  if (!h) {
    out = local; // the world body sits at the origin, unrotated
    return true;
  }
  const b3BodyId id = b3LoadBodyId(h);
  if (!b3Body_IsValid(id))
    return false;
  float q[4] = {local.rot[0], local.rot[1], local.rot[2], local.rot[3]};
  sanitizeQuat(q);
  b3Transform lt;
  lt.p.x = static_cast<float>(local.pos[0]);
  lt.p.y = static_cast<float>(local.pos[1]);
  lt.p.z = static_cast<float>(local.pos[2]);
  lt.q = makeQuat(q);
  frameToWorld(b3Body_GetTransform(id), lt, out);
  return true;
}

int Box3DWorld::brokenJoints(std::vector<void *> &userData) const {
  userData.clear();
  if (!isValid())
    return 0;
  const b3JointEvents ev = b3World_GetJointEvents(myImpl->worldId);
  for (int i = 0; i < ev.count; ++i)
    userData.push_back(ev.jointEvents[i].userData);
  return static_cast<int>(userData.size());
}

int Box3DWorld::jointCount() const {
  return isValid() ? b3World_GetCounters(myImpl->worldId).jointCount : 0;
}

void Box3DWorld::version(int &major, int &minor, int &revision) {
  const b3Version v = b3GetVersion();
  major = v.major;
  minor = v.minor;
  revision = v.revision;
}

int Box3DWorld::assertCount() { return gAssertCount.load(); }

int Box3DWorld::liveWorldCount() { return b3GetWorldCount(); }

int Box3DWorld::maxWorldCount() { return B3_MAX_WORLDS; }

} // namespace b3dop
