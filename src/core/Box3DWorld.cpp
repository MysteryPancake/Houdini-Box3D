// Box3DWorld.cpp
#include "Box3DDebug.h"
#include "Box3DShapes.h"
#include "Box3DWorldInternal.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace b3dop {

static_assert(
    kMaxWorkerCount == B3_MAX_WORKERS,
    "kMaxWorkerCount (Box3DWorld.h) must match Box3D's B3_MAX_WORKERS");

namespace {

// Every shape of a body (newest first), into a reused scratch vector.
int bodyShapes(b3BodyId id, std::vector<b3ShapeId> &out) {
  const int n = b3Body_GetShapeCount(id);
  out.resize(static_cast<size_t>(n > 0 ? n : 0));
  return n > 0 ? b3Body_GetShapes(id, out.data(), n) : 0;
}

// Shape id -> creation index of the first `got` of `shapes` (newest first),
// sorted by id and searched with findShapeIndex. (A hash map cleared for every
// call costs its bucket count, which a large compound left behind.)
using ShapeIndex = std::vector<std::pair<uint64_t, int>>;

void indexShapes(const std::vector<b3ShapeId> &shapes, int got,
                 ShapeIndex &out) {
  out.clear();
  for (int i = 0; i < got; ++i)
    out.emplace_back(b3StoreShapeId(shapes[static_cast<size_t>(i)]),
                     got - 1 - i);
  std::sort(out.begin(), out.end());
}

// The entry of `key` in `index`, or null.
const std::pair<uint64_t, int> *findShapeIndex(const ShapeIndex &index,
                                               uint64_t key) {
  const auto it = std::lower_bound(index.begin(), index.end(), key,
                                   [](const std::pair<uint64_t, int> &e,
                                      uint64_t k) { return e.first < k; });
  return it == index.end() || it->first != key ? nullptr : &*it;
}

// ---- the process-wide hull cache
// Box3D adds a hull to the world's own database when a shape is made (identical
// hulls share one clone), so entries here are only needed until then: they can
// be shared by every world (a rebuilt world finds what its predecessor built)
// and evicted at any time. Worlds hold the entries they use in a step
// (Box3DWorld::Impl::held) so eviction never frees one between findHull and
// createHullShape.
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
    // The hull's whole allocation: header, vertex / point / edge / plane / face
    // arrays, the SoA vertex and normal arrays and the edge cosines.
    fresh->bytes = static_cast<size_t>(hull->byteCount);
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

  // Keys whose points cannot be hulled (flat, too complex): a key always means
  // the same points, so they are not tried again.
  bool knownFailure(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    return myFailed.count(key) != 0;
  }
  void noteFailure(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    if (myFailed.size() >= kMaxFailedKeys)
      myFailed.clear();
    myFailed.insert(key);
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
  static constexpr size_t kMaxFailedKeys = 65536;
  std::unordered_set<HullId> myFailed;
  size_t myBytes = 0;
  size_t myBudget = static_cast<size_t>(256) << 20;
  uint64_t myTick = 0;
};

// ---- the process-wide fit cache ---------------------------------------------
// What fitShapeOriented found for a point set (by content and shape type, or
// under a caller's key: findFit / storeFit), so many packed instances of one
// piece fit once. Half is dropped when it grows too big.
class FitLibrary {
public:
  static FitLibrary &instance() {
    static FitLibrary lib;
    return lib;
  }
  // Under a shared lock: every packed instance of a fresh scan asks, from
  // several threads, and a lookup changes nothing.
  bool find(uint64_t key, ShapeSpec &out) {
    // The thread's last hit first (a fit is a function of its key, so it never
    // changes), skipping the shared lock every thread would take per point.
    thread_local uint64_t memoKey = 0;
    thread_local ShapeSpec memo;
    if (memoKey == key && key != 0) {
      out = memo;
      return true;
    }
    std::shared_lock<std::shared_mutex> lock(myMutex);
    const auto it = myEntries.find(key);
    if (it == myEntries.end())
      return false;
    memoKey = key;
    memo = it->second;
    out = it->second;
    return true;
  }
  void insert(uint64_t key, const ShapeSpec &spec) {
    std::unique_lock<std::shared_mutex> lock(myMutex);
    if (myEntries.size() >= kMaxFitEntries) {
      // Half goes (in the map's order), so a scene past the cap keeps most of
      // what it just used instead of losing all of it.
      for (auto it = myEntries.begin();
           it != myEntries.end() && myEntries.size() > kMaxFitEntries / 2;)
        it = myEntries.erase(it);
    }
    myEntries.emplace(key, spec);
  }

private:
  static constexpr size_t kMaxFitEntries = 262144;
  std::shared_mutex myMutex;
  std::unordered_map<uint64_t, ShapeSpec> myEntries;
};

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
    B3D_DEBUG_LOG(
        "Box3D assertion failed (ignored, simulation continues): %s, %s, "
        "line %d",
        condition ? condition : "?", fileName ? fileName : "?", lineNumber);
  if (n == kMaxAssertLogs)
    B3D_DEBUG_LOG("further Box3D assertion messages are not printed");
  return 0;
}

void logHandler(const char *message) {
  B3D_DEBUG_LOG("%s (from Box3D)", message ? message : "?");
}

// Box3D's world list is not thread-safe: creating or destroying worlds from
// several threads (two DOP networks cooking at once, a cache freeing old
// worlds) must not overlap. Never held while stepping.
std::mutex &worldListMutex() {
  static std::mutex m;
  return m;
}

void installAssertHandler() {
  static std::once_flag once;
  std::call_once(once, [] {
    b3SetAssertFcn(&assertHandler);
    b3SetLogFcn(&logHandler);
  });
}

// b3Pos is float, or double with BOX3D_DOUBLE_PRECISION; these work either way.
b3Pos makePos(const double p[3]) {
  b3Pos r;
  r.x = static_cast<decltype(r.x)>(p[0]);
  r.y = static_cast<decltype(r.y)>(p[1]);
  r.z = static_cast<decltype(r.z)>(p[2]);
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

// A unit quaternion from any input (zero, non-finite or off-unit values become
// a valid rotation): Box3D asserts on anything else.
b3Quat cleanQuat(const float q[4]) {
  float c[4] = {q[0], q[1], q[2], q[3]};
  sanitizeQuat(c);
  return makeQuat(c);
}

void storePos(const b3Pos &p, double out[3]) {
  out[0] = static_cast<double>(p.x);
  out[1] = static_cast<double>(p.y);
  out[2] = static_cast<double>(p.z);
}

void toBodyMove(const b3BodyMoveEvent &e, BodyMove &out) {
  out.userData = e.userData;
  out.handle = b3StoreBodyId(e.bodyId);
  storePos(e.transform.p, out.pos);
  storeQuat(e.transform.q, out.rot);
  storeVec3(e.linearVelocity, out.lin);
  storeVec3(e.angularVelocity, out.ang);
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

// The points as b3Vec3. False if any is not finite (Box3D asserts on them).
bool loadPoints(const float *xyz, int count, std::vector<b3Vec3> &pts) {
  for (int i = 0; i < count * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return false;
  pts.resize(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
    pts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);
  return true;
}

// b3CreateHull fails for degenerate input or when the result would exceed
// Box3D's vertex / face / edge limits; a smaller vertex budget keeps fewer
// vertices, so dense input still gets a (slightly smaller) hull. A
// triangulated hull hits the 256 half-edge limit at about 44 vertices, which
// is why there is no rung between the maximum and 44: a 64-vertex budget
// fails whenever the maximum did.
constexpr int kHullBudgets[] = {B3_MAX_HULL_VERTICES, 44, 32, 16, 8};

// The hull with the largest budget that works, skipping budgets above
// `maxBudget` (0: none skipped). `usedBudget` is the budget that worked,
// `tries` the b3CreateHull calls made. Null if none works.
b3HullData *createHullLadder(const std::vector<b3Vec3> &pts, int maxBudget,
                             int &usedBudget, int &tries) {
  usedBudget = 0;
  tries = 0;
  for (const int budget : kHullBudgets) {
    if (maxBudget && budget > maxBudget)
      continue;
    ++tries;
    b3HullData *hull =
        b3CreateHull(pts.data(), static_cast<int>(pts.size()), budget);
    if (hull) {
      usedBudget = budget;
      return hull;
    }
  }
  return nullptr;
}

// fitShapeOriented: a turned frame must make the box at least this much
// smaller (relative volume) to replace the body axes.
constexpr double kTurnedFitGain = 1e-3;

// Null, or a rotation within float noise of identity (qx, qy, qz ~ 0, w ~ +-1).
bool isIdentityRotation(const float q[4]) {
  return !q || (std::fabs(q[0]) < 1e-6f && std::fabs(q[1]) < 1e-6f &&
                std::fabs(q[2]) < 1e-6f);
}

b3Filter toB3Filter(const CollisionFilter &f) {
  b3Filter out = b3DefaultFilter();
  out.categoryBits = f.category;
  out.maskBits = f.mask;
  out.groupIndex = f.group;
  return out;
}

// Box3D asserts on a negative or non-finite rolling resistance and tangent
// velocity: clean them (the rolling resistance with nonNegOr).
void cleanTangent(const float in[3], float out[3]) {
  const bool ok = finite3(in);
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
  def.baseMaterial.rollingResistance = nonNegOr(p.rollingResistance);
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

// Box3D's task hooks, forwarding to the world's TaskRunner (userContext).
void *enqueueTask(b3TaskCallback *task, void *taskContext, void *userContext,
                  b3TaskType /*type*/) {
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
  std::unique_lock<std::mutex> listLock(worldListMutex());
  if (b3GetWorldCount() >= B3_MAX_WORLDS)
    return false;

  myImpl->settings = s;
  myImpl->settings.subSteps = std::max(1, s.subSteps); // at least one
  myImpl->settings.gravity[0] = def.gravity.x;         // as applied (finite)
  myImpl->settings.gravity[1] = def.gravity.y;
  myImpl->settings.gravity[2] = def.gravity.z;
  myImpl->tasks = std::move(tasks);
  myImpl->worldId = b3CreateWorld(&def);
  listLock.unlock();
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
  if (b3World_IsValid(myImpl->worldId)) {
    std::lock_guard<std::mutex> listLock(worldListMutex());
    b3DestroyWorld(myImpl->worldId); // destroys all bodies/shapes/joints
  }

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
  def.rotation = cleanQuat(rotationXYZW);
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
  setLazyPose(h, false);
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
  case ShapeType::Cylinder:
    return cylinderShape(h, body, shape.radius, shape.halfLength, shape.center,
                         shape.rot, updateMass);
  case ShapeType::Plane: {
    // A ground slab with its top through the body origin, facing its +Y
    // (static bodies only).
    if (!h || b3Body_GetType(b3LoadBodyId(h)) != b3_staticBody)
      return false;
    // The spec's half extents are the ground's size (groundSpec); its top is
    // the body's y = 0.
    const float below[3] = {0.0f, -shape.halfExtents[1], 0.0f};
    return boxShape(h, body, shape.halfExtents, below, nullptr, updateMass);
  }
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
  return !B3_IS_NULL(b3CreateHullShape(
      b3LoadBodyId(h), &def, hull->hull)); // added to the world's database
}

int Box3DWorld::copyShapesByHandle(BodyHandle src, BodyHandle dst,
                                   const float center[3],
                                   const float rotXYZW[4]) {
  if (!src || !dst || src == dst || !center || !rotXYZW || !finite3(center) ||
      !finite3(rotXYZW))
    return -1;
  const b3BodyId from = b3LoadBodyId(src), to = b3LoadBodyId(dst);
  std::vector<b3ShapeId> shapes;
  const int got = bodyShapes(from, shapes);
  // Box3D lists shapes newest first: copy in creation order.
  std::reverse(shapes.begin(), shapes.begin() + got);
  for (int i = 0; i < got; ++i) {
    const b3ShapeType type = b3Shape_GetType(shapes[static_cast<size_t>(i)]);
    if (type != b3_hullShape && type != b3_sphereShape &&
        type != b3_capsuleShape)
      return -1; // meshes, planes...: nothing is copied
  }
  float rot[4] = {rotXYZW[0], rotXYZW[1], rotXYZW[2], rotXYZW[3]};
  sanitizeQuat(rot); // Box3D asserts on a non-unit quaternion
  b3Transform local;
  local.p = makeVec3(center);
  local.q = makeQuat(rot);
  int added = 0;
  for (int i = 0; i < got; ++i) {
    const b3ShapeId s = shapes[static_cast<size_t>(i)];
    b3ShapeDef def = b3DefaultShapeDef();
    def.updateBodyMass = false; // the caller applies the mass once
    def.density = b3Shape_GetDensity(s);
    def.baseMaterial = b3Shape_GetSurfaceMaterial(s);
    def.filter = b3Shape_GetFilter(s);
    def.enableHitEvents = true;
    b3ShapeId made = b3_nullShapeId;
    switch (b3Shape_GetType(s)) {
    case b3_hullShape:
      made = b3CreateTransformedHullShape(to, &def, b3Shape_GetHull(s), local,
                                          b3Vec3_one);
      break;
    case b3_sphereShape: {
      b3Sphere sphere = b3Shape_GetSphere(s);
      sphere.center = b3Add(local.p, b3RotateVector(local.q, sphere.center));
      made = b3CreateSphereShape(to, &def, &sphere);
      break;
    }
    default: {
      b3Capsule capsule = b3Shape_GetCapsule(s);
      capsule.center1 =
          b3Add(local.p, b3RotateVector(local.q, capsule.center1));
      capsule.center2 =
          b3Add(local.p, b3RotateVector(local.q, capsule.center2));
      made = b3CreateCapsuleShape(to, &def, &capsule);
      break;
    }
    }
    if (B3_IS_NULL(made))
      return -1; // the caller drops the partly built body
    ++added;
  }
  return added;
}

bool Box3DWorld::createMeshShape(BodyHandle h, const BodySettings &body,
                                 const float *xyz, int vertexCount,
                                 const int32_t *triangles, int triangleCount,
                                 float weldTolerance, const float scale[3]) {
  if (!h || !xyz || !triangles || vertexCount < 3 || triangleCount < 1)
    return false;
  // Static or kinematic bodies only (a triangle mesh has no volume or mass):
  // refuse a dynamic one.
  if (b3Body_GetType(b3LoadBodyId(h)) == b3_dynamicBody)
    return false;

  // Reject bad input here: Box3D asserts on non-finite vertices or out-of-range
  // indices.
  const size_t floatCount = static_cast<size_t>(vertexCount) * 3;
  const size_t indexCount = static_cast<size_t>(triangleCount) * 3;
  for (int i = 0; i < vertexCount; ++i)
    if (!finite3(xyz + 3 * i))
      return false;
  for (size_t i = 0; i < indexCount; ++i)
    if (triangles[i] < 0 || triangles[i] >= vertexCount)
      return false;
  if (scale && (!finite3(scale) || !(scale[0] > 0.0f) || !(scale[1] > 0.0f) ||
                !(scale[2] > 0.0f)))
    return false;

  // The same mesh (vertices, indices, weld) on another body shares one
  // b3MeshData: Box3D only points at it, and the scale is the shape's own. A
  // hash match is checked against the mesh's own copy of what it was built
  // from; a different mesh with the same hash takes the next free key.
  const float weld = weldTolerance > 0.0f ? weldTolerance : 0.0f;
  uint64_t key = hashFloats(xyz, floatCount);
  key = hashBytes(key, triangles, indexCount * sizeof(int32_t));
  key = mixHash(key, static_cast<uint64_t>(std::hash<float>()(weld)));
  Impl::SharedMesh *shared = nullptr;
  for (;; ++key) {
    const auto it = myImpl->sharedMeshes.find(key);
    if (it == myImpl->sharedMeshes.end())
      break;
    const Impl::SharedMesh &m = it->second;
    if (m.weld == weld && m.xyz.size() == floatCount &&
        m.tris.size() == indexCount &&
        std::memcmp(m.xyz.data(), xyz, floatCount * sizeof(float)) == 0 &&
        std::memcmp(m.tris.data(), triangles, indexCount * sizeof(int32_t)) ==
            0) {
      shared = &it->second;
      break;
    }
  }

  std::vector<b3Vec3> verts; // only for a mesh not built yet
  std::vector<int32_t> idx;
  b3MeshDef def = {};
  if (!shared) {
    verts.resize(static_cast<size_t>(vertexCount));
    for (int i = 0; i < vertexCount; ++i)
      verts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);
    idx.assign(triangles, triangles + indexCount); // b3MeshDef wants non-const
  }
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

  b3MeshData *mesh = shared ? shared->data : b3CreateMesh(&def, nullptr, 0);
  if (!mesh)
    return false;

  const b3ShapeDef sdef = shapeDefFor(body, false);
  const b3ShapeId shape = b3CreateMeshShape(
      b3LoadBodyId(h), &sdef, mesh, scale ? makeVec3(scale) : b3Vec3_one);
  if (B3_IS_NULL(shape)) {
    if (!shared)
      b3DestroyMesh(mesh);
    return false;
  }
  if (!shared) {
    shared = &myImpl->sharedMeshes[key];
    shared->xyz.assign(xyz, xyz + floatCount);
    shared->tris.assign(triangles, triangles + indexCount);
    shared->weld = weld;
  }
  shared->data = mesh;
  ++shared->users;
  myImpl->bodyGeometry[h].meshKeys.push_back(key);
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

  b3HeightFieldDef def = {};
  // Box3D takes float* but only reads the heights (it quantises them into its
  // own data): no copy of a field that can be a gigabyte.
  def.heights = const_cast<float *>(heights);
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

  const bool offset =
      center && (center[0] != 0.0f || center[1] != 0.0f || center[2] != 0.0f);
  b3BoxHull hull;
  if (!isIdentityRotation(rotationXYZW)) {
    b3Transform local;
    local.p = makeVec3(center);
    local.q = cleanQuat(rotationXYZW);
    hull = b3MakeTransformedBoxHull(halfExtents[0], halfExtents[1],
                                    halfExtents[2], local);
  } else if (offset) {
    hull = b3MakeOffsetBoxHull(halfExtents[0], halfExtents[1], halfExtents[2],
                               makeVec3(center));
  } else {
    hull = b3MakeBoxHull(halfExtents[0], halfExtents[1], halfExtents[2]);
  }
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
    const b3Vec3 up = {0.0f, 1.0f, 0.0f};
    const b3Vec3 r = b3RotateVector(cleanQuat(rotationXYZW), up);
    axis[0] = r.x;
    axis[1] = r.y;
    axis[2] = r.z;
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

bool Box3DWorld::cylinderShape(BodyHandle h, const BodySettings &b,
                               float radius, float halfLength,
                               const float center[3],
                               const float rotationXYZW[4], bool updateMass) {
  if (!h || !std::isfinite(radius) || !std::isfinite(halfLength))
    return false;
  // b3CreateCylinder asserts on a zero or negative size.
  radius = std::max(radius, kMinExtent);
  halfLength = std::max(halfLength, kMinExtent);
  const float c[3] = {center ? center[0] : 0.0f, center ? center[1] : 0.0f,
                      center ? center[2] : 0.0f};
  const bool turned = !isIdentityRotation(rotationXYZW);
  const b3Quat q = turned ? cleanQuat(rotationXYZW) : b3Quat_identity;

  // One key per size and pose, so instances share the hull.
  const float keyData[] = {
      radius, halfLength, c[0],  c[1], c[2],
      q.v.x,  q.v.y,      q.v.z, q.s,  static_cast<float>(kCylinderSides)};
  HullId key = hashFloats(keyData, sizeof(keyData) / sizeof(keyData[0]));
  key = mixHash(key, 0xc711d3ull);
  if (key == 0)
    key = 1;

  if (!findHull(key)) {
    // The cylinder is centred on its own origin: b3CreateCylinder starts at
    // yOffset and rises by the height.
    b3HullData *hull = b3CreateCylinder(2.0f * halfLength, radius, -halfLength,
                                        kCylinderSides);
    if (!hull)
      return false;
    if (turned || c[0] != 0.0f || c[1] != 0.0f || c[2] != 0.0f) {
      // Move the prism's points and hull them again.
      const b3Vec3 *src = b3GetHullPoints(hull);
      std::vector<b3Vec3> pts(static_cast<size_t>(hull->vertexCount));
      for (size_t i = 0; i < pts.size(); ++i) {
        const b3Vec3 r = b3RotateVector(q, src[i]);
        pts[i] = {r.x + c[0], r.y + c[1], r.z + c[2]};
      }
      b3DestroyHull(hull);
      hull = b3CreateHull(pts.data(), static_cast<int>(pts.size()),
                          B3_MAX_HULL_VERTICES);
      if (!hull)
        return false;
    }
    const HullRef ref = HullLibrary::instance().insert(key, hull);
    myImpl->hold(key, ref);
  }
  return createHullShape(h, b, key, updateMass);
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
  // Before hashing: a non-finite point must not find a cached hull.
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
  std::vector<b3Vec3> pts;
  if (!loadPoints(xyz, pointCount, pts))
    return 0;

  // The group's last working budget is tried first.
  HullLibrary &lib = HullLibrary::instance();
  if (lib.knownFailure(key))
    return 0;
  const int hint = lib.budgetHint(budgetGroup);
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, hint, budget, tries);
  lib.attempts.fetch_add(tries, std::memory_order_relaxed);
  if (!hull) {
    lib.noteFailure(key);
    return 0;
  }
  if (budget != hint)
    lib.setBudgetHint(budgetGroup, budget);
  lib.builds.fetch_add(1, std::memory_order_relaxed);

  if (!(hull->volume > 1e-9f) || !std::isfinite(hull->volume)) {
    b3DestroyHull(hull); // flat: no mass, no stable contacts
    lib.noteFailure(key);
    return 0;
  }
  const HullRef ref = lib.insert(key, hull);
  myImpl->hold(key, ref);
  if (outVolume)
    *outVolume = ref->volume;
  return key;
}

namespace {

// A mesh's edges worth drawing: boundary edges and ones where the two
// triangles meet at more than ~20 degrees (a flat quad's diagonal is left out).
// More than kMaxMeshOutlineEdges are thinned evenly.
void meshOutline(const b3Mesh &mesh, Box3DWorld::ShapeOutline &o) {
  const b3Vec3 *v = b3GetMeshVertices(mesh.data);
  const b3MeshTriangle *t = b3GetMeshTriangles(mesh.data);
  const int vc = mesh.data->vertexCount, tc = mesh.data->triangleCount;
  if (!v || !t || vc < 3 || tc < 1)
    return;
  o.kind = Box3DWorld::ShapeOutline::Mesh;
  o.verts.resize(static_cast<size_t>(vc) * 3);
  for (int k = 0; k < vc; ++k) {
    o.verts[3 * static_cast<size_t>(k)] = v[k].x * mesh.scale.x;
    o.verts[3 * static_cast<size_t>(k) + 1] = v[k].y * mesh.scale.y;
    o.verts[3 * static_cast<size_t>(k) + 2] = v[k].z * mesh.scale.z;
  }
  struct Edge {
    uint64_t key;
    b3Vec3 normal;
    int count;
  };
  std::vector<Edge> edges;
  edges.reserve(static_cast<size_t>(tc) * 3);
  for (int k = 0; k < tc; ++k) {
    const int idx[3] = {t[k].index1, t[k].index2, t[k].index3};
    if (idx[0] < 0 || idx[1] < 0 || idx[2] < 0 || idx[0] >= vc ||
        idx[1] >= vc || idx[2] >= vc)
      continue;
    const float *p[3];
    for (int j = 0; j < 3; ++j)
      p[j] = &o.verts[3 * static_cast<size_t>(idx[j])];
    const float a[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1],
                        p[1][2] - p[0][2]};
    const float b[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1],
                        p[2][2] - p[0][2]};
    b3Vec3 n = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    const float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (len > 0.0f) {
      n.x /= len;
      n.y /= len;
      n.z /= len;
    }
    for (int j = 0; j < 3; ++j) {
      const uint64_t lo =
          static_cast<uint64_t>(std::min(idx[j], idx[(j + 1) % 3]));
      const uint64_t hi =
          static_cast<uint64_t>(std::max(idx[j], idx[(j + 1) % 3]));
      edges.push_back({(lo << 32) | hi, n, 1});
    }
  }
  std::sort(edges.begin(), edges.end(),
            [](const Edge &x, const Edge &y) { return x.key < y.key; });
  std::vector<uint64_t> keep;
  constexpr float kCreaseDot = 0.94f; // cos 20 degrees
  for (size_t k = 0; k < edges.size();) {
    size_t e = k + 1;
    while (e < edges.size() && edges[e].key == edges[k].key)
      ++e;
    bool draw = e - k != 2; // boundary or non-manifold
    if (!draw) {
      const b3Vec3 &x = edges[k].normal, &y = edges[k + 1].normal;
      draw = x.x * y.x + x.y * y.y + x.z * y.z < kCreaseDot;
    }
    if (draw)
      keep.push_back(edges[k].key);
    k = e;
  }
  const size_t stride = keep.size() / kMaxMeshOutlineEdges + 1;
  for (size_t k = 0; k < keep.size(); k += stride) {
    o.loops.push_back(static_cast<int>(keep[k] >> 32));
    o.loops.push_back(static_cast<int>(keep[k] & 0xffffffffu));
  }
}

} // namespace

bool Box3DWorld::bodyShapeOutlines(BodyHandle h,
                                   std::vector<ShapeOutline> &out) const {
  out.clear();
  if (!h)
    return false;
  const b3BodyId body = b3LoadBodyId(h);
  if (!b3Body_IsValid(body))
    return false;
  std::vector<b3ShapeId> shapes;
  const int got = bodyShapes(body, shapes);
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
    case b3_meshShape: {
      const b3Mesh mesh = b3Shape_GetMesh(shapes[static_cast<size_t>(i)]);
      if (!mesh.data)
        continue;
      meshOutline(mesh, o);
      break;
    }
    default:
      continue; // compounds: not made for bodies here
    }
    out.push_back(std::move(o));
  }
  return true;
}

bool Box3DWorld::hullCentroid(const float *xyz, int count, double out[3]) {
  std::vector<b3Vec3> pts;
  if (!xyz || count < 4 || !loadPoints(xyz, count, pts))
    return false;
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, 0, budget, tries);
  if (!hull)
    return false;
  const b3MassData md = b3ComputeHullMass(hull, 1.0f);
  b3DestroyHull(hull);
  if (!(md.mass > 1e-9f) || !std::isfinite(md.center.x) ||
      !std::isfinite(md.center.y) || !std::isfinite(md.center.z))
    return false;
  out[0] = static_cast<double>(md.center.x);
  out[1] = static_cast<double>(md.center.y);
  out[2] = static_cast<double>(md.center.z);
  return true;
}

namespace {

// fitShapeOriented without the cache.
bool fitOrientedUncached(ShapeType type, const float *xyz, int count,
                         ShapeSpec &out) {
  std::vector<b3Vec3> pts;
  if (!loadPoints(xyz, count, pts))
    return false;
  // Only the hull's directions are used, so a reduced budget is fine (the
  // extents are measured over every point).
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, 0, budget, tries);
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
        const float len = length3(u);
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

} // namespace

bool Box3DWorld::fitShapeOriented(ShapeType type, const float *xyz, int count,
                                  ShapeSpec &out) {
  if (!xyz || count <= 0)
    return false;
  if (count < 4)
    return fitShapeToPoints(type, xyz, count, out);
  // Before hashing: a non-finite point must not find a cached fit.
  for (int i = 0; i < count * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return false;
  const uint64_t key = mixHash(hashPoints(xyz, count),
                               static_cast<uint64_t>(static_cast<int>(type)));
  FitLibrary &lib = FitLibrary::instance();
  if (lib.find(key, out))
    return true;
  if (!fitOrientedUncached(type, xyz, count, out))
    return false;
  lib.insert(key, out);
  return true;
}

bool Box3DWorld::findFit(uint64_t key, ShapeSpec &out) {
  return key != 0 && FitLibrary::instance().find(key, out);
}

void Box3DWorld::storeFit(uint64_t key, const ShapeSpec &spec) {
  if (key != 0)
    FitLibrary::instance().insert(key, spec);
}

void Box3DWorld::pruneHulls() {
  std::unique_lock<std::shared_mutex> lock(myImpl->held_mutex);
  myImpl->held.clear();
}

// The world's own figure (its arrays, contacts, joints and the hulls in
// its database), plus the meshes and height fields we own: Box3D keeps
// pointers to those instead of copying them.
size_t Box3DWorld::estimatedBytes() const {
  if (!isValid())
    return 0;
  const b3Counters c = b3World_GetCounters(myImpl->worldId);
  size_t bytes = c.byteCount > 0 ? static_cast<size_t>(c.byteCount) : 0;
  for (const auto &kv : myImpl->sharedMeshes)
    bytes += static_cast<size_t>(kv.second.data->byteCount);
  for (const auto &kv : myImpl->bodyGeometry) {
    for (const b3HeightFieldData *hf : kv.second.heightFields)
      bytes += static_cast<size_t>(hf->byteCount);
  }
  return bytes;
}

int Box3DWorld::meshCount() const {
  return static_cast<int>(myImpl->sharedMeshes.size());
}

int Box3DWorld::hullCacheSize() const {
  std::shared_lock<std::shared_mutex> lock(myImpl->held_mutex);
  return static_cast<int>(myImpl->held.size());
}

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

// ---- changes to existing bodies -------------------------------------------

namespace {

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
  const float rolling = nonNegOr(rollingResistance);
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

void Box3DWorld::setEnabledByHandle(BodyHandle h, bool enabled) {
  if (!h)
    return;
  const b3BodyId id = b3LoadBodyId(h);
  if (b3Body_IsEnabled(id) == enabled)
    return;
  if (enabled) {
    // A body that leaves its compound is put where it is first.
    freshenPose(h);
    setLazyPose(h, false);
    b3Body_Enable(id);
  } else {
    b3Body_Disable(id);
  }
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

int Box3DWorld::destroyShapeRangesByHandle(
    BodyHandle h, const std::vector<std::pair<int, int>> &ranges) {
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  const int n = bodyShapes(id, tShapes); // newest first
  std::vector<int> doomed; // creation indices, each once (ranges may overlap)
  for (const auto &r : ranges)
    for (int i = r.first; i < r.first + r.second; ++i)
      if (i >= 0 && i < n)
        doomed.push_back(i);
  std::sort(doomed.begin(), doomed.end());
  doomed.erase(std::unique(doomed.begin(), doomed.end()), doomed.end());
  // Collect the ids before destroying any: tShapes is not touched by it, but
  // the list is read newest first.
  std::vector<b3ShapeId> ids;
  ids.reserve(doomed.size());
  for (const int i : doomed)
    ids.push_back(tShapes[static_cast<size_t>(n - 1 - i)]);
  for (const b3ShapeId s : ids)
    b3DestroyShape(s, false); // the caller applies the mass afterwards
  return static_cast<int>(ids.size());
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

bool Box3DWorld::bodyStateByHandle(BodyHandle h, BodyState &out) const {
  if (!h)
    return false;
  freshenPose(h);
  const b3BodyId id = b3LoadBodyId(h);
  out.awake = b3Body_IsAwake(id);
  out.mass = b3Body_GetMass(id);
  storePos(b3Body_GetWorldCenter(id), out.com);
  storeVec3(b3Body_GetLinearVelocity(id), out.lin);
  storeVec3(b3Body_GetAngularVelocity(id), out.ang);
  return true;
}

bool Box3DWorld::extentByHandle(BodyHandle h, float out[3]) const {
  if (!h || b3Body_GetShapeCount(b3LoadBodyId(h)) == 0)
    return false;
  freshenPose(h);
  const b3AABB box = b3Body_ComputeAABB(b3LoadBodyId(h));
  const b3Vec3 e = b3Sub(box.upperBound, box.lowerBound);
  out[0] = e.x;
  out[1] = e.y;
  out[2] = e.z;
  return finite3(out) && out[0] >= 0.0f && out[1] >= 0.0f && out[2] >= 0.0f;
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

bool Box3DWorld::applyMassByHandle(BodyHandle h, float density,
                                   bool comAtOrigin, float inertiaScale,
                                   bool setShapeDensity,
                                   const float *inverseInertia) {
  if (!h || !(density >= 0.0f) || !std::isfinite(density))
    return true;
  const b3BodyId id = b3LoadBodyId(h);
  const int n =
      setShapeDensity ? bodyShapes(id, tShapes) : b3Body_GetShapeCount(id);
  if (setShapeDensity)
    for (int i = 0; i < n; ++i)
      b3Shape_SetDensity(tShapes[static_cast<size_t>(i)], density, false);
  b3Body_ApplyMassFromShapes(id);

  // A custom inertia is the inverse of the tensor given, about the centre of
  // mass, which is the body origin.
  float custom[6];
  const bool haveCustom =
      inverseInertia && invertSymmetric3(inverseInertia, custom);
  const bool customOk = !inverseInertia || haveCustom;

  const bool scale = inertiaScale > 0.0f && std::isfinite(inertiaScale) &&
                     inertiaScale != 1.0f;
  if (b3Body_GetType(id) != b3_dynamicBody || n == 0 ||
      (!comAtOrigin && !scale && !haveCustom))
    return customOk;
  b3MassData md = b3Body_GetMassData(id);
  const bool moveCenter =
      (comAtOrigin || haveCustom) &&
      (md.center.x != 0.0f || md.center.y != 0.0f || md.center.z != 0.0f);
  if (!moveCenter && !scale && !haveCustom)
    return customOk; // already centred
  if (haveCustom) {
    md.inertia.cx = {custom[0], custom[3], custom[4]};
    md.inertia.cy = {custom[3], custom[1], custom[5]};
    md.inertia.cz = {custom[4], custom[5], custom[2]};
    md.center = b3Vec3_zero;
  } else if (moveCenter) {
    // Parallel axis: I_o = I_c + m (|d|^2 E - d d^T). b3Matrix3 is three
    // columns.
    const float d[3] = {md.center.x, md.center.y, md.center.z};
    const float dd = dot3(d, d);
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
  return customOk;
}

bool Box3DWorld::getMassDataByHandle(BodyHandle h, float &mass, float center[3],
                                     float inertiaDiag[3],
                                     float *inertiaOff) const {
  if (!h)
    return false;
  const b3MassData md = b3Body_GetMassData(b3LoadBodyId(h));
  mass = md.mass;
  storeVec3(md.center, center);
  inertiaDiag[0] = md.inertia.cx.x;
  inertiaDiag[1] = md.inertia.cy.y;
  inertiaDiag[2] = md.inertia.cz.z;
  if (inertiaOff) {
    inertiaOff[0] = md.inertia.cy.x; // xy
    inertiaOff[1] = md.inertia.cz.x; // xz
    inertiaOff[2] = md.inertia.cz.y; // yz
  }
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
  t.q = cleanQuat(rotXYZW);
  b3Body_SetTargetTransform(b3LoadBodyId(h), t, timeStep, true);
}

namespace {

bool sameBody(const b3BodyId &a, const b3BodyId &b) {
  return a.index1 == b.index1 && a.world0 == b.world0 &&
         a.generation == b.generation;
}

// The body's contacts into `data` (a scratch buffer that only grows, so a call
// does not zero what it is about to overwrite); how many.
int fetchContacts(b3BodyId id, std::vector<b3ContactData> &data) {
  const int cap = b3Body_GetContactCapacity(id);
  if (cap <= 0)
    return 0;
  if (data.size() < static_cast<size_t>(cap))
    data.resize(static_cast<size_t>(cap));
  return b3Body_GetContactData(id, data.data(), cap);
}

} // namespace

float Box3DWorld::contactImpulseByHandle(
    BodyHandle h, const BodyHandle *skip, size_t skipCount,
    const std::function<bool(BodyHandle)> *ignore) const {
  if (!h)
    return 0.0f;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
  float total = 0.0f;
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    if (skipCount > 0 || ignore) {
      const b3BodyId a = b3Shape_GetBody(cd.shapeIdA);
      const b3BodyId other = sameBody(a, id) ? b3Shape_GetBody(cd.shapeIdB) : a;
      const BodyHandle otherHandle = b3StoreBodyId(other);
      if (std::find(skip, skip + skipCount, otherHandle) != skip + skipCount)
        continue;
      if (ignore && (*ignore)(otherHandle))
        continue;
    }
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k)
        if (man.points[k].totalNormalImpulse > 0.0f)
          total += man.points[k].totalNormalImpulse;
    }
  }
  return total;
}

namespace {

struct SweptBodies {
  std::vector<b3BodyId> ids;
  b3BodyId self;
};

bool collectSweptBody(b3ShapeId shape, void *context) {
  SweptBodies &c = *static_cast<SweptBodies *>(context);
  const b3BodyId body = b3Shape_GetBody(shape);
  if (sameBody(body, c.self))
    return true;
  // One entry per shape: the caller sorts and drops the repeats.
  c.ids.push_back(body);
  return true;
}

} // namespace

void Box3DWorld::approachingBodies(const BodyHandle *movers, size_t count,
                                   float dt, float minSpeed, float minMomentum,
                                   std::vector<Approach> &out) const {
  if (!isValid() || !(dt > 0.0f))
    return;
  SweptBodies hit;
  for (size_t i = 0; i < count; ++i) {
    if (!movers[i])
      continue;
    const b3BodyId id = b3LoadBodyId(movers[i]);
    if (!b3Body_IsValid(id) || b3Body_GetType(id) == b3_staticBody ||
        !b3Body_IsAwake(id) || b3Body_GetShapeCount(id) == 0)
      continue;
    const b3Vec3 v = b3Body_GetLinearVelocity(id);
    if (!b3IsValidVec3(v) || b3Length(v) < minSpeed)
      continue;
    const bool dynamic = b3Body_GetType(id) == b3_dynamicBody;
    const float mass = dynamic ? b3Body_GetMass(id) : 0.0f;
    if (dynamic && !(mass * b3Length(v) >= minMomentum))
      continue;
    b3AABB box = b3Body_ComputeAABB(id);
    const b3Vec3 size = b3Sub(box.upperBound, box.lowerBound);
    const float half = 0.5f * std::max(size.x, std::max(size.y, size.z));
    const b3Vec3 d = b3MulSV(dt, v);
    box.lowerBound = b3Add(box.lowerBound, b3Min(d, b3Vec3_zero));
    box.upperBound = b3Add(box.upperBound, b3Max(d, b3Vec3_zero));
    hit.ids.clear();
    hit.self = id;
    b3World_OverlapAABB(myImpl->worldId, box, b3DefaultQueryFilter(),
                        collectSweptBody, &hit);
    std::sort(hit.ids.begin(), hit.ids.end(),
              [](const b3BodyId &x, const b3BodyId &y) {
                return b3StoreBodyId(x) < b3StoreBodyId(y);
              });
    hit.ids.erase(std::unique(hit.ids.begin(), hit.ids.end(), sameBody),
                  hit.ids.end());
    const b3Pos centre = b3Body_GetWorldCenter(id);
    for (const b3BodyId &target : hit.ids) {
      // The mover's velocity relative to what the target does at the mover's
      // centre: a body that moves with the target (a carrier, a platform) is
      // not coming at it.
      const b3Vec3 r = b3SubPos(centre, b3Body_GetWorldCenter(target));
      const b3Vec3 atMover =
          b3Add(b3Body_GetLinearVelocity(target),
                b3Cross(b3Body_GetAngularVelocity(target), r));
      const b3Vec3 rel = b3Sub(v, atMover);
      if (!b3IsValidVec3(rel) || b3Length(rel) < minSpeed)
        continue;
      Approach a;
      a.mover = movers[i];
      a.target = b3StoreBodyId(target);
      a.moverMass = mass;
      storePos(centre, a.moverCom);
      storeVec3(v, a.moverLin);
      storeVec3(rel, a.relLin);
      a.moverHalf = half;
      out.push_back(a);
    }
  }
}

bool Box3DWorld::boundsByHandle(BodyHandle h, double lo[3],
                                double hi[3]) const {
  if (!h)
    return false;
  const b3BodyId id = b3LoadBodyId(h);
  if (!b3Body_IsValid(id) || b3Body_GetShapeCount(id) == 0)
    return false;
  const b3AABB box = b3Body_ComputeAABB(id);
  lo[0] = box.lowerBound.x;
  lo[1] = box.lowerBound.y;
  lo[2] = box.lowerBound.z;
  hi[0] = box.upperBound.x;
  hi[1] = box.upperBound.y;
  hi[2] = box.upperBound.z;
  return true;
}

int Box3DWorld::contactImpulsesByShape(
    BodyHandle h, std::vector<float> &out,
    const std::function<bool(BodyHandle)> *ignore) const {
  out.clear();
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ShapeId> shapes;
  const int got = bodyShapes(id, shapes);
  if (got <= 0)
    return 0;
  out.assign(static_cast<size_t>(got), 0.0f);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
  if (n <= 0)
    return got; // no contacts: no need to index the shapes
  thread_local ShapeIndex indexOf;
  indexShapes(shapes, got, indexOf);
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    const bool aIsMine = sameBody(b3Shape_GetBody(cd.shapeIdA), id);
    const b3ShapeId mine = aIsMine ? cd.shapeIdA : cd.shapeIdB;
    const uint64_t mineKey = b3StoreShapeId(mine);
    const auto *it = findShapeIndex(indexOf, mineKey);
    if (!it)
      continue;
    if (ignore && (*ignore)(b3StoreBodyId(
                      b3Shape_GetBody(aIsMine ? cd.shapeIdB : cd.shapeIdA))))
      continue;
    float total = 0.0f;
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k)
        if (man.points[k].totalNormalImpulse > 0.0f)
          total += man.points[k].totalNormalImpulse;
    }
    out[static_cast<size_t>(it->second)] += total;
  }
  return got;
}

int Box3DWorld::getContactsByHandle(BodyHandle h,
                                    std::vector<ContactPoint> &out) const {
  out.clear();
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
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

void Box3DWorld::beginBulkCreate() {
  if (isValid())
    b3World_BeginBulkCreate(myImpl->worldId);
}

void Box3DWorld::endBulkCreate() {
  if (isValid())
    b3World_EndBulkCreate(myImpl->worldId);
}

void Box3DWorld::enableContactImpulseEvents(bool on) {
  if (isValid())
    b3World_EnableContactImpulseEvents(myImpl->worldId, on);
}

int Box3DWorld::getContactImpulseEvents(
    std::vector<ContactImpulse> &out) const {
  if (!isValid()) {
    out.clear();
    return 0;
  }
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  // Every element is overwritten below, so a vector of the right size is not
  // cleared first (it would be written twice).
  out.resize(static_cast<size_t>(ev.impulseCount));
  for (int i = 0; i < ev.impulseCount; ++i) {
    const b3ContactImpulseEvent &e = ev.impulseEvents[i];
    ContactImpulse &c = out[static_cast<size_t>(i)];
    c.a = b3StoreBodyId(e.bodyIdA);
    c.b = b3StoreBodyId(e.bodyIdB);
    c.userDataA = e.userDataA;
    c.userDataB = e.userDataB;
    storePos(e.point, c.pos);
    storeVec3(e.normal, c.normal);
    c.impulse = e.impulse;
    c.awakeA = e.awakeA;
    c.awakeB = e.awakeB;
  }
  return ev.impulseCount;
}

int Box3DWorld::getContactImpulseSums(
    std::vector<ContactImpulseSum> &out) const {
  out.clear();
  if (!isValid())
    return 0;
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  out.reserve(static_cast<size_t>(ev.impulseCount) / 2 + 1);
  for (int i = 0; i < ev.impulseCount; ++i) {
    const b3ContactImpulseEvent &e = ev.impulseEvents[i];
    if (!(e.impulse > 0.0f))
      continue;
    const BodyHandle a = b3StoreBodyId(e.bodyIdA);
    const BodyHandle b = b3StoreBodyId(e.bodyIdB);
    if (!out.empty() && out.back().a == a && out.back().b == b)
      out.back().impulse += e.impulse;
    else
      out.push_back({a, b, e.impulse});
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
  next.subSteps = std::max(1, next.subSteps); // at least one
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

void Box3DWorld::updateHitEvents(bool append) {
  if (!append)
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

Box3DWorld::StepProfile Box3DWorld::stepProfile() const {
  StepProfile out;
  if (!isValid())
    return out;
  const b3Profile p = b3World_GetProfile(myImpl->worldId);
  out.step = p.step;
  out.pairs = p.pairs;
  out.collide = p.collide;
  out.solverSetup = p.solverSetup;
  out.constraints = p.constraints;
  out.prepareConstraints = p.prepareConstraints;
  out.integrateVelocities = p.integrateVelocities;
  out.warmStart = p.warmStart;
  out.solveImpulses = p.solveImpulses;
  out.integratePositions = p.integratePositions;
  out.relaxImpulses = p.relaxImpulses;
  out.restitution = p.restitution;
  out.storeImpulses = p.storeImpulses;
  out.splitIslands = p.splitIslands;
  out.transforms = p.transforms;
  out.events = p.jointEvents + p.hitEvents;
  out.refit = p.refit;
  out.bullets = p.bullets;
  out.sensorHits = p.sensorHits;
  out.sleepIslands = p.sleepIslands;
  out.sensors = p.sensors;
  return out;
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
  freshenPose(h);
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

void *Box3DWorld::bodyUserDataByHandle(BodyHandle h) const {
  return h ? b3Body_GetUserData(b3LoadBodyId(h)) : nullptr;
}

void Box3DWorld::setBodyUserData(BodyHandle h, void *userData) {
  if (h)
    b3Body_SetUserData(b3LoadBodyId(h), userData);
}

bool Box3DWorld::setTransformByHandle(BodyHandle h, const double pos[3],
                                      const float rotXYZW[4]) {
  if (!h)
    return false;
  b3Body_SetTransform(b3LoadBodyId(h), makePos(pos), cleanQuat(rotXYZW));
  if (myImpl->lazyCount.load(std::memory_order_relaxed) != 0) {
    const auto it = myImpl->lazyPose.find(h);
    if (it != myImpl->lazyPose.end())
      it->second.store(myImpl->lazyEpoch.load(std::memory_order_acquire),
                       std::memory_order_release);
  }
  return true;
}

void Box3DWorld::setLazyPose(BodyHandle h, bool lazy) {
  if (!h)
    return;
  Impl &d = *myImpl;
  if (lazy) {
    const auto ins = d.lazyPose.try_emplace(h, 0);
    ins.first->second.store(d.lazyEpoch.load(), std::memory_order_release);
    if (ins.second)
      d.lazyCount.store(d.lazyPose.size(), std::memory_order_release);
  } else if (d.lazyPose.erase(h) != 0) {
    d.lazyCount.store(d.lazyPose.size(), std::memory_order_release);
  }
}

void Box3DWorld::invalidateLazyPoses() {
  myImpl->lazyEpoch.fetch_add(1, std::memory_order_acq_rel);
}

void Box3DWorld::setPoseRefresher(std::function<void(BodyHandle)> refresh) {
  myImpl->poseRefresher = std::move(refresh);
}

bool Box3DWorld::hasPoseRefresher() const {
  return static_cast<bool>(myImpl->poseRefresher);
}

void Box3DWorld::freshenPose(BodyHandle h) const {
  Impl &d = *myImpl;
  if (!h || d.lazyCount.load(std::memory_order_acquire) == 0)
    return;
  const auto it = d.lazyPose.find(h);
  if (it == d.lazyPose.end())
    return;
  const uint64_t epoch = d.lazyEpoch.load(std::memory_order_acquire);
  if (it->second.load(std::memory_order_acquire) == epoch)
    return;
  const std::lock_guard<std::mutex> lock(d.lazyMutex);
  if (it->second.load(std::memory_order_acquire) == epoch)
    return;
  // Another thread reading the body meanwhile must wait for the lock: the
  // epoch is stored once the transform is set (setTransformByHandle), not
  // before.
  if (d.poseRefresher)
    d.poseRefresher(h);
  it->second.store(epoch, std::memory_order_release);
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
