// Box3DWorld.cpp
//
// Box3DWorld: the world's life (create, step, destroy), bodies and their
// properties, mass, transforms, lazy poses. Shapes, hulls and contacts are in
// Box3DWorldShapes.cpp, Box3DWorldHulls.cpp and Box3DWorldContacts.cpp, joints
// in Box3DWorldJoints.cpp.
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

namespace {
// Scratch for a body's shapes (reused, so a call does not allocate).
thread_local std::vector<b3ShapeId> tShapes;
} // namespace

static_assert(
    kMaxWorkerCount == B3_MAX_WORKERS,
    "kMaxWorkerCount (Box3DWorld.h) must match Box3D's B3_MAX_WORKERS");

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

int Box3DWorld::bodyCount() const { return myImpl->liveBodies; }

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

void Box3DWorld::beginBulkCreate() {
  if (isValid())
    b3World_BeginBulkCreate(myImpl->worldId);
}

void Box3DWorld::endBulkCreate() {
  if (isValid())
    b3World_EndBulkCreate(myImpl->worldId);
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
