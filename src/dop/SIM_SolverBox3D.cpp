// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DData.h"
#include "SIM_Box3DParms.h"
#include "SIM_Box3DWorld.h"
#include "core/Box3DDebug.h"
#include "sync/Box3DAffectors.h"
#include "sync/Box3DJointSync.h"
#include "sync/Box3DSolverInternal.h"

#include <SIM/SIM_CacheOptions.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <UT/UT_PerfMonAutoEvent.h>
#include <UT/UT_Thread.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <unordered_set>

using namespace box3dsolver;

// ---------------------------------------------------------------------------

SIM_SolverBox3D::SIM_SolverBox3D(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this) {}

SIM_SolverBox3D::~SIM_SolverBox3D() {}

const SIM_DopDescription *SIM_SolverBox3D::getDescription() {
  // Box3D's solver tuning, with Box3D's defaults (Bullet's ERP / CFM have no
  // equivalent).
  using R = Box3DParms::Range;
  static const std::string autoThreadsHelp =
      "Use one worker thread per CPU core (up to " +
      std::to_string(b3dop::kMaxWorkerCount) +
      "). Usually only faster for very large scenes.";
  static Box3DParms p;
  static const PRM_Template *templates =
      p.tab("Simulation")
          .vector("gravity", "Gravity", {0.0, -9.80665, 0.0},
                  "Acceleration of every dynamic body, in units per second "
                  "squared. Scale per body with Gravity Scale on Box3D Data.")
          .toggle("useforces", "Use Force DOPs", true,
                  "Apply Force DOPs connected to the objects (Wind, Fan, "
                  "etc.). Set Gravity to 0 when using a Gravity Force.")
          .toggle("forceswake", "Forces Wake Bodies", false,
                  "Let forces wake sleeping bodies, e.g. wind that should blow "
                  "resting pieces away.",
                  "{ useforces == 0 }")
          // Minimum 1 (Box3D clamps to it); larger than 16 can be typed.
          .integer("substeps", "Substeps", 4, R{1, 16},
                   "Solver passes per time step. More is more stable for "
                   "stacks and fast collisions, but slower.")
          .number("timescale", "Time Scale", 1.0, R{0, 2},
                  "Multiplies the time step: below 1 is slow motion, above 1 "
                  "fast motion.")
          .toggle("enableaging", "Enable Aging", false,
                  "Add the frame time step (not Time Scale-adjusted) to each "
                  "point's age, and mark points dead once age exceeds life.")
          .number("maxlinearspeed", "Max Linear Speed", 400.0, R{0, 1000},
                  "Speed limit for every body, in units per second.")
          .tab("Performance")
          .toggle("autothreads", "Use All Threads", false,
                  autoThreadsHelp.c_str())
          .integer("threads", "Threads", b3dop::kDefaultWorkerCount,
                   R{1, static_cast<double>(b3dop::kMaxWorkerCount), true},
                   "Number of worker threads. More helps large scenes; small "
                   "ones can run faster with fewer.",
                   "{ autothreads == 1 }")
          .toggle("enablesleep", "Allow Sleeping (All Objects)", true,
                  "Let resting bodies stop simulating until touched. Off keeps "
                  "every body awake, whatever Box3D Data says.")
          .number("contactrecycledistance", "Contact Recycle Distance", 0.05,
                  R{0, 0.2},
                  "Reuse contacts for pairs that barely moved: faster but "
                  "slightly less exact. 0 turns it off. "
                  "Box3D Data's Contact Recycling turns it off per object.")
          .tab("Contacts")
          .number("contactstiffness", "Contact Stiffness", 30.0, R{0, 120},
                  "How stiff contacts are (cycles per second). Higher values "
                  "recover from overlap faster but can jitter.")
          .number("contactdamping", "Contact Damping Ratio", 10.0, R{0, 20},
                  "How much contacts damp overlap recovery. Lower values "
                  "recover faster but more energetically.")
          .number("contactpushspeed", "Overlap Push Speed", 3.0, R{0, 10},
                  "How fast overlapping pieces are pushed apart. Lower it if "
                  "pieces that start out overlapping fly apart.")
          .toggle("enablecontinuous", "Continuous Collision", true,
                  "Stop fast bodies passing through static ones (and through "
                  "moving ones, for Box3D Data's Fast Object).")
          .tab("Bounce & Friction")
          .number("bouncethreshold", "Bounce Threshold", 1.0, R{0, 5},
                  "Collisions slower than this speed don't bounce, so resting "
                  "pieces settle instead of jittering.")
          .integer("restitutioniterations", "Bounce Iterations", 2, R{0, 10},
                   "Extra solver passes that compute bounce. More passes give "
                   "better bounce on stacked pieces.")
          .toggle("restitutionpropagation", "Bounce Propagation", false,
                  "Pass bounce on through stacks of touching pieces.")
          // Both menus list the rules in the same order (kMixingMenu*); the
          // stored value is the menu index.
          .menu("frictionmixing", "Friction Mixing", 0,
                {"Geometric Mean", "Minimum", "Maximum", "Average", "Multiply"},
                "How the friction of two touching bodies is combined.")
          .menu("restitutionmixing", "Bounce Mixing", 2,
                {"Geometric Mean", "Minimum", "Maximum", "Average", "Multiply"},
                "How the bounce of two touching bodies is combined.")
          .tab("Advanced")
          .number("isotropicinertia", "Isotropic Inertia Tolerance", 0.2,
                  R{0, 1, true},
                  "Treats nearly spherical bodies as spheres for spin. Applies "
                  "to new bodies and mass changes.")
          .toggle("enablewarmstarting", "Warm Starting", true,
                  "Reuse last step's contact forces. Leave on; off makes "
                  "stacks less stable.")
          .number("impactminspeed", "Impact Min Speed", 0.0, R{0, 10},
                  "Impact data (Box3D Data's Add Impact Data) only includes "
                  "collisions faster than this. 0 includes every contact.")
          .templates();

  static SIM_DopDescription theDopDescription(
      true,           // true: create a DOP node for this solver
      "box3d_solver", // internal node name
      "Box3D Solver", // node label
      "Solver",       // data name
      classname(), templates);

  return &theDopDescription;
}

void SIM_SolverBox3D::message(SIM_ObjectArray &objects, const std::string &text,
                              UT_ErrorSeverity severity) {
  if (objects.entries() > 0)
    addError(objects(0), SIM_MESSAGE, ("Box3D: " + text).c_str(), severity);
}

void SIM_SolverBox3D::warnCount(SIM_ObjectArray &objects, int n,
                                const std::string &text) {
  if (n > 0)
    message(objects, std::to_string(n) + " " + text);
}

int SIM_SolverBox3D::getRequestedCacheSubclass() const {
  // Keep one past timestep (the world we continue from). Debug builds:
  // BOX3DDOP_REQUESTED_CACHE.
  const int value = B3D_ENV_INT("BOX3DDOP_REQUESTED_CACHE", 1);
  return value;
}

int SIM_SolverBox3D::currentWorkerCount() const {
  // Parallel tasks per step, run on Houdini's TBB pool: all Houdini threads
  // (honours HOUDINI_MAXTHREADS), or the Threads parm, never more than Houdini
  // allows.
  const int houdini =
      std::max(1, static_cast<int>(UT_Thread::getNumProcessors()));
  int n = getAutoThreads() ? houdini : static_cast<int>(getThreads());
  n = std::min(n, houdini);
  return std::clamp(n, 1, b3dop::kMaxWorkerCount);
}

b3dop::WorldSettings SIM_SolverBox3D::currentWorldSettings() const {
  b3dop::WorldSettings s;
  const UT_Vector3 g = getGravity();
  s.gravity[0] = g.x();
  s.gravity[1] = g.y();
  s.gravity[2] = g.z();
  s.subSteps = static_cast<int>(getSubSteps());
  s.workerCount = currentWorkerCount();
  s.enableSleep = getEnableSleep();
  s.contactPushSpeed = static_cast<float>(getContactPushSpeed());
  s.contactStiffness = static_cast<float>(getContactStiffness());
  s.contactDampingRatio = static_cast<float>(getContactDamping());
  s.restitutionThreshold = static_cast<float>(getBounceThreshold());
  s.contactRecycleDistance = static_cast<float>(getContactRecycleDistance());
  s.maxLinearSpeed = static_cast<float>(getMaxLinearSpeed());
  s.enableContinuous = getEnableContinuous();
  s.restitutionIterations = static_cast<int>(getRestitutionIterations());
  s.restitutionPropagation = getRestitutionPropagation();
  s.isotropicInertiaTolerance = static_cast<float>(getIsotropicInertia());
  s.warmStarting = getEnableWarmStarting();
  s.impactMinSpeed = static_cast<float>(getImpactMinSpeed());
  // Menu index -> rule. Both menus: Geometric Mean, Minimum, Maximum, Average,
  // Multiply. Out of range (set from a script): Box3D's defaults.
  using FM = b3dop::WorldSettings::FrictionMixing;
  using RM = b3dop::WorldSettings::RestitutionMixing;
  static const FM kFriction[] = {FM::GeometricMean, FM::Minimum, FM::Maximum,
                                 FM::Average, FM::Multiply};
  static const RM kRestitution[] = {RM::GeometricMean, RM::Minimum, RM::Maximum,
                                    RM::Average, RM::Multiply};
  const int fm = static_cast<int>(getFrictionMixing()),
            rm = static_cast<int>(getRestitutionMixing());
  s.frictionMixing = (fm >= 0 && fm < 5) ? kFriction[fm] : FM::GeometricMean;
  s.restitutionMixing = (rm >= 0 && rm < 5) ? kRestitution[rm] : RM::Maximum;
  return s;
}

// ---------------------------------------------------------------------------
// The step
// ---------------------------------------------------------------------------

SIM_Solver::SIM_Result SIM_SolverBox3D::solveObjectsSubclass(
    SIM_Engine &engine, SIM_ObjectArray &allObjects,
    SIM_ObjectArray &newobjects, SIM_ObjectArray & /*feedbacktoobjects*/,
    const SIM_Time &timestep) {
  // Start or end of the step: canContinue() does not care.
  const double now = engine.getSimulationTime();
  const double dt = timestep;

  // Objects created this timestep keep their initial state: they are left out
  // of the whole step (no body, no write-back) and are solved from the next
  // one. Works whether or not `newobjects` is also part of `allObjects`.
  std::unordered_set<int> newIds;
  for (exint i = 0; i < newobjects.entries(); ++i)
    newIds.insert(newobjects(i)->getObjectId());
  SIM_ObjectArray objects;
  for (exint i = 0; i < allObjects.entries(); ++i)
    if (newIds.find(allObjects(i)->getObjectId()) == newIds.end())
      objects.add(allObjects(i));

  if (objects.entries() == 0)
    return SIM_SOLVER_SUCCESS;

  // Box3D assertions logged during this solve (RelWithDebInfo builds; see
  // Box3DWorld).
  const int assertsBefore = b3dop::Box3DWorld::assertCount();
  struct AssertReport {
    SIM_SolverBox3D &solver;
    SIM_ObjectArray &objects;
    int before;
    ~AssertReport() {
      solver.warnCount(objects, b3dop::Box3DWorld::assertCount() - before,
                       "internal Box3D assertion(s) failed this step and were "
                       "ignored (details in Houdini's "
                       "console). Results may be wrong.");
    }
  } assertReport{*this, objects, assertsBefore};

  // Escape (see StepProgress).
  StepProgress progress;
  progress.boss = UTgetInterrupt();
  if (progress.check())
    return SIM_SOLVER_FAIL; // cancelled before anything changed

  SIM_Box3DWorld *worldData = findOrCreateWorldData(objects);
  if (!worldData) {
    message(objects, "could not create world data", UT_ERROR_ABORT);
    return SIM_SOLVER_FAIL;
  }

  // ---- is the cached world usable? ---------------------------------------
  char reasonBuf[200];
  const char *reason = nullptr;
  {
    Box3DSharedState *st = worldData->state();
    if (!st || !st->world.isValid()) {
      reason = "no valid world (first step, reset, or copied from empty)";
    } else {
      std::lock_guard<std::mutex> lock(st->mutex);
      st->objectsSorted = box3dObjectsSortedById(objects);
      if (st->world.settings().workerCount != currentWorkerCount()) {
        // The pool is sized at creation; a rebuild only loses warm starting and
        // sleep.
        reason = "thread count changed";
      } else if (!st->canContinue(now, dt)) {
        std::snprintf(
            reasonBuf, sizeof(reasonBuf),
            "stamp mismatch: stamped=%d stamp=%g now=%g dt=%g (now-stamp=%g)",
            (int)st->stamped, st->stampTime, now, dt, now - st->stampTime);
        reason = reasonBuf;
      }
    }
  }

  // A failed rebuild leaves the old state in place, valid but from another
  // time: never continue it.
  if (reason) {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Rebuild World");
    if (!createFreshWorld(*worldData, reason)) {
      reportWorldCreationFailure(objects);
      return SIM_SOLVER_FAIL;
    }
  }
  Box3DSharedState *st = worldData->state();
  if (!st || !st->world.isValid()) {
    reportWorldCreationFailure(objects);
    return SIM_SOLVER_FAIL;
  }

  std::lock_guard<std::mutex> lock(st->mutex);

  // The physics advances by dt * Time Scale.
  const double timeScale = std::max(0.0, static_cast<double>(getTimeScale()));
  st->stepDt = static_cast<float>(dt * timeScale);
  // Solver parameters may be animated; only the thread count needs a new world.
  const b3dop::WorldSettings tuning = currentWorldSettings();
  if (!std::isfinite(tuning.gravity[0]) || !std::isfinite(tuning.gravity[1]) ||
      !std::isfinite(tuning.gravity[2]))
    message(objects,
            "Gravity is not a finite number; the previous gravity is kept");
  st->world.applyTuning(tuning); // ignores non-finite gravity

  // Cancelled mid-sync, or before the (uninterruptible) Box3D step: the world
  // may be half updated, so no step may continue from it. Unstamped, the next
  // solve that would have continued it builds a fresh world from its geometry
  // instead.
  auto cancel = [&]() {
    st->stamped = false;
    message(objects, "step cancelled", UT_ERROR_MESSAGE);
    return SIM_SOLVER_FAIL;
  };
  if (!syncFromGeometry(*worldData, objects, static_cast<float>(dt), progress))
    return cancel();
  if (progress.check())
    return cancel();

  // Force DOPs go into the bodies now, for this step only (Box3D clears them
  // afterwards).
  if (getUseForces()) {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Apply Forces");
    const ForceStats fs = applyExternalForces(*st, objects, getForcesWake());
    warnCount(objects, fs.nonFinite,
              "body(ies) got a non-finite force or torque from a Force DOP; it "
              "was ignored");
  }

  // From here the step runs to the end (the write-back is quick), so the cached
  // result is always a whole step.
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Step World");
    st->world.step(st->stepDt);
    st->world.getMoveEvents(
        st->moves); // read once: the write-back and impacts use them
  }
  B3D_LOG_VERBOSE("  step: %d contact(s) recycled",
                  st->world.recycledContactCount());
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Write Results");
    writeResultsBack(*worldData, objects);
  }
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Constraint Feedback");
    if (const int broke = jointsAfterStep(*st, objects))
      B3D_LOG("  joints: %d broke", broke);
  }
  // Contacts are read before the queued point writes, aging and deletions.
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Record Impacts");
    recordImpacts(*st, objects, engine.getSimulationTime());
  }
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Point Updates");
    writePointAttrs(*st, objects);
    if (getEnableAging())
      ageObjects(*st, objects, static_cast<float>(dt));
    deleteDeadPoints(*st, objects);
  }

  st->stampTime = now;
  st->stamped = true;
  return SIM_SOLVER_SUCCESS;
}

// ---------------------------------------------------------------------------

SIM_Box3DWorld *
SIM_SolverBox3D::findOrCreateWorldData(SIM_ObjectArray &objects) {
  // The first object carrying world data (the carrier may change). Only the
  // carrier is fetched non-const, which is what copies the (cheap, sharing)
  // data.
  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    if (SIM_DATA_GETCONST(*obj, kBox3DWorldDataName, SIM_Box3DWorld)) {
      SIM_Box3DWorld *w =
          SIM_DATA_GET(*obj, kBox3DWorldDataName, SIM_Box3DWorld);
      return w;
    }
  }

  SIM_Object *carrier = objects(0);
  SIM_Box3DWorld *w =
      SIM_DATA_CREATE(*carrier, kBox3DWorldDataName, SIM_Box3DWorld, 0);
  return w;
}

bool SIM_SolverBox3D::createFreshWorld(SIM_Box3DWorld &worldData,
                                       const char *reason) {
  B3D_COUNT(rebuilds);
  B3D_LOG("REBUILD #%d: %s", B3D_COUNTER(rebuilds), reason);

  // The old state stays for other timesteps; the empty mirror makes every point
  // a new body.
  return worldData.createWorld(currentWorldSettings());
}

void SIM_SolverBox3D::reportWorldCreationFailure(SIM_ObjectArray &objects) {
  const int live = b3dop::Box3DWorld::liveWorldCount();
  const int max = b3dop::Box3DWorld::maxWorldCount();
  if (live >= max) {
    message(objects,
            "world limit reached (" + std::to_string(live) + " of " +
                std::to_string(max) +
                " Box3D worlds alive). Each rebuild keeps the previous world "
                "while cached frames use it. "
                "Reset the simulation or clear the DOP cache to free them, or "
                "raise BOX3DDOP_MAX_WORLDS "
                "when building the plugin.",
            UT_ERROR_ABORT);
  } else
    message(objects, "world creation failed", UT_ERROR_ABORT);
}

bool SIM_SolverBox3D::syncFromGeometry(SIM_Box3DWorld &worldData,
                                       SIM_ObjectArray &objects, float timeStep,
                                       StepProgress &progress) {
  Box3DSharedState *st = worldData.state();

  SyncCounts counts;

  ++st->syncStamp;
  st->stampedCount = 0;
  st->wantNames = anyJointNetwork(objects);
  st->pointWrites.clear();
  st->deletions.clear();
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Sync Bodies");
    for (exint i = 0; i < objects.entries(); ++i) {
      if (progress.check())
        return false;
      syncObject(*st, *objects(i), counts, progress);
      if (progress.cancelled)
        return false; // half synced: the caller discards the world
    }
  }

  // Static colliders from affector objects (Static Object DOP etc. with a
  // collision relationship).
  Box3DAffectorCounts affectorCounts;
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Sync Affectors");
    syncAffectorBodies(*st, objects, affectorCounts);
  }
  warnCount(objects, affectorCounts.failed + affectorCounts.noGeo,
            "affector object(s) have no usable collision geometry (they need "
            "polygons) and are ignored");
  if (affectorCounts.added || affectorCounts.removed || affectorCounts.moved)
    B3D_LOG("  affectors: %d, +%d built, %d moved, -%d removed, %d baked",
            affectorCounts.total, affectorCounts.added, affectorCounts.moved,
            affectorCounts.removed, affectorCounts.baked);

  // Remove bodies whose point (or object) is gone: the unstamped entries.
  int removed = 0;
  if (st->stampedCount != st->mirror.size()) {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Remove Bodies");
    std::unordered_set<int> touched; // objects that lost bodies
    for (auto it = st->mirror.begin(); it != st->mirror.end();) {
      if (!st->isLive(it->second)) {
        touched.insert(it->second.objectId);
        st->world.destroyBody(it->second.handle);
        it = st->mirror.erase(it);
        ++removed;
      } else
        ++it;
    }
    // Those objects' slots may cache pointers to erased entries; other objects
    // keep theirs.
    for (const int id : touched) {
      const auto sl = st->slots.find(id);
      if (sl != st->slots.end())
        for (Box3DPointSlot &slot : sl->second)
          slot.entry = nullptr;
    }
  }

  // Objects not seen lose their records (their bodies were just removed).
  for (auto it = st->objectSync.begin(); it != st->objectSync.end();)
    it = (it->second.visited == st->syncStamp) ? std::next(it)
                                               : st->objectSync.erase(it);

  // Joints, once the bodies they name are current. A changed set of bodies
  // re-reads the networks.
  if (counts.added || removed || affectorCounts.added || affectorCounts.removed)
    ++st->bodyVersion;
  Box3DJointCounts jointCounts;
  {
    UT_PerfMonAutoSolveEvent perf(this, "Box3D: Sync Constraints");
    syncJoints(*st, objects, timeStep, jointCounts);
  }
  {
    warnCount(objects, jointCounts.unknown,
              "constraint primitive(s) have a constraint_name that names no "
              "Box3D, Glue, Hard, Spring, Soft or Cone Twist constraint "
              "relationship and are ignored");
    warnCount(objects, jointCounts.unresolved,
              "constraint primitive(s) name a piece or object that does not "
              "exist and are ignored");
    warnCount(
        objects, jointCounts.notLines,
        "constraint primitive(s) are not two-point lines and are ignored");
    warnCount(objects, jointCounts.failed,
              "constraint primitive(s) could not be made (both ends on the "
              "world, or the same body twice)");
    if (jointCounts.created || jointCounts.removed || jointCounts.updated)
      B3D_LOG("  joints: %d network(s), +%d made, %d updated, -%d removed "
              "(joints now %d)",
              jointCounts.networks, jointCounts.created, jointCounts.updated,
              jointCounts.removed, st->world.jointCount());
  }

  // The shapes are made: let go of the hulls this sync held (the shared cache
  // keeps them).
  st->world.pruneHulls();

  // Node warnings, one per kind with a count. Fallbacks only on the step a
  // shape is built.
  for (const auto &fb : counts.fallbacks)
    warnCount(objects, fb.second, "point(s): " + fb.first);
  for (const auto &fb : counts.objectFallbacks)
    warnCount(objects, fb.second, "object(s): " + fb.first);
  if (counts.overriddenObjects > 0) {
    std::string names;
    for (const auto &od : counts.overriddenData)
      names += (names.empty() ? "" : ", ") + od.first;
    message(objects, "point attribute(s) " + names +
                         " override Box3D Data on " +
                         std::to_string(counts.overriddenObjects) +
                         " object(s). RBD Packed Object writes these from its "
                         "own parameters: set the values there");
  }
  warnCount(objects, counts.failed,
            "point(s) could not be turned into a body and are not simulated");
  warnCount(objects, counts.failedObjects,
            "object(s) could not be turned into a body and are not simulated");
  warnCount(objects, counts.duplicates,
            "point(s) share an `id` with another point; only the first of each "
            "is simulated");
  warnCount(objects, counts.badP,
            "point(s) have a non-finite P and are skipped");
  warnCount(objects, counts.badVelocity,
            "point(s) have a non-finite v or w; it was treated as zero");
  if (counts.dead && !st->deletions.empty())
    B3D_LOG("  sync: %d dead point(s) removed", counts.dead);

  if (counts.skipped)
    B3D_LOG_VERBOSE("  sync: %d of %d object(s) unchanged, scan skipped",
                    counts.skipped, static_cast<int>(objects.entries()));
  // Adds, removals and rebuilds are logged; the rest can repeat every step
  // (BOX3DDOP_VERBOSE=1).
  if (counts.added || counts.edited || counts.retyped || counts.failed ||
      counts.duplicates || counts.badP || counts.rebuilt || counts.updated ||
      removed) {
    const bool event = counts.added || removed || counts.rebuilt;
    B3D_LOG_IF(event,
               "  sync: +%d added, -%d removed, %d edited externally, %d type "
               "changes, "
               "%d shapes rebuilt, %d settings changed, "
               "%d failed, %d duplicate ids, %d non-finite P  (bodies now %d)",
               counts.added, removed, counts.edited, counts.retyped,
               counts.rebuilt, counts.updated, counts.failed, counts.duplicates,
               counts.badP, st->world.bodyCount());
  }
  return true;
}
