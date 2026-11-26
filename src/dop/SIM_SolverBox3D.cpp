// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DData.h"
#include "SIM_Box3DParms.h"
#include "SIM_Box3DWorld.h"
#include "core/Box3DCpu.h"
#include "core/Box3DDebug.h"
#include "sync/Box3DAffectors.h"
#include "sync/Box3DCompoundSync.h"
#include "sync/Box3DJointSync.h"
#include "sync/Box3DSolverInternal.h"

#include <SIM/SIM_CacheOptions.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_GuidePerObject.h>
#include <UT/UT_PerfMonAutoEvent.h>
#include <UT/UT_Thread.h>

#include <algorithm>
#include <cstdio>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace box3dsolver;

namespace {

// Event names are written "Category}/{Stage": Houdini wraps a name in braces
// and nests the event under its parent at each "/" outside them, so the
// Performance Monitor shows {Category} with {Stage} below it, as the Bullet
// solver's tree does. A plain "A/B" or "A/{B}" gives unbalanced braces.

// Steps the world inside an event of its own, so the profile shows the time of
// Box3D's step apart from the plugin's own stages.
void stepWorld(const SIM_Solver *solver, b3dop::Box3DWorld &world, float dt) {
  UT_PerfMonAutoSolveEvent e(solver, "Stepping Simulation}/{Box3D Step");
  world.step(dt);
}

} // namespace

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
      p.tab("Solver")
          .number("timescale", "Time Scale", 1.0, R{0, 2},
                  "Multiplies the time step: below 1 is slow motion, above 1 "
                  "fast motion.")
          // Outer loop: whole Box3D steps, each with its own collision
          // detection. Slider 1 to 100; minimum 1, more can be typed.
          .integer("substeps", "Substeps", 2, R{1, 100},
                   "Box3D steps per time step. 1 steps once per frame (the "
                   "scene FPS), 2 twice, and so on. Each step detects "
                   "collisions again and runs its own Constraint Iterations, "
                   "so more catches fast or thin objects better, at a cost of "
                   "roughly Substeps x Constraint Iterations.")
          // Inner loop. Slider 1 to 100; minimum 1 (Box3D clamps to it), more
          // can be typed.
          .integer("solverpasses", "Constraint Iterations", 4, R{1, 100},
                   "Contact and joint solves per Box3D step. Each one is a "
                   "smaller time step, not another pass at the same "
                   "positions. More is stiffer and more stable for stacks "
                   "and chains, but slower. Raise Substeps instead for "
                   "collisions missed between frames.")
          .subfolder("Contacts")
          // Stiffer and less damped than Box3D's own defaults (30 Hz, 10):
          // a more rigid look. Box3D caps the stiffness at 1/8 of the pass
          // rate (fps x Substeps x Constraint Iterations), so 60 Hz needs 20 in
          // total at 24 fps; the defaults (2 x 4 = 8) give 24 Hz.
          .number("contactstiffness", "Contact Stiffness", 60.0, R{0, 120},
                  "How stiff contacts are (cycles per second). Higher values "
                  "recover from overlap faster but can jitter. Capped at 1/8 "
                  "of FPS x Substeps x Constraint Iterations.")
          .number("contactdamping", "Contact Damping Ratio", 3.0, R{0, 20},
                  "How much contacts damp overlap recovery. Lower values "
                  "recover faster but more energetically.")
          .number("contactpushspeed", "Overlap Push Speed", 3.0, R{0, 10},
                  "How fast overlapping pieces are pushed apart. Lower it if "
                  "pieces that start out overlapping fly apart.")
          .number("contactrecycledistance", "Contact Recycle Distance", 0.05,
                  R{0, 0.2},
                  "Reuse contacts for pairs that barely moved: faster but "
                  "slightly less exact. 0 turns it off. "
                  "Box3D Data's Contact Recycling turns it off per object.")
          .subfolder("Sleeping")
          .toggle("enablesleep", "Allow Sleeping", true,
                  "Let resting bodies stop simulating until touched. Off keeps "
                  "every body awake, whatever Box3D Data says.")
          .subfolder("Aging")
          .toggle("enableaging", "Enable Aging", false,
                  "Add the frame time step (not Time Scale-adjusted) to each "
                  "point's age, and mark points dead once age exceeds life.")
          .toggle("reapdead", "Reap Dead Points", true,
                  "Delete points whose dead attribute is 1. Off keeps them "
                  "where they died, no longer simulated.")
          .toggle("reapatframeend", "Reap At Frame End", true,
                  "Delete points that die during a step at its end, so dead "
                  "points are never seen. Off keeps them for one frame.",
                  "{ reapdead == 0 }")
          .tab("Forces")
          .vector("gravity", "Gravity", {0.0, -9.80665, 0.0},
                  "Acceleration of every dynamic body, in units per second "
                  "squared. Scale per body with Gravity Scale on Box3D Data.")
          .subfolder("Force DOPs")
          .toggle("useforces", "Use Force DOPs", true,
                  "Apply Force DOPs connected to the objects (Wind, Fan, "
                  "etc.), and the drag attributes (airresist, spinresist, "
                  "targetv, targetw). Set Gravity to 0 when using a Gravity "
                  "Force.")
          .toggle("forceswake", "Forces Wake Bodies", false,
                  "Let forces wake sleeping bodies, e.g. wind that should blow "
                  "resting pieces away.",
                  "{ useforces == 0 }")
          .subfolder("Bounce / Friction")
          .toggle("restitutionpropagation", "Bounce Propagation", false,
                  "Pass bounce on through stacks of touching pieces.")
          .number("bouncethreshold", "Bounce Threshold", 1.0, R{0, 5},
                  "Collisions slower than this speed don't bounce, so resting "
                  "pieces settle instead of jittering.")
          .integer("restitutioniterations", "Bounce Iterations", 2, R{0, 10},
                   "Extra solver passes that compute bounce. More passes give "
                   "better bounce on stacked pieces.")
          // Both menus list the rules in the same order (currentWorldSettings);
          // the stored value is the menu index. Defaults are Bullet's rules as
          // measured in Houdini (test bullet_mixing): multiply friction and
          // bounce.
          .menu("frictionmixing", "Friction Mixing", 4,
                {"Geometric Mean", "Minimum", "Maximum", "Average", "Multiply"},
                "How the friction of two touching bodies is combined.")
          .menu("restitutionmixing", "Bounce Mixing", 4,
                {"Geometric Mean", "Minimum", "Maximum", "Average", "Multiply"},
                "How the bounce of two touching bodies is combined.")
          .subfolder("Glue")
          .toggle("gluecompound", "Glue as One Body", true,
                  "Run each group of glued pieces as one rigid body: stiffer "
                  "and faster for large chunks. A fast heavy body frees the "
                  "pieces in its way.")
          .number("gluecompoundload", "Glue Inertial Load", 0.0, R{0, 10},
                  "Also breaks glued bonds away from a hit, by the load that "
                  "slowing the whole body puts on them. 0 is off, 1 is "
                  "physical.",
                  "{ gluecompound == 0 }")
          .toggle("compoundids", "Write Compound IDs", false,
                  "Write each glued piece's compound_id on its point: the "
                  "lowest point id in its compound, -1 once it is out of one.",
                  "{ gluecompound == 0 }")
          // Show Collision Shapes and Shape Color are guide parameters (the
          // node's Guide Options tab), kept out of the tabs.
          .tab("Visualization")
          .subfolder("Geometry")
          .toggle("exportshapes", "Export Collision Shapes", false,
                  "Write every body's collision shapes as data on its object "
                  "(for the Box3D Solver SOP's Show Collision Shapes). Costs "
                  "time every step. Show Collision Shapes and Shape Color are "
                  "on the Guide Options tab.")
          .tab("Advanced")
          .subfolder("Stability")
          .toggle("enablewarmstarting", "Warm Starting", true,
                  "Reuse last step's contact forces. Leave on; off makes "
                  "stacks less stable.")
          .toggle("enablecontinuous", "Continuous Collision", true,
                  "Stop fast bodies passing through static ones (and through "
                  "moving ones, for Box3D Data's Fast Object).")
          .number("maxlinearspeed", "Max Linear Speed", 400.0, R{0, 1000},
                  "Speed limit for every body, in units per second.")
          .number("isotropicinertia", "Isotropic Inertia Tolerance", 0.2,
                  R{0, 1, true},
                  "Treats nearly spherical bodies as spheres for spin. Applies "
                  "to new bodies and mass changes.")
          .subfolder("Ground Box")
          .vector("groundsize", "Ground Box Size", {1000, 10, 1000},
                  "The box a Ground Plane DOP or a static plane piece makes: "
                  "its width "
                  "along x, its depth below the surface (y) and its length "
                  "along z, centred on its position. Nothing collides past "
                  "the edge.",
                  PRM_XYZ_J)
          .subfolder("Performance")
          .toggle("autothreads", "Use All Threads", false,
                  autoThreadsHelp.c_str())
          .integer("threads", "Threads", b3dop::defaultWorkerCount(),
                   R{1, static_cast<double>(b3dop::kMaxWorkerCount), true},
                   "Number of worker threads. Defaults to this computer's "
                   "physical performance cores; small scenes can run faster "
                   "with fewer.",
                   "{ autothreads == 1 }")
          .tab("Output")
          .subfolder("Impact Data")
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

  // Guide parameters (Houdini puts them on the node's Guide Options tab, kept
  // out of the simulation so changing them never re-simulates), read through
  // GET_GUIDE_FUNC_*. Static memory, like the description.
  static Box3DParms g;
  static const PRM_Template *guideTemplates =
      g.toggle(SIM_NAME_SHOWGUIDE, "Show Collision Shapes", false,
               "Draw every body's collision shapes as Box3D sees them. "
               "Sleeping and static bodies are drawn darker.")
          .vector(SIM_NAME_COLOR, "Shape Color", {0.2, 0.8, 1.0},
                  "Colour of the collision shapes.", PRM_RGB)
          .templates();
  static const bool guidesSet =
      (theDopDescription.setGuideTemplates(
           const_cast<PRM_Template *>(guideTemplates)),
       true);
  (void)guidesSet;

  return &theDopDescription;
}

SIM_Guide *SIM_SolverBox3D::createGuideObjectSubclass() const {
  return new SIM_GuidePerObject(this);
}

void SIM_SolverBox3D::buildGuideGeometrySubclass(const SIM_RootData &root,
                                                 const SIM_Options &options,
                                                 const GU_DetailHandle &gdh,
                                                 UT_DMatrix4 *xform,
                                                 const SIM_Time & /*t*/) const {
  if (xform)
    xform->identity(); // the guide is built in world space
  if (!getShowGuide(options) || gdh.isNull())
    return;
  const SIM_Object *obj = dynamic_cast<const SIM_Object *>(&root);
  if (!obj)
    return;
  GU_DetailHandleAutoWriteLock lock(gdh);
  GU_Detail *gdp = lock.getGdp();
  if (gdp)
    buildCollisionGuide(*obj, *gdp, getColor(options));
}

void SIM_SolverBox3D::message(SIM_ObjectArray &objects, const std::string &text,
                              UT_ErrorSeverity severity) {
  if (objects.entries() > 0)
    addError(objects(0), SIM_MESSAGE, text.c_str(), severity);
}

void SIM_SolverBox3D::warnCount(SIM_ObjectArray &objects, int n,
                                const std::string &text) {
  if (n > 0)
    message(objects, std::to_string(n) + " " + text);
}

int SIM_SolverBox3D::getRequestedCacheSubclass() const {
  // Keep one past timestep (the world we continue from). Debug builds:
  // BOX3DDOP_REQUESTED_CACHE.
  const int value = B3D_DEBUG_ENV_INT("BOX3DDOP_REQUESTED_CACHE", 1);
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
  s.subSteps = static_cast<int>(getSolverPasses());
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
  {
    const UT_Vector3 groundSize = getGroundSize();
    for (unsigned i = 0; i < 3; ++i)
      s.ground.size[i] = static_cast<float>(groundSize(i));
  }
  // Menu index -> rule. Both menus: Geometric Mean, Minimum, Maximum, Average,
  // Multiply. Out of range (set from a script): the menus' defaults.
  using FM = b3dop::WorldSettings::FrictionMixing;
  using RM = b3dop::WorldSettings::RestitutionMixing;
  static const FM kFriction[] = {FM::GeometricMean, FM::Minimum, FM::Maximum,
                                 FM::Average, FM::Multiply};
  static const RM kRestitution[] = {RM::GeometricMean, RM::Minimum, RM::Maximum,
                                    RM::Average, RM::Multiply};
  const int fm = static_cast<int>(getFrictionMixing()),
            rm = static_cast<int>(getRestitutionMixing());
  s.frictionMixing = (fm >= 0 && fm < 5) ? kFriction[fm] : FM::Multiply;
  s.restitutionMixing = (rm >= 0 && rm < 5) ? kRestitution[rm] : RM::Multiply;
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

  // Objects created this timestep keep their initial state unless their object
  // node has Solve On Creation Frame on: the others are left out of the whole
  // step (no body, no write-back) and are solved from the next one. Works
  // whether or not `newobjects` is also part of `allObjects`.
  std::unordered_set<int> waitIds; // new, not solved on their creation frame
  for (int i = 0; i < newobjects.entries(); ++i)
    if (!newobjects(i)->getSolveFirstFrame())
      waitIds.insert(newobjects(i)->getObjectId());
  SIM_ObjectArray objects;
  std::unordered_set<int> kept;
  for (int i = 0; i < allObjects.entries(); ++i)
    if (waitIds.find(allObjects(i)->getObjectId()) == waitIds.end() &&
        kept.insert(allObjects(i)->getObjectId()).second)
      objects.add(allObjects(i));
  for (int i = 0; i < newobjects.entries(); ++i)
    if (newobjects(i)->getSolveFirstFrame() &&
        kept.insert(newobjects(i)->getObjectId()).second)
      objects.add(newobjects(i));

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
  progress.solver = this;
  progress.boss = UTgetInterrupt();
  if (progress.check())
    return SIM_SOLVER_FAIL; // cancelled before anything changed

  SIM_Box3DWorld *worldData = findOrCreateWorldData(objects);
  if (!worldData) {
    message(objects, "could not create world data", UT_ERROR_ABORT);
    return SIM_SOLVER_FAIL;
  }

  // The solver parameters now. Animated ones reach the live world through
  // applyTuning; only the thread count needs a new world.
  const b3dop::WorldSettings tuning = currentWorldSettings();

  // ---- is the cached world usable? ---------------------------------------
  char reasonBuf[200];
  const char *reason = nullptr;
  {
    Box3DSharedState *st = worldData->state();
    if (!st || !st->world.isValid()) {
      reason = "no valid world (first step, reset, or copied from empty)";
    } else {
      std::lock_guard<std::mutex> lock(st->mutex);
      if (st->world.settings().workerCount != tuning.workerCount) {
        // The pool is sized at creation; a rebuild only loses warm starting and
        // sleep.
        reason = "thread count changed";
      } else if (!st->canContinue(now, dt)) {
        std::snprintf(
            reasonBuf, sizeof(reasonBuf),
            "stamp mismatch: stamped=%d stamp=%g now=%g dt=%g (now-stamp=%g)",
            static_cast<int>(st->stamped), st->stampTime, now, dt,
            now - st->stampTime);
        reason = reasonBuf;
      }
    }
  }

  // Performance Monitor: one solve event per stage (below), under the
  // Pre-Solve, Stepping Simulation and Post-Solve categories (a "/" in the name
  // nests).

  // A failed rebuild leaves the old state in place, valid but from another
  // time: never continue it.
  if (reason) {
    UT_PerfMonAutoSolveEvent perf(this, "Pre-Solve}/{Rebuilding World");
    if (!createFreshWorld(*worldData, tuning, reason)) {
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
  // Also for a freshly made state, which starts unsorted.
  st->objectsSorted = box3dObjectsSortedById(objects);

  followSourceSops(*st, objects, now, dt);

  // The physics advances by dt * Time Scale.
  const double timeScale = std::max(0.0, static_cast<double>(getTimeScale()));
  st->stepDt = static_cast<float>(dt * timeScale);
  if (!b3dop::finite3(tuning.gravity))
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
  if (!syncFromGeometry(*worldData, objects, progress))
    return cancel();
  if (progress.check())
    return cancel();
  // Impacts and Glue's hits read Box3D's impulse events, made only while
  // something wants them.
  st->glueHitsFromEvents = !st->glueNodes.empty();
  st->world.enableContactImpulseEvents(wantsImpacts(*st) ||
                                       st->glueHitsFromEvents);

  // Substeps: the frame is split into n whole Box3D steps of subDt, each with
  // its own collision detection and Constraint Iterations. st->stepDt stays the
  // whole frame's time: a kinematic target and a Glue's break force are per
  // frame.
  const int n =
      st->stepDt > 0.0f ? std::max(static_cast<int>(getSubsteps()), 1) : 1;
  const float subDt = st->stepDt / static_cast<float>(n);
  reaimJoints(*st); // joints whose anchors are together aim along their line
  refreshMotion(*st, objects); // drag and speed limit attributes

  // From here the step runs to the end (the write-back is quick), so the cached
  // result is always a whole step.
  st->brokenCollected = false;
  st->hitsCollected = false;
  st->impactsCollected = false;
  st->impactKeys = n > 1;
  const char *const kNonFiniteForce =
      "body(ies) got a non-finite force or torque from a Force DOP; it was "
      "ignored";
  // Hit events: only the Impact Min Speed filter reads them.
  const bool wantHits = st->world.settings().impactMinSpeed > 0.0f;
  if (n == 1) { // one step, nothing to merge
    if (getUseForces()) {
      UT_PerfMonAutoSolveEvent perf(this,
                                    "Stepping Simulation}/{Applying Forces");
      const ForceStats fs =
          applyExternalForces(*st, objects, getForcesWake(), subDt);
      warnCount(objects, fs.nonFinite, kNonFiniteForce);
    }
    std::vector<b3dop::BodyMove> left; // pieces of a compound taken apart
    {
      UT_PerfMonAutoSolveEvent e(this, "Stepping Simulation}/{Compound Motion");
      recordCompoundMotion(*st);
      holdCompoundsForApproach(*st, st->moves, left);
    }
    stepWorld(this, st->world, subDt);
    if (wantHits) {
      st->world.updateHitEvents();
      st->hitsCollected = true;
    }
    {
      UT_PerfMonAutoSolveEvent e(this, "Stepping Simulation}/{Glue Hits");
      collectGlueHits(*st);
    }
    st->world.getMoveEvents(st->moves); // read once: write-back and impacts
    applySpeedLimits(*st, st->moves);
    // Before the step's own events: they carry the final poses.
    st->moves.insert(st->moves.begin(), left.begin(), left.end());
    {
      UT_PerfMonAutoSolveEvent e(this,
                                 "Stepping Simulation}/{Compound Bridges");
      pruneCompoundBridges(*st);
    }
  } else {
    // The last frame's events: the bodies to look at before the first step.
    const std::vector<b3dop::BodyMove> lastFrame = std::move(st->moves);
    st->moves.clear();
    st->brokenThisFrame.clear();
    st->brokenCollected = true;
    // Hit events are cleared every step too; only the speed filter reads them.
    const bool collectHits = wantHits;
    st->hitsCollected = collectHits;
    std::unordered_map<b3dop::BodyHandle, size_t> &slot = st->moveSlots;
    std::vector<b3dop::BodyMove> &events = st->stepEvents;
    std::vector<void *> &broken = st->stepBroken;
    slot.clear();
    ForceStats forceStats;
    PointForceCarry carry;
    const bool collectImpactHits = wantsImpacts(*st);
    for (int i = 0; i < n; ++i) {
      // Force DOPs go into the bodies for one Box3D step only (it clears them
      // afterwards), so every step applies them again, the point forces (read
      // and cleared by the first) from `carry`.
      if (getUseForces()) {
        UT_PerfMonAutoSolveEvent perf(this,
                                      "Stepping Simulation}/{Applying Forces");
        carry.replay = i > 0;
        const ForceStats fs =
            applyExternalForces(*st, objects, getForcesWake(), subDt, &carry);
        forceStats.nonFinite = std::max(forceStats.nonFinite, fs.nonFinite);
      }
      bool mayRepeat = false; // a body may be in `events` twice
      {
        std::vector<b3dop::BodyMove> left; // pieces of a compound taken apart
        {
          UT_PerfMonAutoSolveEvent e(this,
                                     "Stepping Simulation}/{Compound Motion");
          recordCompoundMotion(*st);
          holdCompoundsForApproach(*st, i == 0 ? lastFrame : events, left);
        }
        mayRepeat = !left.empty();
        stepWorld(this, st->world, subDt);
        // Before the glue reads them; appended across the frame's steps.
        if (collectHits)
          st->world.updateHitEvents(i > 0);
        {
          UT_PerfMonAutoSolveEvent e(this, "Stepping Simulation}/{Glue Hits");
          collectGlueHits(*st);
        }
        st->world.getMoveEvents(events);
        applySpeedLimits(*st, events);
        // Before the step's own events: they carry the final poses.
        events.insert(events.begin(), left.begin(), left.end());
        {
          UT_PerfMonAutoSolveEvent e(this,
                                     "Stepping Simulation}/{Compound Bridges");
          pruneCompoundBridges(*st);
        }
        // Joint events are cleared by the next step too: keep every joint that
        // went over its threshold (once each).
        st->world.brokenJoints(broken);
        st->brokenThisFrame.insert(st->brokenThisFrame.end(), broken.begin(),
                                   broken.end());
        // This step's contacts are impacts too (their impulses add up over
        // the frame).
        if (collectImpactHits) {
          UT_PerfMonAutoSolveEvent e(
              this, "Stepping Simulation}/{Collecting Contacts");
          collectImpacts(*st, i > 0);
        }
      }
      // Box3D clears its move events every step, so merge them: the last
      // event of a body is its final pose (and its fell-asleep flag).
      mergeMoveEvents(*st, events, slot, mayRepeat);
    }
    // Once each, in the order Box3D gave them.
    {
      std::unordered_set<void *> seen;
      std::vector<void *> &b = st->brokenThisFrame;
      b.erase(std::remove_if(b.begin(), b.end(),
                             [&](void *p) { return !seen.insert(p).second; }),
              b.end());
    }
    warnCount(objects, forceStats.nonFinite, kNonFiniteForce);
  }
  B3D_DEBUG_LOG_VERBOSE("  step: %d contact(s) recycled",
                        st->world.recycledContactCount());
  {
    UT_PerfMonAutoSolveEvent perf(this,
                                  "Post-Solve}/{Exporting Simulation Results");
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Exporting Simulation Results}/{Compound Pieces");
      expandCompoundMoves(*st); // the pieces of moved compounds
    }
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Exporting Simulation Results}/{Writing Back");
      writeResultsBack(*worldData, objects);
    }
  }
  {
    UT_PerfMonAutoSolveEvent perf(this,
                                  "Post-Solve}/{Updating Constraint States");
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Updating Constraint States}/{Advancing Glue");
      advanceGlue(*st, st->stepDt);
    }
    st->constraintAgeDt = getEnableAging() ? static_cast<float>(dt) : 0.0f;
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Updating Constraint States}/{Writing Joints");
      if (const int broke = jointsAfterStep(*st, objects))
        B3D_DEBUG_LOG("  joints: %d broke", broke);
    }
  }
  // Contacts are read before the queued point writes, aging and deletions.
  {
    UT_PerfMonAutoSolveEvent perf(this, "Post-Solve}/{Recording Impacts");
    recordImpacts(*st, objects, engine.getSimulationTime());
  }
  {
    UT_PerfMonAutoSolveEvent perf(this, "Post-Solve}/{Updating Points");
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Updating Points}/{Writing Attributes");
      queueCompoundIds(*st);
      clearMotionInputs(*st, objects);
      writePointAttrs(*st, objects);
    }
    if (getEnableAging()) {
      UT_PerfMonAutoSolveEvent e(this, "Post-Solve}/{Updating Points}/{Aging");
      ageObjects(*st, objects, static_cast<float>(dt));
    }
    {
      UT_PerfMonAutoSolveEvent e(
          this, "Post-Solve}/{Updating Points}/{Deleting Dead Points");
      deleteDeadPoints(*st, objects);
    }
  }
  if (getExportShapes()) {
    UT_PerfMonAutoSolveEvent perf(this,
                                  "Post-Solve}/{Exporting Collision Shapes");
    exportCollisionShapes(*st, objects);
  } else {
    st->shapesFresh = false; // the geometry is left as it was: out of date
  }
  // Every write lock is released: our own writes are not external edits.
  recordFingerprintsAfterWrites(*st, objects);

  st->stampTime = now;
  st->stamped = true;
  worldData->claimMemory();
  return SIM_SOLVER_SUCCESS;
}

// ---------------------------------------------------------------------------

SIM_Box3DWorld *
SIM_SolverBox3D::findOrCreateWorldData(SIM_ObjectArray &objects) {
  // The first object carrying world data (the carrier may change). Only the
  // carrier is fetched non-const, which is what copies the (cheap, sharing)
  // data.
  for (int i = 0; i < objects.entries(); ++i) {
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
                                       const b3dop::WorldSettings &settings,
                                       const char *reason) {
  B3D_DEBUG_COUNT(rebuilds);
  B3D_DEBUG_LOG("REBUILD #%d: %s", B3D_DEBUG_COUNTER(rebuilds), reason);

  // The old state stays for other timesteps; the empty mirror makes every point
  // a new body.
  return worldData.createWorld(settings);
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
                                       SIM_ObjectArray &objects,
                                       StepProgress &progress) {
  Box3DSharedState *st = worldData.state();

  SyncCounts counts;

  ++st->syncStamp;
  st->stampedCount = 0;
  st->fullScans = 0;
  st->wantNames = anyJointNetwork(objects);
  st->reapDead = getReapDead();
  st->reapAtFrameEnd = getReapAtFrameEnd();
  st->glueCompounds = getGlueCompound();
  st->compoundIds = getCompoundIds();
  st->glueCompoundLoad =
      std::max(0.0f, static_cast<float>(getGlueCompoundLoad()));
  st->removedBodies.clear();
  st->bodiesAdded = false;
  st->allNetworksDirty = false;
  st->pointWrites.clear();
  st->deletions.clear();
  st->writtenObjects.clear();
  {
    UT_PerfMonAutoSolveEvent perf(this, "Pre-Solve}/{Updating Bodies");
    for (int i = 0; i < objects.entries(); ++i) {
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
    UT_PerfMonAutoSolveEvent perf(this, "Pre-Solve}/{Updating Colliders");
    syncAffectorBodies(*st, objects, affectorCounts);
  }
  warnCount(objects, affectorCounts.failed + affectorCounts.noGeo,
            "affector object(s) have no usable collision geometry (they need "
            "polygons or a height field) and are ignored");
  if (affectorCounts.added || affectorCounts.removed || affectorCounts.moved)
    B3D_DEBUG_LOG(
        "  affectors: %d, +%d built (%d height field, %d plane), %d moved, "
        "-%d removed, %d baked",
        affectorCounts.total, affectorCounts.added, affectorCounts.heightFields,
        affectorCounts.planes, affectorCounts.moved, affectorCounts.removed,
        affectorCounts.baked);

  // Remove bodies whose point (or object) is gone: the unstamped entries.
  int removed = 0;
  if (st->stampedCount != st->mirror.size()) {
    UT_PerfMonAutoSolveEvent perf(this, "Pre-Solve}/{Removing Bodies");
    std::unordered_set<int> touched; // objects that lost bodies
    removed = static_cast<int>(st->mirror.eraseIf([&](Box3DBodyMirror &e) {
      if (st->isLive(e))
        return false;
      touched.insert(e.objectId);
      st->removedBodies.push_back(e.handle);
      st->world.destroyBody(e.handle);
      return true;
    }));
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
    if (it->second.visited == st->syncStamp) {
      ++it;
    } else {
      // Their per-point and source records go with them.
      st->slots.erase(it->first);
      st->sourcePoses.erase(it->first);
      st->sourceCaches.erase(it->first);
      it = st->objectSync.erase(it);
    }

  // Joints, once the bodies they name are current. A changed set of bodies
  // re-reads the networks.
  st->bodiesAdded = counts.added > 0;
  st->allNetworksDirty = affectorCounts.added || affectorCounts.removed;
  if (counts.added || removed || affectorCounts.added || affectorCounts.removed)
    ++st->bodyVersion;
  Box3DJointCounts jointCounts;
  {
    UT_PerfMonAutoSolveEvent perf(this, "Pre-Solve}/{Updating Constraints");
    syncJoints(*st, objects, jointCounts);
    releaseExpiredHolds(*st, st->stepDt);
    syncCompounds(*st); // glued groups as one body (off by default)
  }
  {
    warnCount(objects, jointCounts.unknown,
              "constraint primitive(s) have a constraint_name that names no "
              "Box3D, Glue, Hard, Spring, Soft or Cone Twist constraint "
              "relationship and are ignored");
    warnCount(objects, jointCounts.typeIgnored,
              "constraint primitive(s) on a Box3D joint node have a "
              "constraint_type of position or rotation, which is only read for "
              "Glue, Hard, Spring, Soft and Cone Twist constraints (the joint "
              "type decides what is constrained)");
    warnCount(objects, jointCounts.unresolved,
              "constraint primitive(s) name a piece or object that does not "
              "exist and are ignored");
    warnCount(objects, jointCounts.badAnchorId,
              "constraint anchor(s) have an anchor_id that is not a point or "
              "vertex of the named object and are ignored");
    warnCount(objects, jointCounts.anchorNoGeometry,
              "constraint anchor(s) have an anchor_id on a body whose "
              "geometry could not be read and are ignored");
    warnCount(objects, jointCounts.agentAnchors,
              "constraint anchor(s) have anchor_type agent, which is not "
              "supported, and are ignored");
    warnCount(
        objects, jointCounts.notLines,
        "constraint primitive(s) are not two-point lines and are ignored");
    warnCount(objects, jointCounts.failed,
              "constraint primitive(s) could not be made (both ends on the "
              "world, or the same body twice)");
    if (jointCounts.created || jointCounts.removed || jointCounts.updated)
      B3D_DEBUG_LOG(
          "  joints: %d network(s), +%d made, %d updated, -%d removed "
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
                         " object(s): the points' values are used. RBD Packed "
                         "Object writes these from its own parameters: set "
                         "them there, or on a Box3D Packed Object");
  }
  for (const std::string &name : counts.unsupportedObjects)
    message(objects,
            name + ": Geometry does not contain any primitive types supported "
                   "by the solver. Pack the pieces (Pack or Assemble SOP), or "
                   "use the Box3D Solver SOP, which packs its inputs. The "
                   "object is not simulated");
  warnCount(objects, counts.unpackedPrimPoints,
            "point(s) belong to primitives that are not packed and are not "
            "simulated (only packed primitives and loose points are)");
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
    B3D_DEBUG_LOG("  sync: %d dead point(s) removed", counts.dead);

  if (counts.skipped)
    B3D_DEBUG_LOG_VERBOSE("  sync: %d of %d object(s) unchanged, scan skipped",
                          counts.skipped, static_cast<int>(objects.entries()));
  // Adds, removals and rebuilds are logged; the rest can repeat every step
  // (BOX3DDOP_VERBOSE=1).
  if (counts.added || counts.edited || counts.retyped || counts.failed ||
      counts.duplicates || counts.badP || counts.rebuilt || counts.updated ||
      removed) {
    const bool event = counts.added || removed || counts.rebuilt;
    B3D_DEBUG_LOG_IF(
        event,
        "  sync: +%d added, -%d removed, %d edited externally, %d type "
        "changes, "
        "%d shapes rebuilt, %d settings changed, "
        "%d failed, %d duplicate ids, %d non-finite P  (bodies now %d)",
        counts.added, removed, counts.edited, counts.retyped, counts.rebuilt,
        counts.updated, counts.failed, counts.duplicates, counts.badP,
        st->world.bodyCount());
  }
  return true;
}
