// SIM_SolverBox3D.h
//
// Multi-object solver: one call per step with all its objects (one Box3D
// world). It holds no simulation state; that lives in SIM_Box3DWorld, attached
// to an object.
#pragma once

#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Solver.h>
#include <UT/UT_Error.h>
#include <UT/UT_Vector3.h>

#include <string>

#include <vector>

#include "core/Box3DWorld.h"

class SIM_Box3DWorld;
class SIM_Guide;
class SIM_Engine;
struct StepProgress;
class SIM_Object;
class SIM_ObjectArray;

class SIM_SolverBox3D : public SIM_Solver, public SIM_OptionsUser {
public:
  GETSET_DATA_FUNCS_V3("gravity", Gravity);
  GETSET_DATA_FUNCS_B(
      "useforces", UseForces); // apply the Force DOPs attached to the objects
  GETSET_DATA_FUNCS_B("forceswake",
                      ForcesWake); // a force also wakes a sleeping body
  // "substeps": solver passes per Box3D step (labelled Solver Passes); keeps
  // its name so saved scenes and scripts still work.
  GETSET_DATA_FUNCS_I("substeps", SubSteps);
  // "stepsperframe": full Box3D steps per DOP step (labelled Substeps).
  GETSET_DATA_FUNCS_I("stepsperframe", StepsPerFrame);
  GETSET_DATA_FUNCS_B("autothreads",
                      AutoThreads);        // every core, ignoring "threads"
  GETSET_DATA_FUNCS_I("threads", Threads); // 1..b3dop::kMaxWorkerCount
  GETSET_DATA_FUNCS_B("enablesleep", EnableSleep);
  GETSET_DATA_FUNCS_F("timescale", TimeScale);
  GETSET_DATA_FUNCS_F("contactpushspeed",
                      ContactPushSpeed); // Box3D contact speed
  GETSET_DATA_FUNCS_F("contactstiffness",
                      ContactStiffness); // Box3D contact stiffness (Hz)
  GETSET_DATA_FUNCS_F("contactdamping",
                      ContactDamping); // Box3D contact damping ratio
  GETSET_DATA_FUNCS_F("bouncethreshold",
                      BounceThreshold); // Box3D restitution threshold
  GETSET_DATA_FUNCS_F("contactrecycledistance",
                      ContactRecycleDistance); // 0 = no contact recycling
  GETSET_DATA_FUNCS_F("maxlinearspeed", MaxLinearSpeed);
  GETSET_DATA_FUNCS_B("enablecontinuous", EnableContinuous);
  GETSET_DATA_FUNCS_I("restitutioniterations", RestitutionIterations);
  GETSET_DATA_FUNCS_B("restitutionpropagation", RestitutionPropagation);
  GETSET_DATA_FUNCS_F("isotropicinertia", IsotropicInertia);
  GETSET_DATA_FUNCS_B("enablewarmstarting", EnableWarmStarting);
  GETSET_DATA_FUNCS_F("impactminspeed", ImpactMinSpeed);
  GETSET_DATA_FUNCS_I("frictionmixing",
                      FrictionMixing); // 0 geometric mean, 1 min, 2 max, 3
                                       // average, 4 multiply (default)
  GETSET_DATA_FUNCS_I(
      "restitutionmixing",
      RestitutionMixing); // same order as friction (1 min is the default)
  GETSET_DATA_FUNCS_B("enableaging",
                      EnableAging); // age += dt; dead = 1 once age > life
  GETSET_DATA_FUNCS_B(
      "reapdead",
      ReapDead); // delete `dead` points (else keep, unsimulated)
  GETSET_DATA_FUNCS_B("reapatframeend",
                      ReapAtFrameEnd); // points dying this step: delete now

  // Guide (the node's Guides tab): every body's collision shapes.
  GET_GUIDE_FUNC_B(SIM_NAME_SHOWGUIDE, ShowGuide, false);
  GET_GUIDE_FUNC_V3(SIM_NAME_COLOR, Color, (0.2, 0.8, 1.0));

protected:
  explicit SIM_SolverBox3D(const SIM_DataFactory *factory);
  ~SIM_SolverBox3D() override;

  SIM_Result solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects,
                                  SIM_ObjectArray &newobjects,
                                  SIM_ObjectArray &feedbacktoobjects,
                                  const SIM_Time &timestep) override;

  int getRequestedCacheSubclass() const override;

  // One guide per object (SIM_GuidePerObject): its bodies' collision shapes.
  SIM_Guide *createGuideObjectSubclass() const override;
  void buildGuideGeometrySubclass(const SIM_RootData &root,
                                  const SIM_Options &options,
                                  const GU_DetailHandle &gdh,
                                  UT_DMatrix4 *xform,
                                  const SIM_Time &t) const override;

private:
  b3dop::WorldSettings currentWorldSettings() const;
  int currentWorkerCount() const;

  // The world data on the first object carrying it, else created on the
  // lowest-id object.
  SIM_Box3DWorld *findOrCreateWorldData(SIM_ObjectArray &objects);

  // A new world for this data (other timesteps keep the old one). False if
  // Box3D could not create one; the data then still holds its old state, which
  // must not be continued.
  bool createFreshWorld(SIM_Box3DWorld &worldData, const char *reason);
  // The node error for a world that could not be created (names the world limit
  // if hit).
  void reportWorldCreationFailure(SIM_ObjectArray &objects);

  // One body per point: adds and removes bodies and pushes the points' changes
  // into Box3D. False if the user cancelled (Escape) part way: the world is
  // then half synced.
  bool syncFromGeometry(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                        float timeStep, StepProgress &progress);

  // P, orient, v, w (and packed transforms) of the awake dynamic bodies. A
  // rebuild only has the geometry to work from.
  void writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

  // A node message on the first object: "Box3D: <text>". warnCount: "Box3D: <n>
  // <text>", only when n > 0.
  void message(SIM_ObjectArray &objects, const std::string &text,
               UT_ErrorSeverity severity = UT_ERROR_WARNING);
  void warnCount(SIM_ObjectArray &objects, int n, const std::string &text);

  static const SIM_DopDescription *getDescription();

  DECLARE_STANDARD_GETCASTTOTYPE();
  DECLARE_DATAFACTORY(SIM_SolverBox3D, SIM_Solver, "Box3D Solver",
                      getDescription());
};
