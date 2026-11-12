// SIM_SolverBox3D.h
//
// Multi-object solver: one call per step with all its objects (one Box3D world). It holds no
// simulation state; that lives in SIM_Box3DWorld, attached to an object.
#pragma once

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DataUtils.h>

#include <vector>

#include "Box3DWorld.h"

class SIM_Box3DWorld;
class SIM_Engine;
class SIM_Object;
class SIM_ObjectArray;

class SIM_SolverBox3D : public SIM_Solver, public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_V3("gravity",     Gravity);
    GETSET_DATA_FUNCS_I("substeps",     SubSteps);
    GETSET_DATA_FUNCS_B("autothreads",  AutoThreads);   // every core, ignoring "threads"
    GETSET_DATA_FUNCS_I("threads",      Threads);       // 1..32
    GETSET_DATA_FUNCS_B("enablesleep",  EnableSleep);
    GETSET_DATA_FUNCS_F("timescale",    TimeScale);
    GETSET_DATA_FUNCS_F("contactpushspeed", ContactPushSpeed);  // Box3D contact speed
    GETSET_DATA_FUNCS_F("contactstiffness", ContactStiffness);  // Box3D contact hertz
    GETSET_DATA_FUNCS_F("contactdamping",   ContactDamping);    // Box3D contact damping ratio
    GETSET_DATA_FUNCS_F("bouncethreshold",  BounceThreshold);   // Box3D restitution threshold
    GETSET_DATA_FUNCS_F("maxlinearspeed",   MaxLinearSpeed);
    GETSET_DATA_FUNCS_B("enablecontinuous", EnableContinuous);
    GETSET_DATA_FUNCS_B("enablespeculative", EnableSpeculative);
    GETSET_DATA_FUNCS_B("enableaging",  EnableAging);   // age += dt; dead = 1 once age > life

protected:
    explicit SIM_SolverBox3D(const SIM_DataFactory *factory);
    ~SIM_SolverBox3D() override;

    SIM_Result solveObjectsSubclass(SIM_Engine &engine,
                                    SIM_ObjectArray &objects,
                                    SIM_ObjectArray &newobjects,
                                    SIM_ObjectArray &feedbacktoobjects,
                                    const SIM_Time &timestep) override;

    int getRequestedCacheSubclass() const override;

private:
    b3dop::WorldSettings currentWorldSettings() const;
    int                  currentWorkerCount() const;

    // The world data on the first object carrying it, else created on the lowest-id object.
    SIM_Box3DWorld *findOrCreateWorldData(SIM_ObjectArray &objects);

    // A new world for this data (other timesteps keep the old one).
    void createFreshWorld(SIM_Box3DWorld &worldData, const char *reason);

    // One body per point: adds and removes bodies and pushes the points' changes into Box3D.
    void syncFromGeometry(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

    void stepWorld(SIM_Box3DWorld &worldData, float timestep);

    // P, orient, v, w (and packed transforms) of the awake dynamic bodies. A rebuild only has
    // the geometry to work from.
    void writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_SolverBox3D, SIM_Solver, "Box3D Solver", getDescription());
};
