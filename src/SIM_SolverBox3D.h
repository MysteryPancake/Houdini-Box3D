// SIM_SolverBox3D.h
//
// Multi-object solver. Unlike SIM_SingleSolver (one call per object), a
// SIM_Solver gets ALL objects it is responsible for in one call, which is
// what a physics engine with a single global world needs.
//
// The solver object itself holds NO simulation state (it is shared data).
// All persistent state lives in SIM_Box3DWorld, attached to an object.
#pragma once

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DataUtils.h>

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
    GETSET_DATA_FUNCS_B("enablesleep",  EnableSleep);

protected:
    explicit SIM_SolverBox3D(const SIM_DataFactory *factory);
    ~SIM_SolverBox3D() override;

    // Called once per DOP timestep with every object this solver owns.
    SIM_Result solveObjectsSubclass(SIM_Engine &engine,
                                    SIM_ObjectArray &objects,
                                    SIM_ObjectArray &newobjects,
                                    SIM_ObjectArray &feedbacktoobjects,
                                    const SIM_Time &timestep) override;

    // How many past timesteps the engine should keep for us. See the .cpp.
    int getRequestedCacheSubclass() const override;

private:
    b3dop::WorldSettings currentWorldSettings() const;

    // Find the world data on any of the objects, or create it on the lowest-id
    // object. Uses the non-const accessor only on the object that carries it.
    SIM_Box3DWorld *findOrCreateWorldData(SIM_ObjectArray &objects);

    // Throw away the cached world and rebuild from the DOP objects.
    void rebuildWorldFromObjects(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                                 const char *reason);

    // World is current: add bodies for new object ids, remove bodies for ids
    // that disappeared.
    void syncObjectSet(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                       SIM_ObjectArray &newobjects);

    // Detect edits made to DOP state since we last wrote it (other nodes,
    // kinematic animation, ...) and push them into Box3D.
    void pushObjectStateIn(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

    void stepWorld(SIM_Box3DWorld &worldData, float timestep);

    // Position, orientation, linear and angular velocity. A rebuild has only
    // this to work from, so everything it needs must be written.
    void writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

    void publishEvents(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects);

    // Debug-only helpers (empty in non-debug builds).
    void debugLogStepHeader(SIM_Engine &engine, SIM_ObjectArray &objects,
                            SIM_ObjectArray &newobjects, SIM_ObjectArray &feedback,
                            double now, double dt);
    void debugProbeBodyDefaults(SIM_ObjectArray &objects);

    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_SolverBox3D, SIM_Solver, "Box3D Solver", getDescription());
};
