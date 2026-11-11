// SIM_SolverBox3D.cpp
#include "SIM_SolverBox3D.h"
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DBody.h"
#include "Box3DDebug.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_CacheOptions.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Error.h>
#include <SIM/SIM_Motion.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Error.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>

#include <cmath>
#include <cstdio>
#include <mutex>
#include <unordered_set>

namespace {

// Names the world / per-object body settings are attached under.
const char *const kWorldDataName = "Box3DWorld";
const char *const kBodyDataName  = "Box3DBody";

// A snapshot of one DOP object's rigid-body state.
using DopBodyState = Box3DBodyMirror;

bool
readObjectState(const SIM_Object &obj, DopBodyState &out)
{
    const SIM_Position *pos = obj.getPosition();
    if (!pos)
        return false;

    UT_Vector3    p;
    UT_Quaternion q;
    pos->getPosition(p);
    pos->getOrientation(q);

    out.pos[0] = p.x();  out.pos[1] = p.y();  out.pos[2] = p.z();
    out.rot[0] = q.x();  out.rot[1] = q.y();  out.rot[2] = q.z();  out.rot[3] = q.w();

    out.lin[0] = out.lin[1] = out.lin[2] = 0.0f;
    out.ang[0] = out.ang[1] = out.ang[2] = 0.0f;

    // Velocities live on the SIM_Motion flavour of the Position data.
    const SIM_Motion *motion = SIM_DATA_GETCONST(obj, SIM_POSITION_DATANAME, SIM_Motion);
    if (motion)
    {
        const UT_Vector3 v = motion->getVelocity();
        const UT_Vector3 w = motion->getAngularVelocity();   // radians/second
        out.lin[0] = v.x();  out.lin[1] = v.y();  out.lin[2] = v.z();
        out.ang[0] = w.x();  out.ang[1] = w.y();  out.ang[2] = w.z();
    }
    return true;
}

// Returns true if velocities could be stored (object has Motion data).
// Without Motion we still write position/orientation so the object moves, but
// velocity stays only inside Box3D.
bool
writeObjectState(SIM_Object &obj, const DopBodyState &s)
{
    SIM_Motion *motion = SIM_DATA_GET(obj, SIM_POSITION_DATANAME, SIM_Motion);
    if (motion)
    {
        motion->setPosition(UT_Vector3(s.pos[0], s.pos[1], s.pos[2]));
        motion->setOrientation(UT_Quaternion(s.rot[0], s.rot[1], s.rot[2], s.rot[3]));
        motion->setVelocity(UT_Vector3(s.lin[0], s.lin[1], s.lin[2]));
        motion->setAngularVelocity(UT_Vector3(s.ang[0], s.ang[1], s.ang[2]));
        return true;
    }

    SIM_PositionSimple *position = SIM_DATA_GET(obj, SIM_POSITION_DATANAME, SIM_PositionSimple);
    if (!position)
    {
        B3D_LOG("  writeBack: object id=%d Position is not writable (not SIM_PositionSimple)",
                obj.getObjectId());
        return false;
    }
    B3D_LOG("  writeBack: object id=%d has no Motion data, writing position/orientation only",
            obj.getObjectId());
    position->setPosition(UT_Vector3(s.pos[0], s.pos[1], s.pos[2]));
    position->setOrientation(UT_Quaternion(s.rot[0], s.rot[1], s.rot[2], s.rot[3]));
    return false;
}

// Returns a short description of what differs, or nullptr if the same.
const char *
describeStateDifference(const DopBodyState &a, const DopBodyState &b)
{
    const double eps = 1e-5;
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.pos[i] - b.pos[i]) > eps) return "position";

    // q and -q are the same rotation.
    double dot = 0.0;
    for (int i = 0; i < 4; ++i) dot += double(a.rot[i]) * double(b.rot[i]);
    if (std::fabs(dot) < 1.0 - 1e-6) return "orientation";

    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.lin[i] - b.lin[i]) > eps) return "linear velocity";
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a.ang[i] - b.ang[i]) > eps) return "angular velocity";
    return nullptr;
}

b3dop::BodySettings
bodySettingsFor(const SIM_Object &obj)
{
    const SIM_Box3DBody *body = SIM_DATA_GETCONST(obj, kBodyDataName, SIM_Box3DBody);
    return body ? body->toBodySettings() : b3dop::BodySettings();
}

// Add one body (and, once the core supports it, its shapes).
bool
addObjectToWorld(b3dop::Box3DWorld &world, const SIM_Object &obj, const DopBodyState &st)
{
    const uint64_t id = static_cast<uint64_t>(obj.getObjectId());
    const bool ok = world.addBody(id, bodySettingsFor(obj), st.pos, st.rot);
    // TODO: shapes from the object's Geometry / collision data.
    // PLACEHOLDER: a 1x1x1 box so the body has mass (a shapeless dynamic body
    // has zero mass and gravity does nothing to it).
    if (ok)
    {
        const float half[3] = {0.5f, 0.5f, 0.5f};
        if (!world.addBoxShape(id, half))
            B3D_LOG("  addBoxShape(id=%d) FAILED", obj.getObjectId());
    }
    if (!ok)
        B3D_LOG("  addBody(id=%d) FAILED", obj.getObjectId());
    return ok;
}

} // namespace

// ---------------------------------------------------------------------------

SIM_SolverBox3D::SIM_SolverBox3D(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_SolverBox3D::~SIM_SolverBox3D()
{
}

const SIM_DopDescription *
SIM_SolverBox3D::getDescription()
{
    static PRM_Name    theGravityName("gravity", "Gravity");
    static PRM_Default theGravityDefault[] = { 0.0, -9.80665, 0.0 };

    static PRM_Name    theSubStepsName("substeps", "Box3D Sub-Steps");
    static PRM_Default theSubStepsDefault(4);

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_FLT_J,  3, &theGravityName,  theGravityDefault),
        PRM_Template(PRM_INT_J,  1, &theSubStepsName, &theSubStepsDefault),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,    &theSleepDefault),
        PRM_Template()
    };

    static SIM_DopDescription theDopDescription(
        true,                   // true: create a DOP node for this solver
        "box3d_solver",         // internal node name
        "Box3D Solver",         // node label
        "Solver",               // data name
        classname(),
        theTemplates);

    return &theDopDescription;
}

int
SIM_SolverBox3D::getRequestedCacheSubclass() const
{
    // SIM_Solver's default is 0. The header describes this as "the amount of
    // history required by this solver". We ask for 1 timestep of history so
    // the engine keeps the previous step (the world we continue from) alive.
    // To experiment without rebuilding (debug builds only):
    //     set BOX3DDOP_REQUESTED_CACHE=0
    const int value = B3D_ENV_INT("BOX3DDOP_REQUESTED_CACHE", 1);
    return value;
}

b3dop::WorldSettings
SIM_SolverBox3D::currentWorldSettings() const
{
    b3dop::WorldSettings s;
    const UT_Vector3 g = getGravity();
    s.gravity[0]  = g.x();
    s.gravity[1]  = g.y();
    s.gravity[2]  = g.z();
    s.subSteps    = static_cast<int>(getSubSteps());
    s.enableSleep = getEnableSleep();
    return s;
}

// ---------------------------------------------------------------------------
// The step
// ---------------------------------------------------------------------------

SIM_Solver::SIM_Result
SIM_SolverBox3D::solveObjectsSubclass(SIM_Engine &engine,
                                      SIM_ObjectArray &objects,
                                      SIM_ObjectArray &newobjects,
                                      SIM_ObjectArray &feedbacktoobjects,
                                      const SIM_Time &timestep)
{
    // Whatever the engine reports as "now" during a solve. We never assume
    // whether that is the start or the end of the step; see canContinue().
    const double now = engine.getSimulationTime();
    const double dt  = timestep;


    if (objects.entries() == 0)
        return SIM_SOLVER_SUCCESS;

    SIM_Box3DWorld *worldData = findOrCreateWorldData(objects);
    if (!worldData)
    {
        addError(objects(0), SIM_MESSAGE, "Box3D: could not create world data", UT_ERROR_ABORT);
        return SIM_SOLVER_FAIL;
    }

    // ---- is the cached world usable? ---------------------------------------
    char        reasonBuf[200];
    const char *reason = nullptr;
    {
        Box3DSharedState *st = worldData->state();
        if (!worldData->hasValidWorld())
        {
            reason = "no valid world (first step, reset, or copied from empty)";
        }
        else
        {
            std::lock_guard<std::mutex> lock(st->mutex);
            if (!st->canContinue(now, dt))
            {
                std::snprintf(reasonBuf, sizeof(reasonBuf),
                              "stamp mismatch: stamped=%d stamp=%g now=%g dt=%g (now-stamp=%g)",
                              (int)st->stamped, st->stampTime, now, dt, now - st->stampTime);
                reason = reasonBuf;
            }
        }
    }

    if (reason)
        rebuildWorldFromObjects(*worldData, objects, reason);
    if (!worldData->hasValidWorld())
    {
        addError(objects(0), SIM_MESSAGE, "Box3D: world creation failed", UT_ERROR_ABORT);
        return SIM_SOLVER_FAIL;
    }

    Box3DSharedState *st = worldData->state();
    std::lock_guard<std::mutex> lock(st->mutex);

    if (!reason)
    {
        syncObjectSet(*worldData, objects, newobjects);
    }

    pushObjectStateIn(*worldData, objects);
    stepWorld(*worldData, static_cast<float>(dt));
    writeResultsBack(*worldData, objects);

    st->markStepped(now);

    publishEvents(*worldData, objects);
    return SIM_SOLVER_SUCCESS;
}

// ---------------------------------------------------------------------------

SIM_Box3DWorld *
SIM_SolverBox3D::findOrCreateWorldData(SIM_ObjectArray &objects)
{
    // objects is sorted by object id. Use the first object that already
    // carries world data, so the carrier can change without losing the world.
    // The const accessor does not trigger a copy; only the chosen carrier is
    // fetched non-const, which is the access that causes the (cheap) copy.
    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        if (SIM_DATA_GETCONST(*obj, kWorldDataName, SIM_Box3DWorld))
        {
            SIM_Box3DWorld *w = SIM_DATA_GET(*obj, kWorldDataName, SIM_Box3DWorld);
            return w;
        }
    }

    SIM_Object *carrier = objects(0);
    SIM_Box3DWorld *w = SIM_DATA_CREATE(*carrier, kWorldDataName, SIM_Box3DWorld, 0);
    return w;
}

void
SIM_SolverBox3D::rebuildWorldFromObjects(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                                         const char *reason)
{
    B3D_COUNT(rebuilds);
    B3D_LOG("REBUILD #%d: %s", B3D_COUNTER(rebuilds), reason);

    // Fresh state; the previous one (if any) is left alone for whichever
    // timestep data still points at it.
    if (!worldData.createWorld(currentWorldSettings()))
        return;

    Box3DSharedState *st = worldData.state();
    for (exint i = 0; i < objects.entries(); ++i)
    {
        const SIM_Object *obj = objects(i);
        DopBodyState s;
        if (!readObjectState(*obj, s))
        {
            B3D_LOG("  object id=%d has no Position data, skipped", obj->getObjectId());
            continue;
        }
        addObjectToWorld(st->world, *obj, s);
        st->mirror[obj->getObjectId()] = s;     // DOP state is the truth
        B3D_LOG("  body id=%d pos=(%g %g %g) lin=(%g %g %g) type=%d", obj->getObjectId(),
                s.pos[0], s.pos[1], s.pos[2], s.lin[0], s.lin[1], s.lin[2],
                (int)bodySettingsFor(*obj).type);
    }
}

void
SIM_SolverBox3D::syncObjectSet(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects,
                               SIM_ObjectArray &newobjects)
{
    Box3DSharedState *st = worldData.state();

    std::unordered_set<int> present;
    for (exint i = 0; i < objects.entries(); ++i)
        present.insert(objects(i)->getObjectId());

    // TODO: the engine says objects in `newobjects` should not be solved on the
    // step they appear in, so they keep their initial conditions. A rigid body
    // world has no per-body "skip this step"; decide how to treat them (e.g.
    // step them as kinematic for one step).

    // Removed objects.
    for (auto it = st->mirror.begin(); it != st->mirror.end();)
    {
        if (!present.count(it->first))
        {
            B3D_LOG("  sync: object id=%d removed -> removeBody", it->first);
            st->world.removeBody(static_cast<uint64_t>(it->first));
            it = st->mirror.erase(it);
        }
        else
            ++it;
    }

    // Added objects.
    for (exint i = 0; i < objects.entries(); ++i)
    {
        const SIM_Object *obj = objects(i);
        if (st->mirror.count(obj->getObjectId()))
            continue;
        DopBodyState s;
        if (!readObjectState(*obj, s))
            continue;
        B3D_LOG("  sync: object id=%d added -> addBody", obj->getObjectId());
        addObjectToWorld(st->world, *obj, s);
        st->mirror[obj->getObjectId()] = s;
    }
}

void
SIM_SolverBox3D::pushObjectStateIn(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects)
{
    Box3DSharedState *st = worldData.state();

    for (exint i = 0; i < objects.entries(); ++i)
    {
        const SIM_Object *obj = objects(i);
        auto it = st->mirror.find(obj->getObjectId());
        if (it == st->mirror.end())
            continue;

        DopBodyState cur;
        if (!readObjectState(*obj, cur))
            continue;

        const char *what = describeStateDifference(cur, it->second);
        if (!what)
            continue;

        // Someone else changed this object since we wrote it. DOPs wins.
        B3D_LOG("  external edit on object id=%d (%s) -> pushing into Box3D",
                obj->getObjectId(), what);
        const uint64_t id = static_cast<uint64_t>(obj->getObjectId());
        st->world.setTransform(id, cur.pos, cur.rot);
        st->world.setVelocity(id, cur.lin, cur.ang);
        it->second = cur;
    }

    // TODO: kinematic targets, forces from other DOP nodes, per-body setting
    //       changes (SIM_Box3DBody), gravity / sleep changes without a rebuild.
}

void
SIM_SolverBox3D::stepWorld(SIM_Box3DWorld &worldData, float timestep)
{
    worldData.state()->world.step(timestep);
}

void
SIM_SolverBox3D::writeResultsBack(SIM_Box3DWorld &worldData, SIM_ObjectArray &objects)
{
    Box3DSharedState *st = worldData.state();
    bool warnedStub = false;

    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        auto it = st->mirror.find(obj->getObjectId());
        if (it == st->mirror.end())
            continue;

        const uint64_t id = static_cast<uint64_t>(obj->getObjectId());
        DopBodyState s = it->second;    // start from what we know

        const bool gotT = st->world.getTransform(id, s.pos, s.rot);
        const bool gotV = st->world.getVelocity(id, s.lin, s.ang);

        if (!gotT || !gotV)
        {
            // Body missing from the world (e.g. addBody failed). Leave the DOP
            // state alone and re-snapshot it so the next step does not
            // misreport an "external edit".
            if (!warnedStub)
            {
                B3D_LOG("  writeBack: Box3D has no body for some objects; DOP state untouched for them");
                warnedStub = true;
            }
            DopBodyState cur;
            if (readObjectState(*obj, cur))
                it->second = cur;
            continue;
        }

        if (!writeObjectState(*obj, s))
        {
            // DOP can't hold velocity here, so what we re-read next step is 0.
            s.lin[0] = s.lin[1] = s.lin[2] = 0.0f;
            s.ang[0] = s.ang[1] = s.ang[2] = 0.0f;
        }
        it->second = s;
    }
}

void
SIM_SolverBox3D::publishEvents(SIM_Box3DWorld & /*worldData*/, SIM_ObjectArray & /*objects*/)
{
    // TODO: begin/end touch, hit, sensor, and sleep events as DOP data.
}
