// Box3DForces.cpp
//
// Force DOPs (SIM_Force data on the objects) and point force / torque attributes, applied to
// the bodies right before the step.
#include "Box3DSolverInternal.h"

namespace box3dsolver {

// Collects the SIM_Force data an object carries (Gravity Force, Wind Force, Fan Force, POP
// Force, any custom force DOP): the lookup Houdini's own solvers use (see SIM_SolverHair).
class ForceCollector : public SIM_EachDataCallback
{
public:
    explicit ForceCollector(std::vector<const SIM_Force *> &out) : myOut(out) {}

    void callbackConst(const SIM_Data *data, const char *) override
    {
        if (const SIM_Force *f = SIM_DATA_CASTCONST(data, SIM_Force))
            myOut.push_back(f);
    }

private:
    std::vector<const SIM_Force *> &myOut;
};

// The data ids of an object's `force` / `torque` attributes (kNoId when absent).
void
forceAttribIds(const GU_Detail *gdp, GA_DataId &forceId, GA_DataId &torqueId)
{
    const GA_Attribute *f = gdp->findPointAttribute("force");
    const GA_Attribute *t = gdp->findPointAttribute("torque");
    forceId  = f ? f->getDataId() : Box3DSyncFingerprint::kNoId;
    torqueId = t ? t->getDataId() : Box3DSyncFingerprint::kNoId;
}

// Force DOPs: every dynamic body takes the sum of the SIM_Force data on its object, evaluated at
// its centre of mass with its mass and velocities, the way Houdini's solvers do, plus its point's
// `force` / `torque` attributes. The force and torque go into Box3D for the coming step (it
// integrates them in every substep and clears them afterwards), so this runs after the sync and
// right before the step. A Gravity Force DOP adds to the solver's own Gravity: set that to 0 when
// using one. Only the bodies of objects that have forces are visited (Box3DObjectSync::bodies).
ForceStats
applyExternalForces(Box3DSharedState &st, SIM_ObjectArray &objects, bool wake)
{
    ForceStats stats;
    b3dop::Box3DWorld &world = st.world;

    // A point's non-zero `force` / `torque` (what POP Force and other POP / SOP nodes write).
    struct PointForce
    {
        GA_Offset  off;
        UT_Vector3 force, torque;
    };
    std::vector<const SIM_Force *> forces;
    std::vector<PointForce>        pointForces;    // sorted by offset
    std::vector<SIM_Object *>      cleared;

    for (exint i = 0; i < objects.entries(); ++i)
    {
        SIM_Object *obj = objects(i);
        const auto osIt = st.objectSync.find(obj->getObjectId());
        if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp || osIt->second.bodies.empty())
            continue;
        Box3DObjectSync &os = osIt->second;

        forces.clear();
        ForceCollector collector(forces);
        obj->forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"), SIM_FORCES_DATANAME,
                                 SIM_DataFilterNone());

        // Point forces. We clear them after applying them; while their data ids are still the ones
        // we recorded then, nothing has written new ones and the read is skipped.
        pointForces.clear();
        const SIM_Geometry *pointGeo = os.wholeObject ? nullptr     // one body: no per-point forces
                                                      : SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
        if (const SIM_Geometry *geo = pointGeo)
        {
            SIM_GeometryAutoReadLock lock(geo);
            if (const GU_Detail *gdp = lock.getGdp())
            {
                GA_DataId fId, tId;
                forceAttribIds(gdp, fId, tId);
                const bool any = fId != Box3DSyncFingerprint::kNoId || tId != Box3DSyncFingerprint::kNoId;
                if (any && !(fId == os.forceId && tId == os.torqueId))
                {
                    const GA_ROHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
                    const GA_ROHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
                    const UT_Vector3    zero(0.0f, 0.0f, 0.0f);
                    auto nonZero = [](const UT_Vector3 &v) { return v.x() != 0.0f || v.y() != 0.0f || v.z() != 0.0f; };
                    for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
                    {
                        const UT_Vector3 f = fh.isValid() ? fh.get(*it) : zero;
                        const UT_Vector3 t = th.isValid() ? th.get(*it) : zero;
                        if (nonZero(f) || nonZero(t))
                            pointForces.push_back({ *it, f, t });
                    }
                    if (pointForces.empty())        // all zero: no need to look again until they change
                    {
                        os.forceId  = fId;
                        os.torqueId = tId;
                    }
                }
            }
        }
        if (forces.empty() && pointForces.empty())
            continue;

        for (Box3DBodyMirror *e : os.bodies)
        {
            if (!e->handle || !e->dynamic() || !e->props.enabled)
                continue;
            if (!wake && !world.isAwakeByHandle(e->handle))
                continue;                       // Box3D would ignore the force anyway

            UT_Vector3 force(0.0f, 0.0f, 0.0f);
            UT_Vector3 torque(0.0f, 0.0f, 0.0f);
            if (!pointForces.empty())
            {
                const GA_Offset po = GA_Offset(e->pointOff);
                const auto it = std::lower_bound(pointForces.begin(), pointForces.end(), po,
                                                 [](const PointForce &pf, GA_Offset o) { return pf.off < o; });
                if (it != pointForces.end() && it->off == po)
                {
                    force  += it->force;
                    torque += it->torque;
                }
            }
            if (!forces.empty())
            {
                const float mass = world.massByHandle(e->handle);
                double com[3];
                float  lin[3], ang[3];
                if (mass > 0.0f && std::isfinite(mass) && world.getCenterOfMassByHandle(e->handle, com) &&
                    world.getVelocityByHandle(e->handle, lin, ang))
                {
                    const UT_Vector3 pos(static_cast<float>(com[0]), static_cast<float>(com[1]), static_cast<float>(com[2]));
                    const UT_Vector3 vel(lin[0], lin[1], lin[2]);
                    const UT_Vector3 angvel(ang[0], ang[1], ang[2]);
                    for (const SIM_Force *sf : forces)
                    {
                        UT_Vector3 f(0.0f, 0.0f, 0.0f), t(0.0f, 0.0f, 0.0f);
                        sf->getForce(*obj, pos, vel, angvel, mass, f, t);
                        force  += f;
                        torque += t;
                    }
                }
            }

            const float fv[3] = { float(force.x()),  float(force.y()),  float(force.z()) };
            const float tv[3] = { float(torque.x()), float(torque.y()), float(torque.z()) };
            if (!b3dop::finite3(fv) || !b3dop::finite3(tv))
            {
                ++stats.nonFinite;
                continue;
            }
            if (fv[0] == 0.0f && fv[1] == 0.0f && fv[2] == 0.0f && tv[0] == 0.0f && tv[1] == 0.0f && tv[2] == 0.0f)
                continue;
            world.applyForceByHandle(e->handle, fv, tv, wake);
            ++stats.bodies;
        }

        // Clear what was read (every body, applied or not), or a POP Force that only adds to
        // `force` would pile up from step to step.
        if (pointForces.empty())
            continue;
        SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
        if (!geo)
            continue;
        {
            SIM_GeometryAutoWriteLock lock(geo);
            GU_Detail *gdp = &lock.getGdp();
            GA_RWHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
            GA_RWHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
            const UT_Vector3 zero(0.0f, 0.0f, 0.0f);
            for (const PointForce &pf : pointForces)
            {
                if (fh.isValid()) fh.set(pf.off, zero);
                if (th.isValid()) th.set(pf.off, zero);
            }
            if (fh.isValid()) fh.getAttribute()->bumpDataId();
            if (th.isValid()) th.getAttribute()->bumpDataId();
        }
        cleared.push_back(obj);
    }

    // With the locks released (releasing may bump ids): the cleared ids, and the fingerprints.
    for (SIM_Object *obj : cleared)
    {
        recordFingerprintAfterWrite(st, *obj);
        const SIM_Geometry *geo = SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
        const auto osIt = st.objectSync.find(obj->getObjectId());
        if (!geo || osIt == st.objectSync.end())
            continue;
        SIM_GeometryAutoReadLock lock(geo);
        if (const GU_Detail *gdp = lock.getGdp())
            forceAttribIds(gdp, osIt->second.forceId, osIt->second.torqueId);
    }
    return stats;
}

} // namespace box3dsolver
