// Box3DForces.cpp
//
// Force DOPs (SIM_Force data on the objects) and point force / torque
// attributes, applied to the bodies right before the step.
#include "Box3DSolverInternal.h"

namespace box3dsolver {

// Collects the SIM_Force data an object carries (Gravity Force, Wind Force, Fan
// Force, POP Force, any custom force DOP): the lookup Houdini's own solvers use
// (see SIM_SolverHair).
class ForceCollector : public SIM_EachDataCallback {
public:
  explicit ForceCollector(std::vector<const SIM_Force *> &out) : myOut(out) {}

  void callbackConst(const SIM_Data *data, const char *) override {
    if (const SIM_Force *f = SIM_DATA_CASTCONST(data, SIM_Force))
      myOut.push_back(f);
  }

private:
  std::vector<const SIM_Force *> &myOut;
};

// The data ids of an object's `force` / `torque` attributes (kNoId when
// absent).
void forceAttribIds(const GU_Detail *gdp, GA_DataId &forceId,
                    GA_DataId &torqueId) {
  const GA_Attribute *f = gdp->findPointAttribute("force");
  const GA_Attribute *t = gdp->findPointAttribute("torque");
  forceId = f ? f->getDataId() : Box3DSyncFingerprint::kNoId;
  torqueId = t ? t->getDataId() : Box3DSyncFingerprint::kNoId;
}

namespace {

// A point's non-zero `force` / `torque` (what POP Force and other POP / SOP
// nodes write).
struct PointForce {
  GA_Offset off;
  UT_Vector3 force, torque;
};

// The SIM_Force data on the object.
void collectForceDops(const SIM_Object &obj,
                      std::vector<const SIM_Force *> &forces) {
  forces.clear();
  ForceCollector collector(forces);
  obj.forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"),
                          SIM_FORCES_DATANAME, SIM_DataFilterNone());
}

// The object's point forces, sorted by offset. We clear them after applying
// them; while their data ids are still the ones we recorded then, nothing has
// written new ones and the read is skipped.
void readPointForces(const SIM_Object &obj, Box3DObjectSync &os,
                     std::vector<PointForce> &pointForces) {
  pointForces.clear();
  if (os.wholeObject)
    return; // one body: no per-point forces
  const SIM_Geometry *geo =
      SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
  if (!geo)
    return;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return;

  GA_DataId fId, tId;
  forceAttribIds(gdp, fId, tId);
  const bool any =
      fId != Box3DSyncFingerprint::kNoId || tId != Box3DSyncFingerprint::kNoId;
  if (!any || (fId == os.forceId && tId == os.torqueId))
    return;

  const GA_ROHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
  const GA_ROHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
  const UT_Vector3 zero(0.0f, 0.0f, 0.0f);
  auto nonZero = [](const UT_Vector3 &v) {
    return v.x() != 0.0f || v.y() != 0.0f || v.z() != 0.0f;
  };
  for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it) {
    const UT_Vector3 f = fh.isValid() ? fh.get(*it) : zero;
    const UT_Vector3 t = th.isValid() ? th.get(*it) : zero;
    if (nonZero(f) || nonZero(t))
      pointForces.push_back({*it, f, t});
  }
  if (pointForces.empty()) // all zero: no need to look again until they change
  {
    os.forceId = fId;
    os.torqueId = tId;
  }
}

// The sum of the Force DOPs at the body's centre of mass, with its mass and
// velocities (`bs`, read once for the body).
void addForceDops(const SIM_Object &obj, const b3dop::BodyState &bs,
                  const std::vector<const SIM_Force *> &forces,
                  UT_Vector3 &force, UT_Vector3 &torque) {
  const float mass = bs.mass;
  if (!(mass > 0.0f && std::isfinite(mass)))
    return;
  const UT_Vector3 pos(static_cast<float>(bs.com[0]),
                       static_cast<float>(bs.com[1]),
                       static_cast<float>(bs.com[2]));
  const UT_Vector3 vel(bs.lin[0], bs.lin[1], bs.lin[2]);
  const UT_Vector3 angvel(bs.ang[0], bs.ang[1], bs.ang[2]);
  for (const SIM_Force *sf : forces) {
    UT_Vector3 f(0.0f, 0.0f, 0.0f), t(0.0f, 0.0f, 0.0f);
    sf->getForce(obj, pos, vel, angvel, mass, f, t);
    force += f;
    torque += t;
  }
}

// The body's point force, if it has one (binary search: pointForces is sorted
// by offset).
void addPointForce(const std::vector<PointForce> &pointForces,
                   const Box3DBodyMirror &e, UT_Vector3 &force,
                   UT_Vector3 &torque) {
  const GA_Offset po = GA_Offset(e.pointOff);
  const auto it = std::lower_bound(
      pointForces.begin(), pointForces.end(), po,
      [](const PointForce &pf, GA_Offset o) { return pf.off < o; });
  if (it != pointForces.end() && it->off == po) {
    force += it->force;
    torque += it->torque;
  }
}

// Force DOPs and point forces onto the object's dynamic bodies.
void applyToObjectBodies(Box3DSharedState &st, const SIM_Object &obj,
                         const Box3DObjectSync &os,
                         const std::vector<const SIM_Force *> &forces,
                         const std::vector<PointForce> &pointForces, bool wake,
                         ForceStats &stats) {
  b3dop::Box3DWorld &world = st.world;
  for (const Box3DBodyMirror *e : os.bodies) {
    if (!e->handle || !e->dynamic() || !e->props.enabled)
      continue;
    // The body's state is read once, and only when a Force DOP needs it.
    b3dop::BodyState bs;
    if (!forces.empty()) {
      if (!world.bodyStateByHandle(e->handle, bs))
        continue;
      if (!wake && !bs.awake)
        continue; // Box3D would ignore the force anyway
    } else if (!wake && !world.isAwakeByHandle(e->handle))
      continue;

    UT_Vector3 force(0.0f, 0.0f, 0.0f);
    UT_Vector3 torque(0.0f, 0.0f, 0.0f);
    if (!pointForces.empty())
      addPointForce(pointForces, *e, force, torque);
    if (!forces.empty())
      addForceDops(obj, bs, forces, force, torque);

    const float fv[3] = {float(force.x()), float(force.y()), float(force.z())};
    const float tv[3] = {float(torque.x()), float(torque.y()),
                         float(torque.z())};
    if (!b3dop::finite3(fv) || !b3dop::finite3(tv)) {
      ++stats.nonFinite;
      continue;
    }
    if (fv[0] == 0.0f && fv[1] == 0.0f && fv[2] == 0.0f && tv[0] == 0.0f &&
        tv[1] == 0.0f && tv[2] == 0.0f)
      continue;
    world.applyForceByHandle(e->handle, fv, tv, wake);
    ++stats.bodies;
  }
}

// Clear what was read (every body, applied or not), or a POP Force that only
// adds to `force` would pile up from step to step. False if the geometry is not
// writable.
bool clearPointForces(SIM_Object &obj,
                      const std::vector<PointForce> &pointForces) {
  SIM_GeometryCopy *geo = getOrMakeWritableGeometry(obj);
  if (!geo)
    return false;
  SIM_GeometryAutoWriteLock lock(geo);
  GU_Detail *gdp = &lock.getGdp();
  GA_RWHandleV3 fh(gdp, GA_ATTRIB_POINT, "force");
  GA_RWHandleV3 th(gdp, GA_ATTRIB_POINT, "torque");
  const UT_Vector3 zero(0.0f, 0.0f, 0.0f);
  for (const PointForce &pf : pointForces) {
    if (fh.isValid())
      fh.set(pf.off, zero);
    if (th.isValid())
      th.set(pf.off, zero);
  }
  if (fh.isValid())
    fh.getAttribute()->bumpDataId();
  if (th.isValid())
    th.getAttribute()->bumpDataId();
  return true;
}

// With the locks released (releasing may bump ids): the cleared ids, and the
// fingerprints.
void recordClearedForces(Box3DSharedState &st,
                         const std::vector<SIM_Object *> &cleared) {
  for (SIM_Object *obj : cleared) {
    recordFingerprintAfterWrite(st, *obj);
    const SIM_Geometry *geo =
        SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (!geo || osIt == st.objectSync.end())
      continue;
    SIM_GeometryAutoReadLock lock(geo);
    if (const GU_Detail *gdp = lock.getGdp())
      forceAttribIds(gdp, osIt->second.forceId, osIt->second.torqueId);
  }
}

} // namespace

// Force DOPs: every dynamic body takes the sum of the SIM_Force data on its
// object, evaluated at its centre of mass with its mass and velocities, the way
// Houdini's solvers do, plus its point's `force` / `torque` attributes. The
// force and torque go into Box3D for the coming step (it integrates them in
// every substep and clears them afterwards), so this runs after the sync and
// right before the step. A Gravity Force DOP adds to the solver's own Gravity:
// set that to 0 when using one. Only the bodies of objects that have forces are
// visited (Box3DObjectSync::bodies).
ForceStats applyExternalForces(Box3DSharedState &st, SIM_ObjectArray &objects,
                               bool wake) {
  ForceStats stats;
  std::vector<const SIM_Force *> forces;
  std::vector<PointForce> pointForces;
  std::vector<SIM_Object *> cleared;

  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp ||
        osIt->second.bodies.empty())
      continue;
    Box3DObjectSync &os = osIt->second;

    collectForceDops(*obj, forces);
    readPointForces(*obj, os, pointForces);
    if (forces.empty() && pointForces.empty())
      continue;

    applyToObjectBodies(st, *obj, os, forces, pointForces, wake, stats);
    if (!pointForces.empty() && clearPointForces(*obj, pointForces))
      cleared.push_back(obj);
  }

  recordClearedForces(st, cleared);
  return stats;
}

} // namespace box3dsolver
