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

// The body's bounding box size, read once and only when a force samples a
// circle or a sphere. Valid is false for a body with no shapes.
struct BodyExtent {
  const b3dop::Box3DWorld &world;
  b3dop::BodyHandle handle;
  bool read = false, valid = false;
  float size[3] = {0, 0, 0};

  bool get() {
    if (!read) {
      read = true;
      valid = world.extentByHandle(handle, size);
    }
    return valid;
  }
};

constexpr float kPi = 3.14159265358979f;

// The force of one Force DOP on the body, by the sampling the force asks for
// (its Sample Mode): at the centre of mass (a point), over the body as a sphere
// of the same volume as its bounding box (force per volume), or over the body
// as a circle facing the force, of the same area as the box seen along it
// (force per area). A circle needs the force's direction, taken from the point
// sample. The box is in world axes, so a turned body counts a little larger.
void sampleForce(const SIM_Force &sf, const SIM_Object &obj,
                 const UT_Vector3 &pos, const UT_Vector3 &vel,
                 const UT_Vector3 &angvel, float mass, BodyExtent &extent,
                 UT_Vector3 &f, UT_Vector3 &t) {
  const SIM_Force::SIM_ForceSample mode = sf.getOptimalForceSampling();
  if (mode == SIM_Force::SIM_FORCE_SAMPLESPHERE && extent.get()) {
    const float volume = extent.size[0] * extent.size[1] * extent.size[2];
    const float radius = std::cbrt(0.75f * volume / kPi);
    if (radius > 0.0f) {
      sf.getForceOnSphere(obj, pos, radius, vel, angvel, mass, f, t);
      return;
    }
  } else if (mode == SIM_Force::SIM_FORCE_SAMPLECIRCLE && extent.get()) {
    sf.getForce(obj, pos, vel, angvel, mass, f, t);
    const float len = static_cast<float>(f.length());
    if (len > 0.0f && std::isfinite(len)) {
      const UT_Vector3 n = f / len;
      const float *s = extent.size;
      const float area = std::fabs(n.x()) * s[1] * s[2] +
                         std::fabs(n.y()) * s[0] * s[2] +
                         std::fabs(n.z()) * s[0] * s[1];
      const float radius = std::sqrt(area / kPi);
      if (radius > 0.0f) {
        f = UT_Vector3(0.0f, 0.0f, 0.0f);
        t = UT_Vector3(0.0f, 0.0f, 0.0f);
        sf.getForceOnCircle(obj, pos, n, radius, vel, angvel, mass, f, t);
      }
    }
    return;
  }
  sf.getForce(obj, pos, vel, angvel, mass, f, t);
}

// The sum of the Force DOPs at the body's centre of mass, with its mass and
// velocities (`bs`, read once for the body).
void addForceDops(const SIM_Object &obj, const b3dop::BodyState &bs,
                  const std::vector<const SIM_Force *> &forces,
                  BodyExtent &extent, UT_Vector3 &force, UT_Vector3 &torque) {
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
    sampleForce(*sf, obj, pos, vel, angvel, mass, extent, f, t);
    force += f;
    torque += t;
  }
}

// The body's point force, if it has one (binary search: pointForces is sorted
// by offset).
void addPointForce(const std::vector<PointForce> &pointForces,
                   const Box3DBodyMirror &e, UT_Vector3 &force,
                   UT_Vector3 &torque) {
  const GA_Offset po = static_cast<GA_Offset>(e.pointOff);
  const auto it = std::lower_bound(
      pointForces.begin(), pointForces.end(), po,
      [](const PointForce &pf, GA_Offset o) { return pf.off < o; });
  if (it != pointForces.end() && it->off == po) {
    force += it->force;
    torque += it->torque;
  }
}

// One body's inputs and result: read and applied serially, the Force DOPs
// between them possibly in parallel.
struct ForceJob {
  const Box3DBodyMirror *body = nullptr;
  b3dop::BodyState state;
  UT_Vector3 force{0.0f, 0.0f, 0.0f};
  UT_Vector3 torque{0.0f, 0.0f, 0.0f};
};

using ResolverPtr = std::unique_ptr<SIM_ForceResolver>;

// The Force DOPs that can run on several threads: sampled at a point, with a
// resolver that says it is thread-safe (Houdini's own forces mostly are; a
// custom force gets the generic resolver, which is not). `serial` gets the
// rest, which keep the per-body evaluation on one thread.
void splitForces(const SIM_Object &obj,
                 const std::vector<const SIM_Force *> &forces,
                 std::vector<ResolverPtr> &parallel,
                 std::vector<const SIM_Force *> &serial) {
  for (const SIM_Force *sf : forces) {
    ResolverPtr resolver;
    if (sf->getOptimalForceSampling() == SIM_Force::SIM_FORCE_SAMPLEPOINT)
      resolver.reset(sf->getForceResolver(obj));
    if (resolver && resolver->threadsafe())
      parallel.push_back(std::move(resolver));
    else
      serial.push_back(sf);
  }
}

// What the resolvers add to each job, on Houdini's TBB pool: every range of
// jobs works with its own copies of the resolvers, as SIM_ForceResolver asks.
void evaluateResolvers(const std::vector<ResolverPtr> &resolvers,
                       std::vector<ForceJob> &jobs) {
  constexpr size_t kGrain = 64;
  UTparallelFor(UT_BlockedRange<size_t>(0, jobs.size(), kGrain),
                [&](const UT_BlockedRange<size_t> &range) {
                  std::vector<ResolverPtr> mine;
                  mine.reserve(resolvers.size());
                  for (const ResolverPtr &r : resolvers)
                    mine.emplace_back(r->copy());
                  for (size_t i = range.begin(); i != range.end(); ++i) {
                    ForceJob &job = jobs[i];
                    const float mass = job.state.mass;
                    if (!(mass > 0.0f && std::isfinite(mass)))
                      continue;
                    const UT_Vector3 pos(static_cast<float>(job.state.com[0]),
                                         static_cast<float>(job.state.com[1]),
                                         static_cast<float>(job.state.com[2]));
                    const UT_Vector3 vel(job.state.lin[0], job.state.lin[1],
                                         job.state.lin[2]);
                    const UT_Vector3 angvel(job.state.ang[0], job.state.ang[1],
                                            job.state.ang[2]);
                    for (const ResolverPtr &r : mine) {
                      UT_Vector3 f(0.0f, 0.0f, 0.0f), t(0.0f, 0.0f, 0.0f);
                      r->getForce(pos, vel, angvel, mass, f, t);
                      job.force += f;
                      job.torque += t;
                    }
                  }
                });
}

// Force DOPs and point forces onto the object's dynamic bodies, in three
// phases: the bodies' inputs are read (serial: it reads Box3D), the Force DOPs
// are evaluated (the thread-safe ones in parallel when there are many bodies),
// and the results go into Box3D (serial).
void applyToObjectBodies(Box3DSharedState &st, const SIM_Object &obj,
                         const Box3DObjectSync &os,
                         const std::vector<const SIM_Force *> &forces,
                         const std::vector<PointForce> &pointForces, bool wake,
                         ForceStats &stats) {
  b3dop::Box3DWorld &world = st.world;

  std::vector<ForceJob> jobs;
  jobs.reserve(os.bodies.size());
  for (const Box3DBodyMirror *e : os.bodies) {
    if (!e->handle || !e->dynamic() || !e->props.enabled)
      continue;
    // The body's state is read once, and only when a Force DOP needs it.
    ForceJob job;
    job.body = e;
    if (!forces.empty()) {
      if (!world.bodyStateByHandle(e->handle, job.state))
        continue;
      if (!wake && !job.state.awake)
        continue; // Box3D would ignore the force anyway
    } else if (!wake && !world.isAwakeByHandle(e->handle))
      continue;
    if (!pointForces.empty())
      addPointForce(pointForces, *e, job.force, job.torque);
    jobs.push_back(job);
  }

  // The Force DOPs.
  if (!forces.empty()) {
    std::vector<ResolverPtr> parallel;
    std::vector<const SIM_Force *> serial;
    if (jobs.size() >= minParallelForces())
      splitForces(obj, forces, parallel, serial);
    else
      serial = forces;
    if (!parallel.empty())
      evaluateResolvers(parallel, jobs);
    if (!serial.empty())
      for (ForceJob &job : jobs) {
        BodyExtent extent{world, job.body->handle};
        addForceDops(obj, job.state, serial, extent, job.force, job.torque);
      }
  }

  for (const ForceJob &job : jobs) {
    const float fv[3] = {static_cast<float>(job.force.x()),
                         static_cast<float>(job.force.y()),
                         static_cast<float>(job.force.z())};
    const float tv[3] = {static_cast<float>(job.torque.x()),
                         static_cast<float>(job.torque.y()),
                         static_cast<float>(job.torque.z())};
    if (!b3dop::finite3(fv) || !b3dop::finite3(tv)) {
      ++stats.nonFinite;
      continue;
    }
    if (fv[0] == 0.0f && fv[1] == 0.0f && fv[2] == 0.0f && tv[0] == 0.0f &&
        tv[1] == 0.0f && tv[2] == 0.0f)
      continue;
    world.applyForceByHandle(job.body->handle, fv, tv, wake);
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
                               bool wake, PointForceCarry *carry) {
  ForceStats stats;
  std::vector<const SIM_Force *> forces;
  std::vector<PointForce> pointForces;
  std::vector<SIM_Object *> cleared;
  const bool replay = carry && carry->replay;

  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp ||
        osIt->second.bodies.empty())
      continue;
    Box3DObjectSync &os = osIt->second;

    collectForceDops(*obj, forces);
    if (replay) {
      // The attributes were cleared by the frame's first step: apply what it
      // read.
      const auto kept = carry->byObject.find(obj->getObjectId());
      pointForces.clear();
      if (kept != carry->byObject.end())
        pointForces = kept->second;
    } else {
      readPointForces(*obj, os, pointForces);
      if (carry && !pointForces.empty())
        carry->byObject[obj->getObjectId()] = pointForces;
    }
    if (forces.empty() && pointForces.empty())
      continue;

    applyToObjectBodies(st, *obj, os, forces, pointForces, wake, stats);
    if (!replay && !pointForces.empty() && clearPointForces(*obj, pointForces))
      cleared.push_back(obj);
  }

  recordClearedForces(st, cleared);
  return stats;
}

} // namespace box3dsolver
