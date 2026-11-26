// Box3DForces.cpp
//
// Force DOPs (SIM_Force data on the objects) and point force / torque
// attributes, applied to the bodies right before the step.
#include "Box3DSolverInternal.h"

#include "Box3DCompoundSync.h"

#include <GA/GA_SplittableRange.h>

#include <algorithm>
#include <mutex>

namespace box3dsolver {

namespace {

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
  const auto collect = [&](GA_Offset off, std::vector<PointForce> &out) {
    const UT_Vector3 f = fh.isValid() ? fh.get(off) : zero;
    const UT_Vector3 t = th.isValid() ? th.get(off) : zero;
    if (nonZero(f) || nonZero(t))
      out.push_back({off, f, t});
  };
  const GA_Range range(gdp->getPointRange());
  if (static_cast<size_t>(range.getEntries()) < minParallelPoints()) {
    for (GA_Iterator it(range); !it.atEnd(); ++it)
      collect(*it, pointForces);
  } else {
    // Read-only: each range gathers its own points, merged under a lock and
    // sorted by offset afterwards (addPointForce searches by offset).
    std::mutex mergeLock;
    UTparallelFor(GA_SplittableRange(range), [&](const GA_SplittableRange &r) {
      std::vector<PointForce> mine;
      GA_Offset start, end;
      for (GA_Iterator it(r); it.blockAdvance(start, end);)
        for (GA_Offset off = start; off < end; ++off)
          collect(off, mine);
      if (mine.empty())
        return;
      const std::lock_guard<std::mutex> guard(mergeLock);
      pointForces.insert(pointForces.end(), mine.begin(), mine.end());
    });
    std::sort(
        pointForces.begin(), pointForces.end(),
        [](const PointForce &a, const PointForce &b) { return a.off < b.off; });
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
    const float radius = std::cbrt(0.75f * volume / b3dop::kPi);
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
      const float radius = std::sqrt(area / b3dop::kPi);
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
const PointForce *findPointForce(const std::vector<PointForce> &pointForces,
                                 const Box3DBodyMirror &e) {
  const GA_Offset po = static_cast<GA_Offset>(e.pointOff);
  const auto it = std::lower_bound(
      pointForces.begin(), pointForces.end(), po,
      [](const PointForce &pf, GA_Offset o) { return pf.off < o; });
  return it != pointForces.end() && it->off == po ? &*it : nullptr;
}

void addPointForce(const PointForce &pf, UT_Vector3 &force,
                   UT_Vector3 &torque) {
  force += pf.force;
  torque += pf.torque;
}

// One body's inputs and result: read and applied serially, the Force DOPs
// between them possibly in parallel.
struct ForceJob {
  const Box3DBodyMirror *body = nullptr;
  const Box3DMotionEntry *drag = nullptr; // the point's drag, if it has one
  b3dop::BodyState state;
  UT_Vector3 force{0.0f, 0.0f, 0.0f};
  UT_Vector3 torque{0.0f, 0.0f, 0.0f};
  // The drag's own force and torque: applied without waking the body, so a
  // resting body is not woken by a drag it does not need.
  UT_Vector3 dragForce{0.0f, 0.0f, 0.0f};
  UT_Vector3 dragTorque{0.0f, 0.0f, 0.0f};
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
    const bool point =
        sf->getOptimalForceSampling() == SIM_Force::SIM_FORCE_SAMPLEPOINT;
    if (point)
      resolver.reset(sf->getForceResolver(obj));
    const bool safe = resolver && resolver->threadsafe();
    B3D_DEBUG_LOG("force %s: point sampling %d, resolver %d, threadsafe %d: %s",
                  sf->getDataType().c_str(), point ? 1 : 0, resolver ? 1 : 0,
                  safe ? 1 : 0, safe ? "parallel" : "serial");
    if (safe)
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

// The drag of a body's point (airresist, spinresist, ...), as a force and a
// torque of its own (`dragForce`, `dragTorque`). Bullet applies it implicitly
// (a strong drag then cannot overshoot): the velocity the body would have after
// the step under the other forces and gravity is slowed by the drag, and the
// force that gives that change is the drag. `gravity`: the world's, and the
// body's scale of it. The body's own linear and angular damping, which Box3D
// applies in its integration, is not in the predicted velocity, so with damping
// the end speed is a little lower than the drag solved for.
void addDrag(const b3dop::Box3DWorld &world, const float gravity[3], float dt,
             ForceJob &job) {
  const b3dop::MotionParams &m = job.drag->params;
  const float mass = job.state.mass;
  if (!(dt > 0.0f) || !(mass > 0.0f) || !std::isfinite(mass))
    return;
  const float invMass = 1.0f / mass;
  if (m.airResist > 0.0f) {
    const float scale = job.body->props.gravityScale;
    float before[3], after[3];
    for (int k = 0; k < 3; ++k)
      before[k] =
          job.state.lin[k] + dt * (invMass * job.force[k] + gravity[k] * scale);
    std::copy(before, before + 3, after);
    b3dop::applyImplicitDrag(m.dragExp, m.airResist, m.targetV, dt, invMass,
                             after);
    for (int k = 0; k < 3; ++k)
      job.dragForce[k] = (after[k] - before[k]) / (invMass * dt);
  }
  float inertia[6], inverse[6];
  if (m.spinResist > 0.0f &&
      world.inertiaWorldByHandle(job.body->handle, inertia) &&
      b3dop::invertSymmetric3(inertia, inverse)) {
    const auto mul = [](const float s[6], const float *v, float out[3]) {
      out[0] = s[0] * v[0] + s[3] * v[1] + s[4] * v[2];
      out[1] = s[3] * v[0] + s[1] * v[1] + s[5] * v[2];
      out[2] = s[4] * v[0] + s[5] * v[1] + s[2] * v[2];
    };
    const float t[3] = {job.torque.x(), job.torque.y(), job.torque.z()};
    float spin[3], before[3], after[3], change[3], twist[3];
    mul(inverse, t, spin);
    for (int k = 0; k < 3; ++k)
      before[k] = job.state.ang[k] + dt * spin[k];
    std::copy(before, before + 3, after);
    b3dop::applyImplicitDrag(m.dragExp, m.spinResist, m.targetW, dt, 1.0f,
                             after);
    for (int k = 0; k < 3; ++k)
      change[k] = after[k] - before[k];
    mul(inertia, change, twist);
    for (int k = 0; k < 3; ++k)
      job.dragTorque[k] = twist[k] / dt;
  }
}

// Force DOPs and point forces onto the object's dynamic bodies, in three
// phases: the bodies' inputs are read (serial: it reads Box3D), the Force DOPs
// are evaluated (the thread-safe ones in parallel when there are many bodies),
// and the results go into Box3D (serial).
void applyToObjectBodies(Box3DSharedState &st, const SIM_Object &obj,
                         Box3DObjectSync &os, PointForceCarry::ObjectForces &of,
                         const std::vector<PointForce> &pointForces, bool wake,
                         float dt, ForceStats &stats) {
  const std::vector<const SIM_Force *> &forces = of.forces;
  b3dop::Box3DWorld &world = st.world;

  // Kept between calls (every object, every substep): no allocation.
  thread_local std::vector<ForceJob> jobs;
  jobs.clear();
  jobs.reserve(os.bodies.size());
  for (Box3DBodyMirror *e : os.bodies) {
    const PointForce *own =
        pointForces.empty() ? nullptr : findPointForce(pointForces, *e);
    // A point force that has ended: a sleeping body does not feel it go, so it
    // is woken once.
    if (e->pointForced && !own) {
      e->pointForced = false;
      if (e->handle && e->dynamic() && !e->inCompound)
        world.wakeByHandle(e->handle);
    }
    if (!e->handle || !e->dynamic() || !e->props.enabled)
      continue;
    // The body's state is read once, and only when a Force DOP needs it.
    ForceJob job;
    job.body = e;
    // A piece of a compound (Box3DCompoundSync.h) is parked: its state is read
    // from the compound.
    const bool piece = e->inCompound;
    // Drag: a compound's pieces are parked and the compound has no point, so
    // only the other bodies take it.
    if (!piece && !os.motion.empty() && !e->wholeObject) {
      const Box3DMotionEntry *m =
          findMotion(os, static_cast<GA_Offset>(e->pointOff));
      if (m && m->params.hasDrag())
        job.drag = m;
    }
    if (!forces.empty() || job.drag) {
      const bool read = piece ? compoundPieceState(st, *e, job.state)
                              : world.bodyStateByHandle(e->handle, job.state);
      if (!read)
        continue;
      if (!wake && !job.state.awake)
        continue; // Box3D would ignore the force anyway
    } else if (!wake && !(piece ? compoundPieceAwake(st, *e)
                                : world.isAwakeByHandle(e->handle)))
      continue;
    if (own) {
      addPointForce(*own, job.force, job.torque);
      e->pointForced = !e->inCompound;
    }
    jobs.push_back(job);
  }
  os.pointForcedAny =
      std::any_of(os.bodies.begin(), os.bodies.end(),
                  [](const Box3DBodyMirror *e) { return e->pointForced; });

  // The Force DOPs.
  if (!forces.empty() && !jobs.empty()) {
    // The split is made once per frame (the resolvers copy the forces' state
    // when made, which does not change between steps).
    const bool many = jobs.size() >= minParallelForces();
    if (many && !of.split) {
      splitForces(obj, forces, of.parallel, of.serial);
      of.split = true;
    }
    const std::vector<const SIM_Force *> &serial = many ? of.serial : forces;
    if (many && !of.parallel.empty())
      evaluateResolvers(of.parallel, jobs);
    if (!serial.empty())
      for (ForceJob &job : jobs) {
        BodyExtent extent{world, job.body->handle};
        addForceDops(obj, job.state, serial, extent, job.force, job.torque);
      }
  }

  // Drag, last: it works on the sum of everything else.
  if (std::any_of(jobs.begin(), jobs.end(),
                  [](const ForceJob &j) { return j.drag != nullptr; })) {
    const float *g = world.settings().gravity;
    for (ForceJob &job : jobs)
      if (job.drag)
        addDrag(world, g, dt, job);
  }

  for (const ForceJob &job : jobs) {
    float fv[3] = {static_cast<float>(job.force.x()),
                   static_cast<float>(job.force.y()),
                   static_cast<float>(job.force.z())};
    float tv[3] = {static_cast<float>(job.torque.x()),
                   static_cast<float>(job.torque.y()),
                   static_cast<float>(job.torque.z())};
    float dfv[3] = {job.dragForce.x(), job.dragForce.y(), job.dragForce.z()};
    float dtv[3] = {job.dragTorque.x(), job.dragTorque.y(), job.dragTorque.z()};
    if (!b3dop::finite3(fv) || !b3dop::finite3(tv) || !b3dop::finite3(dfv) ||
        !b3dop::finite3(dtv)) {
      ++stats.nonFinite;
      continue;
    }
    const auto zero = [](const float v[3]) {
      return v[0] == 0.0f && v[1] == 0.0f && v[2] == 0.0f;
    };
    if (zero(fv) && zero(tv) && zero(dfv) && zero(dtv))
      continue;
    // A piece of a compound pushes its compound, about the compound's centre.
    b3dop::BodyHandle target = job.body->handle;
    if (job.body->inCompound)
      compoundPieceForce(st, *job.body, fv, tv, target);
    if (!zero(fv) || !zero(tv))
      world.applyForceByHandle(target, fv, tv, wake);
    // The drag never wakes a sleeping body.
    if (!zero(dfv) || !zero(dtv))
      world.applyForceByHandle(target, dfv, dtv, false);
    ++stats.bodies;
  }
}

// Clear what was read (every body, applied or not), or a POP Force that only
// adds to `force` would pile up from step to step. False if the geometry is not
// writable.
bool clearPointForces(SIM_Object &obj,
                      const std::vector<PointForce> &pointForces) {
  SIM_GeometryCopy *geo = box3dWritableGeometry(obj, SIM_GEOMETRY_DATANAME);
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

// With the locks released (releasing may bump ids): the cleared ids. The
// fingerprints are re-recorded at the end of the step (noteOwnWrite).
void recordClearedForces(Box3DSharedState &st,
                         const std::vector<SIM_Object *> &cleared) {
  for (SIM_Object *obj : cleared) {
    noteOwnWrite(st, *obj);
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
                               bool wake, float dt, PointForceCarry *carry) {
  ForceStats stats;
  PointForceCarry::ObjectForces single; // without a carry: made for each call
  std::vector<PointForce> pointForces;
  const std::vector<PointForce> noForces;
  std::vector<SIM_Object *> cleared;
  const bool replay = carry && carry->replay;

  for (int i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp ||
        osIt->second.bodies.empty())
      continue;
    Box3DObjectSync &os = osIt->second;

    // The frame's first step finds the object's forces; later steps reuse them.
    PointForceCarry::ObjectForces &of =
        carry ? carry->forcesOf[obj->getObjectId()] : single;
    if (!replay) {
      of = PointForceCarry::ObjectForces();
      collectForceDops(*obj, of.forces);
    }
    const std::vector<PointForce> *applied = &pointForces;
    if (replay) {
      // The attributes were cleared by the frame's first step: apply what it
      // read.
      const auto kept = carry->byObject.find(obj->getObjectId());
      applied = kept != carry->byObject.end() ? &kept->second : &noForces;
    } else {
      readPointForces(*obj, os, pointForces);
      if (carry && !pointForces.empty())
        carry->byObject[obj->getObjectId()] = pointForces;
    }
    // A body whose point force just ended is still visited (to wake it).
    if (of.forces.empty() && applied->empty() && !os.pointForcedAny &&
        os.motion.empty())
      continue;

    applyToObjectBodies(st, *obj, os, of, *applied, wake, dt, stats);
    if (!replay && !pointForces.empty() && clearPointForces(*obj, pointForces))
      cleared.push_back(obj);
  }

  recordClearedForces(st, cleared);
  return stats;
}

} // namespace box3dsolver
