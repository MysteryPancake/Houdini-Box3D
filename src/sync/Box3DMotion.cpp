// Box3DMotion.cpp
//
// The drag and speed limits RBD Bullet Solver reads from POP-style point
// attributes (Box3DMotion.h has the maths): read per object when the attributes
// change, the drag added to the force pass (Box3DForces.cpp), the limits
// applied to the moved bodies after every Box3D step.
#include "Box3DSolverInternal.h"
#include "core/Box3DMath.h"

#include <GA/GA_Handle.h>
#include <GA/GA_SplittableRange.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_map>

namespace box3dsolver {

namespace {

// Changes with the attributes' data ids and the point count (an added or
// removed point moves the offsets).
uint64_t motionSignature(const GU_Detail *gdp) {
  uint64_t h = b3dop::mixHash(b3dop::kHashSeed,
                              static_cast<uint64_t>(gdp->getNumPoints()));
  for (const char *name : kMotionAttribNames) {
    const GA_Attribute *a = gdp->findPointAttribute(name);
    h = b3dop::mixHash(h, a ? static_cast<uint64_t>(a->getDataId()) + 1 : 0);
  }
  return h;
}

bool hasAccel(const std::vector<Box3DMotionEntry> &entries) {
  return std::any_of(
      entries.begin(), entries.end(),
      [](const Box3DMotionEntry &e) { return e.params.hasAccelLimits(); });
}

bool hasAnyMotionAttrib(const GU_Detail *gdp) {
  for (const char *name : kMotionAttribNames)
    if (gdp->findPointAttribute(name))
      return true;
  return false;
}

// `inputs`: some point has a non-zero one-step input (see
// Box3DObjectSync::motionInputs); `accel`: some point has an accel limit.
void readMotion(const GU_Detail *gdp, std::vector<Box3DMotionEntry> &out,
                bool &inputs, bool &accel) {
  out.clear();
  inputs = false;
  accel = false;
  if (!hasAnyMotionAttrib(gdp))
    return;
  const GA_ROHandleF airresist(gdp, GA_ATTRIB_POINT, "airresist"),
      spinresist(gdp, GA_ATTRIB_POINT, "spinresist"),
      drag(gdp, GA_ATTRIB_POINT, "drag"),
      dragexp(gdp, GA_ATTRIB_POINT, "dragexp"),
      speedmin(gdp, GA_ATTRIB_POINT, "speedmin"),
      speedmax(gdp, GA_ATTRIB_POINT, "speedmax"),
      spinmin(gdp, GA_ATTRIB_POINT, "spinmin"),
      spinmax(gdp, GA_ATTRIB_POINT, "spinmax"),
      accelmax(gdp, GA_ATTRIB_POINT, "accelmax"),
      angaccelmax(gdp, GA_ATTRIB_POINT, "angaccelmax");
  const GA_ROHandleV3 targetv(gdp, GA_ATTRIB_POINT, "targetv"),
      targetw(gdp, GA_ATTRIB_POINT, "targetw");
  const auto num = [](float v, float fallback) {
    return std::isfinite(v) ? v : fallback;
  };
  // One point's entry, if it has any drag or limit; `anyInput` is set when it
  // carries a one-step input.
  const auto collect = [&](GA_Offset off, std::vector<Box3DMotionEntry> &dest,
                           bool &anyInput) {
    b3dop::MotionParams p;
    // `drag` scales both resists, as Bullet's getDragParms: 1 without it.
    const float scale = drag.isValid() ? num(drag.get(off), 1.0f) : 1.0f;
    if (airresist.isValid())
      p.airResist = std::max(0.0f, scale * num(airresist.get(off), 0.0f));
    if (spinresist.isValid())
      p.spinResist = std::max(0.0f, scale * num(spinresist.get(off), 0.0f));
    p.dragExp =
        b3dop::clampDragExp(dragexp.isValid() ? dragexp.get(off) : 2.0f);
    if (targetv.isValid()) {
      const UT_Vector3 v = targetv.get(off);
      for (int k = 0; k < 3; ++k)
        p.targetV[k] = num(v[k], 0.0f);
    }
    if (targetw.isValid()) {
      const UT_Vector3 w = targetw.get(off);
      for (int k = 0; k < 3; ++k)
        p.targetW[k] = num(w[k], 0.0f);
    }
    if (speedmin.isValid())
      p.minSpeed = num(speedmin.get(off), -1.0f);
    if (speedmax.isValid())
      p.maxSpeed = num(speedmax.get(off), -1.0f);
    if (spinmin.isValid())
      p.minSpin = num(spinmin.get(off), -1.0f);
    if (spinmax.isValid())
      p.maxSpin = num(spinmax.get(off), -1.0f);
    if (accelmax.isValid())
      p.maxAccel = num(accelmax.get(off), -1.0f);
    if (angaccelmax.isValid())
      p.maxAngAccel = num(angaccelmax.get(off), -1.0f);
    anyInput = anyInput || p.airResist > 0.0f || p.spinResist > 0.0f ||
               p.targetV[0] != 0.0f || p.targetV[1] != 0.0f ||
               p.targetV[2] != 0.0f || p.targetW[0] != 0.0f ||
               p.targetW[1] != 0.0f || p.targetW[2] != 0.0f;
    if (p.any())
      dest.push_back({off, p});
  };
  const GA_Range range(gdp->getPointRange());
  if (static_cast<size_t>(range.getEntries()) < minParallelPoints()) {
    for (GA_Iterator it(range); !it.atEnd(); ++it)
      collect(*it, out, inputs);
    accel = hasAccel(out);
    return;
  }
  // Read-only: each range gathers its own points, merged under a lock and
  // sorted by offset afterwards (findMotion searches by offset).
  std::mutex mergeLock;
  UTparallelFor(GA_SplittableRange(range), [&](const GA_SplittableRange &r) {
    std::vector<Box3DMotionEntry> mine;
    bool mineInputs = false;
    GA_Offset start, end;
    for (GA_Iterator it(r); it.blockAdvance(start, end);)
      for (GA_Offset off = start; off < end; ++off)
        collect(off, mine, mineInputs);
    if (mine.empty() && !mineInputs)
      return;
    const std::lock_guard<std::mutex> guard(mergeLock);
    out.insert(out.end(), mine.begin(), mine.end());
    inputs = inputs || mineInputs;
  });
  std::sort(out.begin(), out.end(),
            [](const Box3DMotionEntry &a, const Box3DMotionEntry &b) {
              return a.off < b.off;
            });
  accel = hasAccel(out);
}

} // namespace

void refreshMotion(Box3DSharedState &st, SIM_ObjectArray &objects) {
  bool any = false;
  for (int i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt == st.objectSync.end() || osIt->second.visited != st.syncStamp)
      continue;
    Box3DObjectSync &os = osIt->second;
    if (os.wholeObject) { // Bullet gives an RBD Object no drag or limits
      os.motion.clear();
      os.motionAccel = false;
      continue;
    }
    const SIM_Geometry *geo =
        SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!geo)
      continue;
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = lock.getGdp();
    if (!gdp)
      continue;
    const uint64_t sig = motionSignature(gdp);
    if (sig != os.motionSig) {
      readMotion(gdp, os.motion, os.motionInputs, os.motionAccel);
      os.motionSig = sig;
    }
    any = any || !os.motion.empty();
  }
  st.anyMotion = any;
}

void refreshCompoundMotion(Box3DSharedState &st) {
  for (const auto &cp : st.compounds) {
    Box3DCompound &c = *cp;
    b3dop::MotionParams sum;
    sum.dragExp = 0.0f;
    double mass = 0.0;
    double air = 0.0, spin = 0.0, exp = 0.0, tv[3] = {}, tw[3] = {};
    const auto lower = [](float &into, float v) { // the smallest limit set
      if (v >= 0.0f)
        into = into >= 0.0f ? std::min(into, v) : v;
    };
    const auto raise = [](float &into, float v) { // the largest limit set
      if (v >= 0.0f)
        into = std::max(into, v);
    };
    if (st.anyMotion)
      for (const Box3DCompoundMember &m : c.members) {
        const Box3DBodyMirror *e = m.entry;
        const double w = m.mass > 0.0f ? m.mass : 0.0;
        mass += w;
        const Box3DMotionEntry *me =
            e && e->sync && !e->sync->motion.empty() && e->pointOff >= 0
                ? findMotion(*e->sync, static_cast<GA_Offset>(e->pointOff))
                : nullptr;
        const b3dop::MotionParams p = me ? me->params : b3dop::MotionParams();
        air += w * p.airResist;
        spin += w * p.spinResist;
        exp += w * p.dragExp;
        for (int k = 0; k < 3; ++k) {
          tv[k] += w * p.targetV[k];
          tw[k] += w * p.targetW[k];
        }
        raise(sum.minSpeed, p.minSpeed);
        lower(sum.maxSpeed, p.maxSpeed);
        raise(sum.minSpin, p.minSpin);
        lower(sum.maxSpin, p.maxSpin);
        lower(sum.maxAccel, p.maxAccel);
        lower(sum.maxAngAccel, p.maxAngAccel);
      }
    if (mass > 0.0) {
      sum.airResist = static_cast<float>(air / mass);
      sum.spinResist = static_cast<float>(spin / mass);
      sum.dragExp = b3dop::clampDragExp(static_cast<float>(exp / mass));
      for (int k = 0; k < 3; ++k) {
        sum.targetV[k] = static_cast<float>(tv[k] / mass);
        sum.targetW[k] = static_cast<float>(tw[k] / mass);
      }
    } else {
      sum = b3dop::MotionParams();
    }
    c.motion = sum;
    c.hasMotion = sum.any();
  }
}

const Box3DMotionEntry *findMotion(const Box3DObjectSync &os, GA_Offset off) {
  const auto it = std::lower_bound(
      os.motion.begin(), os.motion.end(), off,
      [](const Box3DMotionEntry &e, GA_Offset o) { return e.off < o; });
  return it != os.motion.end() && it->off == off ? &*it : nullptr;
}

void applySpeedLimits(Box3DSharedState &st,
                      std::vector<b3dop::BodyMove> &events) {
  if (!st.anyMotion)
    return;
  for (b3dop::BodyMove &mv : events) {
    const Box3DBodyMirror *e =
        static_cast<const Box3DBodyMirror *>(mv.userData);
    if (!e || !e->sync || e->sync->motion.empty() || !e->dynamic() ||
        e->wholeObject || e->inCompound || mv.fellAsleep || e->pointOff < 0)
      continue;
    const Box3DMotionEntry *m =
        findMotion(*e->sync, static_cast<GA_Offset>(e->pointOff));
    if (!m || !m->params.hasLimits())
      continue;
    if (b3dop::clampMotion(m->params, mv.lin, mv.ang))
      st.world.setVelocityByHandle(mv.handle, mv.lin, mv.ang);
  }
  // The compounds with limits: their bodies have no entry (their pieces'
  // velocities are worked out from them afterwards, expandCompoundMoves).
  thread_local std::unordered_map<b3dop::BodyHandle, const Box3DCompound *>
      limited;
  limited.clear();
  for (const auto &c : st.compounds)
    if (c->hasMotion && c->motion.hasLimits())
      limited.emplace(c->body, c.get());
  if (limited.empty())
    return;
  for (b3dop::BodyMove &mv : events) {
    if (mv.userData || mv.fellAsleep)
      continue;
    const auto it = limited.find(mv.handle);
    if (it != limited.end() &&
        b3dop::clampMotion(it->second->motion, mv.lin, mv.ang))
      st.world.setVelocityByHandle(mv.handle, mv.lin, mv.ang);
  }
}

void refreshAccelLimits(Box3DSharedState &st) {
  st.accelSlots.clear();
  st.accelSlotOf.clear();
  if (!st.anyMotion)
    return;
  const auto add = [&st](b3dop::BodyHandle h, const b3dop::MotionParams &p,
                         float gravityScale) {
    Box3DAccelSlot s;
    s.body = h;
    s.maxAccel = p.maxAccel;
    s.maxAngAccel = p.maxAngAccel;
    s.gravityScale = gravityScale;
    st.accelSlotOf.emplace(h, st.accelSlots.size());
    st.accelSlots.push_back(s);
  };
  for (const auto &kv : st.objectSync) {
    const Box3DObjectSync &os = kv.second;
    if (!os.motionAccel || os.visited != st.syncStamp)
      continue;
    for (const Box3DBodyMirror *e : os.bodies) {
      if (!e->handle || !e->dynamic() || e->inCompound || e->pointOff < 0)
        continue;
      const Box3DMotionEntry *m =
          findMotion(os, static_cast<GA_Offset>(e->pointOff));
      if (m && m->params.hasAccelLimits())
        add(e->handle, m->params, e->props.gravityScale);
    }
  }
  for (const auto &c : st.compounds)
    if (c->hasMotion && c->motion.hasAccelLimits() && !c->members.empty())
      add(c->body, c->motion, c->members[0].props.gravityScale);
}

void beginAccelLimits(Box3DSharedState &st) {
  for (Box3DAccelSlot &s : st.accelSlots) {
    if (!st.world.getVelocityByHandle(s.body, s.lin0, s.ang0)) {
      std::fill(s.lin0, s.lin0 + 3, 0.0f);
      std::fill(s.ang0, s.ang0 + 3, 0.0f);
    }
    std::fill(s.force, s.force + 3, 0.0f);
    std::fill(s.torque, s.torque + 3, 0.0f);
  }
}

void noteAccelForce(Box3DSharedState &st, b3dop::BodyHandle body,
                    const float force[3], const float torque[3]) {
  if (st.accelSlots.empty())
    return;
  const auto it = st.accelSlotOf.find(body);
  if (it == st.accelSlotOf.end())
    return;
  Box3DAccelSlot &s = st.accelSlots[it->second];
  for (int k = 0; k < 3; ++k) {
    s.force[k] += force[k];
    s.torque[k] += torque[k];
  }
}

void applyAccelLimits(Box3DSharedState &st,
                      std::vector<b3dop::BodyMove> &events, float dt) {
  if (st.accelSlots.empty() || !(dt > 0.0f))
    return;
  b3dop::Box3DWorld &w = st.world;
  const float *g = w.settings().gravity;
  for (b3dop::BodyMove &mv : events) {
    if (mv.fellAsleep)
      continue;
    const auto it = st.accelSlotOf.find(mv.handle);
    if (it == st.accelSlotOf.end())
      continue;
    const Box3DAccelSlot &s = st.accelSlots[it->second];
    bool changed = false;
    if (s.maxAccel >= 0.0f) {
      // What gravity and the step's forces alone make of the velocity.
      const float mass = w.massByHandle(mv.handle);
      const float invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
      float expected[3];
      for (int k = 0; k < 3; ++k)
        expected[k] =
            s.lin0[k] + dt * (g[k] * s.gravityScale + invMass * s.force[k]);
      changed |= b3dop::clampConstraintChange(s.maxAccel, dt, expected, mv.lin);
    }
    if (s.maxAngAccel >= 0.0f) {
      // The torque's change, on the inertia the body has now.
      float expected[3] = {s.ang0[0], s.ang0[1], s.ang0[2]};
      float inertia[6], inverse[6];
      if ((s.torque[0] != 0.0f || s.torque[1] != 0.0f || s.torque[2] != 0.0f) &&
          w.inertiaWorldByHandle(mv.handle, inertia) &&
          b3dop::invertSymmetric3(inertia, inverse)) {
        const float *t = s.torque;
        expected[0] +=
            dt * (inverse[0] * t[0] + inverse[3] * t[1] + inverse[4] * t[2]);
        expected[1] +=
            dt * (inverse[3] * t[0] + inverse[1] * t[1] + inverse[5] * t[2]);
        expected[2] +=
            dt * (inverse[4] * t[0] + inverse[5] * t[1] + inverse[2] * t[2]);
      }
      changed |=
          b3dop::clampConstraintChange(s.maxAngAccel, dt, expected, mv.ang);
    }
    if (changed)
      w.setVelocityByHandle(mv.handle, mv.lin, mv.ang);
  }
}

void clearMotionInputs(Box3DSharedState &st, SIM_ObjectArray &objects) {
  for (int i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt == st.objectSync.end() || !osIt->second.motionInputs)
      continue;
    {
      SIM_GeometryCopy *geo =
          box3dWritableGeometry(*obj, SIM_GEOMETRY_DATANAME);
      if (!geo)
        continue;
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      const UT_Vector3 zero(0.0f, 0.0f, 0.0f);
      for (const char *name : {"airresist", "spinresist"}) {
        GA_RWHandleF h(gdp, GA_ATTRIB_POINT, name);
        if (h.isValid()) {
          h.makeConstant(0.0f);
          h.bumpDataId();
        }
      }
      for (const char *name : {"targetv", "targetw"}) {
        GA_RWHandleV3 h(gdp, GA_ATTRIB_POINT, name);
        if (h.isValid()) {
          h.makeConstant(zero);
          h.bumpDataId();
        }
      }
    }
    // Our own write is not an edit, and what is read next frame is zero.
    noteOwnWrite(st, *obj);
    osIt->second.motionInputs = false;
  }
}

} // namespace box3dsolver
