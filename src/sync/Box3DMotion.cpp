// Box3DMotion.cpp
//
// The drag and speed limits RBD Bullet Solver reads from POP-style point
// attributes (Box3DMotion.h has the maths): read per object when the attributes
// change, the drag added to the force pass (Box3DForces.cpp), the limits
// applied to the moved bodies after every Box3D step.
#include "Box3DSolverInternal.h"

#include <GA/GA_Handle.h>
#include <GA/GA_SplittableRange.h>

#include <algorithm>
#include <cmath>
#include <mutex>

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

bool hasAnyMotionAttrib(const GU_Detail *gdp) {
  for (const char *name : kMotionAttribNames)
    if (gdp->findPointAttribute(name))
      return true;
  return false;
}

// `inputs`: some point has a non-zero one-step input (see
// Box3DObjectSync::motionInputs).
void readMotion(const GU_Detail *gdp, std::vector<Box3DMotionEntry> &out,
                bool &inputs) {
  out.clear();
  inputs = false;
  if (!hasAnyMotionAttrib(gdp))
    return;
  const GA_ROHandleF airresist(gdp, GA_ATTRIB_POINT, "airresist"),
      spinresist(gdp, GA_ATTRIB_POINT, "spinresist"),
      drag(gdp, GA_ATTRIB_POINT, "drag"),
      dragexp(gdp, GA_ATTRIB_POINT, "dragexp"),
      speedmin(gdp, GA_ATTRIB_POINT, "speedmin"),
      speedmax(gdp, GA_ATTRIB_POINT, "speedmax"),
      spinmin(gdp, GA_ATTRIB_POINT, "spinmin"),
      spinmax(gdp, GA_ATTRIB_POINT, "spinmax");
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
      readMotion(gdp, os.motion, os.motionInputs);
      os.motionSig = sig;
    }
    any = any || !os.motion.empty();
  }
  st.anyMotion = any;
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
