// Box3DGuide.cpp
//
// The Box3D Solver's guide geometry (its Show Collision Shapes toggle): every
// body's collision shapes, as Box3D holds them, drawn as wireframe polylines.
// The shapes come from the live world (body space, so they do not depend on
// the frame); each body is placed at the pose the object's geometry shows on
// the displayed frame, so scrubbing through cached frames draws them where the
// pieces are. A body the world no longer has (deleted, or a world rebuilt since
// that frame) is not drawn.
#include "sync/Box3DSolverInternal.h"

#include <GU/GU_PrimPoly.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Object.h>

#include <cmath>
#include <mutex>
#include <vector>

namespace box3dsolver {

namespace {

constexpr int kCircleSegments = 32;
constexpr float kPi = 3.14159265358979f;
constexpr float kAsleepShade = 0.55f; // sleeping dynamic bodies
constexpr float kStaticShade = 0.35f; // static and kinematic bodies

// The shared state of the Box3D world that has this object's bodies: the world
// data sits on one object of its solver, so look through the engine's objects.
Box3DSharedState *findState(const SIM_Object &obj) {
  const SIM_Engine &engine = obj.getEngine();
  const int id = obj.getObjectId();
  for (int i = 0; i < engine.getNumSimulationObjects(); ++i) {
    const SIM_Object *o = engine.getSimulationObject(i);
    if (!o)
      continue;
    const SIM_Box3DWorld *w =
        SIM_DATA_GETCONST(*o, kBox3DWorldDataName, SIM_Box3DWorld);
    Box3DSharedState *st = w ? w->state() : nullptr;
    if (st && st->world.isValid() && st->objectSync.count(id))
      return st;
  }
  return nullptr;
}

// Polylines with one colour each, written out at the end.
class Lines {
public:
  void begin(const UT_Vector3 &color) { myColor = color; }

  void add(const std::vector<UT_Vector3> &pts) {
    if (pts.size() < 2)
      return;
    myLines.push_back(pts);
    myColors.push_back(myColor);
  }

  // A full circle (closed: the first point repeated).
  void circle(const UT_Vector3 &c, const UT_Vector3 &u, const UT_Vector3 &v,
              float r) {
    arc(c, u, v, r, 0.0f, 2.0f * kPi, kCircleSegments);
  }

  void arc(const UT_Vector3 &c, const UT_Vector3 &u, const UT_Vector3 &v,
           float r, float a0, float a1, int segments) {
    std::vector<UT_Vector3> pts;
    pts.reserve(static_cast<size_t>(segments) + 1);
    for (int k = 0; k <= segments; ++k) {
      const float a =
          a0 + (a1 - a0) * static_cast<float>(k) / static_cast<float>(segments);
      pts.push_back(c + r * (std::cos(a) * u + std::sin(a) * v));
    }
    add(pts);
  }

  void write(GU_Detail &gdp) const {
    GA_RWHandleV3 cd(gdp.addFloatTuple(GA_ATTRIB_PRIMITIVE, "Cd", 3));
    if (cd.isValid())
      cd.getAttribute()->setTypeInfo(GA_TYPE_COLOR);
    for (size_t i = 0; i < myLines.size(); ++i) {
      const std::vector<UT_Vector3> &pts = myLines[i];
      GU_PrimPoly *poly = GU_PrimPoly::build(
          &gdp, static_cast<GA_Size>(pts.size()), /*open=*/true);
      if (!poly)
        continue;
      for (size_t k = 0; k < pts.size(); ++k)
        gdp.setPos3(poly->getPointOffset(static_cast<GA_Size>(k)), pts[k]);
      if (cd.isValid())
        cd.set(poly->getMapOffset(), myColors[i]);
    }
  }

private:
  UT_Vector3 myColor{1.0f, 1.0f, 1.0f};
  std::vector<std::vector<UT_Vector3>> myLines;
  std::vector<UT_Vector3> myColors;
};

// Body space -> world space for one body.
struct Pose {
  double pos[3] = {0, 0, 0};
  float rot[4] = {0, 0, 0, 1};

  UT_Vector3 point(const float p[3]) const {
    float r[3];
    b3dop::quatRotate(rot, p, r);
    return UT_Vector3(static_cast<float>(pos[0] + r[0]),
                      static_cast<float>(pos[1] + r[1]),
                      static_cast<float>(pos[2] + r[2]));
  }
  UT_Vector3 dir(const float d[3]) const {
    float r[3];
    b3dop::quatRotate(rot, d, r);
    return UT_Vector3(r[0], r[1], r[2]);
  }
};

// Two unit vectors perpendicular to unit `a` and to each other.
void perpendiculars(const UT_Vector3 &a, UT_Vector3 &u, UT_Vector3 &v) {
  const UT_Vector3 ref =
      std::fabs(a.x()) < 0.9f ? UT_Vector3(1, 0, 0) : UT_Vector3(0, 1, 0);
  u = cross(a, ref);
  u.normalize();
  v = cross(a, u);
  v.normalize();
}

void drawSphere(Lines &out, const UT_Vector3 &c, float r) {
  const UT_Vector3 x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
  out.circle(c, x, y, r);
  out.circle(c, y, z, r);
  out.circle(c, z, x, r);
}

void drawCapsule(Lines &out, const UT_Vector3 &c1, const UT_Vector3 &c2,
                 float r) {
  UT_Vector3 axis = c2 - c1;
  const float len = axis.length();
  if (len < 1e-6f) {
    drawSphere(out, c1, r);
    return;
  }
  axis /= len;
  UT_Vector3 u, v;
  perpendiculars(axis, u, v);
  out.circle(c1, u, v, r);
  out.circle(c2, u, v, r);
  for (const UT_Vector3 &side : {u, -u, v, -v})
    out.add({c1 + r * side, c2 + r * side});
  // Each cap: two half circles over its end, in the planes of u and v.
  for (const UT_Vector3 &side : {u, v}) {
    out.arc(c2, side, axis, r, 0.0f, kPi, kCircleSegments / 2);
    out.arc(c1, side, -axis, r, 0.0f, kPi, kCircleSegments / 2);
  }
}

void drawBody(Lines &out, const b3dop::Box3DWorld &world,
              const Box3DBodyMirror &e, const Pose &pose,
              const UT_Vector3 &color) {
  std::vector<b3dop::Box3DWorld::ShapeOutline> shapes;
  if (!world.bodyShapeOutlines(e.handle, shapes) || shapes.empty())
    return;
  float shade = 1.0f;
  if (!e.dynamic())
    shade = kStaticShade;
  else if (!world.isAwakeByHandle(e.handle))
    shade = kAsleepShade;
  out.begin(color * shade);
  for (const b3dop::Box3DWorld::ShapeOutline &s : shapes) {
    switch (s.kind) {
    case b3dop::Box3DWorld::ShapeOutline::Sphere:
      drawSphere(out, pose.point(s.c1), s.radius);
      break;
    case b3dop::Box3DWorld::ShapeOutline::Capsule:
      drawCapsule(out, pose.point(s.c1), pose.point(s.c2), s.radius);
      break;
    case b3dop::Box3DWorld::ShapeOutline::Hull: {
      // Each face as a closed loop (shared edges are drawn twice).
      for (size_t k = 0; k < s.loops.size();) {
        const int n = s.loops[k++];
        std::vector<UT_Vector3> loop;
        loop.reserve(static_cast<size_t>(n) + 1);
        for (int j = 0; j < n && k < s.loops.size(); ++j, ++k) {
          const size_t v = static_cast<size_t>(s.loops[k]) * 3;
          if (v + 2 < s.verts.size())
            loop.push_back(pose.point(&s.verts[v]));
        }
        if (!loop.empty())
          loop.push_back(loop.front());
        out.add(loop);
      }
      break;
    }
    }
  }
}

} // namespace

void buildCollisionGuide(const SIM_Object &obj, GU_Detail &out,
                         const UT_Vector3 &color) {
  Box3DSharedState *st = findState(obj);
  if (!st)
    return;
  // The solver holds the lock while it steps; a guide drawn meanwhile is
  // skipped rather than waiting.
  std::unique_lock<std::mutex> lock(st->mutex, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  const int objectId = obj.getObjectId();
  const auto sync = st->objectSync.find(objectId);
  if (sync == st->objectSync.end())
    return;

  Lines lines;
  if (sync->second.wholeObject) {
    // One body: posed by the object's Position data.
    const auto it =
        st->mirror.find(b3dop::makeBodyKey(objectId, kWholeObjectId));
    double pivot[3];
    Box3DBodyPose pose;
    if (it != st->mirror.end() && readWholePose(obj, pivot, pose)) {
      Pose p;
      b3dop::copy3(p.pos, pose.pos);
      b3dop::copy4(p.rot, pose.rot);
      drawBody(lines, st->world, it->second, p, color);
    }
  } else {
    // One body per point, keyed as the sync keys it: `id`, else the point
    // number; posed by the point's P and its packed rotation (else orient).
    const SIM_Geometry *geo = obj.getGeometry();
    if (!geo)
      return;
    SIM_GeometryAutoReadLock geoLock(geo);
    const GU_Detail *gdp = geoLock.getGdp();
    if (!gdp)
      return;
    const GA_ROHandleI id(gdp, GA_ATTRIB_POINT, "id");
    const GA_ROHandleQ orient(gdp, GA_ATTRIB_POINT, "orient");
    const bool anyPrims = gdp->getNumPrimitives() > 0;
    int64_t n = 0;
    for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it, ++n) {
      const GA_Offset off = *it;
      const int64_t bid = id.isValid() ? static_cast<int64_t>(id.get(off)) : n;
      const auto e = st->mirror.find(b3dop::makeBodyKey(objectId, bid));
      if (e == st->mirror.end())
        continue;
      Pose p;
      const PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
      if (pk.prim) {
        b3dop::copy3(p.pos, pk.P);
        b3dop::copy4(p.rot, pk.rot);
      } else {
        const UT_Vector3 P = gdp->getPos3(off);
        p.pos[0] = P.x();
        p.pos[1] = P.y();
        p.pos[2] = P.z();
        if (orient.isValid()) {
          const UT_QuaternionF q = orient.get(off);
          p.rot[0] = q.x();
          p.rot[1] = q.y();
          p.rot[2] = q.z();
          p.rot[3] = q.w();
          b3dop::sanitizeQuat(p.rot);
        }
      }
      drawBody(lines, st->world, e->second, p, color);
    }
  }
  lines.write(out);
}

} // namespace box3dsolver
