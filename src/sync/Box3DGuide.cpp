// Box3DGuide.cpp
//
// The Box3D Solver's guide geometry (its Show Collision Shapes toggle): every
// body's collision shapes, as Box3D holds them, drawn as wireframe polylines.
// The shapes come from the live world (body space, so they do not depend on
// the frame); each body is placed at the pose the object's geometry shows on
// the displayed frame, so scrubbing through cached frames draws them where the
// pieces are. A body the world no longer has (deleted, or a world rebuilt since
// that frame) is not drawn.
//
// The same lines are also written as data on each object (the Export Collision
// Shapes toggle, kShapesDataName): the Box3D Solver SOP imports them to draw
// the shapes, so they follow the cached frames without the guide's lookups.
#include "sync/Box3DSolverInternal.h"

#include <GA/GA_PolyCounts.h>
#include <GEO/GEO_PrimPoly.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <vector>

namespace box3dsolver {

namespace {

constexpr int kCircleSegments = 32;
constexpr float kAsleepShade = 0.55f; // sleeping dynamic bodies
constexpr float kStaticShade = 0.35f; // static and kinematic bodies
// Chunks of bodies (of 256) before the drawing runs in parallel (untuned).
constexpr size_t kMinParallelGuideChunks = 4;

// The shared state of the Box3D world that has this object's bodies: the world
// data sits on one object of its solver, so look through the engine's objects.
// The guide runs on the viewport's thread while the solver may be stepping (or
// rebuilding the world), so the state is kept alive by `keep` and its mutex is
// taken (without waiting) before anything in it is read. Null when the state is
// busy: the guide is skipped.
Box3DSharedState *findState(const SIM_Object &obj,
                            std::shared_ptr<Box3DSharedState> &keep,
                            std::unique_lock<std::mutex> &lock) {
  const SIM_Engine &engine = obj.getEngine();
  const int id = obj.getObjectId();
  for (int i = 0; i < engine.getNumSimulationObjects(); ++i) {
    const SIM_Object *o = engine.getSimulationObject(i);
    if (!o)
      continue;
    const SIM_Box3DWorld *w =
        SIM_DATA_GETCONST(*o, kBox3DWorldDataName, SIM_Box3DWorld);
    std::shared_ptr<Box3DSharedState> sp = w ? w->sharedState() : nullptr;
    if (!sp)
      continue;
    std::unique_lock<std::mutex> l(sp->mutex, std::try_to_lock);
    if (!l.owns_lock())
      return nullptr;
    if (sp->world.isValid() && sp->objectSync.count(id)) {
      keep = std::move(sp);
      lock = std::move(l);
      return keep.get();
    }
  }
  return nullptr;
}

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

// Polylines with one colour each, written out at the end. The points live in
// one array and a line is a range of the index list, so lines can share points
// (a hull's faces share its vertices).
class Lines {
public:
  void begin(const UT_Vector3 &color) { myColor = color; }

  // Points that lines refer to by index; returns the index of the first.
  int addPoints(const UT_Vector3 *pts, size_t n) {
    const int base = static_cast<int>(myPoints.size());
    myPoints.insert(myPoints.end(), pts, pts + n);
    return base;
  }

  // A line through the points `base + idx[k]`.
  void addIndexed(int base, const int *idx, size_t n) {
    if (n < 2)
      return;
    for (size_t k = 0; k < n; ++k)
      myIndices.push_back(base + idx[k]);
    endLine();
  }

  // A line through new points.
  void add(const std::vector<UT_Vector3> &pts) {
    if (pts.size() < 2)
      return;
    const int base = addPoints(pts.data(), pts.size());
    for (size_t k = 0; k < pts.size(); ++k)
      myIndices.push_back(base + static_cast<int>(k));
    endLine();
  }

  const std::vector<UT_Vector3> &points() const { return myPoints; }
  const std::vector<int> &indices() const { return myIndices; }
  const std::vector<size_t> &ends() const { return myEnds; }

  // A full circle (closed: the first point repeated).
  void circle(const UT_Vector3 &c, const UT_Vector3 &u, const UT_Vector3 &v,
              float r) {
    arc(c, u, v, r, 0.0f, 2.0f * b3dop::kPi, kCircleSegments);
  }

  void arc(const UT_Vector3 &c, const UT_Vector3 &u, const UT_Vector3 &v,
           float r, float a0, float a1, int segments) {
    for (int k = 0; k <= segments; ++k) {
      const float a =
          a0 + (a1 - a0) * static_cast<float>(k) / static_cast<float>(segments);
      myIndices.push_back(static_cast<int>(myPoints.size()));
      myPoints.push_back(c + r * (std::cos(a) * u + std::sin(a) * v));
    }
    endLine();
  }

  // The lines of `other`, after this one's (the colours go with them).
  void append(const Lines &other) {
    const int pointBase = static_cast<int>(myPoints.size());
    const size_t indexBase = myIndices.size();
    myPoints.insert(myPoints.end(), other.myPoints.begin(),
                    other.myPoints.end());
    for (const int i : other.myIndices)
      myIndices.push_back(pointBase + i);
    for (const size_t end : other.myEnds)
      myEnds.push_back(indexBase + end);
    myColors.insert(myColors.end(), other.myColors.begin(),
                    other.myColors.end());
  }

  // Every line is an open polygon, built in one block: the points are made
  // with the polygons, and the polygons are at consecutive offsets.
  void write(GU_Detail &gdp) const {
    if (myEnds.empty())
      return;
    GA_RWHandleV3 cd(gdp.addFloatTuple(GA_ATTRIB_PRIMITIVE, "Cd", 3));
    if (cd.isValid())
      cd.getAttribute()->setTypeInfo(GA_TYPE_COLOR);
    GA_PolyCounts sizes;
    size_t begin = 0;
    for (const size_t end : myEnds) {
      sizes.append(static_cast<GA_Size>(end - begin));
      begin = end;
    }
    const GA_Offset first = GEO_PrimPoly::buildBlock(
        &gdp, myPoints.data(), static_cast<GA_Size>(myPoints.size()), sizes,
        myIndices.data(), /*closed=*/false);
    if (cd.isValid() && GAisValid(first))
      for (size_t i = 0; i < myColors.size(); ++i)
        cd.set(first + static_cast<GA_Offset>(i), myColors[i]);
  }

private:
  // The indices added since the last line become a line of the current colour.
  void endLine() {
    myEnds.push_back(myIndices.size());
    myColors.push_back(myColor);
  }

  UT_Vector3 myColor{1.0f, 1.0f, 1.0f};
  std::vector<UT_Vector3> myPoints;
  std::vector<int>
      myIndices; // the points of every line, one line after another
  std::vector<size_t> myEnds; // one past each line's last index
  std::vector<UT_Vector3> myColors;
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
    out.arc(c2, side, axis, r, 0.0f, b3dop::kPi, kCircleSegments / 2);
    out.arc(c1, side, -axis, r, 0.0f, b3dop::kPi, kCircleSegments / 2);
  }
}

// How much a body's lines are dimmed: static and kinematic bodies, and sleeping
// dynamic ones.
float shadeOf(const b3dop::Box3DWorld &world, const Box3DBodyMirror &e) {
  if (!e.dynamic())
    return kStaticShade;
  if (!world.isAwakeByHandle(e.handle))
    return kAsleepShade;
  return 1.0f;
}

// A hull's or mesh's vertices, placed by `pose`, as points the lines refer to;
// the index of the first.
int placeVertices(Lines &out, const b3dop::Box3DWorld::ShapeOutline &s,
                  const Pose &pose) {
  std::vector<UT_Vector3> pts(s.verts.size() / 3);
  for (size_t k = 0; k < pts.size(); ++k)
    pts[k] = pose.point(&s.verts[3 * k]);
  return out.addPoints(pts.data(), pts.size());
}

// A body's shape lines in the current colour, placed by `pose`.
void drawBodyShapes(Lines &out, const b3dop::Box3DWorld &world,
                    const Box3DBodyMirror &e, const Pose &pose) {
  std::vector<b3dop::Box3DWorld::ShapeOutline> shapes;
  if (!world.bodyShapeOutlines(e.handle, shapes) || shapes.empty())
    return;
  for (const b3dop::Box3DWorld::ShapeOutline &s : shapes) {
    switch (s.kind) {
    case b3dop::Box3DWorld::ShapeOutline::Sphere:
      drawSphere(out, pose.point(s.c1), s.radius);
      break;
    case b3dop::Box3DWorld::ShapeOutline::Capsule:
      drawCapsule(out, pose.point(s.c1), pose.point(s.c2), s.radius);
      break;
    case b3dop::Box3DWorld::ShapeOutline::Mesh: {
      const int base = placeVertices(out, s, pose);
      const size_t count = s.verts.size() / 3;
      for (size_t k = 0; k + 1 < s.loops.size(); k += 2) {
        const int pair[2] = {s.loops[k], s.loops[k + 1]};
        if (pair[0] >= 0 && pair[1] >= 0 &&
            static_cast<size_t>(pair[0]) < count &&
            static_cast<size_t>(pair[1]) < count)
          out.addIndexed(base, pair, 2);
      }
      break;
    }
    case b3dop::Box3DWorld::ShapeOutline::Hull: {
      // Each face as a closed loop over the hull's own vertices (shared edges
      // are drawn twice).
      const int base = placeVertices(out, s, pose);
      const size_t count = s.verts.size() / 3;
      std::vector<int> loop; // one buffer for every face
      for (size_t k = 0; k < s.loops.size();) {
        const int n = s.loops[k++];
        loop.clear();
        for (int j = 0; j < n && k < s.loops.size(); ++j, ++k)
          if (s.loops[k] >= 0 && static_cast<size_t>(s.loops[k]) < count)
            loop.push_back(s.loops[k]);
        if (!loop.empty())
          loop.push_back(loop.front());
        out.addIndexed(base, loop.data(), loop.size());
      }
      break;
    }
    }
  }
}

void drawBody(Lines &out, const b3dop::Box3DWorld &world,
              const Box3DBodyMirror &e, const Pose &pose,
              const UT_Vector3 &color) {
  out.begin(color * shadeOf(world, e));
  drawBodyShapes(out, world, e, pose);
}

// A body and where it is.
struct Placed {
  const Box3DBodyMirror *e;
  Pose p;
};

// An object's bodies and the poses its geometry shows: one body per point,
// keyed as the sync keys it (`id`, else the point number), posed by the point's
// P and its packed rotation (else orient); or the one body of a whole object,
// posed by its Position data. The caller holds st.mutex.
void placeBodies(const Box3DSharedState &st, const SIM_Object &obj,
                 std::vector<Placed> &placed) {
  placed.clear();
  const int objectId = obj.getObjectId();
  const auto sync = st.objectSync.find(objectId);
  if (sync == st.objectSync.end())
    return;

  if (sync->second.wholeObject) {
    const Box3DBodyMirror *body =
        st.mirror.find(b3dop::makeBodyKey(objectId, kWholeObjectId));
    double pivot[3];
    Box3DBodyPose pose;
    if (body && readWholePose(obj, pivot, pose)) {
      Placed pl{body, Pose()};
      b3dop::copy3(pl.p.pos, pose.pos);
      b3dop::copy4(pl.p.rot, pose.rot);
      placed.push_back(pl);
    }
    return;
  }
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
  // Read in chunks in parallel (the geometry and the mirror are only read).
  const GA_Size count = gdp->getNumPoints();
  std::vector<Placed> slots(static_cast<size_t>(count),
                            Placed{nullptr, Pose()});
  constexpr size_t kChunk = 256;
  parallelForEach((slots.size() + kChunk - 1) / kChunk,
                  [&](size_t c) {
                    const size_t end = std::min(slots.size(), (c + 1) * kChunk);
                    for (size_t i = c * kChunk; i < end; ++i) {
                      const GA_Offset off =
                          gdp->pointOffset(GA_Index(static_cast<GA_Size>(i)));
                      const int64_t bid =
                          id.isValid() ? static_cast<int64_t>(id.get(off))
                                       : static_cast<int64_t>(i);
                      const Box3DBodyMirror *e =
                          st.mirror.find(b3dop::makeBodyKey(objectId, bid));
                      if (!e)
                        continue;
                      Pose p;
                      const PackedInfo pk =
                          anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
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
                      slots[i] = {e, p};
                    }
                  },
                  kMinParallelGuideChunks);
  for (const Placed &pl : slots)
    if (pl.e)
      placed.push_back(pl);
}

// The same bodies as the solver holds them right after a step: the object's
// body list, each at the pose the write-back stored in the mirror (what its
// point holds now), so no geometry is read. A whole object is placed by its
// Position data as placeBodies does.
void placeBodiesFromMirror(const Box3DSharedState &st, const SIM_Object &obj,
                           std::vector<Placed> &placed) {
  const auto sync = st.objectSync.find(obj.getObjectId());
  if (sync == st.objectSync.end() || sync->second.wholeObject) {
    placeBodies(st, obj, placed);
    return;
  }
  placed.clear();
  placed.reserve(sync->second.bodies.size());
  for (const Box3DBodyMirror *e : sync->second.bodies) {
    if (!e || !e->handle || !st.isLive(*e))
      continue;
    Placed pl{e, Pose()};
    b3dop::copy3(pl.p.pos, e->pos);
    b3dop::copy4(pl.p.rot, e->rot);
    placed.push_back(pl);
  }
}

// One object's bodies drawn into `lines`. The caller holds st.mutex. The
// bodies are drawn in chunks in parallel, each into its own Lines, and joined
// in order.
void drawObjectShapes(Lines &lines, Box3DSharedState &st, const SIM_Object &obj,
                      const UT_Vector3 &color) {
  std::vector<Placed> placed;
  placeBodies(st, obj, placed);
  constexpr size_t kChunk = 256;
  const size_t chunks = (placed.size() + kChunk - 1) / kChunk;
  std::vector<Lines> parts(chunks);
  parallelForEach(
      chunks,
      [&](size_t c) {
        const size_t end = std::min(placed.size(), (c + 1) * kChunk);
        for (size_t i = c * kChunk; i < end; ++i)
          drawBody(parts[c], st.world, *placed[i].e, placed[i].p, color);
      },
      kMinParallelGuideChunks);
  for (const Lines &part : parts)
    lines.append(part);
}

// ---- Export Collision Shapes: the lines are kept between steps ------------
//
// A step that only moves bodies moves the exported points: the outlines are
// kept in body space (Box3DShapeExport) and each body's points are placed again
// only when its pose differs from what the geometry holds. The geometry carries
// the signature of the bodies and shapes it was built for; any change in them
// (a body added or removed, new shapes) or a geometry that does not match
// (another timeline, a discarded step) builds the lines again from scratch.

constexpr const char *kShapesSigName = "b3d_shapes_sig";

// Which bodies and shapes the lines are for: each body's handle, shape
// signature, shape count, body type and scale.
uint64_t shapesSignature(const b3dop::Box3DWorld &world,
                         const std::vector<Placed> &placed) {
  uint64_t h = b3dop::mixHash(0x9e3779b97f4a7c15ull, placed.size());
  for (const Placed &pl : placed) {
    const Box3DBodyMirror &e = *pl.e;
    h = b3dop::mixHash(h, static_cast<uint64_t>(e.handle));
    h = b3dop::mixHash(h, e.shapeSig);
    h = b3dop::mixHash(
        h, static_cast<uint64_t>(world.shapeCountByHandle(e.handle)));
    h = b3dop::mixHash(h, e.dynamic() ? 1u : 0u);
    h = b3dop::hashBytes(h, e.sizeKey, sizeof(e.sizeKey));
  }
  return h;
}

// Moves the exported points to the bodies' poses, and re-colours a body whose
// shade changed. False when the geometry is not the one `cache` describes.
bool updateShapes(GU_Detail &gdp, const Box3DSharedState &st,
                  const std::vector<Placed> &placed, uint64_t sig,
                  const Box3DShapeExport &cache, const UT_Vector3 &color) {
  if (cache.sig != sig || cache.bodies.size() != placed.size())
    return false;
  const GA_ROHandleI marker(&gdp, GA_ATTRIB_DETAIL, kShapesSigName);
  if (!marker.isValid() || marker.getAttribute()->getTupleSize() < 2 ||
      static_cast<uint32_t>(marker.get(GA_Offset(0), 0)) !=
          static_cast<uint32_t>(sig & 0xffffffffu) ||
      static_cast<uint32_t>(marker.get(GA_Offset(0), 1)) !=
          static_cast<uint32_t>(sig >> 32))
    return false;
  if (gdp.getNumPoints() != static_cast<GA_Size>(cache.points) ||
      gdp.getNumPrimitives() != static_cast<GA_Size>(cache.lines))
    return false;
  if (cache.points == 0)
    return true; // nothing is drawn
  // The points and polylines are the consecutive offsets they were built as.
  const GA_Offset p0 = gdp.pointOffset(GA_Index(0));
  const GA_Offset q0 = gdp.primitiveOffset(GA_Index(0));
  if (gdp.pointOffset(GA_Index(cache.points - 1)) !=
          p0 + static_cast<GA_Offset>(cache.points - 1) ||
      gdp.primitiveOffset(GA_Index(cache.lines - 1)) !=
          q0 + static_cast<GA_Offset>(cache.lines - 1))
    return false;
  GA_RWHandleV3 P(gdp.getP());
  GA_RWHandleV3 cd(&gdp, GA_ATTRIB_PRIMITIVE, "Cd");
  if (!P.isValid() || !cd.isValid())
    return false;
  bool movedP = false, movedCd = false;
  for (size_t i = 0; i < placed.size(); ++i) {
    const Box3DShapeExportBody &b = cache.bodies[i];
    const size_t n = b.pts.size() / 3;
    if (n == 0)
      continue;
    const Pose &pose = placed[i].p;
    // A body that has not moved has the same points: three of them say so.
    const size_t probes[3] = {0, n / 2, n - 1};
    bool same = true;
    for (const size_t k : probes)
      if (!(P.get(p0 + static_cast<GA_Offset>(b.pointStart + k)) ==
            pose.point(&b.pts[3 * k])))
        same = false;
    if (!same) {
      for (size_t k = 0; k < n; ++k)
        P.set(p0 + static_cast<GA_Offset>(b.pointStart + k),
              pose.point(&b.pts[3 * k]));
      movedP = true;
    }
    const UT_Vector3 want = color * shadeOf(st.world, *placed[i].e);
    const GA_Offset first = q0 + static_cast<GA_Offset>(b.lineStart);
    if (!(cd.get(first) == want)) {
      for (size_t k = 0; k < b.ends.size(); ++k)
        cd.set(first + static_cast<GA_Offset>(k), want);
      movedCd = true;
    }
  }
  if (movedP)
    P.getAttribute()->bumpDataId();
  if (movedCd)
    cd.getAttribute()->bumpDataId();
  return true;
}

// Builds the lines from scratch and remembers the outlines.
void rebuildShapes(GU_Detail &gdp, const Box3DSharedState &st,
                   const std::vector<Placed> &placed, uint64_t sig,
                   Box3DShapeExport &cache, const UT_Vector3 &color) {
  cache.sig = sig;
  cache.bodies.assign(placed.size(), Box3DShapeExportBody());
  constexpr size_t kChunk = 256;
  const size_t chunks = (placed.size() + kChunk - 1) / kChunk;
  parallelForEach(
      chunks,
      [&](size_t c) {
        const size_t end = std::min(placed.size(), (c + 1) * kChunk);
        for (size_t i = c * kChunk; i < end; ++i) {
          Lines local;
          drawBodyShapes(local, st.world, *placed[i].e, Pose());
          Box3DShapeExportBody &b = cache.bodies[i];
          b.pts.reserve(local.points().size() * 3);
          for (const UT_Vector3 &v : local.points()) {
            b.pts.push_back(v.x());
            b.pts.push_back(v.y());
            b.pts.push_back(v.z());
          }
          b.idx = local.indices();
          for (const size_t e : local.ends())
            b.ends.push_back(static_cast<uint32_t>(e));
        }
      },
      kMinParallelGuideChunks);
  Lines world;
  uint32_t points = 0, lines = 0;
  for (size_t i = 0; i < placed.size(); ++i) {
    Box3DShapeExportBody &b = cache.bodies[i];
    b.pointStart = points;
    b.lineStart = lines;
    world.begin(color * shadeOf(st.world, *placed[i].e));
    const size_t n = b.pts.size() / 3;
    std::vector<UT_Vector3> pts(n);
    for (size_t k = 0; k < n; ++k)
      pts[k] = placed[i].p.point(&b.pts[3 * k]);
    const int base = world.addPoints(pts.data(), n);
    uint32_t from = 0;
    for (const uint32_t end : b.ends) {
      world.addIndexed(base, &b.idx[from], end - from);
      from = end;
    }
    points += static_cast<uint32_t>(n);
    lines += static_cast<uint32_t>(b.ends.size());
  }
  cache.points = points;
  cache.lines = lines;
  gdp.clearAndDestroy();
  world.write(gdp);
  GA_RWHandleI marker(gdp.addIntTuple(GA_ATTRIB_DETAIL, kShapesSigName, 2));
  if (marker.isValid()) {
    marker.set(GA_Offset(0), 0, static_cast<int>(sig & 0xffffffffu));
    marker.set(GA_Offset(0), 1, static_cast<int>(sig >> 32));
  }
}

} // namespace

void buildCollisionGuide(const SIM_Object &obj, GU_Detail &out,
                         const UT_Vector3 &color) {
  // `lock` is declared after `keep`, so it is released first.
  std::shared_ptr<Box3DSharedState> keep;
  std::unique_lock<std::mutex> lock;
  Box3DSharedState *st = findState(obj, keep, lock);
  if (!st)
    return;
  Lines lines;
  drawObjectShapes(lines, *st, obj, color);
  lines.write(out);
}

void exportCollisionShapes(Box3DSharedState &st, SIM_ObjectArray &objects) {
  const UT_Vector3 white(1.0f, 1.0f, 1.0f); // the SOP tints it (Shape Color)
  std::vector<Placed> placed;
  for (int i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    if (!obj || !st.objectSync.count(obj->getObjectId()))
      continue;
    placeBodiesFromMirror(st, *obj, placed);
    const uint64_t sig = shapesSignature(st.world, placed);
    SIM_GeometryCopy *data = SIM_DATA_CREATE(
        *obj, kShapesDataName, SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING);
    if (!data)
      continue;
    SIM_GeometryAutoWriteLock lock(data);
    GU_Detail &gdp = lock.getGdp();
    Box3DShapeExport &cache = st.shapeExports[obj->getObjectId()];
    if (!updateShapes(gdp, st, placed, sig, cache, white))
      rebuildShapes(gdp, st, placed, sig, cache, white);
  }
}

} // namespace box3dsolver
