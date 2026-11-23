// Box3DShapeBuild.cpp
//
// Collision source geometry (packed primitives' pieces, cached per source) and
// the Box3D shapes built from it, for points and for whole objects.
#include "Box3DSolverInternal.h"

#include "core/Box3DQuadrics.h"

#include <GA/GA_PrimitiveTypes.h>
#include <GEO/GEO_PrimTube.h>
#include <GEO/GEO_Quadric.h>
#include <UT/UT_Matrix3.h>

#include <cmath>

namespace box3dsolver {

// Packed 3x3 (row-vector, M = S * R) split into per-row scale and a unit
// quaternion with the HDK, so packed rotations follow Houdini's own quaternion
// convention. Shear is dropped. A mirror is split off as a flip of the third
// row (the hull points are mirrored to match, packedScale; the write-back puts
// the flip back, keepMirror). False (identity, scale 1) if degenerate.
bool splitPackedMatrix(const double m3[9], float q[4], double scale[3]) {
  q[0] = q[1] = q[2] = 0.0f;
  q[3] = 1.0f;
  scale[0] = scale[1] = scale[2] = 1.0;
  UT_Matrix3D m(m3[0], m3[1], m3[2], m3[3], m3[4], m3[5], m3[6], m3[7], m3[8]);
  const double det = m.determinant();
  if (!std::isfinite(det) || std::fabs(det) < 1e-30)
    return false;
  if (det < 0.0) // mirrored: flip the third row
    for (unsigned c = 0; c < 3; ++c)
      m(2, c) = -m(2, c);
  UT_Vector3D s;
  m.extractScales(s); // m is left a pure rotation
  UT_QuaternionD rq;
  rq.updateFromRotationMatrix(m);
  rq.normalize();
  q[0] = static_cast<float>(rq.x());
  q[1] = static_cast<float>(rq.y());
  q[2] = static_cast<float>(rq.z());
  q[3] = static_cast<float>(rq.w());
  scale[0] = std::fabs(s.x());
  scale[1] = std::fabs(s.y());
  scale[2] = std::fabs(s.z());
  return true;
}

// A per-row scale (splitPackedMatrix's, kept from the sync) with q's rotation:
// a packed primitive's new local 3x3.
void composePackedMatrix(const float q[4], const double sc[3], double out[9]) {
  UT_DMatrix3 r;
  UT_QuaternionD(q[0], q[1], q[2], q[3]).getRotationMatrix(r);
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col)
      out[row * 3 + col] = sc[row] * r(row, col);
}

// The rotation of a packed point's `restxform` (row-vector 4x4 as stored), the
// way splitPackedMatrix splits a packed 3x3. False while it is unset (all zero:
// the value of a point added after the attribute) or degenerate.
bool restRotation(const UT_Matrix4F &m, float q[4]) {
  bool any = false;
  for (unsigned r = 0; r < 4 && !any; ++r)
    for (unsigned c = 0; c < 4 && !any; ++c)
      any = m(r, c) != 0.0f;
  if (!any)
    return false;
  double m3[9];
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c)
      m3[r * 3 + c] = static_cast<double>(m(r, c));
  double scale[3];
  return splitPackedMatrix(m3, q, scale);
}

// The primitive's full transform, row-major.
void fetchFull4(const GU_PrimPacked &p, double out[16]) {
  UT_Matrix4D m;
  p.getFullTransform4(m);
  box3dToArray(m, out);
}

// Writes the packed primitive's local 3x3.
void setLocal3(GU_PrimPacked &p, const double m[9]) {
  UT_Matrix3D mat;
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c)
      mat(r, c) = m[r * 3 + c];
  p.setLocalTransform(mat);
  // It caches bounds derived from the transform.
  p.transformDirty();
}

// The point's packed primitive, if any (prim == nullptr otherwise), without the
// pivot.
PackedInfo getPackedInfo(const GU_Detail *gdp, GA_Offset ptoff) {
  PackedInfo pk;
  const GA_Offset vtx = gdp->pointVertex(ptoff);
  if (!GAisValid(vtx))
    return pk;
  const GA_Offset po = gdp->vertexPrimitive(vtx);
  if (!GAisValid(po))
    return pk;
  const GA_Primitive *prim = gdp->getPrimitive(po);
  if (!prim || !GU_PrimPacked::isPackedPrimitive(*prim))
    return pk;

  pk.prim = static_cast<const GU_PrimPacked *>(prim);
  pk.primOff = po;
  const UT_Vector3 pos = gdp->getPos3(ptoff);
  pk.P[0] = pos.x();
  pk.P[1] = pos.y();
  pk.P[2] = pos.z();
  fetchFull4(*pk.prim, pk.m4);
  b3dop::upper3x3(pk.m4, pk.m3);
  splitPackedMatrix(pk.m3, pk.rot, pk.scale);
  return pk;
}

// Fills pk.pivot (only needed to build shapes).
void readPackedPivot(PackedInfo &pk) {
  if (!pk.prim)
    return;
  UT_Vector3 pv(0, 0, 0);
  pk.prim->getPivot(pv);
  pk.pivot[0] = pv.x();
  pk.pivot[1] = pv.y();
  pk.pivot[2] = pv.z();
}

// ---------------------------------------------------------------------------
// Collision source geometry
// ---------------------------------------------------------------------------

namespace {

// A Tube's two end rings as Houdini evaluates them, which includes its taper
// (one radius at each end). False if the primitive cannot be evaluated, then
// the unit tube is sampled instead.
bool appendTubePoints(const GA_Primitive &prim, NamedPiece &np) {
  const GEO_PrimTube *tube = dynamic_cast<const GEO_PrimTube *>(&prim);
  if (!tube)
    return false;
  std::vector<float> ring;
  for (int end = 0; end < 2; ++end)
    for (int j = 0; j < b3dop::kQuadricSegments; ++j) {
      UT_Vector4 pos;
      // u around, v along; v = 1 is the end u wraps to, so stay inside [0, 1).
      if (tube->evaluatePoint(pos,
                              static_cast<float>(j) /
                                  static_cast<float>(b3dop::kQuadricSegments),
                              static_cast<float>(end)) != 0 ||
          !std::isfinite(pos.x() + pos.y() + pos.z()))
        return false;
      ring.push_back(pos.x());
      ring.push_back(pos.y());
      ring.push_back(pos.z());
    }
  np.samples.insert(np.samples.end(), ring.begin(), ring.end());
  return true;
}

// A Sphere, Tube, Circle or Metaball primitive has one point and a transform,
// so its points alone make no shape: its surface is sampled too
// (Box3DQuadrics.h). Their transform is GEO_Quadric::getTransform (no
// translation: the position is the point).
void appendQuadricSamples(const GU_Detail *gdp, const GA_Primitive &prim,
                          NamedPiece &np) {
  if (prim.getTypeId() == GA_PRIMPOLY || prim.getVertexCount() < 1)
    return;
  const b3dop::QuadricKind kind =
      b3dop::quadricKindFromName(prim.getTypeName());
  if (kind == b3dop::QuadricKind::None)
    return;
  const GEO_Quadric *quadric = dynamic_cast<const GEO_Quadric *>(&prim);
  if (!quadric)
    return;
  const UT_Matrix3 &m = quadric->getTransform();
  // UT_Matrix3 multiplies row vectors (p' = p * m): the math matrix is its
  // transpose.
  double a[9];
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c)
      a[c * 3 + r] = m(r, c);
  const UT_Vector3 p = gdp->getPos3(prim.getPointOffset(0));
  const double centre[3] = {p.x(), p.y(), p.z()};
  if (!(kind == b3dop::QuadricKind::Tube && appendTubePoints(prim, np)))
    b3dop::appendQuadricPoints(kind, a, centre, np.samples);
  double radius = 0;
  if ((kind == b3dop::QuadricKind::Sphere ||
       kind == b3dop::QuadricKind::Metaball) &&
      b3dop::quadricSphereRadius(a, radius)) {
    if (kind == b3dop::QuadricKind::Metaball)
      radius *= b3dop::kMetaballRadiusScale;
    np.sphereValid = true;
    np.sphereRadius = radius;
    std::copy(centre, centre + 3, np.sphereCentre);
  }
}

const GU_PackedFragment *asFragment(const GU_PrimPacked &prim);
void appendPrimPoints(const GU_Detail *gdp, const GA_Primitive &prim,
                      NamedPiece &np, int depth = 0);

// A packed primitive inside a source: its own points (recursively, for packed
// primitives nested deeper) carried into the source's space, (p - pivot) * M +
// P with M its full transform's 3x3 and P its point's position. Appended to
// np.samples; the primitive's own point is not a collision point.
void appendNestedPacked(const GU_Detail *gdp, const GU_PrimPacked &packed,
                        NamedPiece &np, int depth) {
  constexpr int kMaxDepth = 8; // a cycle cannot be built, but stay bounded
  if (depth >= kMaxDepth || packed.getVertexCount() < 1)
    return;
  const GU_PackedFragment *frag = asFragment(packed);
  GU_ConstDetailHandle handle =
      frag ? frag->detailPtr() : packed.getPackedDetail();
  GU_DetailHandleAutoReadLock lock(handle);
  const GU_Detail *inner = lock.getGdp();
  if (!inner)
    return;

  // The inner points in the packed primitive's own space.
  NamedPiece in;
  bool anyPrim = false;
  const GA_Range prims =
      frag ? frag->getPrimitiveRange() : inner->getPrimitiveRange();
  for (GA_Iterator it(prims); !it.atEnd(); ++it)
    if (const GA_Primitive *prim = inner->getPrimitive(*it)) {
      anyPrim = true;
      appendPrimPoints(inner, *prim, in, depth + 1);
    }
  if (!anyPrim)
    for (GA_Iterator it(inner->getPointRange()); !it.atEnd(); ++it)
      in.points.push_back(*it);
  std::sort(in.points.begin(), in.points.end());
  in.points.erase(std::unique(in.points.begin(), in.points.end()),
                  in.points.end());
  std::vector<float> pts;
  pts.reserve(3 * in.points.size() + in.samples.size());
  for (const GA_Offset pt : in.points) {
    const UT_Vector3 p = inner->getPos3(pt);
    pts.push_back(p.x());
    pts.push_back(p.y());
    pts.push_back(p.z());
  }
  pts.insert(pts.end(), in.samples.begin(), in.samples.end());

  double m4[16], m3[9];
  fetchFull4(packed, m4);
  b3dop::upper3x3(m4, m3);
  UT_Vector3 pv(0, 0, 0);
  packed.getPivot(pv);
  const UT_Vector3 P = gdp->getPos3(packed.getPointOffset(0));
  const double pivot[3] = {pv.x(), pv.y(), pv.z()};
  const double origin[3] = {P.x(), P.y(), P.z()};
  for (size_t i = 0; i + 2 < pts.size(); i += 3) {
    double d[3];
    for (int k = 0; k < 3; ++k)
      d[k] = static_cast<double>(pts[i + static_cast<size_t>(k)]) - pivot[k];
    for (int j = 0; j < 3; ++j)
      np.samples.push_back(static_cast<float>(d[0] * m3[j] + d[1] * m3[3 + j] +
                                              d[2] * m3[6 + j] + origin[j]));
  }
}

void appendPrimPoints(const GU_Detail *gdp, const GA_Primitive &prim,
                      NamedPiece &np, int depth) {
  ++np.primCount;
  if (GU_PrimPacked::isPackedPrimitive(prim)) {
    appendNestedPacked(gdp, static_cast<const GU_PrimPacked &>(prim), np,
                       depth);
    return;
  }
  const GA_Size nv = prim.getVertexCount();
  for (GA_Size i = 0; i < nv; ++i)
    np.points.push_back(prim.getPointOffset(i));
  appendQuadricSamples(gdp, prim, np);
}

// Each piece's points once (in offset order) into xyz, then the hashes of each
// piece and of all of them together (once per shared source).
void finishPieces(const GU_Detail *gdp, PieceGroup &g) {
  g.hash = 0;
  for (NamedPiece &np : g.pieces) {
    std::sort(np.points.begin(), np.points.end());
    np.points.erase(std::unique(np.points.begin(), np.points.end()),
                    np.points.end());
    np.xyz.clear();
    np.xyz.reserve(np.points.size() * 3);
    for (const GA_Offset pt : np.points) {
      const UT_Vector3 p = gdp->getPos3(pt);
      np.xyz.push_back(p.x());
      np.xyz.push_back(p.y());
      np.xyz.push_back(p.z());
    }
    if (np.primCount != 1) // only a piece that is one sphere is a sphere
      np.sphereValid = false;
    np.xyz.insert(np.xyz.end(), np.samples.begin(), np.samples.end());
    np.samples = std::vector<float>();
    np.points = std::vector<GA_Offset>();
    np.hash = b3dop::hashFloats(np.xyz.data(), np.xyz.size());
    g.hash = b3dop::mixHash(g.hash, np.hash);
  }
}

// The fragment implementation of a packed primitive, or null if it is not one.
const GU_PackedFragment *asFragment(const GU_PrimPacked &prim) {
  return dynamic_cast<const GU_PackedFragment *>(prim.sharedImplementation());
}

} // namespace

// Pieces by the primitive `name` attribute (unnamed prims share one; no prims:
// one piece).
void collectPieces(const GU_Detail *gdp, PieceGroup &out) {
  const GA_ROHandleS nameH(gdp, GA_ATTRIB_PRIMITIVE, "name");
  bool anyPrim = false;
  for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
    const GA_Primitive *prim = gdp->getPrimitive(*it);
    if (!prim)
      continue;
    anyPrim = true;
    std::string name;
    if (nameH.isValid())
      if (const char *n = nameH.get(*it))
        name = n;
    appendPrimPoints(gdp, *prim, out.piece(name));
  }
  if (!anyPrim) {
    NamedPiece &np = out.piece(std::string());
    for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
      np.points.push_back(*it);
  }
  finishPieces(gdp, out);
}

// One piece per set of connected primitives (b3dop::connectedSets).
void collectConnectedPieces(const GU_Detail *gdp, const GA_Range &prims,
                            PieceGroup &out) {
  std::vector<std::vector<GA_Offset>> primPoints;
  std::vector<const GA_Primitive *> primList;
  for (GA_Iterator it(prims); !it.atEnd(); ++it) {
    const GA_Primitive *prim = gdp->getPrimitive(*it);
    if (!prim)
      continue;
    primList.push_back(prim);
    primPoints.emplace_back();
    const GA_Size nv = prim->getVertexCount();
    for (GA_Size i = 0; i < nv; ++i)
      primPoints.back().push_back(prim->getPointOffset(i));
  }

  const std::vector<int> setOf = b3dop::connectedSets(primPoints);
  for (size_t i = 0; i < primPoints.size(); ++i) {
    NamedPiece &np = out.piece(std::to_string(setOf[i]));
    // Quadrics and nested packed primitives add their surface points (the
    // same as appendPrimPoints); their own point is a collision point only for
    // a quadric.
    if (GU_PrimPacked::isPackedPrimitive(*primList[i])) {
      ++np.primCount;
      appendNestedPacked(gdp, static_cast<const GU_PrimPacked &>(*primList[i]),
                         np, 0);
      continue;
    }
    np.points.insert(np.points.end(), primPoints[i].begin(),
                     primPoints[i].end());
    ++np.primCount;
    appendQuadricSamples(gdp, *primList[i], np);
  }
  finishPieces(gdp, out);
}

namespace {

// A packed fragment shares the whole source detail but stands for the
// primitives in its range: those points, as one piece.
void collectFragmentPiece(const GU_Detail *gdp, const GA_Range &range,
                          const std::string &name, PieceGroup &out) {
  NamedPiece &np = out.piece(name);
  for (GA_Iterator it(range); !it.atEnd(); ++it)
    if (const GA_Primitive *prim = gdp->getPrimitive(*it))
      appendPrimPoints(gdp, *prim, np);
  finishPieces(gdp, out);
}

} // namespace

// What a source's pieces hash to, so a packed hull can be looked up without
// reading the source geometry. Non-empty pieces only, in piece order.
struct SourceSummary {
  uint64_t groupHash = 0;
  std::vector<uint64_t> pieceHashes;
};

namespace {

// Source summaries by the source detail's identity (unique id, P and primitive
// list data ids, counts, fragment, connected sets or not). Process-wide, so a
// rebuilt world skips reading sources whose hulls are still cached. Relies on
// whoever edits a source bumping its data ids, as Houdini's own caches do.
class SourceSummaryCache {
public:
  static SourceSummaryCache &instance() {
    static SourceSummaryCache c;
    return c;
  }
  // Null when none was recorded. The summary is shared, not copied, and looked
  // up under a shared lock: every point of a fresh world asks.
  std::shared_ptr<const SourceSummary> find(uint64_t key) {
    // The thread's last hit first. A key is made of the source's identity, so
    // its summary never changes, and the lock below is a cache line every
    // thread of a parallel scan would fight over for every point.
    thread_local uint64_t memoKey = 0;
    thread_local std::shared_ptr<const SourceSummary> memo;
    if (memo && memoKey == key)
      return memo;
    std::shared_lock<std::shared_mutex> lock(myMutex);
    const auto it = myMap.find(key);
    if (it == myMap.end())
      return nullptr;
    memoKey = key;
    memo = it->second;
    return memo;
  }
  void insert(uint64_t key, const PieceGroup &g) {
    auto sum = std::make_shared<SourceSummary>();
    sum->groupHash = g.hash;
    for (const NamedPiece &np : g.pieces)
      if (!np.xyz.empty())
        sum->pieceHashes.push_back(np.hash);
    std::unique_lock<std::shared_mutex> lock(myMutex);
    if (myMap.size() >= kMaxEntries) {
      // Half goes (in the map's order), so a scene past the cap keeps most of
      // what it just used instead of losing all of it.
      for (auto it = myMap.begin();
           it != myMap.end() && myMap.size() > kMaxEntries / 2;)
        it = myMap.erase(it);
    }
    myMap[key] = std::move(sum);
  }

private:
  static constexpr size_t kMaxEntries = 262144;
  std::shared_mutex myMutex;
  std::unordered_map<uint64_t, std::shared_ptr<const SourceSummary>> myMap;
};

// The identity SourceSummaryCache is keyed by: the geometry's, and the
// primitive `name` attribute the pieces are grouped by (editing it changes
// neither P nor the primitive list).
uint64_t sourceIdentity(const GU_Detail *gdp, GA_Offset fragmentFirst,
                        bool connected) {
  uint64_t h = geometryIdentity(gdp);
  const GA_Attribute *name = gdp->findPrimitiveAttribute("name");
  h = b3dop::mixHash(h,
                     name ? static_cast<uint64_t>(name->getDataId()) : ~0ull);
  h = b3dop::mixHash(
      h, static_cast<uint64_t>(static_cast<int64_t>(fragmentFirst)));
  return b3dop::mixHash(h, connected ? 1 : 2);
}

} // namespace

// The source's summary, if one was recorded for it (no geometry is read).
std::shared_ptr<const SourceSummary>
ShapeSources::summary(const GU_PrimPacked &prim, bool connected) {
  const GU_PackedFragment *frag = asFragment(prim);
  GU_DetailHandleAutoReadLock lock(sourceHandle(prim, frag));
  const GU_Detail *gdp = lock.getGdp();
  return gdp ? SourceSummaryCache::instance().find(
                   sourceIdentity(gdp, fragmentFirst(frag), connected))
             : nullptr;
}

// The primitive's pieces (by `name`, or connected sets), or null. Records the
// summary.
const PieceGroup *ShapeSources::innerPieces(const GU_PrimPacked &prim,
                                            bool connected) {
  const GU_PackedFragment *frag = asFragment(prim);
  GU_ConstDetailHandle handle = sourceHandle(prim, frag);
  GU_DetailHandleAutoReadLock lock(handle);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return nullptr;

  const GA_Offset firstOff = fragmentFirst(frag);
  InnerEntry *entry = nullptr;
  if (frag) {
    entry = &fragments.get(std::make_pair(gdp, firstOff));
    std::call_once(entry->piecesOnce, [&] {
      entry->handle = handle;
      collectFragmentPiece(gdp, frag->getPrimitiveRange(),
                           std::to_string(static_cast<int64_t>(firstOff)),
                           entry->pieces);
      SourceSummaryCache::instance().insert(
          sourceIdentity(gdp, firstOff, false), entry->pieces);
    });
  } else {
    entry = &inner.get(gdp);
    std::call_once(entry->piecesOnce, [&] {
      entry->handle = handle;
      collectPieces(gdp, entry->pieces);
      SourceSummaryCache::instance().insert(
          sourceIdentity(gdp, firstOff, false), entry->pieces);
    });
  }

  if (!connected)
    return anyPieceNonEmpty(entry->pieces) ? &entry->pieces : nullptr;

  std::call_once(entry->connectedOnce, [&] {
    if (frag)
      collectConnectedPieces(gdp, frag->getPrimitiveRange(), entry->connected);
    else {
      collectConnectedPieces(gdp, gdp->getPrimitiveRange(), entry->connected);
      if (entry->connected.pieces.empty()) // points only, no primitives
        collectPieces(gdp, entry->connected);
    }
    SourceSummaryCache::instance().insert(sourceIdentity(gdp, firstOff, true),
                                          entry->connected);
  });
  return entry->connected.pieces.empty() ? nullptr : &entry->connected;
}

namespace {

// The polygons of `prims` as triangles in Houdini's order (box3dTriangulate).
std::shared_ptr<const MeshPiece>
collectTriangles(const GU_Detail *gdp, const GA_Range &prims, bool fragment) {
  auto mesh = std::make_shared<MeshPiece>();
  box3dTriangulate(gdp, prims, fragment, false, mesh->xyz, mesh->tris);
  if (mesh->tris.empty())
    return nullptr;
  return mesh;
}

} // namespace

std::shared_ptr<const MeshPiece>
ShapeSources::innerTriangles(const GU_PrimPacked &prim) {
  const GU_PackedFragment *frag = asFragment(prim);
  GU_ConstDetailHandle handle = sourceHandle(prim, frag);
  GU_DetailHandleAutoReadLock lock(handle);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return nullptr;
  TriEntry &entry = triangles.get(std::make_pair(gdp, fragmentFirst(frag)));
  std::call_once(entry.once, [&] {
    entry.handle = handle;
    entry.mesh = collectTriangles(
        gdp, frag ? frag->getPrimitiveRange() : gdp->getPrimitiveRange(),
        frag != nullptr);
  });
  return entry.mesh;
}

// ---------------------------------------------------------------------------
// Building the Box3D shapes for one point
// ---------------------------------------------------------------------------

namespace {

// The volume a closed mesh encloses (the divergence theorem); an open one gives
// some value near it, never negative.
float meshVolume(const MeshPiece &m) {
  double v = 0.0;
  for (size_t t = 0; t + 2 < m.tris.size(); t += 3) {
    const float *a = &m.xyz[3 * static_cast<size_t>(m.tris[t])];
    const float *b = &m.xyz[3 * static_cast<size_t>(m.tris[t + 1])];
    const float *c = &m.xyz[3 * static_cast<size_t>(m.tris[t + 2])];
    v += static_cast<double>(a[0]) * (static_cast<double>(b[1]) * c[2] -
                                      static_cast<double>(b[2]) * c[1]) -
         static_cast<double>(a[1]) * (static_cast<double>(b[0]) * c[2] -
                                      static_cast<double>(b[2]) * c[0]) +
         static_cast<double>(a[2]) * (static_cast<double>(b[0]) * c[1] -
                                      static_cast<double>(b[1]) * c[0]);
  }
  return static_cast<float>(std::fabs(v) / 6.0);
}

// A static `concave` point's mesh: its packed polygons, body-local, with
// Box3D's winding. Null without polygons.
std::shared_ptr<const MeshPiece> staticMeshPiece(ShapeSources &src,
                                                 const PackedInfo &pk) {
  const std::shared_ptr<const MeshPiece> raw = src.innerTriangles(*pk.prim);
  if (!raw)
    return nullptr;
  auto mesh = std::make_shared<MeshPiece>();
  b3dop::packedPointsToBodyLocal(raw->xyz.data(),
                                 static_cast<int>(raw->xyz.size() / 3), pk.m4,
                                 pk.pivot, mesh->xyz);
  mesh->tris = raw->tris;
  // Houdini's front faces wind clockwise, Box3D's counter-clockwise (meshes are
  // one-sided), as for Static Object meshes. A mirrored piece's points are
  // mirrored too (packedPointsToBodyLocal), which flips the winding again.
  double m3[9];
  b3dop::upper3x3(pk.m4, m3);
  const bool mirrored = b3dop::matrixDeterminant(m3) < 0.0;
  if (!mirrored)
    for (size_t t = 0; t + 2 < mesh->tris.size(); t += 3)
      std::swap(mesh->tris[t + 1], mesh->tris[t + 2]);
  return mesh;
}

const char *const kHullFallbackNote =
    "a piece could not be hulled (flat, < 4 points or too complex); used its "
    "bounding box";

// A hull of body-local points, cached under `key` (by content when 0), else a
// box fitted to the points. fill(pts) produces the points; with a key it is
// only called when the hull is not cached yet. `budgetGroup`: hulls of one
// source share their vertex budget (Box3DWorld). False if there are no usable
// points.
template <class Fill>
bool pieceFromPoints(b3dop::Box3DWorld &world, uint64_t key,
                     uint64_t budgetGroup, const Fill &fill,
                     std::vector<float> &pts, Piece &out, const char *&note) {
  float vol = 0.0f;
  b3dop::HullId h = key ? world.findHull(key, &vol) : 0;
  if (!h) {
    fill(pts);
    const int n = static_cast<int>(pts.size() / 3);
    h = key ? world.createHullWithKey(key, pts.data(), n, &vol, budgetGroup)
            : world.createHull(pts.data(), n, &vol);
    if (!h) {
      // Fewer than 4 points, flat, or too complex to hull: fall back to the
      // bounds.
      if (!b3dop::fitShapeToPoints(b3dop::ShapeType::Box, pts.data(), n,
                                   out.spec))
        return false;
      // A flat piece (a Circle as polygons, a grid) gets some thickness, as
      // the Bullet Solver's flat hull has.
      float widest = 0.0f;
      for (const float half : out.spec.halfExtents)
        widest = std::max(widest, half);
      if (widest > b3dop::kFlatHalfThickness)
        for (float &half : out.spec.halfExtents)
          if (half < 0.01f * widest)
            half = b3dop::kFlatHalfThickness;
      out.isHull = false;
      out.volume = b3dop::shapeVolume(out.spec);
      note = kHullFallbackNote;
      return true;
    }
  }
  out.isHull = true;
  out.hull = h;
  out.volume = vol;
  return true;
}

// A Box3D sphere for a piece that is one Sphere primitive (sphereValid): its
// centre and a point one radius from it, carried into body space by
// toBody(sourcePoint, bodyPoint), which returns false on failure.
template <class ToBody>
bool spherePiece(const NamedPiece &np, const ToBody &toBody, Piece &out) {
  const double c[3] = {np.sphereCentre[0], np.sphereCentre[1],
                       np.sphereCentre[2]};
  const double e[3] = {c[0] + np.sphereRadius, c[1], c[2]};
  double oc[3], oe[3];
  if (!toBody(c, oc) || !toBody(e, oe))
    return false;
  out.spec.type = b3dop::ShapeType::Sphere;
  for (int k = 0; k < 3; ++k)
    out.spec.center[k] = static_cast<float>(oc[k]);
  out.spec.radius = b3dop::clampExtent(static_cast<float>(std::sqrt(
      (oe[0] - oc[0]) * (oe[0] - oc[0]) + (oe[1] - oc[1]) * (oe[1] - oc[1]) +
      (oe[2] - oc[2]) * (oe[2] - oc[2]))));
  out.volume = b3dop::shapeVolume(out.spec);
  return true;
}

} // namespace

// A point's shapes (docs/agents/bodies.md, "Pieces and packed geometry"):
//  box / sphere / capsule: fitted to the packed geometry with autofit, else
//                          pscale * scale; with autofit off, plus the
//                          bullet_prim* / radius / length overrides
//  convex:  one hull of the packed geometry (hullPerConnected: one per
//           connected set)
//  concave: one hull per `name` piece (hullPerConnected: one per connected
//           set)
// Convex / concave without packed geometry fall back to a box.
BuiltShapes buildShapes(b3dop::Box3DWorld &world, ShapeSources &src,
                        b3dop::ShapeType type, bool hullPerConnected,
                        float pscale, const float scale[3],
                        const PackedInfo &pk, bool autofit,
                        const b3dop::PrimOverrides &ov, bool staticMesh,
                        const b3dop::GroundSize &ground) {
  using namespace b3dop;
  BuiltShapes out;
  auto add = [&](const Piece &pc) {
    out.totalVolume += pc.volume;
    out.pieces.push_back(pc);
  };

  // `plane`: a ground box (groundSpec, the size from Box3D Data), only on a
  // static body. Otherwise a box (fitted or pscale sized) with a note.
  if (type == ShapeType::Plane) {
    if (staticMesh) {
      Piece pc;
      pc.spec = groundSpec(ground);
      add(pc);
      out.mesh = true; // static-only shapes
      return out;
    }
    out.note = "a plane needs a static piece; used a box";
    type = ShapeType::Box;
  }

  // Static concave: the exact polygons as one Box3D triangle mesh (meshes only
  // collide as static). No polygons in it: hulls as for a moving body.
  if (staticMesh && type == ShapeType::Concave && pk.prim) {
    if (std::shared_ptr<const MeshPiece> mesh = staticMeshPiece(src, pk)) {
      Piece pc;
      pc.volume = meshVolume(*mesh);
      pc.mesh = std::move(mesh);
      add(pc);
      out.mesh = true;
      return out;
    }
  }

  const bool wantGeometry =
      (type == ShapeType::Convex || type == ShapeType::Concave);
  const bool mergeAll = (type == ShapeType::Convex) && !hullPerConnected;
  const bool connected =
      (type == ShapeType::Convex || type == ShapeType::Concave) &&
      hullPerConnected;

  // Packed convex / concave hulls are keyed by (source, pivot, scale), so
  // instances share them and the points are built only on a miss. A degenerate
  // transform has no scale: by content.
  double S[3];
  const bool scaled =
      pk.prim && (wantGeometry || autofit) && packedScale(pk.m4, S);
  const bool keyed = scaled && wantGeometry;
  // AutoFit of a box / sphere / capsule / cylinder: the fit is keyed by source
  // too, so an instance of a known source is looked up without its points
  // being built.
  const bool fitKeyed = scaled && !wantGeometry;

  // A source seen before (in any world): its summary gives the keys, and when
  // every hull is still cached the source geometry is not read at all.
  if (keyed) {
    const std::shared_ptr<const SourceSummary> sum =
        ShapeSources::summary(*pk.prim, connected);
    if (sum && !sum->pieceHashes.empty()) {
      std::vector<Piece> found;
      auto lookup = [&](uint64_t sourceHash, uint32_t kind) {
        Piece pc;
        pc.hull = world.findHull(packedHullKey(sourceHash, pk.pivot, S, kind),
                                 &pc.volume);
        pc.isHull = pc.hull != 0;
        if (pc.isHull)
          found.push_back(pc);
        return pc.isHull;
      };
      bool all = true;
      if (mergeAll)
        all = lookup(sum->groupHash, 1);
      else
        for (const uint64_t h : sum->pieceHashes) {
          all = lookup(h, 2);
          if (!all)
            break;
        }
      if (all) {
        for (const Piece &pc : found)
          add(pc);
        return out;
      }
    }
  }
  if (fitKeyed) {
    const std::shared_ptr<const SourceSummary> sum =
        ShapeSources::summary(*pk.prim, false);
    if (sum && !sum->pieceHashes.empty()) {
      Piece pc;
      if (Box3DWorld::findFit(packedFitKey(sum->groupHash, pk.pivot, S, type),
                              pc.spec)) {
        pc.volume = shapeVolume(pc.spec);
        add(pc);
        return out;
      }
    }
  }

  // The packed geometry's non-empty pieces (none for a plain point; box /
  // sphere / capsule only need them to autofit). Connected sets for
  // hullPerConnected.
  std::vector<const NamedPiece *> used;
  const PieceGroup *group = (pk.prim && (wantGeometry || autofit))
                                ? src.innerPieces(*pk.prim, connected)
                                : nullptr;
  if (group)
    for (const NamedPiece &np : group->pieces)
      if (!np.xyz.empty())
        used.push_back(&np);

  // Body-local points of `from`, merged.
  std::vector<float> pts, local;
  auto toBodyLocal = [&](const std::vector<const NamedPiece *> &from,
                         std::vector<float> &dst) {
    dst.clear();
    for (const NamedPiece *np : from) {
      packedPointsToBodyLocal(np->xyz.data(),
                              static_cast<int>(np->xyz.size() / 3), pk.m4,
                              pk.pivot, local);
      dst.insert(dst.end(), local.begin(), local.end());
    }
  };

  if (wantGeometry && !used.empty()) {
    auto addHull = [&](uint64_t sourceHash, uint32_t kind,
                       const std::vector<const NamedPiece *> &from) {
      const uint64_t key =
          keyed ? packedHullKey(sourceHash, pk.pivot, S, kind) : 0;
      Piece pc;
      // Through the packed transform.
      const auto sphereToBody = [&](const double in[3], double out[3]) {
        const float f[3] = {static_cast<float>(in[0]),
                            static_cast<float>(in[1]),
                            static_cast<float>(in[2])};
        std::vector<float> o;
        packedPointsToBodyLocal(f, 1, pk.m4, pk.pivot, o);
        if (o.size() < 3)
          return false;
        for (size_t k = 0; k < 3; ++k)
          out[k] = static_cast<double>(o[k]);
        return true;
      };
      if (from.size() == 1 && from[0]->sphereValid &&
          spherePiece(*from[0], sphereToBody, pc)) {
        add(pc);
        return;
      }
      if (pieceFromPoints(
              world, key, b3dop::mixHash(sourceHash, kind),
              [&](std::vector<float> &dst) { toBodyLocal(from, dst); }, pts, pc,
              out.note))
        add(pc);
    };
    if (mergeAll)
      addHull(group->hash, 1, used); // one hull from everything
    else
      for (const NamedPiece *np : used)
        addHull(np->hash, 2, {np}); // one hull per piece
  } else if (!wantGeometry && !used.empty()) {
    // Box / sphere / capsule on a packed point with autofit: fit to the
    // geometry, turned with it.
    toBodyLocal(used, pts);
    Piece pc;
    if (Box3DWorld::fitShapeOriented(
            type, pts.data(), static_cast<int>(pts.size() / 3), pc.spec)) {
      if (fitKeyed)
        Box3DWorld::storeFit(packedFitKey(group->hash, pk.pivot, S, type),
                             pc.spec);
      pc.volume = shapeVolume(pc.spec);
      add(pc);
    }
  }

  // Nothing usable: pscale * scale.
  if (out.pieces.empty()) {
    if (wantGeometry)
      out.note = pk.prim
                     ? "packed geometry is empty; used a box"
                     : "convex / concave needs a packed primitive; used a box";
    Piece pc;
    pc.spec =
        makeShapeSpec(wantGeometry ? ShapeType::Box : type, pscale, scale);
    // The bullet_prim* sizes only with autofit off (Bullet's meaning). RBD
    // Packed Object puts default bullet_primS / bullet_radius on every point,
    // so applying them with autofit on would override pscale on plain points.
    if (!wantGeometry && !autofit)
      applyPrimOverrides(pc.spec, ov);
    pc.volume = shapeVolume(pc.spec);
    add(pc);
  }
  return out;
}

// A whole object's shapes, body-local: the geometry through its own transform
// `m` (object space; scale and shear are baked in), relative to `pivot`. Box /
// sphere / capsule fit the whole geometry, convex is one hull (or one per
// connected set), concave one hull per `name` piece (or per connected set).
BuiltShapes buildWholeObjectShapes(b3dop::Box3DWorld &world,
                                   const GU_Detail *gdp, const double m[16],
                                   const double pivot[3], b3dop::ShapeType type,
                                   bool hullPerConnected, bool staticMesh) {
  using b3dop::ShapeType;
  BuiltShapes out;
  if (type == ShapeType::None)
    return out;

  // Static concave: the object's polygons as one triangle mesh (meshes only
  // collide as static). No polygons: hulls as for a moving body.
  if (staticMesh && type == ShapeType::Concave) {
    auto mesh = std::make_shared<MeshPiece>();
    box3dTriangulate(gdp, gdp->getPrimitiveRange(), false, false, mesh->xyz,
                     mesh->tris);
    if (!mesh->tris.empty()) {
      for (size_t k = 0; k + 2 < mesh->xyz.size(); k += 3) {
        const double p[3] = {mesh->xyz[k], mesh->xyz[k + 1], mesh->xyz[k + 2]};
        double o[3];
        b3dop::xformPoint(m, p, o);
        for (int c = 0; c < 3; ++c)
          mesh->xyz[k + static_cast<size_t>(c)] =
              static_cast<float>(o[c] - pivot[c]);
      }
      // Houdini's front faces wind clockwise, Box3D's counter-clockwise; a
      // mirroring transform flips the winding again.
      double m3[9];
      b3dop::upper3x3(m, m3);
      if (b3dop::matrixDeterminant(m3) >= 0.0)
        for (size_t t = 0; t + 2 < mesh->tris.size(); t += 3)
          std::swap(mesh->tris[t + 1], mesh->tris[t + 2]);
      Piece pc;
      pc.volume = meshVolume(*mesh);
      pc.mesh = std::move(mesh);
      out.totalVolume += pc.volume;
      out.pieces.push_back(std::move(pc));
      out.mesh = true;
      return out;
    }
  }

  PieceGroup g;
  const bool connected =
      (type == ShapeType::Convex || type == ShapeType::Concave) &&
      hullPerConnected;
  if (connected)
    collectConnectedPieces(gdp, gdp->getPrimitiveRange(), g);
  if (g.pieces.empty())
    collectPieces(gdp, g);

  auto toBody = [&](const NamedPiece &np, std::vector<float> &dst) {
    for (size_t k = 0; k + 2 < np.xyz.size(); k += 3) {
      const double p[3] = {np.xyz[k], np.xyz[k + 1], np.xyz[k + 2]};
      double o[3];
      b3dop::xformPoint(m, p, o);
      for (int c = 0; c < 3; ++c)
        dst.push_back(static_cast<float>(o[c] - pivot[c]));
    }
  };
  std::vector<float> pts;
  auto addHull = [&](const std::vector<const NamedPiece *> &from) {
    Piece pc;
    const auto sphereToBody = [&](const double in[3], double out[3]) {
      double o[3];
      b3dop::xformPoint(m, in, o);
      for (size_t k = 0; k < 3; ++k)
        out[k] = o[k] - pivot[k];
      return true;
    };
    if (from.size() == 1 && from[0]->sphereValid &&
        spherePiece(*from[0], sphereToBody, pc)) {
      out.totalVolume += pc.volume;
      out.pieces.push_back(pc);
      return;
    }
    if (pieceFromPoints(
            world, 0, 0,
            [&](std::vector<float> &dst) {
              dst.clear();
              for (const NamedPiece *np : from)
                toBody(*np, dst);
            },
            pts, pc, out.note)) {
      out.totalVolume += pc.volume;
      out.pieces.push_back(pc);
    }
  };

  std::vector<const NamedPiece *> all;
  for (const NamedPiece &np : g.pieces)
    if (!np.xyz.empty())
      all.push_back(&np);
  if (all.empty())
    return out;

  if (type == ShapeType::Concave || connected) {
    for (const NamedPiece *np : all)
      addHull({np});
  } else if (type == ShapeType::Convex)
    addHull(all);
  else {
    pts.clear();
    for (const NamedPiece *np : all)
      toBody(*np, pts);
    Piece pc;
    if (b3dop::Box3DWorld::fitShapeOriented(
            type, pts.data(), static_cast<int>(pts.size() / 3), pc.spec)) {
      pc.volume = b3dop::shapeVolume(pc.spec);
      out.totalVolume += pc.volume;
      out.pieces.push_back(pc);
    }
  }
  return out;
}

} // namespace box3dsolver
