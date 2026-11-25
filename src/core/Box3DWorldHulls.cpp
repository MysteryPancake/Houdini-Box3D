// Box3DWorldHulls.cpp
//
// Box3DWorld's hulls and fits: building a hull from a point set (shared by
// every world), finding and storing a fit, pruning, and the cache's statistics.
#include "Box3DWorldHulls.h"
#include "Box3DDebug.h"
#include "Box3DShapes.h"
#include "Box3DWorldInternal.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace b3dop {

namespace {

// fitShapeOriented: a turned frame must make the box at least this much
// smaller (relative volume) to replace the body axes.
constexpr double kTurnedFitGain = 1e-3;

} // namespace

HullId Box3DWorld::createHull(const float *xyz, int pointCount,
                              float *outVolume) {
  if (outVolume)
    *outVolume = 0.0f;
  if (!xyz || pointCount < 4)
    return 0;
  // Before hashing: a non-finite point must not find a cached hull.
  for (int i = 0; i < pointCount * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return 0;
  return createHullWithKey(hashPoints(xyz, pointCount), xyz, pointCount,
                           outVolume);
}

HullId Box3DWorld::findHull(HullId key, float *outVolume) const {
  if (outVolume)
    *outVolume = 0.0f;
  if (key == 0)
    return 0;
  HullRef ref = myImpl->heldHull(key);
  if (!ref) {
    ref = HullLibrary::instance().find(key);
    if (!ref)
      return 0;
    myImpl->hold(key, ref);
  }
  if (outVolume)
    *outVolume = ref->volume;
  return key;
}

HullId Box3DWorld::createHullWithKey(HullId key, const float *xyz,
                                     int pointCount, float *outVolume,
                                     uint64_t budgetGroup) {
  if (outVolume)
    *outVolume = 0.0f;
  if (key == 0 || !xyz || pointCount < 4)
    return 0;
  if (const HullId hit = findHull(key, outVolume))
    return hit;
  std::vector<b3Vec3> pts;
  if (!loadPoints(xyz, pointCount, pts))
    return 0;

  // The group's last working budget is tried first.
  HullLibrary &lib = HullLibrary::instance();
  if (lib.knownFailure(key))
    return 0;
  const int hint = lib.budgetHint(budgetGroup);
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, hint, budget, tries);
  lib.attempts.fetch_add(tries, std::memory_order_relaxed);
  if (!hull) {
    lib.noteFailure(key);
    return 0;
  }
  if (budget != hint)
    lib.setBudgetHint(budgetGroup, budget);
  lib.builds.fetch_add(1, std::memory_order_relaxed);

  if (!(hull->volume > 1e-9f) || !std::isfinite(hull->volume)) {
    b3DestroyHull(hull); // flat: no mass, no stable contacts
    lib.noteFailure(key);
    return 0;
  }
  const HullRef ref = lib.insert(key, hull);
  myImpl->hold(key, ref);
  if (outVolume)
    *outVolume = ref->volume;
  return key;
}

bool Box3DWorld::hullCentroid(const float *xyz, int count, double out[3]) {
  std::vector<b3Vec3> pts;
  if (!xyz || count < 4 || !loadPoints(xyz, count, pts))
    return false;
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, 0, budget, tries);
  if (!hull)
    return false;
  const b3MassData md = b3ComputeHullMass(hull, 1.0f);
  b3DestroyHull(hull);
  if (!(md.mass > 1e-9f) || !std::isfinite(md.center.x) ||
      !std::isfinite(md.center.y) || !std::isfinite(md.center.z))
    return false;
  out[0] = static_cast<double>(md.center.x);
  out[1] = static_cast<double>(md.center.y);
  out[2] = static_cast<double>(md.center.z);
  return true;
}

namespace {

// fitShapeOriented without the cache.
bool fitOrientedUncached(ShapeType type, const float *xyz, int count,
                         ShapeSpec &out) {
  std::vector<b3Vec3> pts;
  if (!loadPoints(xyz, count, pts))
    return false;
  // Only the hull's directions are used, so a reduced budget is fine (the
  // extents are measured over every point).
  int budget = 0, tries = 0;
  b3HullData *hull = createHullLadder(pts, 0, budget, tries);
  if (!hull)
    return fitShapeToPoints(type, xyz, count, out);

  const b3Vec3 *hp = b3GetHullPoints(hull);
  const b3Plane *planes = b3GetHullPlanes(hull);
  const b3HullFace *faces = b3GetHullFaces(hull);
  const b3HullHalfEdge *edges = b3GetHullEdges(hull);
  const int nv = hull->vertexCount;

  // The box volume along `axes`, over the hull's vertices.
  auto volumeAlong = [&](const float axes[3][3]) {
    double v = 1.0;
    for (int k = 0; k < 3; ++k) {
      float lo = 0.0f, hi = 0.0f;
      for (int i = 0; i < nv; ++i) {
        const float d =
            hp[i].x * axes[k][0] + hp[i].y * axes[k][1] + hp[i].z * axes[k][2];
        if (i == 0 || d < lo)
          lo = d;
        if (i == 0 || d > hi)
          hi = d;
      }
      v *= static_cast<double>(hi - lo);
    }
    return v;
  };

  float best[3][3];
  std::memcpy(best, identityFrame(), sizeof best);
  const double bodyAxesVolume = volumeAlong(best);
  double bestVolume = bodyAxesVolume;
  if (hp && planes && faces && edges)
    for (int f = 0; f < hull->faceCount; ++f) {
      const b3Vec3 n = planes[f].normal;
      const int first = faces[f].edge;
      int e = first;
      for (int guard = 0; guard < hull->edgeCount; ++guard) {
        const b3Vec3 a = hp[edges[e].origin];
        const b3Vec3 b = hp[edges[edges[e].next].origin];
        // The edge direction made perpendicular to the normal.
        float u[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
        const float dn = u[0] * n.x + u[1] * n.y + u[2] * n.z;
        u[0] -= dn * n.x;
        u[1] -= dn * n.y;
        u[2] -= dn * n.z;
        const float len = length3(u);
        if (len > 1e-6f) {
          float axes[3][3] = {
              {u[0] / len, u[1] / len, u[2] / len}, {n.x, n.y, n.z}, {0, 0, 0}};
          axes[2][0] = axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1];
          axes[2][1] = axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2];
          axes[2][2] = axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0];
          const double v = volumeAlong(axes);
          if (v < bestVolume) {
            bestVolume = v;
            std::memcpy(best, axes, sizeof best);
          }
        }
        e = edges[e].next;
        if (e == first)
          break;
      }
    }
  b3DestroyHull(hull);

  // Turn only for a clearly smaller box, so float noise never turns geometry
  // that lies along the body axes.
  if (!(bestVolume < bodyAxesVolume * (1.0 - kTurnedFitGain)))
    std::memcpy(best, identityFrame(), sizeof best);
  canonicalFrame(best);
  return fitShapeInFrame(type, xyz, count, best, out);
}

} // namespace

bool Box3DWorld::fitShapeOriented(ShapeType type, const float *xyz, int count,
                                  ShapeSpec &out) {
  if (!xyz || count <= 0)
    return false;
  if (count < 4)
    return fitShapeToPoints(type, xyz, count, out);
  // Before hashing: a non-finite point must not find a cached fit.
  for (int i = 0; i < count * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return false;
  const uint64_t key = mixHash(hashPoints(xyz, count),
                               static_cast<uint64_t>(static_cast<int>(type)));
  FitLibrary &lib = FitLibrary::instance();
  if (lib.find(key, out))
    return true;
  if (!fitOrientedUncached(type, xyz, count, out))
    return false;
  lib.insert(key, out);
  return true;
}

bool Box3DWorld::findFit(uint64_t key, ShapeSpec &out) {
  return key != 0 && FitLibrary::instance().find(key, out);
}

void Box3DWorld::storeFit(uint64_t key, const ShapeSpec &spec) {
  if (key != 0)
    FitLibrary::instance().insert(key, spec);
}

void Box3DWorld::pruneHulls() {
  std::unique_lock<std::shared_mutex> lock(myImpl->held_mutex);
  myImpl->held.clear();
}

// The world's own figure (its arrays, contacts, joints and the hulls in
// its database), plus the meshes and height fields we own: Box3D keeps
// pointers to those instead of copying them.
size_t Box3DWorld::estimatedBytes() const {
  if (!isValid())
    return 0;
  const b3Counters c = b3World_GetCounters(myImpl->worldId);
  size_t bytes = c.byteCount > 0 ? static_cast<size_t>(c.byteCount) : 0;
  for (const auto &kv : myImpl->sharedMeshes)
    bytes += static_cast<size_t>(kv.second.data->byteCount);
  for (const auto &kv : myImpl->bodyGeometry) {
    for (const b3HeightFieldData *hf : kv.second.heightFields)
      bytes += static_cast<size_t>(hf->byteCount);
  }
  return bytes;
}

int Box3DWorld::meshCount() const {
  return static_cast<int>(myImpl->sharedMeshes.size());
}

int Box3DWorld::hullCacheSize() const {
  std::shared_lock<std::shared_mutex> lock(myImpl->held_mutex);
  return static_cast<int>(myImpl->held.size());
}

size_t Box3DWorld::sharedHullBytes() { return HullLibrary::instance().bytes(); }

void Box3DWorld::setSharedHullBudget(size_t b) {
  HullLibrary::instance().setBudget(b);
}

int Box3DWorld::hullBuildCount() {
  return HullLibrary::instance().builds.load();
}

int Box3DWorld::hullBuildAttempts() {
  return HullLibrary::instance().attempts.load();
}

} // namespace b3dop
