// physics_shapes.cpp
//
// Physics cases: shapes, hulls, primitive overrides and body extent.
// The case list and main are in core_physics.cpp.
#include "KeyedWorld.h"
#include "PhysicsCases.h"
#include "TestUtil.h"
#include "core/Box3DShapes.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

// Row-vector rotation matrix for unit quaternion q (x, y, z, w); builds test
// matrices only.
static void quatToMatrix3(const float q[4], double m[9]) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  m[0] = 1 - 2 * (y * y + z * z);
  m[1] = 2 * (x * y + z * w);
  m[2] = 2 * (x * z - y * w);
  m[3] = 2 * (x * y - z * w);
  m[4] = 1 - 2 * (x * x + z * z);
  m[5] = 2 * (y * z + x * w);
  m[6] = 2 * (x * z + y * w);
  m[7] = 2 * (y * z - x * w);
  m[8] = 1 - 2 * (x * x + y * y);
}

// Convex shapes: names, packed-prim math, hulls, compounds.
void testConvexShapes() {
  using namespace b3dop;
  ShapeType t = ShapeType::Box;
  // every bullet_georep value Houdini defines
  CHECK(parseShapeType("convexhull", t) && t == ShapeType::Convex);
  CHECK(parseShapeType("concave", t) && t == ShapeType::Concave);
  CHECK(parseShapeType("box", t) && t == ShapeType::Box);
  CHECK(parseShapeType("capsule", t) && t == ShapeType::Capsule);
  CHECK(parseShapeType("compound", t) && t == ShapeType::Concave);
  CHECK(parseShapeType("sphere", t) && t == ShapeType::Sphere);
  CHECK(parseShapeType("none", t) && t == ShapeType::None);
  CHECK(!parseShapeType("Convexhull", t) &&
        !parseShapeType("CONCAVE", t)); // case-sensitive
  // the old `shape` attribute names are gone
  CHECK(!parseShapeType("convex", t) && !parseShapeType("hull", t));
  CHECK(!parseShapeType("cube", t) && !parseShapeType("ball", t));
  CHECK(!parseShapeType("proxy", t));
  // Houdini values Box3D cannot do are treated as unknown (caller uses its
  // default)
  t = ShapeType::Box;
  CHECK(parseShapeType("plane", t) && t == ShapeType::Plane);
  CHECK(parseShapeType("cylinder", t) && t == ShapeType::Cylinder);
  t = ShapeType::Box;

  // test fixture: S(2,3,4) * R(0.7 rad about Y), row-vector layout
  const float s45 = std::sin(0.5f * 0.7f), c45 = std::cos(0.5f * 0.7f);
  const float qa[4] = {0.0f, s45, 0.0f, c45};
  double rotM[9];
  quatToMatrix3(qa, rotM);
  double ms[9];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      ms[r * 3 + c] = (2.0 + r) * rotM[r * 3 + c];
  {
    double sc[3];
    CHECK(matrixRowLengths(ms, sc));
    CHECK(std::fabs(sc[0] - 2) < 1e-6 && std::fabs(sc[1] - 3) < 1e-6 &&
          std::fabs(sc[2] - 4) < 1e-6);
    const double zero9[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(!matrixRowLengths(zero9, sc) && sc[0] == 1.0);
  }

  // connected sets: prims 0-1-2 chain through shared points, 3 is alone, 4
  // joins 3
  {
    const std::vector<std::vector<int>> prims = {{10, 11}, {11, 12}, {12, 13},
                                                 {20, 21}, {21, 22}, {30}};
    const std::vector<int> sets = connectedSets(prims);
    CHECK(sets.size() == 6);
    CHECK(sets[0] == 0 && sets[1] == 0 && sets[2] == 0);
    CHECK(sets[3] == 1 && sets[4] == 1);
    CHECK(sets[5] == 2);
    // a late primitive that bridges two earlier sets merges them
    const std::vector<std::vector<int>> bridge = {{1, 2}, {3, 4}, {2, 3}};
    const std::vector<int> bs = connectedSets(bridge);
    CHECK(bs[0] == 0 && bs[1] == 0 && bs[2] == 0);
    CHECK(connectedSets(std::vector<std::vector<int>>()).empty());
  }

  // Packed prim: inner points around a pivot, M = S*R, translation t = P -
  // pivot*S*R. Body-local points must come out as (p - pivot) * S, rotation
  // removed.
  const double pivot[3] = {1.0, 2.0, 3.0};
  const double P[3] = {10.0, -4.0, 7.0};
  double m4[16] = {0};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      m4[r * 4 + c] = ms[r * 3 + c];
  for (int c = 0; c < 3; ++c)
    m4[3 * 4 + c] =
        P[c] - (pivot[0] * ms[0 * 3 + c] + pivot[1] * ms[1 * 3 + c] +
                pivot[2] * ms[2 * 3 + c]);
  m4[15] = 1;
  const std::vector<float> inner = {1, 2, 3, 2, 2, 3, 1, 4, 3, 1, 2, 6};
  std::vector<float> localPts;
  packedPointsToBodyLocal(inner.data(), 4, m4, pivot, localPts);
  CHECK(std::fabs(localPts[0]) < 1e-4f && std::fabs(localPts[1]) < 1e-4f &&
        std::fabs(localPts[2]) < 1e-4f); // the pivot is the origin
  CHECK(std::fabs(localPts[3] - 2.0f) < 1e-4f &&
        std::fabs(localPts[4]) < 1e-4f);          // +1 in x, S = 2
  CHECK(std::fabs(localPts[7] - 6.0f) < 1e-4f);   // +2 in y, S = 3
  CHECK(std::fabs(localPts[11] - 12.0f) < 1e-4f); // +3 in z, S = 4

  // Same source, 3 different rotations / positions (float noise in m4 and P):
  // the body-local points must come out byte-identical so the hull cache
  // hits.
  {
    KeyedWorld iw;
    CHECK(iw.create(WorldSettings()));
    std::vector<float> src; // an off-centre cube around the pivot
    for (int i = 0; i < 8; ++i) {
      src.push_back(static_cast<float>(pivot[0]) + ((i & 1) ? 0.37f : -0.61f));
      src.push_back(static_cast<float>(pivot[1]) + ((i & 2) ? 0.52f : -0.23f));
      src.push_back(static_cast<float>(pivot[2]) + ((i & 4) ? 0.91f : -0.44f));
    }
    HullId first = 0;
    for (int k = 0; k < 3; ++k) {
      const float ang = 0.9f + 1.7f * static_cast<float>(k);
      const float qk[4] = {0.3f * std::sin(0.5f * ang),
                           std::sin(0.5f * ang) * 0.8f,
                           0.1f * std::sin(0.5f * ang), std::cos(0.5f * ang)};
      float qn2[4];
      const float ln = std::sqrt(qk[0] * qk[0] + qk[1] * qk[1] + qk[2] * qk[2] +
                                 qk[3] * qk[3]);
      for (int c = 0; c < 4; ++c)
        qn2[c] = qk[c] / ln;
      double rk[9];
      quatToMatrix3(qn2, rk);
      const double Pk[3] = {3.1 * k + 0.7, -2.2 * k, 100.0 + 13.37 * k};
      const double sk = 1.5; // uniform scale
      double mk[16] = {0};
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          mk[r * 4 + c] = sk * rk[r * 3 + c];
      for (int c = 0; c < 3; ++c)
        mk[12 + c] =
            Pk[c] - (pivot[0] * mk[0 * 4 + c] + pivot[1] * mk[1 * 4 + c] +
                     pivot[2] * mk[2 * 4 + c]);
      mk[15] = 1;
      std::vector<float> lk;
      packedPointsToBodyLocal(src.data(), 8, mk, pivot, lk);
      const HullId hk = iw.createHull(lk.data(), 8);
      CHECK(hk != 0);
      if (k == 0)
        first = hk;
      CHECK(hk == first); // identical bytes -> one cache entry
    }
    CHECK(iw.hullCacheSize() == 1);
  }

  // Keyed hulls: Houdini says the instances share one source, so the source
  // is hashed once and an instance only adds pivot + scale. Same source at 3
  // rotations -> one key, built once, and the points are only needed on a
  // miss.
  {
    KeyedWorld kw;
    CHECK(kw.create(WorldSettings()));
    std::vector<float> src;
    for (int i = 0; i < 8; ++i) {
      src.push_back(static_cast<float>(pivot[0]) + ((i & 1) ? 0.37f : -0.61f));
      src.push_back(static_cast<float>(pivot[1]) + ((i & 2) ? 0.52f : -0.23f));
      src.push_back(static_cast<float>(pivot[2]) + ((i & 4) ? 0.91f : -0.44f));
    }
    const uint64_t sh = hashFloats(src.data(), src.size());
    auto instanceM4 = [&](int k, double sk, double *m) {
      const float ang = 0.9f + 1.7f * static_cast<float>(k);
      float q[4] = {0.3f * std::sin(0.5f * ang), std::sin(0.5f * ang) * 0.8f,
                    0.1f * std::sin(0.5f * ang), std::cos(0.5f * ang)};
      const float ln =
          std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      for (float &c : q)
        c /= ln;
      double r[9];
      quatToMatrix3(q, r);
      for (int i = 0; i < 16; ++i)
        m[i] = 0;
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
          m[a * 4 + b] = sk * r[a * 3 + b];
      m[15] = 1;
    };
    uint64_t key0 = 0;
    int builds = 0;
    for (int k = 0; k < 3; ++k) {
      double m[16];
      instanceM4(k, 1.5, m);
      double S[3];
      CHECK(packedScale(m, S));
      const uint64_t key = packedHullKey(sh, pivot, S, 1);
      if (k == 0)
        key0 = key;
      CHECK(key == key0); // rotation / noise does not change the key
      float vol = 0.0f;
      if (!kw.findHull(key, &vol)) {
        std::vector<float> loc;
        packedPointsToBodyLocal(src.data(), 8, m, pivot, loc);
        CHECK(kw.createHullWithKey(key, loc.data(), 8, &vol) == key);
        ++builds;
      }
      CHECK(vol > 0.0f);
    }
    CHECK(builds == 1 && kw.hullCacheSize() == 1);
    double m2[16];
    instanceM4(0, 2.0, m2);
    double S2[3];
    CHECK(packedScale(m2, S2));
    CHECK(packedHullKey(sh, pivot, S2, 1) != key0); // other scale: other hull
    double S1[3];
    double m1[16];
    instanceM4(0, 1.5, m1);
    CHECK(packedScale(m1, S1));
    CHECK(packedHullKey(sh, pivot, S1, 2) !=
          key0); // per-piece vs merged: other key
    CHECK(kw.createHullWithKey(0, src.data(), 8) == 0); // 0 is not a key
  }

  // fit a box / sphere to an off-centre point set
  ShapeSpec fit;
  const std::vector<float> off = {2, 0, 0, 4, 0, 0, 2, 2, 0, 2, 0, 6};
  CHECK(fitShapeToPoints(ShapeType::Box, off.data(), 4, fit));
  CHECK(fit.type == ShapeType::Box && std::fabs(fit.center[0] - 3) < 1e-6f &&
        std::fabs(fit.halfExtents[2] - 3) < 1e-6f);
  // Sphere: encloses every point (each is sqrt(11) from the centre 3 1 3).
  CHECK(fitShapeToPoints(ShapeType::Sphere, off.data(), 4, fit) &&
        std::fabs(fit.radius - std::sqrt(11.0f)) < 1e-5f);
  const std::vector<float> flat = {0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1};
  CHECK(fitShapeToPoints(ShapeType::Box, flat.data(), 4, fit) &&
        fit.halfExtents[1] > 0.0f); // clamped
  CHECK(!fitShapeToPoints(ShapeType::Box, off.data(), 0, fit));

  // Capsule: along Y through the bounds' centre (3 1 3), radius through the
  // points (each sqrt(10) from the axis), ending where the bounds end: the 2
  // tall bounds are shorter than the radius, so no straight section.
  CHECK(fitShapeToPoints(ShapeType::Capsule, off.data(), 4, fit));
  CHECK(fit.type == ShapeType::Capsule &&
        std::fabs(fit.radius - std::sqrt(10.0f)) < 1e-5f &&
        fit.halfLength == 0.0f && std::fabs(fit.center[0] - 3.0f) < 1e-6f &&
        std::fabs(fit.center[2] - 3.0f) < 1e-6f && fit.rot[3] == 1.0f);
  const std::vector<float> tall = {-1, 0, -1, 1, 0, -1, -1, 10, 1, 1, 0, 1};
  CHECK(fitShapeToPoints(ShapeType::Capsule, tall.data(), 4, fit));
  CHECK(std::fabs(fit.radius - std::sqrt(2.0f)) < 1e-5f &&
        std::fabs(fit.halfLength - (5.0f - std::sqrt(2.0f))) <
            1e-5f); // 10 tall, caps included
  // A ring of radius sqrt(2) at y 0 and tips at y +-3: 6 tall, caps
  // included.
  const std::vector<float> tips = {-1, 0, -1, 1, 0, -1, -1, 0,  1,
                                   1,  0, 1,  0, 3, 0,  0,  -3, 0};
  CHECK(fitShapeToPoints(ShapeType::Capsule, tips.data(), 6, fit));
  CHECK(std::fabs(fit.radius - std::sqrt(2.0f)) < 1e-5f &&
        std::fabs(fit.halfLength - (3.0f - std::sqrt(2.0f))) < 1e-4f);

  // AutoFit follows geometry turned inside the packed piece: a 0.6 x 0.4 x
  // 0.8 box turned 30 degrees about z gets that box back, turned 30 degrees.
  {
    const float c = std::cos(30.0f * kDegToRad),
                sn = std::sin(30.0f * kDegToRad);
    std::vector<float> box;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          const float x = 0.3f * static_cast<float>(sx),
                      y = 0.2f * static_cast<float>(sy),
                      z = 0.4f * static_cast<float>(sz);
          box.insert(box.end(), {x * c - y * sn + 1.0f, x * sn + y * c, z});
        }
    ShapeSpec o;
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Box, box.data(), 8, o));
    CHECK(o.type == ShapeType::Box &&
          std::fabs(o.halfExtents[0] - 0.3f) < 1e-4f &&
          std::fabs(o.halfExtents[1] - 0.2f) < 1e-4f &&
          std::fabs(o.halfExtents[2] - 0.4f) < 1e-4f);
    CHECK(std::fabs(o.center[0] - 1.0f) < 1e-4f &&
          std::fabs(o.center[1]) < 1e-4f);
    float ex[3];
    const float x1[3] = {1, 0, 0};
    quatRotate(o.rot, x1, ex);
    CHECK(std::fabs(ex[0] - c) < 1e-4f && std::fabs(ex[1] - sn) < 1e-4f);
    // The body-axes fit would be the larger bounding box.
    ShapeSpec aabb;
    CHECK(fitShapeToPoints(ShapeType::Box, box.data(), 8, aabb));
    CHECK(shapeVolume(o) < 0.7f * shapeVolume(aabb));
    const float firstHalf[3] = {o.halfExtents[0], o.halfExtents[1],
                                o.halfExtents[2]};
    const float firstCenter[3] = {o.center[0], o.center[1], o.center[2]};
    const float firstRot[4] = {o.rot[0], o.rot[1], o.rot[2], o.rot[3]};

    // Sphere: centred on the turned box, through its corners (Bullet: 0.538).
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Sphere, box.data(), 8, o));
    CHECK(std::fabs(o.radius - std::sqrt(0.29f)) < 1e-4f &&
          std::fabs(o.center[0] - 1.0f) < 1e-4f);
    // Capsule: along the box's own y, around its 0.6 x 0.8 cross-section
    // (Bullet: radius 0.5); the box is only 0.4 tall, so a sphere.
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Capsule, box.data(), 8, o));
    CHECK(std::fabs(o.radius - 0.5f) < 1e-4f && o.halfLength == 0.0f);
    float ax[3];
    const float up[3] = {0, 1, 0};
    quatRotate(o.rot, up, ax);
    CHECK(std::fabs(ax[0] + sn) < 1e-4f && std::fabs(ax[1] - c) < 1e-4f);

    // The fit cache: the same points fit again give the same box (a hit keyed
    // by shape type too, so the sphere and capsule above did not replace it).
    ShapeSpec again;
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Box, box.data(), 8, again));
    CHECK(again.type == ShapeType::Box);
    for (int i = 0; i < 3; ++i)
      CHECK(again.halfExtents[i] == firstHalf[i] &&
            again.center[i] == firstCenter[i]);
    for (int i = 0; i < 4; ++i)
      CHECK(again.rot[i] == firstRot[i]);
    // A non-finite point never finds a cached fit.
    std::vector<float> bad = box;
    bad[4] = std::nanf("");
    CHECK(!Box3DWorld::fitShapeOriented(ShapeType::Box, bad.data(), 8, again));

    // The same cache under a packed source's key: a miss, a store, then a hit
    // that is that fit; another type, pivot or scale is another key, and 0 is
    // not a key.
    const double fitPivot[3] = {0.1, 0.2, 0.3}, fitOther[3] = {0.1, 0.2, 0.4};
    const double fitScale[3] = {1.0, 2.0, 3.0}, fitScale2[3] = {1.0, 2.0, 3.5};
    const uint64_t fk = packedFitKey(4242, fitPivot, fitScale, ShapeType::Box);
    CHECK(fk != 0 &&
          fk != packedFitKey(4242, fitPivot, fitScale, ShapeType::Sphere) &&
          fk != packedFitKey(4242, fitOther, fitScale, ShapeType::Box) &&
          fk != packedFitKey(4242, fitPivot, fitScale2, ShapeType::Box) &&
          fk != packedFitKey(4243, fitPivot, fitScale, ShapeType::Box) &&
          fk != packedHullKey(4242, fitPivot, fitScale, 1) &&
          fk != packedHullKey(4242, fitPivot, fitScale, 2));
    ShapeSpec hit;
    CHECK(!Box3DWorld::findFit(fk, hit));
    Box3DWorld::storeFit(fk, again);
    CHECK(Box3DWorld::findFit(fk, hit) && hit.type == again.type);
    for (int i = 0; i < 3; ++i)
      CHECK(hit.halfExtents[i] == again.halfExtents[i] &&
            hit.center[i] == again.center[i]);
    CHECK(!Box3DWorld::findFit(
        packedFitKey(4242, fitPivot, fitScale, ShapeType::Sphere), hit));
    Box3DWorld::storeFit(0, again);
    CHECK(!Box3DWorld::findFit(0, hit));
  }
  // Geometry along the body axes keeps them exactly (no turn from float
  // noise).
  {
    ShapeSpec o;
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Box, off.data(), 4, o));
    CHECK(o.rot[0] == 0.0f && o.rot[1] == 0.0f && o.rot[2] == 0.0f &&
          o.rot[3] == 1.0f);
    std::vector<float> cube;
    for (int i = 0; i < 8; ++i)
      cube.insert(cube.end(), {(i & 1) ? 0.5f : -0.5f, (i & 2) ? 0.5f : -0.5f,
                               (i & 4) ? 0.5f : -0.5f});
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Box, cube.data(), 8, o));
    CHECK(o.rot[3] == 1.0f && std::fabs(o.halfExtents[0] - 0.5f) < 1e-6f);
    // Fewer than 4 points: the body axes.
    CHECK(Box3DWorld::fitShapeOriented(ShapeType::Box, off.data(), 3, o) &&
          o.rot[3] == 1.0f);
  }
  // quatFromAxes / canonicalFrame
  {
    float q[4];
    const float x[3] = {0, 1, 0}, y[3] = {-1, 0, 0}, z[3] = {0, 0, 1};
    quatFromAxes(x, y, z, q); // 90 degrees about z
    float r[3];
    const float x1[3] = {1, 0, 0};
    quatRotate(q, x1, r);
    CHECK(std::fabs(r[0]) < 1e-6f && std::fabs(r[1] - 1.0f) < 1e-6f);
    float f[3][3] = {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}}; // the axes, shuffled
    canonicalFrame(f);
    CHECK(f[0][0] == 1.0f && f[1][1] == 1.0f && f[2][2] == 1.0f &&
          f[0][1] == 0.0f);
  }

  // makeShapeSpec for the capsule, and its volume
  const float tallScale[3] = {1, 4, 2};
  const ShapeSpec cap = makeShapeSpec(ShapeType::Capsule, 1.0f, tallScale);
  CHECK(cap.type == ShapeType::Capsule &&
        std::fabs(cap.radius - 1.0f) < 1e-6f &&
        std::fabs(cap.halfLength - 1.0f) < 1e-6f);
  CHECK(std::fabs(shapeVolume(cap) -
                  (3.14159265f * 2.0f + (4.0f / 3.0f) * 3.14159265f)) < 1e-3f);
  const float oneS[3] = {1, 1, 1};
  CHECK(makeShapeSpec(ShapeType::Capsule, 1.0f, oneS).halfLength ==
        0.0f); // unit capsule is a sphere

  // ---- hulls ------------------------------------------------------------
  KeyedWorld hw;
  CHECK(hw.create(WorldSettings()));
  auto cube = [](float cx, float cy, float cz, float h) {
    std::vector<float> v;
    for (int i = 0; i < 8; ++i) {
      v.push_back(cx + ((i & 1) ? h : -h));
      v.push_back(cy + ((i & 2) ? h : -h));
      v.push_back(cz + ((i & 4) ? h : -h));
    }
    return v;
  };
  float vol = 0.0f;
  const std::vector<float> unitCube = cube(0, 0, 0, 0.5f);
  const HullId h1 = hw.createHull(unitCube.data(), 8, &vol);
  CHECK(h1 != 0 && std::fabs(vol - 1.0f) < 1e-3f);
  CHECK(hw.createHull(unitCube.data(), 8) == h1 &&
        hw.hullCacheSize() == 1);                // cached by content
  CHECK(hw.createHull(unitCube.data(), 3) == 0); // < 4 points
  const std::vector<float> flatQuad = {0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1};
  CHECK(hw.createHull(flatQuad.data(), 4) == 0); // flat: no volume
  std::vector<float> bad = unitCube;
  bad[4] = std::nanf("");
  CHECK(hw.createHull(bad.data(), 8) == 0); // non-finite
  // a dense cloud (500 points on a sphere) still gives a valid hull
  std::vector<float> cloud;
  for (int i = 0; i < 500; ++i) {
    const float fi = static_cast<float>(i);
    const float zz = 1.0f - 2.0f * (fi + 0.5f) / 500.0f,
                rr = std::sqrt(1.0f - zz * zz), a = 2.39996323f * fi;
    cloud.push_back(rr * std::cos(a));
    cloud.push_back(rr * std::sin(a));
    cloud.push_back(zz);
  }
  float cvol = 0.0f;
  CHECK(hw.createHull(cloud.data(), 500, &cvol) != 0 && cvol > 3.3f &&
        // unit sphere is 4.19; the hull is cut down to fit the limits
        cvol < 4.2f);
  hw.pruneHulls(); // nothing used them yet
  CHECK(hw.hullCacheSize() == 0);

  // Ground, then: (a) a hull whose pivot is NOT its centre, (b) a two-piece
  // dynamic compound, (c) 50 instances of one hull.
  BodySettings stat;
  stat.type = BodySettings::Type::Static;
  const double gp[3] = {0, -0.5, 0};
  const float gh[3] = {30, 0.5f, 30};
  CHECK(hw.addBody(1, stat, gp, qIdentity) && hw.addBoxShape(1, gh));

  // (a) cube centred at x = +2 from the body origin, dropped with the origin
  // at x = 0.
  const std::vector<float> offCube = cube(2, 0, 0, 0.5f);
  float offVol = 0.0f;
  const HullId ho = hw.createHull(offCube.data(), 8, &offVol);
  CHECK(ho != 0);
  BodySettings db;
  db.props.density = 4.0f;
  const double sp[3] = {0, 3, 0};
  CHECK(hw.addBody(10, db, sp, qIdentity));
  CHECK(hw.setBodyDensity(10, 40.0f / offVol)); // mass attribute 40 -> density
  CHECK(hw.addHullShape(10, ho));
  CHECK(std::fabs(hw.bodyMass(10) - 40.0f) < 0.05f);
  CHECK(hw.shapeCount(10) == 1);

  // (b) L shape from two cubes, as two pieces; total mass 10.
  const std::vector<float> pA = cube(0, 0, 0, 0.5f), pB = cube(1, 0, 0, 0.5f);
  float vA = 0, vB = 0;
  const HullId hA = hw.createHull(pA.data(), 8, &vA),
               hB = hw.createHull(pB.data(), 8, &vB);
  CHECK(hA != 0 && hB != 0 && hA != hB);
  const double lp[3] = {6, 3, 0};
  CHECK(hw.addBody(11, db, lp, qIdentity));
  CHECK(hw.setBodyDensity(11, 10.0f / (vA + vB)));
  CHECK(hw.addHullShape(11, hA) && hw.addHullShape(11, hB));
  CHECK(hw.shapeCount(11) == 2 && std::fabs(hw.bodyMass(11) - 10.0f) < 0.05f);

  // (c) instancing: 50 bodies, one hull build
  for (uint64_t i = 0; i < 50; ++i) {
    const double ip[3] = {-20.0 + static_cast<double>(i) * 0.8, 3, 8};
    CHECK(hw.addBody(100 + i, db, ip, qIdentity) &&
          hw.addHullShape(100 + i, hw.createHull(unitCube.data(), 8)));
  }
  // pruneHulls lets go of what this world holds; the shared cache keeps the
  // hulls, so asking again finds them without a build.
  hw.pruneHulls();
  CHECK(hw.hullCacheSize() == 0);
  {
    const int builds = Box3DWorld::hullBuildCount();
    CHECK(hw.createHull(unitCube.data(), 8) == h1 &&
          hw.createHull(pB.data(), 8) == hB);
    CHECK(Box3DWorld::hullBuildCount() == builds);
    hw.pruneHulls();
  }
  for (int i = 0; i < 400; ++i)
    hw.step(1.0f / 60.0f);

  double hp[3];
  float hq[4];
  CHECK(hw.getTransform(10, hp, hq));
  std::printf("offset hull  origin y=%.4f x=%.4f\n", hp[1], hp[0]);
  CHECK(std::fabs(hp[1] - 0.5) < 0.06); // cube centre is on the origin's height
  CHECK(hw.getTransform(11, hp, hq));
  std::printf("L compound   origin y=%.4f\n", hp[1]);
  CHECK(std::fabs(hp[1] - 0.5) < 0.06); // lies flat on the ground
  CHECK(hw.getTransform(120, hp, hq));
  CHECK(std::fabs(hp[1] - 0.5) < 0.06);

  // Removing bodies releases their hulls.
  for (uint64_t i = 0; i < 50; ++i)
    hw.removeBody(100 + i);
  hw.removeBody(10);
  hw.removeBody(11);
  hw.pruneHulls();
  CHECK(hw.hullCacheSize() == 0);

  // box / sphere with a centre offset (fitted to packed bounds)
  CHECK(hw.addBody(20, db, sp, qIdentity));
  ShapeSpec offBox;
  offBox.type = ShapeType::Box;
  offBox.center[0] = 3.0f;
  CHECK(hw.addShape(20, offBox));
  CHECK(hw.addBody(21, db, sp, qIdentity));
  ShapeSpec offSph;
  offSph.type = ShapeType::Sphere;
  offSph.center[1] = 2.0f;
  offSph.radius = 0.5f;
  CHECK(hw.addShape(21, offSph));
  std::printf("convex-shape checks done\n");
}

// Explicit primitive overrides (bullet_primS / primR / primT / radius /
// length).
void testPrimitiveOverrides() {
  using namespace b3dop;
  auto near3 = [](const float a[3], float x, float y, float z) {
    return std::fabs(a[0] - x) < 1e-5f && std::fabs(a[1] - y) < 1e-5f &&
           std::fabs(a[2] - z) < 1e-5f;
  };

  // quatRotate: 90 degrees about X turns Y into Z (the Euler -> quaternion
  // step is the HDK's now, so only the rotation itself is checked here).
  {
    const float h = 0.70710678f;
    const float qx90[4] = {h, 0, 0, h};
    const float up[3] = {0, 1, 0};
    float r[3];
    quatRotate(qx90, up, r);
    CHECK(near3(r, 0, 0, 1));
  }

  // applyPrimOverrides
  {
    const float one[3] = {1, 1, 1};
    PrimOverrides o;
    CHECK(!o.any());
    ShapeSpec box = makeShapeSpec(ShapeType::Box, 1.0f, one);
    o.hasSize = true;
    o.size[0] = 2;
    o.size[1] = 0.5f;
    o.size[2] = 1; // HALF extents
    o.hasPos = true;
    o.pos[0] = 3;
    applyPrimOverrides(box, o);
    CHECK(near3(box.halfExtents, 2, 0.5f, 1) && box.center[0] == 3.0f);
    CHECK(std::fabs(shapeVolume(box) - 8.0f * 2 * 0.5f * 1) < 1e-5f);

    ShapeSpec cap = makeShapeSpec(ShapeType::Capsule, 1.0f, one);
    PrimOverrides oc;
    oc.hasRadius = true;
    oc.radius = 0.25f;
    oc.hasLength = true;
    oc.length = 3.0f; // straight section
    applyPrimOverrides(cap, oc);
    CHECK(std::fabs(cap.radius - 0.25f) < 1e-6f &&
          std::fabs(cap.halfLength - 1.5f) < 1e-6f);

    // size does not touch a sphere, radius does; only what is given changes
    ShapeSpec sph = makeShapeSpec(ShapeType::Sphere, 1.0f, one);
    applyPrimOverrides(sph, o);
    CHECK(std::fabs(sph.radius - 0.5f) < 1e-6f && sph.center[0] == 3.0f);
    applyPrimOverrides(sph, oc);
    CHECK(std::fabs(sph.radius - 0.25f) < 1e-6f);

    // a rotation override is carried through; a non-finite one is ignored
    ShapeSpec rb = makeShapeSpec(ShapeType::Box, 1.0f, one);
    PrimOverrides orr;
    orr.hasRot = true;
    orr.rot[2] = 0.70710678f;
    orr.rot[3] = 0.70710678f;
    applyPrimOverrides(rb, orr);
    CHECK(std::fabs(rb.rot[2] - 0.70710678f) < 1e-6f &&
          std::fabs(rb.rot[3] - 0.70710678f) < 1e-6f);
    orr.rot[0] = NAN;
    ShapeSpec rb2 = makeShapeSpec(ShapeType::Box, 1.0f, one);
    applyPrimOverrides(rb2, orr);
    CHECK(rb2.rot[3] == 1.0f);

    // bad values are ignored / clamped
    ShapeSpec bad = makeShapeSpec(ShapeType::Box, 1.0f, one);
    PrimOverrides ob;
    ob.hasSize = true;
    ob.size[0] = NAN;
    ob.size[1] = 1;
    ob.size[2] = 1;
    applyPrimOverrides(bad, ob);
    CHECK(near3(bad.halfExtents, 0.5f, 0.5f, 0.5f));
    ob.size[0] = 0;
    applyPrimOverrides(bad, ob);
    CHECK(bad.halfExtents[0] > 0.0f);
    ShapeSpec flat = makeShapeSpec(ShapeType::Capsule, 1.0f, one);
    PrimOverrides ol;
    ol.hasLength = true;
    ol.length = -2.0f;
    applyPrimOverrides(flat, ol);
    CHECK(flat.halfLength == 0.0f);
  }

  // Rotated shapes under real physics, resting on a ground plane (top at y =
  // 0).
  {
    KeyedWorld rw;
    CHECK(rw.create(WorldSettings()));
    const float qid[4] = {0, 0, 0, 1};
    BodySettings st;
    st.type = BodySettings::Type::Static;
    BodySettings dyn;
    const double g[3] = {0, -0.5, 0};
    CHECK(rw.addBody(1, st, g, qid));
    const float gh[3] = {20, 0.5f, 20};
    CHECK(rw.addBoxShape(1, gh));

    // capsule radius 0.5, straight section 2, lying on its side (turned 90
    // about Z): rests at y = radius.
    const float rz90[4] = {0, 0, 0.70710678f,
                           0.70710678f}; // 90 degrees about Z
    ShapeSpec cap;
    cap.type = ShapeType::Capsule;
    cap.radius = 0.5f;
    cap.halfLength = 1.0f;
    for (int i = 0; i < 4; ++i)
      cap.rot[i] = rz90[i];
    const double cp[3] = {0, 3, 0};
    CHECK(rw.addBody(2, dyn, cp, qid));
    CHECK(rw.addShape(2, cap));

    // box 4 x 1 x 1 turned 90 about Z: stands 4 tall, rests at y = 2.
    ShapeSpec box;
    box.type = ShapeType::Box;
    box.halfExtents[0] = 2.0f;
    box.halfExtents[1] = 0.5f;
    box.halfExtents[2] = 0.5f;
    for (int i = 0; i < 4; ++i)
      box.rot[i] = rz90[i];
    const double bp[3] = {8, 4, 0};
    CHECK(rw.addBody(3, dyn, bp, qid));
    CHECK(rw.addShape(3, box));
    CHECK(rw.shapeCount(3) == 1);
    CHECK(std::fabs(rw.bodyMass(3) - shapeVolume(box)) <
          0.05f); // density 1: mass = volume

    // an unrotated box with the same offset, placed 3 to the side, keeps its
    // body origin
    ShapeSpec off;
    off.type = ShapeType::Box;
    off.center[0] = 3.0f;
    const double op[3] = {-8, 3, 0};
    CHECK(rw.addBody(4, dyn, op, qid));
    CHECK(rw.addShape(4, off));

    for (int i = 0; i < 420; ++i)
      rw.step(1.0f / 60.0f);
    double p[3];
    float q[4];
    CHECK(rw.getTransform(2, p, q));
    std::printf("rotated capsule rest y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.06);
    CHECK(rw.getTransform(3, p, q));
    std::printf("rotated box     rest y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 2.0) < 0.08);
  }

  // A rotation that is not a valid quaternion (NaN, infinity) on a box, capsule
  // or cylinder is taken as no rotation: no Box3D assert, and a normal body.
  {
    const int before = Box3DWorld::assertCount();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float badRots[3][4] = {{nan, 0.0f, 0.0f, 1.0f},
                                 {0.0f, 0.0f, 0.0f, nan},
                                 {inf, 0.0f, 0.0f, 0.0f}};
    for (const ShapeType type :
         {ShapeType::Box, ShapeType::Capsule, ShapeType::Cylinder})
      for (const auto &rot : badRots) {
        KeyedWorld bw;
        CHECK(bw.create(WorldSettings()));
        BodySettings bs;
        ShapeSpec spec;
        spec.type = type;
        spec.radius = 0.25f;
        spec.halfLength = 0.25f;
        std::copy(rot, rot + 4, spec.rot);
        const float qid[4] = {0, 0, 0, 1};
        // A static slab under it, top at y = 0.
        BodySettings ground;
        ground.type = BodySettings::Type::Static;
        ShapeSpec slab;
        slab.halfExtents[0] = slab.halfExtents[2] = 10.0f;
        slab.halfExtents[1] = 0.5f;
        const double gat[3] = {0, -0.5, 0};
        const BodyHandle g = bw.createBody(ground, gat, qid);
        CHECK(g != 0 && bw.createShape(g, ground, slab));
        const double at[3] = {0, 1, 0};
        const BodyHandle h = bw.createBody(bs, at, qid);
        CHECK(h != 0 && bw.createShape(h, bs, spec));
        for (int i = 0; i < 120; ++i)
          bw.step(1.0f / 60.0f);
        double p[3];
        float q[4];
        CHECK(bw.getTransformByHandle(h, p, q));
        CHECK(std::isfinite(bw.massByHandle(h)) && bw.massByHandle(h) > 0.0f);
        // It lands on the slab instead of falling through or vanishing.
        CHECK(std::isfinite(p[1]) && p[1] > 0.2 && p[1] < 0.8);
      }
    CHECK(Box3DWorld::assertCount() == before);
  }
  std::printf("primitive-override checks done\n");
}

// The shared hull cache: across worlds, budget hints, eviction.
void testHullCache() {
  using namespace b3dop;
  std::vector<float> sphere; // dense and round: the full vertex budget fails
  for (int i = 0; i < 2000; ++i) {
    const float fi = static_cast<float>(i);
    const float zz = 1.0f - 2.0f * (fi + 0.5f) / 2000.0f,
                rr = std::sqrt(1.0f - zz * zz), a = 2.39996323f * fi;
    sphere.push_back(rr * std::cos(a));
    sphere.push_back(rr * std::sin(a));
    sphere.push_back(zz);
  }
  HullId first = 0;
  {
    KeyedWorld a;
    CHECK(a.create(WorldSettings()));
    const int builds = Box3DWorld::hullBuildCount();
    first = a.createHull(sphere.data(), 2000);
    CHECK(first != 0 && Box3DWorld::hullBuildCount() == builds + 1);
  } // world gone
  {
    KeyedWorld b; // a rebuilt world finds it without building
    CHECK(b.create(WorldSettings()));
    const int builds = Box3DWorld::hullBuildCount();
    float vol = 0.0f;
    CHECK(b.findHull(first, &vol) == first && vol > 3.0f);
    CHECK(b.createHull(sphere.data(), 2000) == first &&
          Box3DWorld::hullBuildCount() == builds);
    const double p[3] = {0, 2, 0};
    const float q[4] = {0, 0, 0, 1};
    CHECK(b.addBody(1, BodySettings(), p, q) && b.addHullShape(1, first));

    // Budget hint: the first hull of a group tries the full budget and fails,
    // the next one of the group starts at the budget that worked.
    const uint64_t group = 0x5eed;
    int att = Box3DWorld::hullBuildAttempts();
    CHECK(b.createHullWithKey(0x1001, sphere.data(), 2000, nullptr, group) !=
          0);
    const int firstAttempts = Box3DWorld::hullBuildAttempts() - att;
    std::vector<float> scaled = sphere;
    for (float &v : scaled)
      v *= 2.0f;
    att = Box3DWorld::hullBuildAttempts();
    CHECK(b.createHullWithKey(0x1002, scaled.data(), 2000, nullptr, group) !=
          0);
    CHECK(firstAttempts >= 2 && Box3DWorld::hullBuildAttempts() - att == 1);

    // Eviction: past the budget, least recently used entries go; held hulls
    // stay usable.
    const size_t oldBudget = static_cast<size_t>(256) << 20;
    Box3DWorld::setSharedHullBudget(1);
    CHECK(Box3DWorld::sharedHullBytes() <= 1);
    const double p2[3] = {3, 2, 0};
    CHECK(b.addBody(2, BodySettings(), p2, q) &&
          b.addHullShape(2, first)); // still held by b
    b.pruneHulls();
    CHECK(!b.findHull(first)); // evicted and no longer held
    Box3DWorld::setSharedHullBudget(oldBudget);
    for (int i = 0; i < 10; ++i)
      b.step(1.0f / 60.0f);
  }
  std::printf("shared hull cache checks done\n");
}

// The centroid of a convex hull of points: a box, a pyramid, two boxes apart.
void testHullCentroid() {
  using b3dop::Box3DWorld;
  auto box = [](std::vector<float> &v, float cx, float cy, float cz, float sx,
                float sy, float sz) {
    for (int dx = -1; dx <= 1; dx += 2)
      for (int dy = -1; dy <= 1; dy += 2)
        for (int dz = -1; dz <= 1; dz += 2) {
          v.push_back(cx + 0.5f * sx * static_cast<float>(dx));
          v.push_back(cy + 0.5f * sy * static_cast<float>(dy));
          v.push_back(cz + 0.5f * sz * static_cast<float>(dz));
        }
  };
  double c[3];
  {
    std::vector<float> v;
    box(v, 1, 2, 3, 1, 2, 3);
    CHECK(Box3DWorld::hullCentroid(v.data(), 8, c));
    CHECK(std::fabs(c[0] - 1) < 1e-4 && std::fabs(c[1] - 2) < 1e-4 &&
          std::fabs(c[2] - 3) < 1e-4);
  }
  {
    // Square pyramid, base y 0 (half side 1), apex y 2: centroid a quarter of
    // the height up. The mean of the points is not.
    const float v[] = {-1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0, 1, 0, 2, 0};
    CHECK(Box3DWorld::hullCentroid(v, 5, c));
    CHECK(std::fabs(c[0]) < 1e-4 && std::fabs(c[1] - 0.5) < 1e-4 &&
          std::fabs(c[2]) < 1e-4);
  }
  {
    // An L of two boxes: the hull's centroid, not the boxes' (0.4, 0.6).
    std::vector<float> v;
    box(v, 0, 0, 0, 3, 1, 1);
    box(v, 1, 1.5f, 0, 1, 2, 1);
    CHECK(Box3DWorld::hullCentroid(v.data(), 16, c));
    CHECK(std::fabs(c[0] - 10.0 / 42.0) < 2e-3 &&
          std::fabs(c[1] - 32.0 / 42.0) < 2e-3 && std::fabs(c[2]) < 1e-4);
  }
  {
    // Refused: too few points, flat input, non-finite input.
    const float few[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    CHECK(!Box3DWorld::hullCentroid(few, 3, c));
    const float flat[] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    CHECK(!Box3DWorld::hullCentroid(flat, 4, c));
    std::vector<float> v;
    box(v, 0, 0, 0, 1, 1, 1);
    v[4] = std::nanf("");
    CHECK(!Box3DWorld::hullCentroid(v.data(), 8, c));
  }
}

// The size of a body's bounding box (what a Force DOP sampling a circle or
// sphere measures).
void testBodyExtent() {
  b3dop::KeyedWorld w;
  CHECK(w.create(b3dop::WorldSettings()));
  const double pos[3] = {0, 3.0, 0};
  b3dop::BodySettings dyn;
  float e[3] = {-1, -1, -1};

  // No shapes yet: nothing to measure.
  const b3dop::BodyHandle empty = w.createBody(dyn, pos, qIdentity);
  CHECK(!w.extentByHandle(empty, e));
  CHECK(!w.extentByHandle(0, e));

  // A 1 x 2 x 4 box: its bounding box is the same size.
  CHECK(w.addBody(1, dyn, pos, qIdentity));
  const float half[3] = {0.5f, 1.0f, 2.0f};
  CHECK(w.addBoxShape(1, half));
  CHECK(w.extentByHandle(w.bodyHandle(1), e));
  CHECK(std::fabs(e[0] - 1.0f) < 0.05f && std::fabs(e[1] - 2.0f) < 0.05f &&
        std::fabs(e[2] - 4.0f) < 0.05f);

  // A sphere of radius 0.5 is 1 across in every direction.
  CHECK(w.addBody(2, dyn, pos, qIdentity));
  CHECK(w.addSphereShape(2, 0.5f));
  CHECK(w.extentByHandle(w.bodyHandle(2), e));
  CHECK(std::fabs(e[0] - 1.0f) < 0.05f && std::fabs(e[1] - 1.0f) < 0.05f &&
        std::fabs(e[2] - 1.0f) < 0.05f);
}

// Mirrored packed transforms. M = diag(1, 1, -1) * S * R (the sync reads the
// rotation and scale from the matrix with its third row negated): the hull
// points come out mirrored so that body rotation R puts them where M does, the
// hull key differs from the unmirrored piece's, and keepMirror puts back the
// flip that a matrix composed from a rotation and a scale does not have.
void testMirroredPacked() {
  using namespace b3dop;
  const double S[3] = {2.0, 3.0, 4.0};
  auto rotationRows = [](double degrees, double rows[3][3]) {
    const double a = degrees * 3.14159265358979323846 / 180.0;
    const double c = std::cos(a), s = std::sin(a);
    const double r[3][3] = {{c, 0, -s}, {0, 1, 0}, {s, 0, c}}; // about Y
    std::memcpy(rows, r, sizeof r);
  };
  // S * R as a 3x3 (row scale), optionally mirrored (third row negated).
  auto compose = [&](double degrees, bool mirror, double m3[9]) {
    double rows[3][3];
    rotationRows(degrees, rows);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        m3[r * 3 + c] = S[r] * rows[r][c] * ((mirror && r == 2) ? -1.0 : 1.0);
  };
  auto toM4 = [](const double m3[9], double m4[16]) {
    std::memset(m4, 0, 16 * sizeof(double));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        m4[r * 4 + c] = m3[r * 3 + c];
    m4[15] = 1.0;
  };

  double mirrored3[9], plain3[9], mirrored4[16], plain4[16];
  compose(30.0, true, mirrored3);
  compose(30.0, false, plain3);
  toM4(mirrored3, mirrored4);
  toM4(plain3, plain4);
  CHECK(matrixDeterminant(mirrored3) < 0.0);
  CHECK(matrixDeterminant(plain3) > 0.0);

  double Sm[3], Sp[3];
  CHECK(packedScale(mirrored4, Sm) && packedScale(plain4, Sp));
  CHECK(Sm[0] > 0.0 && Sm[1] > 0.0 && Sm[2] < 0.0); // the local z flip
  CHECK(Sp[0] > 0.0 && Sp[1] > 0.0 && Sp[2] > 0.0);
  CHECK(std::fabs(std::fabs(Sm[2]) - Sp[2]) < 1e-9);
  const double pivot[3] = {0, 0, 0};
  CHECK(packedHullKey(77, pivot, Sm, 1) != packedHullKey(77, pivot, Sp, 1));

  // The body-local points turned by the split rotation land where the packed
  // matrix puts the source points (p * M).
  const std::vector<float> src = {1, 2, 3, -2, 0.5f, 1, 0.25f, -1, 2};
  std::vector<float> local;
  packedPointsToBodyLocal(src.data(), 3, mirrored4, pivot, local);
  double rows[3][3];
  rotationRows(30.0, rows);
  for (int i = 0; i < 3; ++i) {
    for (int c = 0; c < 3; ++c) {
      double viaBody = 0.0, viaMatrix = 0.0;
      for (int r = 0; r < 3; ++r) {
        viaBody += static_cast<double>(local[static_cast<size_t>(3 * i + r)]) *
                   rows[r][c];
        viaMatrix += static_cast<double>(src[static_cast<size_t>(3 * i + r)]) *
                     mirrored3[r * 3 + c];
      }
      CHECK(std::fabs(viaBody - viaMatrix) < 1e-4);
    }
  }

  // Write-back: a matrix composed from a new rotation and the same scale has
  // no mirror. keepMirror gives it back only when the last read had one.
  double composed[9], kept[9], untouched[9];
  compose(75.0, false, composed);
  std::memcpy(kept, composed, sizeof composed);
  keepMirror(mirrored3, kept);
  CHECK(matrixDeterminant(kept) < 0.0);
  for (int c = 0; c < 3; ++c) {
    CHECK(kept[c] == composed[c] && kept[3 + c] == composed[3 + c]);
    CHECK(kept[6 + c] == -composed[6 + c]);
  }
  std::memcpy(untouched, composed, sizeof composed);
  keepMirror(plain3, untouched);
  for (int i = 0; i < 9; ++i)
    CHECK(untouched[i] == composed[i]);
  std::printf("mirrored packed checks done\n");
}

// The size estimate for the cache: an empty world is small, it grows with
// bodies, shapes and (after a step) contacts, and a destroyed world is 0.
void testEstimatedBytes() {
  b3dop::KeyedWorld w;
  CHECK(w.create(b3dop::WorldSettings()));
  const size_t empty = w.estimatedBytes();
  const float half[3] = {0.5f, 0.5f, 0.5f};
  const double groundPos[3] = {0, -0.5, 0};
  b3dop::BodySettings groundBody;
  groundBody.type = b3dop::BodySettings::Type::Static;
  CHECK(w.addBody(1, groundBody, groundPos, qIdentity));
  const float groundHalf[3] = {20, 0.5f, 20};
  CHECK(w.addBoxShape(1, groundHalf));
  const int count = 100;
  for (int i = 0; i < count; ++i) {
    const double p[3] = {(i % 10) * 1.0, 0.5 + (i - i % 10) / 10.0, 0};
    const auto id = static_cast<uint64_t>(100 + i);
    CHECK(w.addBody(id, b3dop::BodySettings(), p, qIdentity));
    CHECK(w.addBoxShape(id, half));
  }
  const size_t bodies = w.estimatedBytes();
  // At least a body and a shape each (Box3D's own figure, about 900 bytes
  // apiece here), and not wildly more.
  CHECK(bodies >= empty + static_cast<size_t>(count) * 500);
  CHECK(bodies <= empty + static_cast<size_t>(count) * 3000);
  for (int i = 0; i < 10; ++i)
    w.step(1.0f / 60.0f);
  CHECK(w.estimatedBytes() > bodies); // touching boxes: contacts
  w.destroy();
  CHECK(w.estimatedBytes() == 0);
  std::printf("estimated bytes checks done\n");
}
