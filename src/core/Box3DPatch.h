// Box3DPatch.h
//
// The contact patch of two bonded pieces: where their convex faces touch, as a
// polygon, with its area, centroid, normal and second moments. A Bond's stress
// follows from it: the section moduli about each in-plane axis for bending, the
// polar one for twisting. Houdini-free.
//
// Each piece gives its convex faces (world space, outward normals). The patch
// is the overlap of every face of one that faces a face of the other across a
// plane, within `tolerance` of touching (a gap, or an overlap, of that much),
// clipped against each other in the plane. Faces in the dominant plane are
// added together, so a piece made of several convex shapes still gives one
// patch.
#pragma once

#include "core/Box3DMath.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace b3dop {

// A convex polygon of a piece's surface: its corners in order (either way
// round) and its outward unit normal.
struct PatchFace {
  std::vector<std::array<double, 3>> corners;
  double normal[3] = {0, 0, 1};
};

struct ContactPatch {
  bool ok = false;
  double area = 0.0;
  double centroid[3] = {0, 0, 0};
  double normal[3] = {0, 0, 1}; // from the first piece to the second
  double u[3] = {1, 0, 0};      // in-plane axes, u x v = normal
  double v[3] = {0, 1, 0};
  double momentU = 0.0; // second moment of area about the u axis (m^4)
  double momentV = 0.0; // about the v axis
  double extentU = 0.0; // farthest corner from the centroid along u
  double extentV = 0.0;
  double extentR = 0.0; // farthest corner from the centroid, in the plane
};

namespace patch_detail {

using Vec = std::array<double, 3>;

struct Pt2 {
  double x, y;
};

// Sutherland-Hodgman: `subject` clipped against the convex polygon `clip`
// (counter-clockwise).
inline std::vector<Pt2> clipPolygon(const std::vector<Pt2> &subject,
                                    const std::vector<Pt2> &clip) {
  std::vector<Pt2> out = subject;
  const size_t n = clip.size();
  for (size_t i = 0; i < n && !out.empty(); ++i) {
    const Pt2 a = clip[i], b = clip[(i + 1) % n];
    const auto inside = [&](const Pt2 &p) {
      return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) >= 0.0;
    };
    const auto cut = [&](const Pt2 &p, const Pt2 &q) {
      const double dx = q.x - p.x, dy = q.y - p.y;
      const double ex = b.x - a.x, ey = b.y - a.y;
      const double den = dx * ey - dy * ex;
      const double t =
          den != 0.0 ? ((a.x - p.x) * ey - (a.y - p.y) * ex) / den : 0.0;
      return Pt2{p.x + t * dx, p.y + t * dy};
    };
    std::vector<Pt2> in;
    for (size_t k = 0; k < out.size(); ++k) {
      const Pt2 cur = out[k], prev = out[(k + out.size() - 1) % out.size()];
      if (inside(cur)) {
        if (!inside(prev))
          in.push_back(cut(prev, cur));
        in.push_back(cur);
      } else if (inside(prev))
        in.push_back(cut(prev, cur));
    }
    out.swap(in);
  }
  return out;
}

inline double signedArea(const std::vector<Pt2> &p) {
  double a = 0.0;
  for (size_t i = 0; i < p.size(); ++i) {
    const Pt2 &s = p[i], &t = p[(i + 1) % p.size()];
    a += s.x * t.y - t.x * s.y;
  }
  return 0.5 * a;
}

// The polygon's area, first and second moments about the origin, added to
// `m` (area, int x, int y, int x^2, int y^2), for a counter-clockwise polygon.
inline void addMoments(const std::vector<Pt2> &p, double m[5]) {
  for (size_t i = 0; i < p.size(); ++i) {
    const Pt2 &s = p[i], &t = p[(i + 1) % p.size()];
    const double c = s.x * t.y - t.x * s.y;
    m[0] += 0.5 * c;
    m[1] += c * (s.x + t.x) / 6.0;
    m[2] += c * (s.y + t.y) / 6.0;
    m[3] += c * (s.x * s.x + s.x * t.x + t.x * t.x) / 12.0;
    m[4] += c * (s.y * s.y + s.y * t.y + t.y * t.y) / 12.0;
  }
}

} // namespace patch_detail

// The patch between piece `a` and piece `b`. Not ok when no pair of faces face
// each other across a plane within `tolerance` and overlap.
inline ContactPatch contactPatchOf(const std::vector<PatchFace> &a,
                                   const std::vector<PatchFace> &b,
                                   double tolerance) {
  using namespace patch_detail;
  struct Pair {
    const PatchFace *fa, *fb;
    double area;
  };
  // The pairs that face each other across a plane, with their overlap.
  std::vector<Pair> pairs;
  const auto basisOf = [](const double n[3], Vec &u, Vec &v) {
    const Vec ref = std::fabs(n[0]) < 0.9 ? Vec{1, 0, 0} : Vec{0, 1, 0};
    u = {ref[1] * n[2] - ref[2] * n[1], ref[2] * n[0] - ref[0] * n[2],
         ref[0] * n[1] - ref[1] * n[0]};
    const double l = length3(u.data());
    for (double &x : u)
      x /= l;
    v = {n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0]};
  };
  const auto project = [&](const PatchFace &f, const Vec &o, const Vec &u,
                           const Vec &v) {
    std::vector<Pt2> pts;
    for (const auto &c : f.corners) {
      const double d[3] = {c[0] - o[0], c[1] - o[1], c[2] - o[2]};
      pts.push_back({dot3(d, u.data()), dot3(d, v.data())});
    }
    if (signedArea(pts) < 0.0)
      std::reverse(pts.begin(), pts.end());
    return pts;
  };
  ContactPatch best;
  double bestArea = 0.0;
  const PatchFace *bestA = nullptr;
  for (const PatchFace &fa : a)
    for (const PatchFace &fb : b) {
      if (dot3(fa.normal, fb.normal) > -0.985 || fa.corners.size() < 3 ||
          fb.corners.size() < 3)
        continue;
      // Across a plane: fb's corners within `tolerance` of fa's plane.
      const auto &p0 = fa.corners[0];
      bool near = true;
      for (const auto &c : fb.corners) {
        const double d[3] = {c[0] - p0[0], c[1] - p0[1], c[2] - p0[2]};
        if (std::fabs(dot3(d, fa.normal)) > tolerance)
          near = false;
      }
      if (!near)
        continue;
      Vec u, v;
      basisOf(fa.normal, u, v);
      const auto pa = project(fa, p0, u, v), pb = project(fb, p0, u, v);
      const double area = signedArea(clipPolygon(pa, pb));
      if (area > bestArea) {
        bestArea = area;
        bestA = &fa;
      }
    }
  if (!bestA)
    return best;
  // Every pair in the dominant plane, added together.
  const PatchFace &dom = *bestA;
  Vec u, v;
  basisOf(dom.normal, u, v);
  const Vec origin = dom.corners[0];
  double m[5] = {0, 0, 0, 0, 0};
  std::vector<std::vector<Pt2>> pieces;
  for (const PatchFace &fa : a) {
    if (dot3(fa.normal, dom.normal) < 0.985)
      continue;
    const double da[3] = {fa.corners[0][0] - origin[0],
                          fa.corners[0][1] - origin[1],
                          fa.corners[0][2] - origin[2]};
    if (std::fabs(dot3(da, dom.normal)) > tolerance)
      continue;
    for (const PatchFace &fb : b) {
      if (dot3(fb.normal, dom.normal) > -0.985)
        continue;
      const double db[3] = {fb.corners[0][0] - origin[0],
                            fb.corners[0][1] - origin[1],
                            fb.corners[0][2] - origin[2]};
      if (std::fabs(dot3(db, dom.normal)) > tolerance)
        continue;
      const auto clipped =
          clipPolygon(project(fa, origin, u, v), project(fb, origin, u, v));
      if (clipped.size() >= 3 && signedArea(clipped) > 0.0) {
        addMoments(clipped, m);
        pieces.push_back(clipped);
      }
    }
  }
  if (!(m[0] > 0.0))
    return best;
  const double cx = m[1] / m[0], cy = m[2] / m[0];
  best.ok = true;
  best.area = m[0];
  for (size_t i = 0; i < 3; ++i) {
    best.normal[i] = dom.normal[i];
    best.u[i] = u[i];
    best.v[i] = v[i];
    best.centroid[i] = origin[i] + cx * u[i] + cy * v[i];
  }
  // About the centroid: I_x = int y^2 dA (bending about u), I_y = int x^2 dA.
  best.momentU = m[4] - m[0] * cy * cy;
  best.momentV = m[3] - m[0] * cx * cx;
  for (const auto &poly : pieces)
    for (const Pt2 &p : poly) {
      best.extentU = std::max(best.extentU, std::fabs(p.x - cx));
      best.extentV = std::max(best.extentV, std::fabs(p.y - cy));
      best.extentR = std::max(best.extentR, std::hypot(p.x - cx, p.y - cy));
    }
  return best;
}

} // namespace b3dop
