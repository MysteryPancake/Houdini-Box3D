// Surface points for Houdini's quadric primitives (Box3DQuadrics.h): sizes, position, transform.
#include "core/Box3DQuadrics.h"
#include <algorithm>
#include <cstdio>

namespace {
struct Bounds {
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  explicit Bounds(const std::vector<float> &p) {
    for (size_t i = 0; i + 2 < p.size(); i += 3)
      for (int c = 0; c < 3; ++c) {
        lo[c] = std::min<double>(lo[c], p[i + c]);
        hi[c] = std::max<double>(hi[c], p[i + c]);
      }
  }
};
bool near(double a, double b, double tol = 1e-5) { return std::fabs(a - b) <= tol; }
int failures = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}
void diag(double s, double a[9]) {
  std::fill(a, a + 9, 0.0);
  a[0] = a[4] = a[8] = s;
}
} // namespace

int main() {
  using namespace b3dop;
  const double c[3] = {1, 2, 3};
  double a[9];

  check(quadricKindFromName("Sphere") == QuadricKind::Sphere && quadricKindFromName("Tube") == QuadricKind::Tube &&
            quadricKindFromName("Circle") == QuadricKind::Circle &&
            quadricKindFromName("Metaball") == QuadricKind::Metaball &&
            quadricKindFromName("METABALL") == QuadricKind::Metaball &&
            quadricKindFromName("Sphere2") == QuadricKind::None && quadricKindFromName("Poly") == QuadricKind::None && quadricKindFromName(nullptr) == QuadricKind::None,
        "kind from name");

  { // a radius 0.5 sphere: bounds are exactly the radius at the poles, and no point is outside it
    diag(0.5, a);
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Sphere, a, c, p);
    const Bounds b(p);
    check(near(b.lo[1], 2 - 0.5) && near(b.hi[1], 2 + 0.5), "sphere height is its diameter");
    check(b.hi[0] - b.lo[0] > 0.98 && b.hi[0] - b.lo[0] <= 1.0 + 1e-5, "sphere width is about its diameter");
    for (size_t i = 0; i + 2 < p.size(); i += 3) {
      const double d = std::sqrt(std::pow(p[i] - c[0], 2) + std::pow(p[i + 1] - c[1], 2) + std::pow(p[i + 2] - c[2], 2));
      check(d <= 0.5 + 1e-5, "sphere point inside the radius");
    }
  }
  { // an ellipsoid: the matrix scales each axis
    std::fill(a, a + 9, 0.0);
    a[0] = 2.0, a[4] = 0.5, a[8] = 1.0;
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Sphere, a, c, p);
    const Bounds b(p);
    check(near(b.hi[0] - c[0], 2.0) || b.hi[0] - c[0] > 1.9, "ellipsoid x extent");
    check(near(b.hi[1] - c[1], 0.5) && near(c[1] - b.lo[1], 0.5), "ellipsoid y extent");
  }
  { // the transform is applied as a * p: a 90 degree turn about z sends the tube's axis along x
    std::fill(a, a + 9, 0.0);
    a[1] = -1.0, a[3] = 0.5, a[8] = 0.5; // x' = -y, y' = 0.5 x: height 1 along x, radius 0.5
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Tube, a, c, p);
    const Bounds b(p);
    check(near(b.hi[0] - c[0], 0.5) && near(c[0] - b.lo[0], 0.5), "tube turned about z lies along x");
    check(near(b.hi[1] - c[1], 0.5, 0.01), "tube turned about z: radius in y");
  }
  { // tube radius 0.5, height 1: Houdini's matrix is (0.5, 1, 0.5) on the unit tube (y -0.5 to 0.5)
    std::fill(a, a + 9, 0.0);
    a[0] = 0.5, a[4] = 1.0, a[8] = 0.5;
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Tube, a, c, p);
    const Bounds b(p);
    check(near(b.hi[1] - b.lo[1], 1.0) && near(b.hi[0] - b.lo[0], 1.0, 1e-4), "tube size");
  }
  { // a circle in XY, radius 0.5: thin along its normal (z), full across
    diag(0.5, a);
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Circle, a, c, p);
    const Bounds b(p);
    check(near(b.hi[2] - b.lo[2], 2 * kCircleHalfThickness, 1e-5), "circle thickness along its normal");
    check(near(b.hi[0] - b.lo[0], 1.0, 1e-4) && near(b.hi[1] - b.lo[1], 1.0, 1e-4), "circle width");
  }
  { // a metaball of radius 0.5 reaches half that, as the Bullet Solver's does
    diag(0.5, a);
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Metaball, a, c, p);
    check(near(Bounds(p).hi[1] - c[1], 0.25), "metaball radius");
  }
  { // an exact sphere: a uniform scale, turned or not; not an ellipsoid or a shear
    double r = 0;
    diag(0.5, a);
    check(quadricSphereRadius(a, r) && near(r, 0.5), "uniform scale is a sphere");
    const double k = 0.5 * std::sqrt(0.5); // 45 degrees about z, radius 0.5
    const double turned[9] = {k, -k, 0, k, k, 0, 0, 0, 0.5};
    check(quadricSphereRadius(turned, r) && near(r, 0.5), "a turned uniform scale is a sphere");
    const double ellipsoid[9] = {2, 0, 0, 0, 0.5, 0, 0, 0, 1};
    check(!quadricSphereRadius(ellipsoid, r), "an ellipsoid is not a sphere");
    const double shear[9] = {0.5, 0.2, 0, 0, 0.5, 0, 0, 0, 0.5};
    check(!quadricSphereRadius(shear, r), "a shear is not a sphere");
    const double zero[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    check(!quadricSphereRadius(zero, r), "a zero matrix is not a sphere");
  }
  { // nothing for a non-quadric
    std::vector<float> p;
    diag(1.0, a);
    appendQuadricPoints(QuadricKind::None, a, c, p);
    check(p.empty(), "None adds nothing");
  }
  { // a degenerate circle matrix still gives points with finite values
    std::fill(a, a + 9, 0.0);
    std::vector<float> p;
    appendQuadricPoints(QuadricKind::Circle, a, c, p);
    check(!p.empty() && std::isfinite(p[0]), "degenerate circle");
  }

  std::printf(failures ? "FAILED\n" : "OK\n");
  return failures ? 1 : 0;
}
