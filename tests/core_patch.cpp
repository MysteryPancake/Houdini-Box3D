// The contact patch of two pieces (core/Box3DPatch.h): area, centroid, normal
// and second moments of where their faces touch, against hand-worked shapes,
// and the faces Box3DWorld gives for a box.
#include "TestUtil.h"
#include "core/Box3DBond.h"
#include "core/Box3DPatch.h"
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>

using namespace b3dop;

namespace {

// The six faces of a box of half sizes `h` at `c`, turned `yaw` about y.
std::vector<PatchFace> boxFaces(const double c[3], const double h[3],
                                double yaw = 0.0) {
  std::vector<PatchFace> faces;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const auto place = [&](double x, double y, double z) {
    return std::array<double, 3>{c[0] + cs * x + sn * z, c[1] + y,
                                 c[2] - sn * x + cs * z};
  };
  const auto turn = [&](double x, double y, double z) {
    return std::array<double, 3>{cs * x + sn * z, y, -sn * x + cs * z};
  };
  for (int axis = 0; axis < 3; ++axis)
    for (const double sign : {1.0, -1.0}) {
      PatchFace f;
      double n[3] = {0, 0, 0};
      n[axis] = sign;
      const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
      const double corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
      for (const auto &q : corners) {
        double p[3];
        p[axis] = sign * h[axis];
        p[a1] = q[0] * h[a1];
        p[a2] = q[1] * h[a2] * sign;
        f.corners.push_back(place(p[0], p[1], p[2]));
      }
      const auto nn = turn(n[0], n[1], n[2]);
      for (int i = 0; i < 3; ++i)
        f.normal[i] = nn[static_cast<size_t>(i)];
      faces.push_back(f);
    }
  return faces;
}

} // namespace

int main() {
  const int assertsBefore = Box3DWorld::assertCount();
  const double unit[3] = {0.5, 0.5, 0.5};

  // Two unit cubes stacked: the whole face, centred, normal up.
  {
    const double a[3] = {0, 0.5, 0}, b[3] = {0, 1.5, 0};
    const ContactPatch p =
        contactPatchOf(boxFaces(a, unit), boxFaces(b, unit), 0.02);
    CHECK(p.ok && near(p.area, 1.0, 1e-9));
    CHECK(near(p.normal[1], 1.0, 1e-9));
    CHECK(near(p.centroid[0], 0.0, 1e-9) && near(p.centroid[1], 1.0, 1e-9) &&
          near(p.centroid[2], 0.0, 1e-9));
    CHECK(near(p.momentU, 1.0 / 12.0, 1e-9) &&
          near(p.momentV, 1.0 / 12.0, 1e-9));
    CHECK(near(p.extentR, std::sqrt(0.5), 1e-9));
  }
  // Shifted by 0.5 along x: half the face, its centroid half way to the shift,
  // the moment about the axis along z (stress along x) small, the other large.
  {
    const double a[3] = {0, 0.5, 0}, b[3] = {0.5, 1.5, 0};
    const ContactPatch p =
        contactPatchOf(boxFaces(a, unit), boxFaces(b, unit), 0.02);
    CHECK(p.ok && near(p.area, 0.5, 1e-9));
    CHECK(near(p.centroid[0], 0.25, 1e-9));
    // The patch is 0.5 along x and 1 along z.
    const double alongX = std::fabs(p.u[0]) > 0.5 ? p.momentU : p.momentV;
    const double alongZ = std::fabs(p.u[0]) > 0.5 ? p.momentV : p.momentU;
    // The moment about the axis in x's direction: int z^2 dA = 0.5 * 1/12.
    CHECK(near(alongX, 0.5 / 12.0, 1e-9));
    // About the z axis: int x^2 dA = 1 * 0.5^3 / 12.
    CHECK(near(alongZ, 0.125 / 12.0, 1e-9));
  }
  // One cube turned 45 degrees about the normal on the other: the octagon of
  // the overlap, 2 * (sqrt(2) - 1) of the face.
  {
    const double a[3] = {0, 0.5, 0}, b[3] = {0, 1.5, 0};
    const ContactPatch p = contactPatchOf(
        boxFaces(a, unit), boxFaces(b, unit, 0.78539816339744830962), 0.02);
    CHECK(p.ok && near(p.area, 2.0 * (std::sqrt(2.0) - 1.0), 1e-9));
    CHECK(near(p.centroid[0], 0.0, 1e-9) && near(p.centroid[2], 0.0, 1e-9));
  }
  // A gap or an overlap within the tolerance still touches; beyond it does not.
  {
    const double a[3] = {0, 0.5, 0};
    const double within[3] = {0, 1.51, 0}, beyond[3] = {0, 1.6, 0},
                 overlap[3] = {0, 1.49, 0};
    CHECK(contactPatchOf(boxFaces(a, unit), boxFaces(within, unit), 0.02).ok);
    CHECK(contactPatchOf(boxFaces(a, unit), boxFaces(overlap, unit), 0.02).ok);
    CHECK(!contactPatchOf(boxFaces(a, unit), boxFaces(beyond, unit), 0.02).ok);
  }
  // Side by side (normal along x), a tall thin piece against a wide one: the
  // patch is the smaller face, in the plane x = 1.
  {
    const double ha[3] = {0.5, 1.0, 0.25}, hb[3] = {0.5, 0.5, 1.0};
    const double a[3] = {0, 1.0, 0}, b[3] = {1.0, 0.5, 0};
    const ContactPatch p =
        contactPatchOf(boxFaces(a, ha), boxFaces(b, hb), 0.02);
    // The faces are 2 (y) x 0.5 (z) and 1 (y) x 2 (z): they overlap in 1 x 0.5.
    CHECK(p.ok && near(p.area, 0.5, 1e-9) && near(p.normal[0], 1.0, 1e-9));
  }
  // Nothing touching, and faces that do not face each other.
  {
    const double a[3] = {0, 0.5, 0}, far[3] = {5, 1.5, 0};
    CHECK(!contactPatchOf(boxFaces(a, unit), boxFaces(far, unit), 0.02).ok);
    CHECK(!contactPatchOf({}, boxFaces(a, unit), 0.02).ok);
  }

  // The patch's moduli against a beam's: a section 1 (z) wide and 0.5 (y) high
  // is stronger across its width than a square of the same area would be, and
  // the square would be 40% off.
  {
    const double a[3] = {-1, 0.25, 0}, b[3] = {1, 0.25, 0};
    const double h[3] = {1.0, 0.25, 0.5};
    const ContactPatch p = contactPatchOf(boxFaces(a, h), boxFaces(b, h), 0.02);
    CHECK(p.ok && near(p.area, 0.5, 1e-9));
    // Bending about z (vertical load): W = b h^2 / 6 = 1 * 0.25 / 6.
    // The axis is the one with the larger extent along y.
    const bool uIsZ = std::fabs(p.u[2]) > 0.5;
    const double extentY = uIsZ ? p.extentV : p.extentU;
    const double momentAboutZ = uIsZ ? p.momentU : p.momentV;
    CHECK(near(extentY, 0.25, 1e-9));
    CHECK(near(momentAboutZ / extentY, 0.25 / 6.0, 1e-9));
    const double square = p.area * std::sqrt(p.area) / 6.0;
    std::printf("beam section 1 x 0.5: modulus %.5f, square patch %.5f\n",
                momentAboutZ / extentY, square);
    CHECK(square >
          1.3 * (momentAboutZ / extentY)); // the old model was well over
  }

  // The faces of a box body in the world: six, outward normals, 1 m^2 each.
  {
    WorldSettings ws;
    ws.workerCount = 1;
    Box3DWorld w;
    CHECK(w.create(ws));
    BodySettings bs;
    bs.type = BodySettings::Type::Static;
    const double pos[3] = {0, 2, 0};
    const float q[4] = {0, 0, 0, 1};
    const BodyHandle h = w.createBody(bs, pos, q);
    ShapeSpec s;
    s.type = ShapeType::Box;
    s.halfExtents[0] = s.halfExtents[1] = s.halfExtents[2] = 0.5f;
    w.createShape(h, bs, s);
    std::vector<PatchFace> faces;
    CHECK(w.hullFacesByHandle(h, faces) == 6);
    int up = 0;
    for (const PatchFace &f : faces) {
      CHECK(f.corners.size() == 4);
      double mean[3] = {0, 0, 0};
      for (const auto &c : f.corners)
        for (int i = 0; i < 3; ++i)
          mean[i] += c[static_cast<size_t>(i)] / 4.0;
      // The face's centre is half a metre out along its normal.
      for (int i = 0; i < 3; ++i)
        CHECK(near(mean[i], pos[i] + 0.5 * f.normal[i], 1e-5));
      up += f.normal[1] > 0.99 ? 1 : 0;
    }
    CHECK(up == 1);
    // Two bodies on top of each other: the patch from the world's faces.
    const double pos2[3] = {0.5, 3, 0};
    const BodyHandle h2 = w.createBody(bs, pos2, q);
    w.createShape(h2, bs, s);
    std::vector<PatchFace> faces2;
    w.hullFacesByHandle(h2, faces2);
    const ContactPatch p = contactPatchOf(faces, faces2, 0.02);
    CHECK(p.ok && near(p.area, 0.5, 1e-5) && near(p.normal[1], 1.0, 1e-5));
  }

  CHECK(Box3DWorld::assertCount() == assertsBefore);
  if (!fails)
    std::printf("OK\n");
  return fails ? 1 : 0;
}
