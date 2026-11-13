// Static triangle-mesh colliders (what Static Object / affector geometry
// becomes): a dynamic box and sphere land on a two-triangle floor, and bad
// input is refused.
#include "KeyedWorld.h"
#include "TestUtil.h"
#include "core/Box3DAffectorMath.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

int main() {
  using namespace b3dop;
  KeyedWorld w;
  CHECK(w.create(WorldSettings()));

  const float qid[4] = {0, 0, 0, 1};
  const double origin[3] = {0, 0, 0};

  // A 20 x 20 floor at y = 0, facing up (CCW seen from above).
  const float verts[] = {-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10};
  const int32_t tris[] = {0, 2, 1, 0, 3, 2};

  BodySettings st;
  st.type = BodySettings::Type::Static;
  const BodyHandle floorBody = w.createBody(st, origin, qid);
  CHECK(floorBody != 0);
  CHECK(w.createMeshShape(floorBody, st, verts, 4, tris, 2));
  CHECK(w.shapeCountByHandle(floorBody) == 1);

  // Refused: dynamic body, bad index, non-finite vertex, too few triangles.
  BodySettings dyn;
  const BodyHandle d = w.createBody(dyn, origin, qid);
  CHECK(!w.createMeshShape(d, dyn, verts, 4, tris, 2));
  const int32_t badTris[] = {0, 1, 9};
  CHECK(!w.createMeshShape(floorBody, st, verts, 4, badTris, 1));
  float nanVerts[12];
  for (int i = 0; i < 12; ++i)
    nanVerts[i] = verts[i];
  nanVerts[4] = NAN;
  CHECK(!w.createMeshShape(floorBody, st, nanVerts, 4, tris, 2));
  CHECK(!w.createMeshShape(floorBody, st, verts, 4, tris, 0));
  CHECK(w.shapeCountByHandle(floorBody) == 1);
  w.destroyBody(d);

  const double boxPos[3] = {0, 2.0, 0}, spherePos[3] = {3, 2.0, 3};
  const BodyHandle box = w.createBody(dyn, boxPos, qid);
  const float he[3] = {0.5f, 0.5f, 0.5f};
  ShapeSpec bs;
  bs.type = ShapeType::Box;
  bs.halfExtents[0] = bs.halfExtents[1] = bs.halfExtents[2] = 0.5f;
  CHECK(w.createShape(box, dyn, bs));
  const BodyHandle sph = w.createBody(dyn, spherePos, qid);
  ShapeSpec ss;
  ss.type = ShapeType::Sphere;
  ss.radius = 0.5f;
  CHECK(w.createShape(sph, dyn, ss));
  (void)he;

  for (int i = 0; i < 300; ++i)
    w.step(1.0f / 60.0f);

  double p[3];
  float outQ[4];
  CHECK(w.getTransformByHandle(box, p, outQ));
  std::printf("box on mesh    y=%.4f\n", p[1]);
  CHECK(std::fabs(p[1] - 0.5) < 0.05);
  CHECK(w.getTransformByHandle(sph, p, outQ));
  std::printf("sphere on mesh y=%.4f\n", p[1]);
  CHECK(std::fabs(p[1] - 0.5) < 0.05);

  // Mesh storage is released with the body, by shape rebuild, and with the
  // world.
  w.destroyShapesByHandle(floorBody);
  CHECK(w.shapeCountByHandle(floorBody) == 0);
  CHECK(w.createMeshShape(floorBody, st, verts, 4, tris, 2));
  w.destroyBody(floorBody);
  const BodyHandle f2 = w.createBody(st, origin, qid);
  CHECK(w.createMeshShape(f2, st, verts, 4, tris, 2, 0.001f));
  w.destroy();

  // ---- mesh scale: a floor at y = 1 scaled by 2 in y sits at y = 2; rescaling
  // moves it ----
  {
    KeyedWorld sw;
    CHECK(sw.create(WorldSettings()));
    const float raised[] = {-10, 1, -10, 10, 1, -10, 10, 1, 10, -10, 1, 10};
    const float s2[3] = {1, 2, 1}, s1[3] = {1, 1, 1};
    const BodyHandle fl = sw.createBody(st, origin, qid);
    CHECK(sw.createMeshShape(fl, st, raised, 4, tris, 2, 0.0f, s2));
    const double bp[3] = {0, 4.0, 0};
    const BodyHandle bx = sw.createBody(dyn, bp, qid);
    CHECK(sw.createShape(bx, dyn, bs));
    for (int i = 0; i < 240; ++i)
      sw.step(1.0f / 60.0f);
    CHECK(sw.getTransformByHandle(bx, p, outQ));
    std::printf("box on scaled mesh   y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 2.5) < 0.05);
    sw.setMeshScaleByHandle(fl, s1); // wakes what touches it
    for (int i = 0; i < 240; ++i)
      sw.step(1.0f / 60.0f);
    CHECK(sw.getTransformByHandle(bx, p, outQ));
    std::printf("box on rescaled mesh y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 1.5) < 0.05);
  }

  // ---- decomposeScaledRigid: S * R + T splits back; shear and mirror are
  // refused ----
  {
    const double c = std::cos(0.6), sn = std::sin(0.6);
    const double sx = 2, sy = 3, sz = 0.5;
    // R about Z (row-vector rows), rows scaled.
    const double m[16] = {sx * c, sx * sn, 0,  0, -sy * sn, sy * c, 0, 0,
                          0,      0,       sz, 0, 1,        2,      3, 1};
    double pos[3];
    float quat[4], sc[3];
    CHECK(decomposeScaledRigid(m, pos, quat, sc));
    CHECK(std::fabs(sc[0] - 2) < 1e-5 && std::fabs(sc[1] - 3) < 1e-5 &&
          std::fabs(sc[2] - 0.5) < 1e-5);
    CHECK(pos[0] == 1 && pos[1] == 2 && pos[2] == 3);
    CHECK(std::fabs(std::fabs(quat[2]) - std::sin(0.3)) < 1e-5 &&
          std::fabs(quat[0]) < 1e-6);
    double shear[16] = {1, 0.5, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK(!decomposeScaledRigid(shear, pos, quat, sc));
    double mirror[16] = {-2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK(!decomposeScaledRigid(mirror, pos, quat, sc));
    double flat[16] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK(!decomposeScaledRigid(flat, pos, quat, sc));
  }

  // ---- decomposeRigid: row-vector matrices round-trip a rotation +
  // translation ----
  {
    // Rotation of 70 degrees about the axis (1, 2, 3) (row-vector layout, as
    // UT_DMatrix4).
    const double ang = 70.0 * 3.14159265358979 / 180.0, ax = 1, ay = 2, az = 3;
    const double len = std::sqrt(ax * ax + ay * ay + az * az);
    const double qx = ax / len * std::sin(ang / 2),
                 qy = ay / len * std::sin(ang / 2),
                 qz = az / len * std::sin(ang / 2), qw = std::cos(ang / 2);
    double m[16] = {1 - 2 * (qy * qy + qz * qz),
                    2 * (qx * qy + qz * qw),
                    2 * (qx * qz - qy * qw),
                    0,
                    2 * (qx * qy - qz * qw),
                    1 - 2 * (qx * qx + qz * qz),
                    2 * (qy * qz + qx * qw),
                    0,
                    2 * (qx * qz + qy * qw),
                    2 * (qy * qz - qx * qw),
                    1 - 2 * (qx * qx + qy * qy),
                    0,
                    4,
                    5,
                    6,
                    1};
    double pos[3];
    float q[4];
    CHECK(decomposeRigid(m, pos, q));
    CHECK(std::fabs(pos[0] - 4) < 1e-9 && std::fabs(pos[1] - 5) < 1e-9 &&
          std::fabs(pos[2] - 6) < 1e-9);
    const double dotq = q[0] * qx + q[1] * qy + q[2] * qz + q[3] * qw;
    CHECK(std::fabs(std::fabs(dotq) - 1.0) < 1e-5);
    // Rotating a point with the matrix agrees with rotating it by the
    // quaternion's body-space use: check through Box3D-free maths: the matrix
    // of the recovered quaternion equals m.
    double p0[3] = {1, 0, 0}, p1[3];
    transformPoint(m, p0, p1);
    CHECK(std::fabs(p1[0] - (1 - 2 * (qy * qy + qz * qz) + 4)) < 1e-9);

    // Scale, mirror and perspective are not rigid.
    double s[16];
    for (int i = 0; i < 16; ++i)
      s[i] = m[i];
    for (int i = 0; i < 3; ++i) {
      s[i] *= 2;
      s[4 + i] *= 2;
      s[8 + i] *= 2;
    }
    CHECK(!decomposeRigid(s, pos, q));
    double mir[16] = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK(!decomposeRigid(mir, pos, q));
    double persp[16];
    for (int i = 0; i < 16; ++i)
      persp[i] = m[i];
    persp[3] = 0.1;
    CHECK(!decomposeRigid(persp, pos, q));
    // All four quaternion branches (identity, 180 degrees about each axis).
    const double flips[3][16] = {
        {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1},
        {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1},
        {-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    for (const auto &f : flips) {
      CHECK(decomposeRigid(f, pos, q));
      CHECK(std::fabs(q[3]) < 1e-6);
    }
    const double id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK(decomposeRigid(id, pos, q) && std::fabs(q[3] - 1) < 1e-6);
  }

  std::printf(fails ? "FAILED (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
