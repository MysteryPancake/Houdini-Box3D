// Grounds (what a Ground Plane DOP becomes: a wide, deep static box whose top
// is the plane y = 0 of its body): spheres, capsules, boxes and tumbling hulls
// rest on them, two grounds make a trough, a tilted ground slopes, fast bodies
// do not tunnel, and a dynamic body is refused.
#include "KeyedWorld.h"
#include "TestUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

using namespace b3dop;

const float qid[4] = {0, 0, 0, 1};

BodySettings staticBody() {
  BodySettings b;
  b.type = BodySettings::Type::Static;
  return b;
}

// A static body at `pos` turned by `angle` radians about z, with the ground
// whose top is the plane y = 0 of its own space.
BodyHandle addPlane(KeyedWorld &w, const double pos[3], float angle) {
  const BodySettings st = staticBody();
  const float q[4] = {0, 0, std::sin(0.5f * angle), std::cos(0.5f * angle)};
  const BodyHandle h = w.createBody(st, pos, q);
  CHECK(w.createShape(h, st, groundSpec()));
  return h;
}

BodyHandle addDynamic(KeyedWorld &w, const ShapeSpec &shape,
                      const double pos[3]) {
  const BodySettings dyn;
  const BodyHandle h = w.createBody(dyn, pos, qid);
  CHECK(w.createShape(h, dyn, shape));
  return h;
}

void run(KeyedWorld &w, int steps) {
  for (int i = 0; i < steps; ++i)
    w.step(1.0f / 60.0f);
}

} // namespace

int main() {
  const double origin[3] = {0, 0, 0};
  double p[3];
  float q[4];

  ShapeSpec box;
  box.type = ShapeType::Box;
  box.halfExtents[0] = box.halfExtents[1] = box.halfExtents[2] = 0.5f;
  ShapeSpec ball;
  ball.type = ShapeType::Sphere;
  ball.radius = 0.5f;
  ShapeSpec cap;
  cap.type = ShapeType::Capsule;
  cap.radius = 0.25f;
  cap.halfLength = 0.5f;

  // ---- each shape rests on a flat plane at its own height ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    addPlane(w, origin, 0.0f);
    const double a[3] = {-3, 2, 0}, b[3] = {0, 2, 0}, c[3] = {3, 2, 0};
    const BodyHandle hb = addDynamic(w, box, a);
    const BodyHandle hs = addDynamic(w, ball, b);
    const BodyHandle hc = addDynamic(w, cap, c);
    run(w, 240);
    CHECK(w.getTransformByHandle(hb, p, q));
    std::printf("box on plane      y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.03);
    CHECK(w.getTransformByHandle(hs, p, q));
    std::printf("sphere on plane   y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.03);
    CHECK(w.getTransformByHandle(hc, p, q));
    std::printf("capsule on plane  y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.75) < 0.03);
  }

  // ---- the plane's height and turn come from its body ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    const double at[3] = {0, 3, 0};
    addPlane(w, at, 0.0f);
    const double s[3] = {0, 6, 0};
    const BodyHandle hs = addDynamic(w, ball, s);
    run(w, 240);
    CHECK(w.getTransformByHandle(hs, p, q));
    std::printf("sphere on raised  y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 3.5) < 0.03);
  }

  // ---- a tilted plane (rising towards +x): a sphere rolls towards -x ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    addPlane(w, origin, 0.4f);
    const double s[3] = {2, 3, 0};
    const BodyHandle hs = addDynamic(w, ball, s);
    run(w, 90);
    CHECK(w.getTransformByHandle(hs, p, q));
    std::printf("sphere on slope   x=%.4f y=%.4f\n", p[0], p[1]);
    CHECK(p[0] < 1.0);
    // Resting on the surface y = tan(0.4) x, half a diameter off it along the
    // normal.
    CHECK(std::fabs((p[1] - std::tan(0.4f) * p[0]) * std::cos(0.4f) - 0.5) <
          0.05);
  }

  // ---- two planes make a trough: a sphere settles in the middle at r /
  // cos(45) ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    const float quarter = 0.78539816f;
    addPlane(w, origin, quarter);  // rises towards +x
    addPlane(w, origin, -quarter); // rises towards -x
    const double s[3] = {0.3, 3, 0};
    const BodyHandle hs = addDynamic(w, ball, s);
    run(w, 600);
    CHECK(w.getTransformByHandle(hs, p, q));
    std::printf("sphere in trough  x=%.4f y=%.4f\n", p[0], p[1]);
    CHECK(std::fabs(p[0]) < 0.05);
    CHECK(std::fabs(p[1] - 0.5 / std::cos(quarter)) < 0.05);
  }

  // ---- a fast sphere does not tunnel ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    addPlane(w, origin, 0.0f);
    ShapeSpec pea;
    pea.type = ShapeType::Sphere;
    pea.radius = 0.1f;
    const double s[3] = {0, 5, 0};
    const BodyHandle hs = addDynamic(w, pea, s);
    const float v[3] = {0, -400, 0}, zero[3] = {0, 0, 0};
    CHECK(w.setVelocityByHandle(hs, v, zero));
    run(w, 30);
    CHECK(w.getTransformByHandle(hs, p, q));
    std::printf("fast sphere       y=%.4f\n", p[1]);
    CHECK(p[1] > 0.05);
  }

  // ---- convex hulls tumbling onto the ground end up resting on it, not sunk
  // into it or stuck: a lopsided hull from several turns and spins ----
  {
    const float pts[] = {-0.6f, -0.3f, -0.4f, 0.7f, -0.2f, -0.3f, 0.1f,
                         0.5f,  -0.5f, -0.2f, 0.4f, 0.6f,  0.3f,  -0.4f,
                         0.5f,  0.9f,  0.3f,  0.4f, -0.8f, 0.2f,  0.1f};
    const int count = static_cast<int>(sizeof(pts) / sizeof(pts[0]) / 3);
    for (int trial = 0; trial < 12; ++trial) {
      KeyedWorld w;
      CHECK(w.create(WorldSettings()));
      addPlane(w, origin, 0.0f);
      const HullId hull = w.createHull(pts, count);
      CHECK(hull != 0);
      const BodySettings dyn;
      const float a = 0.5f * (0.4f + 0.9f * static_cast<float>(trial));
      const float axis[3] = {std::sin(1.3f * static_cast<float>(trial)),
                             std::cos(0.7f * static_cast<float>(trial)),
                             std::sin(0.3f * static_cast<float>(trial) + 1.0f)};
      const float len =
          std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
      const float r[4] = {std::sin(a) * axis[0] / len,
                          std::sin(a) * axis[1] / len,
                          std::sin(a) * axis[2] / len, std::cos(a)};
      const double s[3] = {0.3 * trial, 2.0, 0};
      const BodyHandle h = w.createBody(dyn, s, r);
      CHECK(w.createHullShape(h, dyn, hull));
      const float v[3] = {0.5f, -1.0f, 0.2f}, spin[3] = {2.0f, 1.0f, -3.0f};
      CHECK(w.setVelocityByHandle(h, v, spin));
      run(w, 360);
      // The lowest vertex of the hull in the world.
      CHECK(w.getTransformByHandle(h, p, q));
      double low = 1e9;
      for (int i = 0; i < count; ++i) {
        const double x = pts[3 * i], y = pts[3 * i + 1], z = pts[3 * i + 2];
        const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
        // y of the rotated point (quaternion * vector, y row).
        const double wy = (2 * (qx * qy + qw * qz)) * x +
                          (1 - 2 * (qx * qx + qz * qz)) * y +
                          (2 * (qy * qz - qw * qx)) * z;
        low = std::min(low, p[1] + wy);
      }
      std::printf("hull %2d lowest vertex y=%.4f\n", trial, low);
      CHECK(low > -0.02 && low < 0.05);
      float lin[3], ang[3];
      CHECK(w.getVelocityByHandle(h, lin, ang));
      CHECK(std::fabs(lin[1]) < 0.05f);
    }
  }

  // ---- the size is the user's: a ball inside a narrow ground rests, one past
  // its edge falls; a thin ground is as deep as it is set ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    const BodySettings st = staticBody();
    const BodyHandle g = w.createBody(st, origin, qid);
    GroundSize size;
    size.size[0] = 4.0f; // x
    size.size[1] = 1.0f; // depth
    size.size[2] = 4.0f; // z
    CHECK(w.createShape(g, st, groundSpec(size)));
    const double in[3] = {1.0, 2, 0}, out[3] = {3.0, 2, 0};
    const BodyHandle hin = addDynamic(w, ball, in);
    const BodyHandle hout = addDynamic(w, ball, out);
    run(w, 240);
    CHECK(w.getTransformByHandle(hin, p, q));
    std::printf("ball on a 4 wide ground    y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.03);
    CHECK(w.getTransformByHandle(hout, p, q));
    std::printf("ball past its edge         y=%.4f\n", p[1]);
    CHECK(p[1] < -5.0);
    // Sizes that are zero, negative or not numbers fall back to the limits.
    GroundSize odd;
    odd.size[0] = NAN;
    odd.size[1] = -3.0f;
    odd.size[2] = 1e9f;
    const ShapeSpec spec = groundSpec(odd);
    CHECK(near(spec.halfExtents[0], 0.5f * GroundSize().size[0], 1e-3));
    CHECK(near(spec.halfExtents[1], 0.5f * kMinGroundSize, 1e-6));
    CHECK(near(spec.halfExtents[2], 0.5f * kMaxGroundSize[2], 1e-3));
  }

  // ---- refused: a dynamic body ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    const BodySettings dyn;
    const BodyHandle d = w.createBody(dyn, origin, qid);
    CHECK(!w.createShape(d, dyn, groundSpec()));
    CHECK(w.shapeCountByHandle(d) == 0);
  }

  return fails ? 1 : 0;
}
