// Static infinite planes (what a Ground Plane DOP becomes): spheres, capsules
// and boxes rest on them, several planes make a trough, a tilted plane slopes,
// fast bodies do not tunnel, and bad input is refused.
#include "KeyedWorld.h"
#include "TestUtil.h"
#include <cmath>
#include <cstdio>

namespace {

using namespace b3dop;

const float qid[4] = {0, 0, 0, 1};
const float up[3] = {0, 1, 0};

BodySettings staticBody() {
  BodySettings b;
  b.type = BodySettings::Type::Static;
  return b;
}

// A static body at `pos` turned by `angle` radians about z, holding the plane y
// = 0 of its own space.
BodyHandle addPlane(KeyedWorld &w, const double pos[3], float angle) {
  const BodySettings st = staticBody();
  const float q[4] = {0, 0, std::sin(0.5f * angle), std::cos(0.5f * angle)};
  const BodyHandle h = w.createBody(st, pos, q);
  CHECK(w.createPlaneShape(h, st, up, 0.0f));
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

  // ---- refused: a dynamic body, a zero or non-finite normal ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    const BodySettings dyn;
    const BodyHandle d = w.createBody(dyn, origin, qid);
    CHECK(!w.createPlaneShape(d, dyn, up, 0.0f));
    const BodySettings st = staticBody();
    const BodyHandle h = w.createBody(st, origin, qid);
    const float zero[3] = {0, 0, 0}, bad[3] = {NAN, 1, 0};
    CHECK(!w.createPlaneShape(h, st, zero, 0.0f));
    CHECK(!w.createPlaneShape(h, st, bad, 0.0f));
    CHECK(!w.createPlaneShape(h, st, up, NAN));
    CHECK(w.shapeCountByHandle(h) == 0);
    // A normal that is not a unit vector is normalised, with the offset scaled
    // to match: 2y = 4 is y = 2.
    const float twice[3] = {0, 2, 0};
    CHECK(w.createPlaneShape(h, st, twice, 4.0f));
    const double s[3] = {0, 6, 0};
    const BodyHandle hs = addDynamic(w, ball, s);
    run(w, 240);
    CHECK(w.getTransformByHandle(hs, p, q));
    CHECK(std::fabs(p[1] - 2.5) < 0.03);
  }

  return fails ? 1 : 0;
}
