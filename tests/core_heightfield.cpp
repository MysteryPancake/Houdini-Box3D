// Static height-field colliders (what a Houdini height field on an affector
// becomes): bodies land on a flat field and roll down a ramp, holes let them
// through, and bad input is refused.
#include "KeyedWorld.h"
#include "TestUtil.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

int main() {
  using namespace b3dop;
  const float qid[4] = {0, 0, 0, 1};
  const double origin[3] = {0, 0, 0};
  BodySettings st;
  st.type = BodySettings::Type::Static;
  BodySettings dyn;
  ShapeSpec bs;
  bs.type = ShapeType::Box;
  bs.halfExtents[0] = bs.halfExtents[1] = bs.halfExtents[2] = 0.5f;
  ShapeSpec ss;
  ss.type = ShapeType::Sphere;
  ss.radius = 0.5f;
  double p[3];
  float q[4];

  const int nx = 9, nz = 9;
  const float unit[3] = {1, 1, 1};

  // ---- a flat field at height 1 over x, z in [0, 8]: a box rests at 1.5 ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    std::vector<float> flat(nx * nz, 1.0f);
    const BodyHandle hf = w.createBody(st, origin, qid);
    CHECK(w.createHeightFieldShape(hf, st, flat.data(), nx, nz, unit));
    CHECK(w.shapeCountByHandle(hf) == 1);

    // Refused: dynamic body, too few points, bad scale, non-finite height.
    const BodyHandle d = w.createBody(dyn, origin, qid);
    CHECK(!w.createHeightFieldShape(d, dyn, flat.data(), nx, nz, unit));
    w.destroyBody(d);
    CHECK(!w.createHeightFieldShape(hf, st, flat.data(), 1, nz, unit));
    const float zeroY[3] = {1, 0, 1}, negX[3] = {-1, 1, 1};
    CHECK(!w.createHeightFieldShape(hf, st, flat.data(), nx, nz, zeroY));
    CHECK(!w.createHeightFieldShape(hf, st, flat.data(), nx, nz, negX));
    std::vector<float> bad = flat;
    bad[10] = NAN;
    CHECK(!w.createHeightFieldShape(hf, st, bad.data(), nx, nz, unit));
    CHECK(w.shapeCountByHandle(hf) == 1);

    const double bp[3] = {4, 3, 4};
    const BodyHandle box = w.createBody(dyn, bp, qid);
    CHECK(w.createShape(box, dyn, bs));
    for (int i = 0; i < 240; ++i)
      w.step(1.0f / 60.0f);
    CHECK(w.getTransformByHandle(box, p, q));
    std::printf("box on flat field     y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 1.5) < 0.05);

    // Released by a shape rebuild, with the body and with the world.
    w.destroyShapesByHandle(hf);
    CHECK(w.shapeCountByHandle(hf) == 0);
    CHECK(w.createHeightFieldShape(hf, st, flat.data(), nx, nz, unit));
    w.destroyBody(hf);
    const BodyHandle hf2 = w.createBody(st, origin, qid);
    CHECK(w.createHeightFieldShape(hf2, st, flat.data(), nx, nz, unit));
    w.destroy();
  }

  // ---- scale and pose: spacing 2 in x and z, heights x3, body moved and
  // turned 90 degrees about y: still a floor at 3 + 3 = 6 over its area ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    std::vector<float> flat(nx * nz, 1.0f);
    const float sc[3] = {2, 3, 2};
    const double at[3] = {-8, 3, 8};
    const float s = std::sqrt(0.5f);
    const float turn[4] = {0, s, 0, s}; // +90 degrees about y: x -> -z
    const BodyHandle hf = w.createBody(st, at, turn);
    CHECK(w.createHeightFieldShape(hf, st, flat.data(), nx, nz, sc));
    // Local (8, 3, 8) -> world (-8 + 8, 6, 8 - 8) = (0, 6, 0): the middle.
    const double bp[3] = {0, 9, 0};
    const BodyHandle box = w.createBody(dyn, bp, qid);
    CHECK(w.createShape(box, dyn, bs));
    for (int i = 0; i < 240; ++i)
      w.step(1.0f / 60.0f);
    CHECK(w.getTransformByHandle(box, p, q));
    std::printf("box on posed field    y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 6.5) < 0.05);
  }

  // ---- a ramp rising along +z: a sphere rolls towards -z ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    std::vector<float> ramp(nx * nz);
    for (int r = 0; r < nz; ++r)
      for (int c = 0; c < nx; ++c)
        ramp[static_cast<size_t>(r * nx + c)] = 0.5f * static_cast<float>(r);
    const BodyHandle hf = w.createBody(st, origin, qid);
    CHECK(w.createHeightFieldShape(hf, st, ramp.data(), nx, nz, unit));
    const double sp[3] = {4, 4, 6};
    const BodyHandle sph = w.createBody(dyn, sp, qid);
    CHECK(w.createShape(sph, dyn, ss));
    for (int i = 0; i < 60; ++i)
      w.step(1.0f / 60.0f);
    CHECK(w.getTransformByHandle(sph, p, q));
    std::printf("sphere on ramp        z=%.4f y=%.4f\n", p[2], p[1]);
    CHECK(p[2] < 5.5);        // rolled downhill
    CHECK(p[1] > 0.5 * p[2]); // above the surface
    CHECK(std::fabs(p[0] - 4) < 0.1);
  }

  // ---- holes: the cells under the box are open, so it falls through ----
  {
    KeyedWorld w;
    CHECK(w.create(WorldSettings()));
    std::vector<float> flat(nx * nz, 0.0f);
    std::vector<uint8_t> holes(static_cast<size_t>((nx - 1) * (nz - 1)), 0);
    for (int r = 2; r < 6; ++r)
      for (int c = 2; c < 6; ++c)
        holes[static_cast<size_t>(r * (nx - 1) + c)] = 1;
    const BodyHandle hf = w.createBody(st, origin, qid);
    CHECK(w.createHeightFieldShape(hf, st, flat.data(), nx, nz, unit,
                                   holes.data()));
    const double bp[3] = {4, 2, 4}, bp2[3] = {7.5, 2, 7.5};
    const BodyHandle box = w.createBody(dyn, bp, qid);
    CHECK(w.createShape(box, dyn, bs));
    const BodyHandle box2 = w.createBody(dyn, bp2, qid);
    CHECK(w.createShape(box2, dyn, bs));
    for (int i = 0; i < 120; ++i)
      w.step(1.0f / 60.0f);
    CHECK(w.getTransformByHandle(box, p, q));
    std::printf("box over hole         y=%.4f\n", p[1]);
    CHECK(p[1] < -1.0);
    CHECK(w.getTransformByHandle(box2, p, q));
    std::printf("box beside hole       y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.05);
  }

  std::printf(fails ? "FAILED (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
