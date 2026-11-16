// SideFX Glue's impulse model (core/Box3DGlue.h): propagation as the Glue help
// works it, decay by the half-life, breaking over the strength, and that a
// bond nothing hits never breaks.
#include "TestUtil.h"
#include "core/Box3DGlue.h"

#include <cmath>
#include <cstdio>

using namespace b3dop;

namespace {

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

GlueBond bond(int a, int b, float rate = 1.0f, float strength = 1e9f,
              int iterations = 1, float halfLife = 0.1f) {
  GlueBond g;
  g.a = a;
  g.b = b;
  g.params.rate = rate;
  g.params.strength = strength;
  g.params.iterations = iterations;
  g.params.halfLife = halfLife;
  return g;
}

} // namespace

int main() {
  // ---- the help's example: A hit with 1000, bonds A-B (rate 1) and A-C (0.5),
  // one round: A 400, B 400, C 200; the bonds then hold 800 and 600.
  {
    std::vector<GlueBond> bonds = {bond(0, 1, 1.0f), bond(0, 2, 0.5f)};
    std::vector<float> hits = {1000.0f, 0.0f, 0.0f};
    glueSpread(bonds, hits);
    CHECK(near(hits[0], 400.0, 1e-3) && near(hits[1], 400.0, 1e-3) &&
          near(hits[2], 200.0, 1e-3));
    std::vector<int> broken;
    bonds = {bond(0, 1, 1.0f), bond(0, 2, 0.5f)};
    glueAdvance(bonds, {1000.0f, 0.0f, 0.0f}, 1.0f / 24.0f, broken);
    CHECK(near(bonds[0].impact, 800.0, 1e-3) &&
          near(bonds[1].impact, 600.0, 1e-3) && broken.empty());
  }

  // ---- rounds: an impulse goes at most `iterations` bonds along a chain
  {
    std::vector<GlueBond> chain = {bond(0, 1, 1.0f, 1e9f, 1),
                                   bond(1, 2, 1.0f, 1e9f, 1)};
    std::vector<float> hits = {900.0f, 0.0f, 0.0f};
    glueSpread(chain, hits);
    CHECK(hits[2] == 0.0f && hits[1] > 0.0f);
    chain = {bond(0, 1, 1.0f, 1e9f, 2), bond(1, 2, 1.0f, 1e9f, 2)};
    hits = {900.0f, 0.0f, 0.0f};
    glueSpread(chain, hits);
    CHECK(hits[2] > 0.0f);
  }

  // ---- rate 0: nothing leaves the body; the bond only sees its own ends
  {
    std::vector<GlueBond> bonds = {bond(0, 1, 0.0f)};
    std::vector<float> hits = {500.0f, 0.0f};
    glueSpread(bonds, hits);
    CHECK(near(hits[0], 500.0, 1e-3) && near(hits[1], 0.0, 1e-3));
  }

  // ---- breaking: over the strength; -1 never; an unhit bond never
  {
    std::vector<int> broken;
    std::vector<GlueBond> bonds = {bond(0, 1, 1.0f, 100.0f),
                                   bond(2, 3, 1.0f, -1.0f)};
    glueAdvance(bonds, {400.0f, 0.0f, 1e6f, 0.0f}, 1.0f / 24.0f, broken);
    CHECK(broken.size() == 1 && broken[0] == 0);
    // just under and over the scaled strength (rate 0: the bond sees its own
    // end's hits)
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {100.0f / kGlueStrengthScale * 0.99f, 0.0f}, 0.0f,
                broken);
    CHECK(broken.empty());
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {100.0f / kGlueStrengthScale * 1.01f, 0.0f}, 0.0f,
                broken);
    CHECK(broken.size() == 1);
    bonds = {bond(0, 1, 1.0f, 1.0f)};
    for (int i = 0; i < 100; ++i)
      glueAdvance(bonds, {0.0f, 0.0f}, 1.0f / 24.0f, broken);
    CHECK(broken.empty() && bonds[0].impact == 0.0f); // weight is no hit
  }

  // ---- decay: the half-life halves the impact; summed hits accumulate
  {
    std::vector<int> broken;
    std::vector<GlueBond> bonds = {bond(0, 1, 0.0f, 1e9f, 1, 0.5f)};
    bonds[0].impact = 1000.0f;
    glueAdvance(bonds, {0.0f, 0.0f}, 0.5f, broken); // one half-life
    CHECK(near(bonds[0].impact, 500.0, 1e-2));
    glueAdvance(bonds, {100.0f, 0.0f}, 0.0f, broken); // no time: no decay
    CHECK(near(bonds[0].impact, 600.0, 1e-2));
    // a zero half-life forgets at once
    bonds = {bond(0, 1, 0.0f, 1e9f, 1, 0.0f)};
    bonds[0].impact = 1000.0f;
    glueAdvance(bonds, {10.0f, 0.0f}, 0.1f, broken);
    CHECK(near(bonds[0].impact, 10.0, 1e-3));
  }

  // ---- bonds naming a missing node are ignored
  {
    std::vector<int> broken;
    std::vector<GlueBond> bonds = {bond(0, 5, 1.0f, 0.0f)};
    glueAdvance(bonds, {10.0f, 10.0f}, 0.1f, broken);
    CHECK(broken.empty());
  }

  if (fails) {
    std::printf("%d check(s) failed\n", fails);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
