// SideFX Glue's impulse model (core/Box3DGlue.h): propagation as the Glue help
// works it, decay by the half-life, breaking over the strength, and that a
// bond nothing hits never breaks; and the break estimate of Bullet's
// pre-breaking (core/Box3DGlueEstimate.h).
#include "TestUtil.h"
#include "core/Box3DGlue.h"
#include "core/Box3DGlueEstimate.h"

#include <cmath>
#include <cstdio>
#include <limits>

using namespace b3dop;

namespace {

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

  // ---- a rate that is not a number or is out of range: the default, 0 or 1,
  // and no NaN reaches the shares, so the bond still sees the hit and breaks
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK(glueRate(nan) == GlueParams().rate &&
          glueRate(inf) == GlueParams().rate);
    CHECK(glueRate(-inf) == GlueParams().rate && glueRate(-0.5f) == 0.0f);
    CHECK(glueRate(2.0f) == 1.0f && glueRate(0.25f) == 0.25f);
    std::vector<GlueBond> bonds = {bond(0, 1, nan, 100.0f)};
    std::vector<int> broken;
    glueAdvance(bonds, {1000.0f, 0.0f}, 1.0f / 24.0f, broken);
    CHECK(std::isfinite(bonds[0].impact) && bonds[0].impact > 0.0f);
    CHECK(broken.size() == 1);
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
    // just under and over the strength (rate 0: the bond sees its own end's
    // hits)
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {99.0f, 0.0f}, 0.0f, broken);
    CHECK(broken.empty());
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {101.0f, 0.0f}, 0.0f, broken);
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

  // ---- a decayed impact is zero, so a quiet frame is idle again
  {
    std::vector<int> broken;
    std::vector<GlueBond> bonds = {bond(0, 1, 0.0f, 1e9f, 1, 0.1f)};
    bonds[0].impact = 1e5f;
    int frames = 0;
    for (; frames < 1000 && bonds[0].impact > 0.0f; ++frames)
      glueAdvance(bonds, {0.0f, 0.0f}, 1.0f / 24.0f, broken);
    // 1e5 down to 1e-6 is 37 half-lives of 0.1 s, at 24 frames a second.
    CHECK(bonds[0].impact == 0.0f && frames > 80 && frames < 100);
    // A NaN is not hidden by it.
    bonds[0].impact = std::numeric_limits<float>::quiet_NaN();
    glueAdvance(bonds, {0.0f, 0.0f}, 1.0f / 24.0f, broken);
    CHECK(std::isnan(bonds[0].impact));
  }

  // ---- estimates: a bond breaks when its decayed impact plus its ends'
  // estimates passes the strength, and reports that as its impact; the real
  // hits are tested alone
  {
    std::vector<int> broken;
    std::vector<GlueBond> bonds = {bond(0, 1, 0.0f, 100.0f)};
    // under: nothing breaks, the impact is the hits'
    glueAdvance(bonds, {10.0f, 0.0f}, {60.0f, 30.0f}, 0.0f, broken);
    CHECK(broken.empty() && near(bonds[0].impact, 10.0, 1e-4));
    // over: it breaks and keeps the estimate as its impact
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {10.0f, 0.0f}, {60.0f, 50.0f}, 0.0f, broken);
    CHECK(broken.size() == 1 && near(bonds[0].impact, 110.0, 1e-4));
    // the decayed impact counts with it: 100 over one half-life is 50
    bonds = {bond(0, 1, 0.0f, 100.0f, 1, 0.5f)};
    bonds[0].impact = 100.0f;
    glueAdvance(bonds, {0.0f, 0.0f}, {30.0f, 30.0f}, 0.5f, broken);
    CHECK(broken.size() == 1 && near(bonds[0].impact, 110.0, 1e-4));
    // estimates spread like hits: A's 1000 over a bond of rate 1 is 500 at
    // each end, so the bond sees the whole 1000
    bonds = {bond(0, 1, 1.0f, 900.0f)};
    glueAdvance(bonds, {0.0f, 0.0f}, {1000.0f, 0.0f}, 0.0f, broken);
    CHECK(broken.size() == 1 && near(bonds[0].impact, 1000.0, 1e-4));
    // never-breaking and estimates of zero change nothing
    bonds = {bond(0, 1, 0.0f, -1.0f)};
    glueAdvance(bonds, {0.0f, 0.0f}, {1e9f, 1e9f}, 0.0f, broken);
    CHECK(broken.empty());
    bonds = {bond(0, 1, 0.0f, 100.0f)};
    glueAdvance(bonds, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, broken);
    CHECK(broken.empty() && bonds[0].impact == 0.0f);
  }

  // ---- the estimate of Bullet's pre-breaking: (1 + e v) v / d at each contact
  // point. Two 1000 kg unit boxes in a row landing flat on the ground at 7 m/s:
  // a rigid body of 2000 kg, inertia 333.3 about x and 833.3 about y and z,
  // with eight points under it at x = -1, 0, 0, 1 and z = +-0.5. Each point's
  // denominator is 1 / 2000 + x^2 / 833.3 + z^2 / 333.3 (0.00245 at x = +-1,
  // 0.00125 at x = 0), so a point is worth 408 v and 800 v, which a pair of
  // pieces sums to 4832 v, against a body of 2000 v of momentum.
  {
    GlueBody row;
    row.moves = true;
    row.mass = 2000.0f;
    row.inertia[0] = 2000.0 * 2.0 / 12.0;
    row.inertia[4] = row.inertia[8] = 2000.0 * 5.0 / 12.0;
    GlueBody ground; // does not move
    const float down[3] = {0.0f, -1.0f, 0.0f};
    auto landing = [&](float approach, float restitution) {
      double total = 0.0;
      for (const double x : {-1.0, 0.0, 0.0, 1.0})
        for (const double z : {-0.5, 0.5}) {
          const double at[3] = {x, -0.5, z};
          total +=
              glueContactEstimate(row, ground, at, down, approach, restitution);
        }
      return total;
    };
    CHECK(near(landing(7.0f, 0.0f), 4832.0 * 7.0, 4832.0 * 7.0 * 2e-3));
    CHECK(near(landing(7.0f, 0.25f), 4832.0 * 7.0 * 2.75,
               4832.0 * 7.0 * 2.75 * 2e-3));
    // moving away from the ground, or not at all: no impulse
    CHECK(landing(-7.0f, 0.25f) == 0.0);
    CHECK(landing(0.0f, 0.25f) == 0.0);
    // straight under the centre of mass only the mass counts
    const double centre[3] = {0.0, -0.5, 0.0};
    CHECK(near(glueContactEstimate(row, ground, centre, down, 2.0f, 0.0f),
               2000.0 * 2.0, 4.0 * 2e-3 * 1000.0));
    // two bodies that move: the denominators add
    GlueBody other = row;
    other.com[1] = -1.0;
    CHECK(near(glueContactEstimate(row, other, centre, down, 7.0f, 0.0f),
               7.0 / (1.0 / 2000.0 + 1.0 / 2000.0), 14.0));
    // nothing there takes an impulse: none
    CHECK(glueContactEstimate(ground, ground, centre, down, 7.0f, 0.5f) == 0.0);
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
