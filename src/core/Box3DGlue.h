// Box3DGlue.h
//
// Header-only, Houdini-free: SideFX Glue's impulse model (the Glue Constraint
// Relationship help, as the Bullet solver runs it). A glued pair moves as one
// until things colliding with it put enough impulse on the bond:
//   * every body collects the impulse of its contacts (hits) over the step;
//   * that impulse spreads over the bond network, `iterations` rounds: a body
//     keeps a share of 1 and gives each neighbour a share of the bond's
//     propagation rate (0 to 1), all divided by the sum of the shares;
//   * each bond adds the impulses its two bodies ended up with to its `impact`,
//     after that has decayed by the half-life;
//   * a bond whose impact passes its strength (< 0: never) divided by
//   kGlueStrengthScale breaks.
// A hit is the normal impulse of a body's contacts, as in Bullet, so a body
// resting on something adds its weight every step. A piece hanging from its
// bond has no contacts and holds at any strength.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace b3dop {

// A bond breaks when its impact passes its strength divided by this. The
// impacts agree with Bullet's (the `impact` attribute, same peak within ~20%
// and the same decay: `glue_impact_trace`), but a Bullet glue only holds from
// about 5 to 6 times its peak impact (its Strength has to be that much higher),
// so the break test is scaled to match: the Strength at which a landing stops
// breaking the glue is within ~14% of Bullet's at both heights tried
// (`glue_impact_scale`: Box3D's was 4.7 and 5.9 times lower without it).
// Measured, not derived, from one scene.
constexpr float kGlueStrengthScale = 5.3f;

// A compound is taken apart (see holdCompoundsForApproach) for a body that is
// faster than this (m/s) and can deliver more than the weakest bond's break
// impulse, and stays apart this many seconds after the last such body came
// near it.
constexpr float kGlueHoldMinSpeed = 2.0f;
constexpr float kGlueHoldSeconds = 2.0f;
// How far ahead (seconds of the body's motion) it is looked for: only when the
// release starts, which has to be before the body touches the compound (the
// pieces must be free in the step it hits; at least two frames are used, see
// holdCompoundsForApproach). Which pieces are freed is the body's whole way
// through the compound, not what it reaches in this time.
constexpr float kGlueHoldLeadSeconds = 0.25f;
// The pieces taken out of a compound are those in the body's swept bounds grown
// by this many times its own half size, so the pieces around its path can
// give way too.
constexpr float kGlueHoldHaloScale = 3.0f;
// The most rounds of propagation a bond may ask for: each costs a pass over
// every bond every frame, so a typo (1e6) must not freeze the solve.
constexpr int kMaxGlueIterations = 1000;

struct GlueParams {
  float strength = 10000.0f; // impulse to break; < 0 never
  float halfLife = 0.1f;     // seconds for the impact to halve
  float rate = 1.0f;         // propagation rate, clamped to 0..1
  int iterations = 1;        // rounds of propagation; < 1 counts as 1
};

// One bond between two nodes (bodies). `impact` is the Glue's `impact`
// attribute: it persists from step to step.
struct GlueBond {
  int a = -1, b = -1;
  GlueParams params;
  float impact = 0.0f;
};

// The impulse each node ends up with after `rounds` rounds of propagation.
// `hits` per node; bonds with fewer iterations than the round stop carrying
// (an impulse never travels more bonds than the bond's iterations).
inline void glueSpread(const std::vector<GlueBond> &bonds,
                       std::vector<float> &hits) {
  int rounds = 1;
  for (const GlueBond &bond : bonds)
    rounds =
        std::min(std::max(rounds, bond.params.iterations), kMaxGlueIterations);
  const size_t n = hits.size();
  // Kept between calls (every frame): no allocation. `next` trades buffers
  // with `hits` below; both are fully overwritten each round.
  thread_local std::vector<float> next, weight;
  next.resize(n);
  weight.resize(n);
  for (int round = 0; round < rounds; ++round) {
    // Each node's total share: itself (1) and its bonds' rates.
    std::fill(weight.begin(), weight.end(), 1.0f);
    for (const GlueBond &bond : bonds) {
      if (bond.params.iterations <= round)
        continue;
      const float r = std::clamp(bond.params.rate, 0.0f, 1.0f);
      weight[static_cast<size_t>(bond.a)] += r;
      weight[static_cast<size_t>(bond.b)] += r;
    }
    for (size_t i = 0; i < n; ++i)
      next[i] = hits[i] / weight[i];
    for (const GlueBond &bond : bonds) {
      if (bond.params.iterations <= round)
        continue;
      const float r = std::clamp(bond.params.rate, 0.0f, 1.0f);
      const size_t a = static_cast<size_t>(bond.a),
                   b = static_cast<size_t>(bond.b);
      next[b] += hits[a] * r / weight[a];
      next[a] += hits[b] * r / weight[b];
    }
    hits.swap(next);
  }
}

// One step of `dt` seconds: the bonds' impacts decay, take the propagated
// hits, and those over their strength are listed in `broken` (indices into
// `bonds`). `hits` (per node) is consumed. A bond with a < 0 or b < 0 is
// ignored.
inline void glueAdvance(std::vector<GlueBond> &bonds, std::vector<float> hits,
                        float dt, std::vector<int> &broken) {
  broken.clear();
  // Node counts in `bonds` must fit `hits`.
  // Kept between calls: no allocation.
  thread_local std::vector<GlueBond> live;
  thread_local std::vector<int> index;
  live.clear();
  index.clear();
  live.reserve(bonds.size());
  index.reserve(bonds.size());
  for (size_t i = 0; i < bonds.size(); ++i) {
    const GlueBond &bond = bonds[i];
    if (bond.a < 0 || bond.b < 0 ||
        static_cast<size_t>(bond.a) >= hits.size() ||
        static_cast<size_t>(bond.b) >= hits.size())
      continue;
    live.push_back(bond);
    index.push_back(static_cast<int>(i));
  }
  glueSpread(live, hits);
  for (size_t k = 0; k < live.size(); ++k) {
    GlueBond &bond = bonds[static_cast<size_t>(index[k])];
    const float hl = bond.params.halfLife;
    const float decay = hl > 0.0f && dt > 0.0f ? std::exp2(-dt / hl)
                                               : (hl > 0.0f ? 1.0f : 0.0f);
    bond.impact = bond.impact * decay + hits[static_cast<size_t>(bond.a)] +
                  hits[static_cast<size_t>(bond.b)];
    if (bond.params.strength >= 0.0f && std::isfinite(bond.params.strength) &&
        bond.impact > bond.params.strength / kGlueStrengthScale)
      broken.push_back(index[k]);
  }
}

} // namespace b3dop
