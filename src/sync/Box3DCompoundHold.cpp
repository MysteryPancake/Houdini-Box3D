// Box3DCompoundHold.cpp
//
// Pieces freed from a compound for a fast body coming through
// (`holdCompoundsForApproach`), the bridges that keep them joined to what is
// still in it, and when the holds expire.
#include "Box3DCompoundInternal.h"
#include "Box3DCompoundSync.h"
#include "Box3DHoudiniUtil.h"

#include "Box3DJointSync.h"
#include "Box3DState.h"
#include "core/Box3DCompound.h"
#include "core/Box3DDebug.h"
#include "core/Box3DMath.h"
#include "core/Box3DShapes.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace compoundsync;

namespace compoundsync {

// Destroys the bridges at the compound's pieces in `members` (null: all of
// them) and forgets them. A bridge to a piece that leaves, or all of them when
// the compound goes.
void dropBridges(Box3DSharedState &st, Box3DCompound &c,
                 const std::unordered_set<BodyHandle> *members) {
  for (auto it = c.bridges.begin(); it != c.bridges.end();) {
    if (members && !members->count(it->member)) {
      ++it;
      continue;
    }
    st.world.destroyJoint(it->handle);
    it = c.bridges.erase(it);
  }
}

// Bridges whose bond's joint is gone (the bond broke, its network went) are
// destroyed and forgotten.
void pruneBridges(Box3DSharedState &st) {
  for (auto &c : st.compounds)
    for (auto it = c->bridges.begin(); it != c->bridges.end();) {
      if (st.world.isJointValid(it->original) &&
          st.world.isJointValid(it->handle)) {
        ++it;
        continue;
      }
      st.world.destroyJoint(it->handle);
      it = c->bridges.erase(it);
      st.bridgesDirty = true;
    }
}

} // namespace compoundsync

namespace {

// A weld from the free piece `other` to the compound's body for the Glue bond
// `e` between it and the compound's piece `m`: at the piece's place in the
// compound and with the bond's own frames, so it holds what the bond's parked
// weld would. `have`: the bonds that have a bridge. False if none was made.
bool bridgeBond(Box3DSharedState &st, Box3DCompound &c,
                const Box3DCompoundMember &m, const Box3DJointEntry &e,
                BodyHandle other,
                std::unordered_set<b3dop::JointHandle> &have) {
  b3dop::Box3DWorld &w = st.world;
  if (!e.handle || e.broken || have.count(e.handle) ||
      !w.isJointValid(e.handle))
    return false;
  const bool memberIsA = e.bodyA == m.body;
  if (!memberIsA && e.bodyB != m.body)
    return false;
  b3dop::JointFrame fa, fb;
  if (!w.jointLocalFrames(e.handle, fa, fb))
    return false;
  const b3dop::JointFrame &onFree = memberIsA ? fb : fa;
  const b3dop::JointFrame &inMember = memberIsA ? fa : fb;
  // The frame in the compound's body: the piece's place in it, then the bond's
  // frame in the piece.
  b3dop::JointFrame onBody;
  b3dop::frameInCompound(m.lp, m.lq, inMember.pos, inMember.rot, onBody.pos,
                         onBody.rot);
  const b3dop::JointHandle h = w.createJoint(e.type, other, c.body, onFree,
                                             onBody, e.settings, nullptr, true);
  if (!h)
    return false;
  Box3DBridge b;
  b.original = e.handle;
  b.handle = h;
  b.member = m.body;
  c.bridges.push_back(b);
  have.insert(e.handle);
  return true;
}

} // namespace

namespace compoundsync {

// A bridge for each Glue bond between a piece of the compound and a piece that
// is not in a compound (held out). All the compound's pieces are looked at:
// for after the groups were worked out again.
void bridgeBoundaries(Box3DSharedState &st, Box3DCompound &c) {
  std::unordered_set<b3dop::JointHandle> have;
  for (const Box3DBridge &b : c.bridges)
    have.insert(b.original);
  for (const Box3DCompoundMember &m : c.members) {
    const auto n = st.glueNodeOf.find(m.body);
    if (n == st.glueNodeOf.end())
      continue;
    for (Box3DJointEntry *e :
         st.glueNodes[static_cast<size_t>(n->second)].bonds) {
      const BodyHandle other = e->bodyA == m.body ? e->bodyB : e->bodyA;
      // Inside the compound (or in another one): nothing to hold.
      if (!other || st.compoundOf.count(other))
        continue;
      bridgeBond(st, c, m, *e, other, have);
    }
  }
}

} // namespace compoundsync

namespace {

// The same for the pieces just taken out of the compound (`released`): only
// their bonds are looked at.
void bridgeReleased(Box3DSharedState &st, Box3DCompound &c,
                    const std::vector<BodyHandle> &released) {
  std::unordered_set<b3dop::JointHandle> have;
  for (const Box3DBridge &b : c.bridges)
    have.insert(b.original);
  for (const BodyHandle piece : released) {
    const auto n = st.glueNodeOf.find(piece);
    if (n == st.glueNodeOf.end())
      continue;
    for (Box3DJointEntry *e :
         st.glueNodes[static_cast<size_t>(n->second)].bonds) {
      if (e->bodyA != piece && e->bodyB != piece)
        continue;
      const auto in =
          st.compoundOf.find(e->bodyA == piece ? e->bodyB : e->bodyA);
      if (in == st.compoundOf.end() || in->second.first != &c)
        continue;
      bridgeBond(st, c, c.members[static_cast<size_t>(in->second.second)], *e,
                 piece, have);
    }
  }
}

} // namespace

void pruneCompoundBridges(Box3DSharedState &st) {
  if (!st.compounds.empty())
    pruneBridges(st);
}

void releaseExpiredHolds(Box3DSharedState &st, float frameDt) {
  st.glueClock += static_cast<double>(frameDt);
  if (st.glueHold.empty())
    return;
  const bool let = st.glueHold.expire(st.glueClock);
  if (let)
    st.groupsRecheck = true; // the pieces can merge again
}

namespace {

void releasePieces(Box3DSharedState &st, Box3DCompound &c,
                   const std::vector<int> &cid, size_t leaving, double until,
                   std::vector<b3dop::BodyMove> &moves,
                   std::vector<int> &nodes);

// Takes the pieces of `c` out of it (releasePieces) that reach the box
// [lo, hi] grown by `halo`: a piece counts when its centre of mass is within
// its full bounds diagonal of the box (the centre can be at a corner).
void releaseWithin(Box3DSharedState &st, Box3DCompound &c, const double lo[3],
                   const double hi[3], double halo, double until,
                   std::vector<b3dop::BodyMove> &moves,
                   std::vector<int> &nodes) {
  // A piece gone or edited: syncCompounds dissolves the compound.
  if (resolveMembers(st, c))
    return;
  Pose cp;
  if (!bodyPose(st.world, c.body, cp))
    return;
  std::vector<int> cid(c.members.size(), 0); // 0: stays, 1: leaves
  size_t leaving = 0;
  for (size_t k = 0; k < c.members.size(); ++k) {
    const Box3DCompoundMember &m = c.members[k];
    if (!m.entry)
      continue;
    double com[3];
    centreOf(placed(cp, m), m, com);
    const double reach = halo + 2.0 * static_cast<double>(m.radius);
    bool inside = true;
    for (int i = 0; i < 3; ++i)
      inside = inside && com[i] >= lo[i] - reach && com[i] <= hi[i] + reach;
    if (inside) {
      cid[k] = 1;
      ++leaving;
    }
  }
  if (leaving == 0)
    return;
  releasePieces(st, c, cid, leaving, until, moves, nodes);
}

// Takes the pieces of `c` that a body is about to plow through out of it and
// holds them until `until`: the body's bounds swept along its velocity relative
// to the compound until its centre leaves the compound's bounds
// (pathThroughBounds), grown by kGlueHoldHaloScale times its size. They are
// enabled where the compound has them (a move event each in `moves`), and the
// compound keeps the rest as one body, joined to them by bridges. The path is
// the whole way through, not a time ahead: a piece left in the compound behind
// the freed ones is rigid and throws the body back. Nothing is done for a body
// that is not heading into or through the compound. If fewer than two pieces
// would stay, all of it is taken apart. `nodes` collects the glue nodes whose
// bonds changed sides.
void releaseNear(Box3DSharedState &st, Box3DCompound &c,
                 const b3dop::Box3DWorld::Approach &a, double until,
                 std::vector<b3dop::BodyMove> &moves, std::vector<int> &nodes) {
  double lo[3], hi[3];
  if (!st.world.boundsByHandle(c.body, lo, hi))
    return;
  double pathLo[3], pathHi[3];
  if (!b3dop::pathThroughBounds(a.moverCom, a.relLin,
                                static_cast<double>(a.moverHalf), lo, hi,
                                pathLo, pathHi))
    return;
  const double halo =
      static_cast<double>(b3dop::kGlueHoldHaloScale) * a.moverHalf;
  // Nothing of the compound within reach of the path: nothing to free, and the
  // pieces need not be looked at.
  for (int i = 0; i < 3; ++i)
    if (pathHi[i] + halo < lo[i] || pathLo[i] - halo > hi[i])
      return;
  releaseWithin(st, c, pathLo, pathHi, halo, until, moves, nodes);
}

// Takes the pieces of `c` marked 1 in `cid` out of it and holds them until
// `until` (`leaving` of them); all of it is taken apart when fewer than two
// would stay.
void releasePieces(Box3DSharedState &st, Box3DCompound &c,
                   const std::vector<int> &cid, size_t leaving, double until,
                   std::vector<b3dop::BodyMove> &moves,
                   std::vector<int> &nodes) {
  const auto hold = [&](const Box3DCompoundMember &m) {
    st.glueHold.hold(m.body, until);
    const auto n = st.glueNodeOf.find(m.body);
    if (n != st.glueNodeOf.end())
      nodes.push_back(n->second);
  };
  const auto takeAllApart = [&]() {
    for (const Box3DCompoundMember &m : c.members)
      hold(m);
    const Box3DCompound *raw = &c;
    dissolve(st, c, &moves);
    eraseCompound(st, raw);
  };
  if (c.members.size() - leaving < 2) {
    takeAllApart();
    return;
  }
  std::vector<BodyHandle> released;
  released.reserve(leaving);
  for (size_t k = 0; k < c.members.size(); ++k)
    if (cid[k] == 1) {
      hold(c.members[k]);
      released.push_back(c.members[k].body);
    }
  if (shrink(st, c, cid, 0, &moves))
    bridgeReleased(st, c, released);
  else
    takeAllApart();
}

// Takes the pieces of the moving compound `c` that are about to reach `target`
// (a static body, the ground, or any body that is not of its group) out of it:
// those within reach of where the compound's bounds, swept along its velocity
// for `lead` seconds, overlap the target's. A compound is rigid, so it would
// take the whole hit through the pieces that touch and be stopped before its
// bonds could break; free and welded, the pieces near the contact give way as
// they do on welds.
void releaseTowards(Box3DSharedState &st, Box3DCompound &c, BodyHandle target,
                    const float lin[3], float lead, double until,
                    std::vector<b3dop::BodyMove> &moves,
                    std::vector<int> &nodes) {
  double lo[3], hi[3], tlo[3], thi[3];
  if (!st.world.boundsByHandle(c.body, lo, hi) ||
      !st.world.boundsByHandle(target, tlo, thi))
    return;
  double sweptLo[3], sweptHi[3];
  for (int i = 0; i < 3; ++i) {
    const double d = static_cast<double>(lin[i]) * static_cast<double>(lead);
    sweptLo[i] = lo[i] + std::min(d, 0.0);
    sweptHi[i] = hi[i] + std::max(d, 0.0);
  }
  double overLo[3], overHi[3];
  for (int i = 0; i < 3; ++i) {
    overLo[i] = std::max(sweptLo[i], tlo[i]);
    overHi[i] = std::min(sweptHi[i], thi[i]);
    if (overLo[i] > overHi[i])
      return;
  }
  releaseWithin(st, c, overLo, overHi, 0.0, until, moves, nodes);
}

// The impulse a bond takes to break: the Glue's Strength over
// kGlueStrengthScale (a body that cannot deliver that in about a frame cannot
// break it). Infinity when it cannot break.
float bondBreakImpulse(const Box3DJointEntry &e) {
  constexpr float kNever = std::numeric_limits<float>::infinity();
  if (!e.extra)
    return kNever;
  const float strength = e.extra->glueParams.strength;
  return strength >= 0.0f && std::isfinite(strength)
             ? strength / b3dop::kGlueStrengthScale
             : kNever;
}

// The weakest break impulse (bondBreakImpulse) of the bonds of one glue node;
// infinity when none can break.
float nodeWeakest(const Box3DSharedState &st, int node) {
  float least = std::numeric_limits<float>::infinity();
  for (const Box3DJointEntry *e : st.glueNodes[static_cast<size_t>(node)].bonds)
    least = std::min(least, bondBreakImpulse(*e));
  return least;
}

// The weakest break impulse of the bonds of a compound's pieces.
float compoundWeakest(Box3DSharedState &st, Box3DCompound &c) {
  float least = std::numeric_limits<float>::infinity();
  for (const Box3DCompoundMember &m : c.members) {
    const auto n = st.glueNodeOf.find(m.body);
    if (n != st.glueNodeOf.end())
      least = std::min(least, nodeWeakest(st, n->second));
  }
  return least;
}

// The weakest break impulse of any bond: a body that cannot deliver that much
// cannot break anything anywhere. Infinity when no bond can break. Worked out
// once per frame, or again when the bonds changed.
float weakestBreak(Box3DSharedState &st) {
  if (st.glueMinBreakClock == st.glueClock &&
      st.glueMinBreakSignature == st.glueSignature)
    return st.glueMinBreak;
  float least = std::numeric_limits<float>::infinity();
  for (const Box3DJointEntry *e : st.glueBonds)
    least = std::min(least, bondBreakImpulse(*e));
  st.glueMinBreak = least;
  st.glueMinBreakClock = st.glueClock;
  st.glueMinBreakSignature = st.glueSignature;
  return least;
}

} // namespace

void holdCompoundsForApproach(Box3DSharedState &st,
                              const std::vector<b3dop::BodyMove> &seen,
                              std::vector<b3dop::BodyMove> &moves) {
  // Held pieces stay to be looked at after the last compound is gone.
  if (!st.glueCompounds || st.glueBonds.empty() || seen.empty() ||
      (st.compounds.empty() && st.glueHold.empty()))
    return;
  // What a body has to be able to deliver to break any bond: nothing smaller
  // is looked at (and with no bond that can break there is nothing to do).
  const float floorBreak = weakestBreak(st);
  if (!std::isfinite(floorBreak))
    return;
  refreshGlueGroups(st);
  thread_local std::vector<BodyHandle> movers;
  thread_local std::vector<b3dop::Box3DWorld::Approach> approaches;
  movers.clear();
  approaches.clear();
  // Only bodies fast enough to matter.
  const float minSpeed2 = b3dop::kGlueHoldMinSpeed * b3dop::kGlueHoldMinSpeed;
  for (const b3dop::BodyMove &mv : seen) {
    const float s2 =
        mv.lin[0] * mv.lin[0] + mv.lin[1] * mv.lin[1] + mv.lin[2] * mv.lin[2];
    if (s2 >= minSpeed2)
      movers.push_back(mv.handle);
  }
  if (movers.empty())
    return;
  // A piece can be in `seen` twice (released ahead of the step's own event).
  std::sort(movers.begin(), movers.end());
  movers.erase(std::unique(movers.begin(), movers.end()), movers.end());
  // Two frames at least: the pieces must be free before the step the body
  // touches the compound in (a release in that frame still threw it back).
  const float lead = std::max(b3dop::kGlueHoldLeadSeconds, 2.0f * st.stepDt);
  st.world.approachingBodies(movers.data(), movers.size(), lead,
                             b3dop::kGlueHoldMinSpeed, floorBreak, approaches);
  if (approaches.empty())
    return;

  std::unordered_map<BodyHandle, Box3DCompound *> byBody;
  byBody.reserve(st.compounds.size());
  st.compoundGroup.clear();
  for (auto &c : st.compounds) {
    byBody[c->body] = c.get();
    st.compoundGroup[c->body] = compoundGlueGroup(st, *c);
  }
  const double until =
      st.glueClock + static_cast<double>(b3dop::kGlueHoldSeconds);
  const auto neighbours = [&st](uint64_t piece, const auto &visit) {
    const auto n = st.glueNodeOf.find(piece);
    if (n == st.glueNodeOf.end())
      return;
    for (const BodyHandle partner :
         st.glueNodes[static_cast<size_t>(n->second)].partners)
      visit(partner);
  };
  struct Target {
    Box3DCompound *compound;
    const b3dop::Box3DWorld::Approach *by;
  };
  std::vector<Target> take;
  for (const b3dop::Box3DWorld::Approach &a : approaches) {
    const auto c = byBody.find(a.target);
    const auto node = st.glueNodeOf.find(a.target);
    const bool held = st.glueHold.held(a.target) && node != st.glueNodeOf.end();
    if (c == byBody.end() && !held)
      continue;
    // A piece of the structure itself is not the body that has to be let
    // through (a body of another group is, glued or not).
    const int targetGroup =
        c != byBody.end() ? st.compoundGroup[a.target]
                          : st.glueGroup[static_cast<size_t>(node->second)];
    if (targetGroup >= 0 && glueSameGroup(st, a.mover, targetGroup))
      continue;
    // What it can deliver to the bonds there, from its motion relative to the
    // structure. A kinematic body (mass 0) is not slowed by anything it hits.
    const float breakAt = c != byBody.end() ? compoundWeakest(st, *c->second)
                                            : nodeWeakest(st, node->second);
    if (!std::isfinite(breakAt))
      continue; // no bond there can break
    const double relSpeed =
        std::sqrt(static_cast<double>(a.relLin[0]) * a.relLin[0] +
                  static_cast<double>(a.relLin[1]) * a.relLin[1] +
                  static_cast<double>(a.relLin[2]) * a.relLin[2]);
    if (a.moverMass > 0.0f &&
        !(static_cast<double>(a.moverMass) * relSpeed >= breakAt))
      continue;
    if (c != byBody.end())
      take.push_back({c->second, &a});
    else
      st.glueHold.refresh(a.target, until, neighbours);
  }
  // A compound heading into something that is not of its group (the ground,
  // another body): the pieces about to touch it are freed too.
  for (const b3dop::Box3DWorld::Approach &a : approaches) {
    const auto m = byBody.find(a.mover);
    if (m == byBody.end())
      continue;
    const int group = st.compoundGroup[a.mover];
    if (group >= 0 && glueSameGroup(st, a.target, group))
      continue;
    const float breakAt = compoundWeakest(st, *m->second);
    const double speed =
        std::sqrt(static_cast<double>(a.relLin[0]) * a.relLin[0] +
                  static_cast<double>(a.relLin[1]) * a.relLin[1] +
                  static_cast<double>(a.relLin[2]) * a.relLin[2]);
    if (!std::isfinite(breakAt) ||
        !(static_cast<double>(a.moverMass) * speed >= breakAt))
      continue;
    const auto live =
        std::find_if(st.compounds.begin(), st.compounds.end(),
                     [&m](const std::unique_ptr<Box3DCompound> &p) {
                       return p.get() == m->second;
                     });
    if (live == st.compounds.end())
      continue;
    std::vector<int> moved;
    releaseTowards(st, *m->second, a.target, a.moverLin, lead, until, moves,
                   moved);
    if (!moved.empty())
      markCompoundBonds(st, &moved);
  }

  std::vector<int> nodes;
  for (const Target &t : take) {
    // An earlier body may have taken all of it apart already.
    const auto live =
        std::find_if(st.compounds.begin(), st.compounds.end(),
                     [&t](const std::unique_ptr<Box3DCompound> &p) {
                       return p.get() == t.compound;
                     });
    if (live != st.compounds.end())
      releaseNear(st, *t.compound, *t.by, until, moves, nodes);
  }
  if (!nodes.empty())
    markCompoundBonds(st, &nodes);
}
