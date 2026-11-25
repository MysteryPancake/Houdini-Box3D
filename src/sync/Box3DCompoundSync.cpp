// Box3DCompoundSync.cpp
//
// Glued groups as compound bodies: see Box3DCompoundSync.h. The maths is in
// core/Box3DCompound.h; this file keeps the groups in step with the bodies:
// merging, dissolving and shrinking compounds, and `syncCompounds`.
#include "Box3DCompoundSync.h"
#include "Box3DCompoundInternal.h"
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

bool glueSameGroup(const Box3DSharedState &st, b3dop::BodyHandle body,
                   int group) {
  const auto n = st.glueNodeOf.find(body);
  if (n != st.glueNodeOf.end())
    return static_cast<size_t>(n->second) < st.glueGroup.size() &&
           st.glueGroup[static_cast<size_t>(n->second)] == group;
  const auto g = st.compoundGroup.find(body);
  return g != st.compoundGroup.end() && g->second == group;
}

int compoundGlueGroup(const Box3DSharedState &st, const Box3DCompound &c) {
  for (const Box3DCompoundMember &m : c.members) {
    const auto n = st.glueNodeOf.find(m.body);
    if (n != st.glueNodeOf.end() &&
        static_cast<size_t>(n->second) < st.glueGroup.size())
      return st.glueGroup[static_cast<size_t>(n->second)];
  }
  return -1;
}

namespace compoundsync {

bool bodyPose(const b3dop::Box3DWorld &w, BodyHandle h, Pose &out) {
  return w.getTransformByHandle(h, out.p, out.q);
}

// Where a piece is when its compound body is at `body`.
Pose placed(const Pose &body, const Box3DCompoundMember &m) {
  Pose out;
  const double lp[3] = {m.lp[0], m.lp[1], m.lp[2]};
  double r[3];
  b3dop::quatRotate(body.q, lp, r);
  for (int i = 0; i < 3; ++i)
    out.p[i] = body.p[i] + r[i];
  b3dop::quatMul(body.q, m.lq, out.q);
  b3dop::sanitizeQuat(out.q);
  return out;
}

// The centre of mass of a piece placed at `pp`.
void centreOf(const Pose &pp, const Box3DCompoundMember &m, double out[3]) {
  const double c[3] = {m.centre[0], m.centre[1], m.centre[2]};
  double r[3];
  b3dop::quatRotate(pp.q, c, r);
  for (int i = 0; i < 3; ++i)
    out[i] = pp.p[i] + r[i];
}

// A body's centre of mass and velocity, as the rigid body pieceVelocity needs.
bool bodyMotion(const b3dop::Box3DWorld &w, BodyHandle h, b3dop::MergedBody &mb,
                bool &awake) {
  b3dop::BodyState s;
  if (!w.bodyStateByHandle(h, s))
    return false;
  for (int i = 0; i < 3; ++i) {
    mb.com[i] = s.com[i];
    mb.lin[i] = s.lin[i];
    mb.ang[i] = s.ang[i];
  }
  mb.ok = true;
  awake = s.awake;
  return true;
}

// The settings that belong to a body, not a shape: pieces that differ here
// cannot share one.
bool sameBodyLevel(const b3dop::BodyProps &a, const b3dop::BodyProps &b) {
  using b3dop::sameFloat;
  return sameFloat(a.linearDamping, b.linearDamping) &&
         sameFloat(a.angularDamping, b.angularDamping) &&
         sameFloat(a.gravityScale, b.gravityScale) &&
         sameFloat(a.sleepThreshold, b.sleepThreshold) &&
         sameFloat(a.ccdThreshold, b.ccdThreshold) &&
         a.contactRecycling == b.contactRecycling &&
         a.enableSleep == b.enableSleep && a.fastObject == b.fastObject &&
         a.fastRotation == b.fastRotation;
}

} // namespace compoundsync

namespace {

// The glue nodes that can be reached from `seeds` through the live bonds
// (their nodes' partners), grouped by component: `of` is a node's component
// number, `nodes` the nodes of each component. Work in proportion to the
// components, not to the scene.
struct Reach {
  std::unordered_map<int, int> of;
  std::vector<std::vector<int>> nodes;
  int componentOf(int node) const {
    const auto it = of.find(node);
    return it == of.end() ? -1 : it->second;
  }
};

} // namespace

namespace compoundsync {

// Whether the piece of a glue node is held out of its compound
// (holdCompoundsForApproach): held pieces are not part of any group, they and
// their bonds are left out of the grouping.
bool heldNode(const Box3DSharedState &st, int node) {
  return !st.glueHold.empty() &&
         st.glueHold.held(st.glueNodes[static_cast<size_t>(node)].body);
}

} // namespace compoundsync

namespace {

// Whether glue nodes `a` and `b` are still joined through live bonds, found by
// searching from both at once, a node at a time, until the searches meet. In a
// mesh of bonds (a wall) the neighbours of a broken bond are joined by a short
// way round and the searches meet within a few nodes; when one side is used up
// without meeting, or the search grows past `kMaxNodes`, the answer is false
// (the caller works the components out the full way). Held pieces are left out
// as reachFrom leaves them.
bool stillJoined(const Box3DSharedState &st, int a, int b) {
  constexpr size_t kMaxNodes = 512;
  if (a == b)
    return true;
  if (heldNode(st, a) || heldNode(st, b))
    return false;
  std::unordered_map<int, char> mark; // 1: reached from a, 2: from b
  std::vector<int> queue[2] = {{a}, {b}};
  size_t head[2] = {0, 0};
  mark[a] = 1;
  mark[b] = 2;
  for (int side = 0;; side ^= 1) {
    if (head[side] >= queue[side].size()) {
      // This side is used up. Without meeting the other, they are apart.
      return false;
    }
    const int n = queue[side][head[side]++];
    for (const BodyHandle h : st.glueNodes[static_cast<size_t>(n)].partners) {
      const auto it = st.glueNodeOf.find(h);
      if (it == st.glueNodeOf.end() || heldNode(st, it->second))
        continue;
      const auto ins = mark.emplace(it->second, static_cast<char>(side + 1));
      if (!ins.second) {
        if (ins.first->second != side + 1)
          return true; // reached by the other search
        continue;
      }
      queue[side].push_back(it->second);
    }
    if (mark.size() > kMaxNodes)
      return false;
  }
}

Reach reachFrom(const Box3DSharedState &st, const std::vector<int> &seeds) {
  Reach r;
  std::vector<int> stack;
  for (const int seed : seeds) {
    if (r.of.count(seed) || heldNode(st, seed))
      continue;
    const int id = static_cast<int>(r.nodes.size());
    r.nodes.emplace_back();
    r.of.emplace(seed, id);
    stack.push_back(seed);
    while (!stack.empty()) {
      const int n = stack.back();
      stack.pop_back();
      r.nodes[static_cast<size_t>(id)].push_back(n);
      for (const BodyHandle h : st.glueNodes[static_cast<size_t>(n)].partners) {
        const auto it = st.glueNodeOf.find(h);
        if (it != st.glueNodeOf.end() && !heldNode(st, it->second) &&
            r.of.emplace(it->second, id).second)
          stack.push_back(it->second);
      }
    }
  }
  return r;
}

} // namespace

namespace compoundsync {

// A piece the compound can carry: a plain dynamic body whose mass and inertia
// come from its shapes.
bool eligible(const Box3DBodyMirror &e) {
  const b3dop::BodyProps &p = e.props;
  return e.handle && !e.wholeObject && e.dynamic() && !e.animated &&
         !e.shapeless && !e.meshShapes && p.enabled && p.inertiaScale == 1.0f &&
         !p.customInertia && !p.boundsInertia;
}

uint64_t keyOf(const Box3DBodyMirror &e) {
  return b3dop::makeBodyKey(e.objectId, e.pointId);
}

} // namespace compoundsync

namespace {

// A parked piece's transform is not set every step (expandCompoundMoves):
// the world brings it up to date from here when something reads it.
void installPoseRefresher(Box3DSharedState &st) {
  if (st.world.hasPoseRefresher())
    return;
  Box3DSharedState *const sp = &st;
  st.world.setPoseRefresher([sp](BodyHandle h) {
    const auto it = sp->compoundOf.find(h);
    if (it == sp->compoundOf.end())
      return;
    const Box3DCompound &c = *it->second.first;
    Pose cp;
    if (!bodyPose(sp->world, c.body, cp))
      return;
    const Pose pp =
        placed(cp, c.members[static_cast<size_t>(it->second.second)]);
    sp->world.setTransformByHandle(h, pp.p, pp.q);
  });
}

} // namespace

namespace compoundsync {

// A move event for a piece of `entry` put at `pp`.
b3dop::BodyMove moveAt(Box3DBodyMirror &entry, BodyHandle body, const Pose &pp,
                       bool asleep, const float lin[3], const float ang[3]) {
  b3dop::BodyMove mv;
  mv.userData = &entry;
  mv.handle = body;
  b3dop::copy3(mv.pos, pp.p);
  b3dop::copy4(mv.rot, pp.q);
  if (lin)
    b3dop::copy3(mv.lin, lin);
  if (ang)
    b3dop::copy3(mv.ang, ang);
  mv.fellAsleep = asleep;
  return mv;
}

} // namespace compoundsync

namespace {

// Takes a piece out of its compound: enabled (a piece the user disabled stays
// disabled) and, if the compound's pose `cp` is known, put where the compound
// has it, moving as `motion` says (asleep if the compound is not `awake`) and
// given a move event in `moves` (optional, for a split in the middle of a
// frame). A piece another node edited keeps the pose the edit gave it, and the
// velocity of its point.
void releasePiece(b3dop::Box3DWorld &w, const Box3DCompoundMember &m,
                  const Pose *cp, bool awake, const b3dop::MergedBody &motion,
                  std::vector<b3dop::BodyMove> *moves) {
  Box3DBodyMirror &e = *m.entry;
  if (e.props.enabled)
    w.setEnabledByHandle(m.body, true);
  if (e.glueEdited) {
    w.setVelocityByHandle(m.body, e.lin, e.ang);
  } else if (cp) {
    const Pose pp = placed(*cp, m);
    w.setTransformByHandle(m.body, pp.p, pp.q);
    float lin[3] = {0, 0, 0}, ang[3] = {0, 0, 0};
    if (awake) {
      double com[3];
      centreOf(pp, m, com);
      b3dop::pieceVelocity(motion, com, lin, ang);
      w.setVelocityByHandle(m.body, lin, ang);
    } else {
      w.sleepByHandle(m.body);
    }
    if (moves)
      moves->push_back(moveAt(e, m.body, pp, !awake, lin, ang));
  }
  e.inCompound = false;
  e.glueEdited = false;
}

// The component with the most pieces (the lowest number on a tie), -1 if there
// are none; `best` is its piece count.
int largestComponent(const std::unordered_map<int, size_t> &count,
                     size_t &best) {
  int stay = -1;
  best = 0;
  for (const auto &kv : count)
    if (kv.second > best || (kv.second == best && kv.first < stay)) {
      stay = kv.first;
      best = kv.second;
    }
  return stay;
}

} // namespace

namespace compoundsync {

// Enables the pieces where the compound left them and destroys the compound
// body. Pieces whose entry is gone (`entry` null) were destroyed with their
// points. `moves` (optional): gets a move event for each piece, for a release
// in the middle of a frame.
void dissolve(Box3DSharedState &st, Box3DCompound &c,
              std::vector<b3dop::BodyMove> *moves) {
  b3dop::Box3DWorld &w = st.world;
  dropBridges(st, c);
  Pose cp;
  b3dop::MergedBody mb;
  bool awake = false;
  const bool have = bodyPose(w, c.body, cp) && bodyMotion(w, c.body, mb, awake);
  for (Box3DCompoundMember &m : c.members) {
    st.compoundOf.erase(m.body);
    if (m.entry)
      releasePiece(w, m, have ? &cp : nullptr, awake, mb, moves);
  }
  w.destroyBody(c.body);
}

// Points each piece at its entry now (null: its point, and body, are gone).
// True if some piece is gone, was edited, or differs from what was merged.
bool resolveMembers(Box3DSharedState &st, Box3DCompound &c) {
  bool changed = false;
  for (Box3DCompoundMember &m : c.members) {
    Box3DBodyMirror *e = st.mirror.find(m.key);
    if (!e || e->handle != m.body || !st.isLive(*e)) {
      m.entry = nullptr;
      changed = true;
      continue;
    }
    m.entry = e;
    if (e->glueEdited || !eligible(*e) || e->shapeSig != m.shapeSig ||
        !(e->props == m.props))
      changed = true;
  }
  return changed;
}

// Which Glue bonds join two pieces of one compound. `nodes`: only the bonds of
// those glue nodes (the components a change was in); null: all.
void markCompoundBonds(Box3DSharedState &st, const std::vector<int> *nodes) {
  auto mark = [&st](Box3DJointEntry *e) {
    const auto a = st.compoundOf.find(e->bodyA);
    const auto b = st.compoundOf.find(e->bodyB);
    e->inCompound = a != st.compoundOf.end() && b != st.compoundOf.end() &&
                    a->second.first == b->second.first;
  };
  if (!nodes) {
    for (Box3DJointEntry *e : st.glueBonds)
      mark(e);
    return;
  }
  for (const int n : *nodes)
    for (Box3DJointEntry *e : st.glueNodes[static_cast<size_t>(n)].bonds)
      mark(e);
}

// Takes the compound out of the list (its body and pieces are dissolve's).
void eraseCompound(Box3DSharedState &st, const Box3DCompound *c) {
  const auto it = std::find_if(
      st.compounds.begin(), st.compounds.end(),
      [c](const std::unique_ptr<Box3DCompound> &p) { return p.get() == c; });
  if (it != st.compounds.end())
    st.compounds.erase(it);
}

} // namespace compoundsync

namespace {

void dissolveAll(Box3DSharedState &st) {
  for (auto &c : st.compounds) {
    resolveMembers(st, *c);
    dissolve(st, *c);
  }
  st.compounds.clear();
  st.compoundOf.clear();
}

} // namespace

namespace compoundsync {

// Takes the pieces whose component is not `mainId` out of the compound and
// keeps the body (and the contacts of the pieces that stay): they are enabled
// where the compound has them, their shapes destroyed, and the body keeps the
// momentum of the pieces that stay, which still move as one rigid body. False,
// with nothing changed, if the compound's body cannot be read.
// `leave` and `moves` as for dissolve: the pieces that leave take `leave`'s
// motion, the ones that stay keep the compound's now.
bool shrink(Box3DSharedState &st, Box3DCompound &c, const std::vector<int> &cid,
            int mainId, std::vector<b3dop::BodyMove> *moves) {
  b3dop::Box3DWorld &w = st.world;
  Pose cp;
  b3dop::MergedBody was;
  bool awake = false;
  if (!bodyPose(w, c.body, cp) || !bodyMotion(w, c.body, was, awake))
    return false;

  {
    std::unordered_set<BodyHandle> leaving;
    for (size_t k = 0; k < c.members.size(); ++k)
      if (cid[k] != mainId)
        leaving.insert(c.members[k].body);
    dropBridges(st, c, &leaving);
  }
  std::vector<Box3DCompoundMember> kept;
  std::vector<std::pair<int, int>> doomed;
  kept.reserve(c.members.size());
  for (size_t k = 0; k < c.members.size(); ++k) {
    Box3DCompoundMember &m = c.members[k];
    if (cid[k] == mainId) {
      kept.push_back(m);
      continue;
    }
    doomed.emplace_back(m.shapeFirst, m.shapeCount);
    st.compoundOf.erase(m.body);
    releasePiece(w, m, &cp, awake, was, moves);
  }
  w.destroyShapeRangesByHandle(c.body, doomed);

  int first = 0;
  for (size_t k = 0; k < kept.size(); ++k) {
    kept[k].shapeFirst = first;
    first += kept[k].shapeCount;
    st.compoundOf[kept[k].body] = {&c, static_cast<int>(k)};
  }
  c.members = std::move(kept);
  w.applyMassFromShapes(c.body);

  // The pieces that stay move as a rigid body with the old motion: its
  // centre of mass moved, so its velocity is v + w x (new centre - old centre).
  b3dop::BodyState now;
  if (awake && w.bodyStateByHandle(c.body, now)) {
    float lin[3], ang[3];
    b3dop::pieceVelocity(was, now.com, lin, ang);
    w.setVelocityByHandle(c.body, lin, ang);
  }
  return true;
}

// One compound from `group` (sorted by key, the first is the leader, the
// compound's origin). False, with nothing changed, if Box3D refuses a shape.
bool merge(Box3DSharedState &st, const std::vector<Box3DBodyMirror *> &group) {
  b3dop::Box3DWorld &w = st.world;
  Pose lead;
  if (!bodyPose(w, group[0]->handle, lead))
    return false;

  b3dop::BodySettings bs = b3dop::nodeBodyDefaults();
  bs.type = b3dop::BodySettings::Type::Dynamic;
  bs.props = group[0]->props;
  bs.startAwake = false;
  for (const Box3DBodyMirror *e : group)
    bs.startAwake = bs.startAwake || e->awake;
  const BodyHandle body = w.createBody(bs, lead.p, lead.q);
  if (!body)
    return false;

  auto c = std::make_unique<Box3DCompound>();
  c->body = body;
  std::vector<b3dop::RigidPiece> pieces;
  float conj[4];
  b3dop::quatConj(lead.q, conj);
  int first = 0;
  bool ok = true;
  for (Box3DBodyMirror *e : group) {
    Pose mp;
    float mass = 0, centre[3] = {}, inertia[3] = {}, off[3] = {};
    if (!bodyPose(w, e->handle, mp) ||
        !w.getMassDataByHandle(e->handle, mass, centre, inertia, off)) {
      ok = false;
      break;
    }
    Box3DCompoundMember m;
    m.key = keyOf(*e);
    m.body = e->handle;
    m.entry = e;
    const double d[3] = {mp.p[0] - lead.p[0], mp.p[1] - lead.p[1],
                         mp.p[2] - lead.p[2]};
    double r[3];
    b3dop::quatRotate(conj, d, r);
    for (int i = 0; i < 3; ++i)
      m.lp[i] = static_cast<float>(r[i]);
    b3dop::quatMul(conj, mp.q, m.lq);
    b3dop::sanitizeQuat(m.lq);
    const int n = w.copyShapesByHandle(e->handle, body, m.lp, m.lq);
    if (n <= 0) {
      ok = false;
      break;
    }
    std::copy(centre, centre + 3, m.centre);
    m.mass = mass;
    float extent[3];
    if (w.extentByHandle(e->handle, extent))
      m.radius =
          0.5f * std::sqrt(extent[0] * extent[0] + extent[1] * extent[1] +
                           extent[2] * extent[2]);
    m.shapeFirst = first;
    m.shapeCount = n;
    first += n;
    m.props = e->props;
    m.shapeSig = e->shapeSig;
    c->members.push_back(m);

    b3dop::RigidPiece pc;
    const double centre3[3] = {centre[0], centre[1], centre[2]};
    double c3[3];
    b3dop::quatRotate(mp.q, centre3, c3);
    for (int i = 0; i < 3; ++i) {
      pc.pos[i] = mp.p[i] + c3[i];
      pc.rot[i] = mp.q[i];
      pc.inertia[i] = inertia[i];
      pc.inertiaOff[i] = off[i];
    }
    pc.rot[3] = mp.q[3];
    pc.mass = mass;
    w.getVelocityByHandle(e->handle, pc.lin, pc.ang);
    pieces.push_back(pc);
  }
  if (!ok) {
    w.destroyBody(body); // its shapes go with it; the pieces are untouched
    return false;
  }

  w.applyMassFromShapes(body);
  const b3dop::MergedBody mb = b3dop::mergePieces(pieces);
  if (!bs.startAwake)
    w.sleepByHandle(body);
  else if (mb.ok)
    w.setVelocityByHandle(body, mb.lin, mb.ang);

  Box3DCompound *raw = c.get();
  installPoseRefresher(st);
  for (size_t i = 0; i < raw->members.size(); ++i) {
    Box3DCompoundMember &m = raw->members[i];
    w.setEnabledByHandle(m.body, false);
    // Parked where it is: from now on it is brought up to date only when read.
    w.setLazyPose(m.body, true);
    m.entry->inCompound = true;
    m.entry->glueEdited = false;
    st.compoundOf[m.body] = {raw, static_cast<int>(i)};
  }
  st.compounds.push_back(std::move(c));
  return true;
}

} // namespace compoundsync

void queueCompoundIds(Box3DSharedState &st) {
  if (!st.compoundIds || (st.compounds.empty() && st.compoundIdKeys.empty()))
    return;
  const auto queue = [&st](Box3DBodyMirror &entry, int id) {
    Box3DPointWrite w;
    w.entry = &entry;
    w.objectId = entry.objectId;
    w.flags = Box3DPointWrite::kCompound;
    w.compoundId = id;
    st.pointWrites.push_back(w);
    entry.compoundShown = id;
  };
  for (const std::unique_ptr<Box3DCompound> &c : st.compounds) {
    int64_t lowest = std::numeric_limits<int64_t>::max();
    for (const Box3DCompoundMember &m : c->members)
      if (m.entry)
        lowest = std::min(lowest, m.entry->pointId);
    const int id = static_cast<int>(lowest);
    for (const Box3DCompoundMember &m : c->members) {
      if (!m.entry || m.entry->compoundShown == id)
        continue;
      if (m.entry->compoundShown < 0)
        st.compoundIdKeys.push_back(m.key);
      queue(*m.entry, id);
    }
  }
  // Pieces that left their compound show -1 again.
  size_t kept = 0;
  for (const uint64_t key : st.compoundIdKeys) {
    Box3DBodyMirror *entry = st.mirror.find(key);
    if (!entry || entry->compoundShown < 0)
      continue;
    if (!entry->inCompound)
      queue(*entry, -1);
    else
      st.compoundIdKeys[kept++] = key;
  }
  st.compoundIdKeys.resize(kept);
}

namespace {

// The glue groups of pieces this frame and what became of them (syncCompounds).
struct GroupPlan {
  bool partial = false;       // only the components of the touched nodes
  b3dop::GlueComponents comp; // all the nodes' components
  Reach reach;                // or only those of the touched nodes
  std::vector<std::vector<Box3DBodyMirror *>> groups;
  std::vector<char> ok, covered;
  int componentOf(int node) const {
    return partial ? reach.componentOf(node)
                   : comp.of[static_cast<size_t>(node)];
  }
};

// Pieces held out of their compound stay joined to it: the bridges are made
// again when the compounds or the bonds changed, also when nothing is grouped
// again.
void rebuildBridges(Box3DSharedState &st) {
  if (!st.bridgesDirty)
    return;
  st.bridgesDirty = false;
  if (!st.glueHold.empty())
    for (auto &c : st.compounds)
      bridgeBoundaries(st, *c);
}

// 1. Compounds whose pieces changed since they were merged. Only a full
// scan of an object (or a body added or removed) can change a piece's entry,
// settings or shapes; a step with none leaves them as merged, and checking
// every piece of every compound each frame was most of what a held compound
// cost. True if one was dissolved; `membership` is set then.
bool dissolveChangedCompounds(Box3DSharedState &st, bool &membership) {
  bool dissolved = false;
  const bool piecesMayDiffer = st.fullScans > 0 ||
                               st.bodyVersion != st.groupsBodyVersion ||
                               st.groupsRecheck;
  for (size_t i = 0; piecesMayDiffer && i < st.compounds.size();) {
    Box3DCompound &c = *st.compounds[i];
    if (resolveMembers(st, c)) {
      dissolve(st, c);
      st.compounds.erase(st.compounds.begin() + static_cast<std::ptrdiff_t>(i));
      dissolved = true;
      membership = true;
    } else {
      ++i;
    }
  }
  return dissolved;
}

// 2. The groups again, only when the glue graph or the bodies changed. When
// only bonds broke (the lists were taken apart in place), only the components
// those bonds were in are worked out again. True when the compounds and groups
// are settled without working them out (bridges made again as needed).
bool settledWithoutRegrouping(Box3DSharedState &st, bool dissolved) {
  const bool bodiesSame = st.bodyVersion == st.groupsBodyVersion;
  if (!dissolved && !st.groupsRecheck && bodiesSame &&
      st.glueSignature == st.groupsSignature) {
    st.glueTouched.clear();
    rebuildBridges(st);
    return true;
  }
  // Only bonds broke (their two nodes are in glueTouched, a pair each), and
  // each broke inside one compound whose pieces are still joined some other
  // way: the components are as they were, so the compounds and the groups need
  // no work. (A bond that cuts a group in two, or one between pieces that are
  // not in a compound, takes the full way below.)
  if (!dissolved && !st.groupsRecheck && bodiesSame && st.tiedKnown &&
      !st.glueTouched.empty() && st.glueTouched.size() % 2 == 0) {
    bool unchanged = true;
    for (size_t k = 0; unchanged && k + 1 < st.glueTouched.size(); k += 2) {
      const int a = st.glueTouched[k], b = st.glueTouched[k + 1];
      const auto ca =
          st.compoundOf.find(st.glueNodes[static_cast<size_t>(a)].body);
      const auto cb =
          st.compoundOf.find(st.glueNodes[static_cast<size_t>(b)].body);
      unchanged = ca != st.compoundOf.end() && cb != st.compoundOf.end() &&
                  ca->second.first == cb->second.first && stillJoined(st, a, b);
    }
    if (unchanged) {
      st.groupsSignature = st.glueSignature;
      st.glueTouched.clear();
      rebuildBridges(st);
      return true;
    }
  }
  // Bonds that cut pieces off a compound: those pieces leave it, with the work
  // in proportion to them. Whatever that cannot do stays in glueTouched.
  if (!dissolved && !st.groupsRecheck && bodiesSame && st.tiedKnown &&
      !st.glueTouched.empty()) {
    splitOffSmallPieces(st);
    if (st.glueTouched.empty() && !st.groupsRecheck) {
      st.groupsSignature = st.glueSignature;
      rebuildBridges(st);
      return true;
    }
  }
  return false;
}

// The pieces of the components the touched nodes are in.
void planPartialGroups(Box3DSharedState &st, GroupPlan &plan) {
  b3dop::Box3DWorld &w = st.world;
  plan.reach = reachFrom(st, st.glueTouched);
  plan.groups.resize(plan.reach.nodes.size());
  plan.ok.assign(plan.reach.nodes.size(), 1);
  for (size_t g = 0; g < plan.reach.nodes.size(); ++g)
    for (const int n : plan.reach.nodes[g]) {
      const BodyHandle h = st.glueNodes[static_cast<size_t>(n)].body;
      Box3DBodyMirror *e =
          static_cast<Box3DBodyMirror *>(w.bodyUserDataByHandle(h));
      if (!e || e->handle != h || e->wholeObject || !st.isLive(*e) ||
          !eligible(*e) || st.tiedBodies.count(h))
        plan.ok[g] = 0;
      else
        plan.groups[g].push_back(e);
    }
}

// The pieces of every component of the glue graph.
void planAllGroups(Box3DSharedState &st, GroupPlan &plan) {
  b3dop::Box3DWorld &w = st.world;
  // Held pieces are out of the graph: their bonds join nothing here, so each
  // is a group of one that is never merged.
  std::vector<b3dop::GlueBond> list = glueBondList(st, false, false);
  if (!st.glueHold.empty())
    for (b3dop::GlueBond &g : list)
      if (g.a >= 0 && g.b >= 0 && (heldNode(st, g.a) || heldNode(st, g.b)))
        g.a = g.b = -1;
  plan.comp = b3dop::glueComponents(st.glueNodes.size(), list);

  std::unordered_map<BodyHandle, Box3DBodyMirror *> byHandle;
  for (auto &kv : st.objectSync)
    for (Box3DBodyMirror *e : kv.second.bodies)
      if (e && e->handle && !e->wholeObject)
        byHandle[e->handle] = e;

  // Pieces another joint touches would go dormant with their bodies.
  st.tiedBodies.clear();
  {
    const std::unordered_set<const Box3DJointEntry *> bondSet(
        st.glueBonds.begin(), st.glueBonds.end());
    for (const auto &kv : st.joints) {
      const Box3DJointEntry &j = kv.second;
      if (bondSet.count(&j) || !w.isJointValid(j.handle))
        continue;
      if (j.bodyA)
        st.tiedBodies.insert(j.bodyA);
      if (j.bodyB)
        st.tiedBodies.insert(j.bodyB);
    }
  }
  st.tiedKnown = true;

  const size_t groupCount = static_cast<size_t>(plan.comp.count);
  plan.groups.resize(groupCount);
  plan.ok.assign(groupCount, 1);
  for (size_t i = 0; i < st.glueNodes.size(); ++i) {
    const size_t g = static_cast<size_t>(plan.comp.of[i]);
    const BodyHandle h = st.glueNodes[i].body;
    const auto it = byHandle.find(h);
    if (it == byHandle.end() || !eligible(*it->second) ||
        st.tiedBodies.count(h))
      plan.ok[g] = 0;
    else
      plan.groups[g].push_back(it->second);
  }
}

// A group is only merged when its pieces are alike.
void checkGroupsAlike(GroupPlan &plan) {
  const size_t groupCount = plan.groups.size();
  plan.covered.assign(groupCount, 0);
  for (size_t g = 0; g < groupCount; ++g) {
    if (!plan.ok[g])
      continue;
    for (const Box3DBodyMirror *e : plan.groups[g])
      if (!sameBodyLevel(e->props, plan.groups[g][0]->props))
        plan.ok[g] = 0;
  }
}

// A compound still the group it was made from stays. One that lost pieces
// (a bond broke) keeps its body for the largest group of pieces left and
// lets the others go; the rest are dissolved. Pieces that leave may join a
// new compound afterwards. Only `affected` ones are looked at when partial.
void keepShrinkOrDissolve(
    Box3DSharedState &st, GroupPlan &plan,
    const std::unordered_set<const Box3DCompound *> &affected,
    bool &membership) {
  for (size_t i = 0; i < st.compounds.size();) {
    Box3DCompound &c = *st.compounds[i];
    if (plan.partial && !affected.count(&c)) {
      ++i;
      continue;
    }
    std::vector<int> cid(c.members.size(), -1); // component of each piece
    std::unordered_map<int, size_t> count;      // pieces per component here
    for (size_t k = 0; k < c.members.size(); ++k) {
      const auto n = st.glueNodeOf.find(c.members[k].body);
      if (n == st.glueNodeOf.end())
        continue; // no bonds left
      cid[k] = plan.componentOf(n->second);
      if (cid[k] >= 0)
        ++count[cid[k]];
    }
    size_t best = 0;
    const int stay = largestComponent(count, best);
    const bool usable = stay >= 0 && plan.ok[static_cast<size_t>(stay)] &&
                        plan.groups[static_cast<size_t>(stay)].size() == best &&
                        best >= 2 && best * 2 >= c.members.size();
    if (usable && best == c.members.size()) {
      plan.covered[static_cast<size_t>(stay)] = 1;
      ++i;
    } else if (usable && shrink(st, c, cid, stay)) {
      plan.covered[static_cast<size_t>(stay)] = 1;
      membership = true;
      ++i;
    } else {
      dissolve(st, c);
      st.compounds.erase(st.compounds.begin() + static_cast<std::ptrdiff_t>(i));
      membership = true;
    }
  }
}

// 3. New compounds from the groups no compound covers.
void mergeNewGroups(Box3DSharedState &st, GroupPlan &plan, bool &membership) {
  for (size_t g = 0; g < plan.groups.size(); ++g) {
    if (plan.covered[g] || !plan.ok[g] || plan.groups[g].size() < 2)
      continue;
    std::sort(plan.groups[g].begin(), plan.groups[g].end(),
              [](const Box3DBodyMirror *a, const Box3DBodyMirror *b) {
                return keyOf(*a) < keyOf(*b);
              });
    if (merge(st, plan.groups[g]))
      membership = true;
  }
}

} // namespace

void syncCompounds(Box3DSharedState &st) {
  if (!st.glueCompounds) {
    if (!st.compounds.empty()) {
      dissolveAll(st);
      markCompoundBonds(st);
    }
    st.glueHold = b3dop::GlueHolds();
    st.groupsRecheck = true;
    return;
  }
  pruneBridges(st);

  bool membership = false; // a compound was dissolved or made
  const bool dissolved = dissolveChangedCompounds(st, membership);
  if (settledWithoutRegrouping(st, dissolved))
    return;

  GroupPlan plan;
  plan.partial = !dissolved && !st.groupsRecheck &&
                 st.bodyVersion == st.groupsBodyVersion &&
                 !st.glueTouched.empty() && st.tiedKnown;
  st.groupsSignature = st.glueSignature;
  st.groupsBodyVersion = st.bodyVersion;
  st.groupsRecheck = false;
  if (plan.partial)
    planPartialGroups(st, plan);
  else
    planAllGroups(st, plan);
  checkGroupsAlike(plan);

  // The compounds that can have changed: those with a piece in a component
  // worked out again.
  std::unordered_set<const Box3DCompound *> affected;
  std::vector<int> affectedNodes;
  if (plan.partial)
    for (const auto &kv : plan.reach.of) {
      affectedNodes.push_back(kv.first);
      const auto it =
          st.compoundOf.find(st.glueNodes[static_cast<size_t>(kv.first)].body);
      if (it != st.compoundOf.end())
        affected.insert(it->second.first);
    }

  keepShrinkOrDissolve(st, plan, affected, membership);
  mergeNewGroups(st, plan, membership);
  st.glueTouched.clear();
  if (membership) {
    markCompoundBonds(st, plan.partial ? &affectedNodes : nullptr);
    st.bridgesDirty = true;
  }
  rebuildBridges(st);
}
