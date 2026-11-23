// Box3DCompoundSync.cpp
//
// Glued groups as compound bodies: see Box3DCompoundSync.h. The maths is in
// core/Box3DCompound.h; this file keeps the groups in step with the bodies.
#include "Box3DCompoundSync.h"
#include "Box3DHoudiniUtil.h"

#include "Box3DJointSync.h"
#include "Box3DState.h"
#include "core/Box3DBondNetwork.h"
#include "core/Box3DCompound.h"
#include "core/Box3DDebug.h"
#include "core/Box3DMath.h"
#include "core/Box3DShapes.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

namespace {

using b3dop::BodyHandle;

using Pose = b3dop::BodyPose;

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

// Whether the piece of a glue node is held out of its compound
// (holdCompoundsForApproach): held pieces are not part of any group, they and
// their bonds are left out of the grouping.
bool heldNode(const Box3DSharedState &st, int node) {
  return !st.glueHold.empty() &&
         st.glueHold.held(st.glueNodes[static_cast<size_t>(node)].body);
}

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

// A move event for a piece of `entry` put at `pp`.
b3dop::BodyMove moveAt(Box3DBodyMirror &entry, BodyHandle body, const Pose &pp,
                       bool asleep, const float lin[3] = nullptr,
                       const float ang[3] = nullptr) {
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

// Destroys the bridges at the compound's pieces in `members` (null: all of
// them) and forgets them. A bridge to a piece that leaves, or all of them when
// the compound goes.
void dropBridges(Box3DSharedState &st, Box3DCompound &c,
                 const std::unordered_set<BodyHandle> *members = nullptr) {
  for (auto it = c.bridges.begin(); it != c.bridges.end();) {
    if (members && !members->count(it->member)) {
      ++it;
      continue;
    }
    st.world.destroyJoint(it->handle);
    it = c.bridges.erase(it);
  }
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

// Enables the pieces where the compound left them and destroys the compound
// body. Pieces whose entry is gone (`entry` null) were destroyed with their
// points. `moves` (optional): gets a move event for each piece, for a release
// in the middle of a frame.
void dissolve(Box3DSharedState &st, Box3DCompound &c,
              std::vector<b3dop::BodyMove> *moves = nullptr) {
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
void markCompoundBonds(Box3DSharedState &st,
                       const std::vector<int> *nodes = nullptr) {
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

void dissolveAll(Box3DSharedState &st) {
  for (auto &c : st.compounds) {
    resolveMembers(st, *c);
    dissolve(st, *c);
  }
  st.compounds.clear();
  st.compoundOf.clear();
}

// Takes the pieces whose component is not `mainId` out of the compound and
// keeps the body (and the contacts of the pieces that stay): they are enabled
// where the compound has them, their shapes destroyed, and the body keeps the
// momentum of the pieces that stay, which still move as one rigid body. False,
// with nothing changed, if the compound's body cannot be read.
// `leave` and `moves` as for dissolve: the pieces that leave take `leave`'s
// motion, the ones that stay keep the compound's now.
bool shrink(Box3DSharedState &st, Box3DCompound &c, const std::vector<int> &cid,
            int mainId, std::vector<b3dop::BodyMove> *moves = nullptr) {
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
  c.bondWarm.clear();
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
  for (size_t i = 0; i < raw->members.size(); ++i) {
    Box3DCompoundMember &m = raw->members[i];
    w.setEnabledByHandle(m.body, false);
    m.entry->inCompound = true;
    m.entry->glueEdited = false;
    st.compoundOf[m.body] = {raw, static_cast<int>(i)};
  }
  st.compounds.push_back(std::move(c));
  return true;
}

} // namespace

void queueCompoundIds(Box3DSharedState &st) {
  if (!st.bondReadouts || (st.compounds.empty() && st.compoundIdKeys.empty()))
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
  b3dop::Box3DWorld &w = st.world;
  pruneBridges(st);
  // Pieces held out of their compound stay joined to it: the bridges are made
  // again when the compounds or the bonds changed, also when nothing is grouped
  // again.
  const auto rebuildBridges = [&st]() {
    if (!st.bridgesDirty)
      return;
    st.bridgesDirty = false;
    if (!st.glueHold.empty())
      for (auto &c : st.compounds)
        bridgeBoundaries(st, *c);
  };

  // 1. Compounds whose pieces changed since they were merged. Only a full
  // scan of an object (or a body added or removed) can change a piece's entry,
  // settings or shapes; a step with none leaves them as merged, and checking
  // every piece of every compound each frame was most of what a held compound
  // cost.
  bool dissolved = false;
  bool membership = false; // a compound was dissolved or made
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

  // 2. The groups again, only when the glue graph or the bodies changed. When
  // only bonds broke (the lists were taken apart in place), only the components
  // those bonds were in are worked out again.
  const bool bodiesSame = st.bodyVersion == st.groupsBodyVersion;
  if (!dissolved && !st.groupsRecheck && bodiesSame &&
      st.glueSignature == st.groupsSignature) {
    st.glueTouched.clear();
    rebuildBridges();
    return;
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
      rebuildBridges();
      return;
    }
  }
  const bool partial = !dissolved && !st.groupsRecheck && bodiesSame &&
                       !st.glueTouched.empty() && st.tiedKnown;
  st.groupsSignature = st.glueSignature;
  st.groupsBodyVersion = st.bodyVersion;
  st.groupsRecheck = false;

  b3dop::GlueComponents comp; // all the nodes' components
  Reach reach;                // or only those of the touched nodes
  std::vector<std::vector<Box3DBodyMirror *>> groups;
  std::vector<char> ok, covered;
  if (partial) {
    reach = reachFrom(st, st.glueTouched);
    groups.resize(reach.nodes.size());
    ok.assign(reach.nodes.size(), 1);
    for (size_t g = 0; g < reach.nodes.size(); ++g)
      for (const int n : reach.nodes[g]) {
        const BodyHandle h = st.glueNodes[static_cast<size_t>(n)].body;
        Box3DBodyMirror *e =
            static_cast<Box3DBodyMirror *>(w.bodyUserDataByHandle(h));
        if (!e || e->handle != h || e->wholeObject || !st.isLive(*e) ||
            !eligible(*e) || st.tiedBodies.count(h))
          ok[g] = 0;
        else
          groups[g].push_back(e);
      }
  } else {
    // Held pieces are out of the graph: their bonds join nothing here, so each
    // is a group of one that is never merged.
    std::vector<b3dop::GlueBond> list = glueBondList(st, false, false);
    if (!st.glueHold.empty())
      for (b3dop::GlueBond &g : list)
        if (g.a >= 0 && g.b >= 0 && (heldNode(st, g.a) || heldNode(st, g.b)))
          g.a = g.b = -1;
    comp = b3dop::glueComponents(st.glueNodes.size(), list);

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

    const size_t groupCount = static_cast<size_t>(comp.count);
    groups.resize(groupCount);
    ok.assign(groupCount, 1);
    for (size_t i = 0; i < st.glueNodes.size(); ++i) {
      const size_t g = static_cast<size_t>(comp.of[i]);
      const BodyHandle h = st.glueNodes[i].body;
      const auto it = byHandle.find(h);
      if (it == byHandle.end() || !eligible(*it->second) ||
          st.tiedBodies.count(h))
        ok[g] = 0;
      else
        groups[g].push_back(it->second);
    }
  }
  const size_t groupCount = groups.size();
  covered.assign(groupCount, 0);
  for (size_t g = 0; g < groupCount; ++g) {
    if (!ok[g])
      continue;
    for (const Box3DBodyMirror *e : groups[g])
      if (!sameBodyLevel(e->props, groups[g][0]->props))
        ok[g] = 0;
  }
  auto componentOf = [&](int node) {
    return partial ? reach.componentOf(node)
                   : comp.of[static_cast<size_t>(node)];
  };

  // The compounds that can have changed: those with a piece in a component
  // worked out again.
  std::unordered_set<const Box3DCompound *> affected;
  std::vector<int> affectedNodes;
  if (partial)
    for (const auto &kv : reach.of) {
      affectedNodes.push_back(kv.first);
      const auto it =
          st.compoundOf.find(st.glueNodes[static_cast<size_t>(kv.first)].body);
      if (it != st.compoundOf.end())
        affected.insert(it->second.first);
    }

  // A compound still the group it was made from stays. One that lost pieces
  // (a bond broke) keeps its body for the largest group of pieces left and
  // lets the others go; the rest are dissolved. Pieces that leave may join a
  // new compound below.
  for (size_t i = 0; i < st.compounds.size();) {
    Box3DCompound &c = *st.compounds[i];
    if (partial && !affected.count(&c)) {
      ++i;
      continue;
    }
    std::vector<int> cid(c.members.size(), -1); // component of each piece
    std::unordered_map<int, size_t> count;      // pieces per component here
    for (size_t k = 0; k < c.members.size(); ++k) {
      const auto n = st.glueNodeOf.find(c.members[k].body);
      if (n == st.glueNodeOf.end())
        continue; // no bonds left
      cid[k] = componentOf(n->second);
      if (cid[k] >= 0)
        ++count[cid[k]];
    }
    size_t best = 0;
    const int stay = largestComponent(count, best);
    const bool usable = stay >= 0 && ok[static_cast<size_t>(stay)] &&
                        groups[static_cast<size_t>(stay)].size() == best &&
                        best >= 2 && best * 2 >= c.members.size();
    if (usable && best == c.members.size()) {
      covered[static_cast<size_t>(stay)] = 1;
      ++i;
    } else if (usable && shrink(st, c, cid, stay)) {
      covered[static_cast<size_t>(stay)] = 1;
      membership = true;
      ++i;
    } else {
      dissolve(st, c);
      st.compounds.erase(st.compounds.begin() + static_cast<std::ptrdiff_t>(i));
      membership = true;
    }
  }

  // 3. New compounds.
  for (size_t g = 0; g < groupCount; ++g) {
    if (covered[g] || !ok[g] || groups[g].size() < 2)
      continue;
    std::sort(groups[g].begin(), groups[g].end(),
              [](const Box3DBodyMirror *a, const Box3DBodyMirror *b) {
                return keyOf(*a) < keyOf(*b);
              });
    if (merge(st, groups[g]))
      membership = true;
  }
  st.glueTouched.clear();
  if (membership) {
    markCompoundBonds(st, partial ? &affectedNodes : nullptr);
    st.bridgesDirty = true;
  }
  rebuildBridges();
}

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
// kGlueStrengthScale, or for a Bond its weakest strength over its area, held
// for kBondHoldSeconds (a body that cannot deliver that in about a frame cannot
// break it). The strengths are the tensile, shear, compressive and bending ones
// (a bending load at an arm of the piece's own size, sqrt(area), breaks it at a
// sixth of the force over the area), so a lever or a crushing load is not
// passed over. Infinity when it cannot break; 0 for a Bond whose area is not
// known yet, so a body is not passed over before the first step.
constexpr float kBondHoldSeconds = 0.02f;
constexpr float kBondBendingArm = 1.0f / 6.0f;
float bondBreakImpulse(const Box3DJointEntry &e) {
  constexpr float kNever = std::numeric_limits<float>::infinity();
  if (e.type == b3dop::JointType::Bond) {
    float least = kNever;
    const auto consider = [&least](float strength, float scale) {
      if (strength > 0.0f && std::isfinite(strength))
        least = std::min(least, strength * scale);
    };
    consider(e.settings.bondTensile, 1.0f);
    consider(e.settings.bondShear, 1.0f);
    consider(e.settings.bondCompressive, 1.0f);
    // A bending strength of 0 is the tensile one.
    consider(e.settings.bondBending > 0.0f ? e.settings.bondBending
                                           : e.settings.bondTensile,
             kBondBendingArm);
    if (least == kNever)
      return kNever;
    const float area = e.extra && e.extra->bondReady ? e.extra->bondAreaUsed
                                                     : e.settings.bondArea;
    return area > 0.0f ? least * area * kBondHoldSeconds : 0.0f;
  }
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

void recordCompoundMotion(Box3DSharedState &st) {
  if (st.glueCompoundLoad <= 0.0f && !st.bondsInGraph)
    return;
  for (auto &c : st.compounds) {
    b3dop::MergedBody mb;
    bool awake = false;
    c->stepPreOk = st.world.isAwakeByHandle(c->body) &&
                   bodyMotion(st.world, c->body, mb, awake) && awake;
    c->stepPre = mb;
  }
}

void expandCompoundMoves(Box3DSharedState &st) {
  if (st.compounds.empty())
    return;
  b3dop::Box3DWorld &w = st.world;
  std::unordered_map<BodyHandle, Box3DCompound *> byBody;
  byBody.reserve(st.compounds.size());
  for (auto &c : st.compounds)
    byBody[c->body] = c.get();
  const size_t original = st.moves.size();
  for (size_t i = 0; i < original; ++i) {
    const auto it = byBody.find(st.moves[i].handle);
    if (it == byBody.end())
      continue;
    const b3dop::BodyMove ev = st.moves[i]; // push_back may move the vector
    Box3DCompound &c = *it->second;
    Pose cp;
    b3dop::copy3(cp.p, ev.pos);
    b3dop::copy4(cp.q, ev.rot);
    b3dop::MergedBody mb;
    bool awake = false;
    if (!bodyMotion(w, c.body, mb, awake))
      continue;
    for (Box3DCompoundMember &m : c.members) {
      if (!m.entry)
        continue;
      const Pose pp = placed(cp, m);
      double com[3];
      centreOf(pp, m, com);
      b3dop::pieceVelocity(mb, com, m.entry->glueLin, m.entry->glueAng);
      st.moves.push_back(moveAt(*m.entry, m.body, pp, ev.fellAsleep,
                                m.entry->glueLin, m.entry->glueAng));
      // The parked body stays where the piece is: joint frames read it.
      w.setTransformByHandle(m.body, pp.p, pp.q);
    }
  }
}

bool compoundGlueHits(Box3DSharedState &st, BodyHandle body, float &hits) {
  const auto it = st.compoundOf.find(body);
  if (it == st.compoundOf.end())
    return false;
  Box3DCompound &c = *it->second.first;
  const Box3DCompoundMember &m =
      c.members[static_cast<size_t>(it->second.second)];
  hits = 0.0f;
  // Per compound, once a tick: hits land on moving bodies, a sleeping one has
  // none.
  if (c.hitsTick != st.hitsTick) {
    c.hitsTick = st.hitsTick;
    c.hitsActive =
        st.world.typeByHandle(c.body) == b3dop::BodySettings::Type::Dynamic &&
        st.world.isAwakeByHandle(c.body);
    if (c.hitsActive) {
      // Contacts with pieces of its own group (the pieces held out of it, say,
      // resting on it) carry weight, not hits.
      const int group = compoundGlueGroup(st, c);
      const std::function<bool(BodyHandle)> sameGroup =
          [&st, group](BodyHandle other) {
            return glueSameGroup(st, other, group);
          };
      st.world.contactImpulsesByShape(c.body, c.shapeImpulse,
                                      group >= 0 ? &sameGroup : nullptr);
    }
  }
  if (!c.hitsActive)
    return true;
  for (int i = m.shapeFirst; i < m.shapeFirst + m.shapeCount &&
                             static_cast<size_t>(i) < c.shapeImpulse.size();
       ++i)
    hits += c.shapeImpulse[static_cast<size_t>(i)];
  // Inertial load: slowing the whole body at one point also loads the pieces
  // away from it. A piece takes at least the impulse it needs to follow the
  // body's change of motion (a piece touching the ground keeps its contact
  // hit when that is larger).
  if (st.glueCompoundLoad > 0.0f && c.stepPreOk) {
    if (c.loadTick != st.hitsTick) {
      c.loadTick = st.hitsTick;
      bool awake = false;
      if (!bodyPose(st.world, c.body, c.postPose) ||
          !bodyMotion(st.world, c.body, c.stepPost, awake))
        c.stepPost.ok = false;
    }
    if (c.stepPost.ok && c.stepPre.ok) {
      double com[3];
      centreOf(placed(c.postPose, m), m, com);
      hits = std::max(hits,
                      st.glueCompoundLoad *
                          b3dop::pieceLoad(c.stepPre, c.stepPost, com, m.mass));
    }
  }
  return true;
}

namespace {
// The stiffness of the contacts a compound rests on against its bonds', and of
// what they do across their normal (friction) against their own. The bonds are
// far stiffer, so the compound settles on its supports as the rigid body it is
// and they share its weight the way a rigid beam on a soft bed does; the
// stiffness across only keeps the solve well posed and shares the friction the
// way the normal load is.
constexpr double kSupportStiffness = 0.01;
constexpr double kSupportTangent = 0.02;
// A contact that carried less than this share of the mean impulse is not a
// support (a corner just touching an edge).
constexpr double kSupportMinShare = 0.05;
// The conjugate gradient's iterations and tolerance for a compound's bond
// loads.
constexpr int kBondSolveIterations = 100;
constexpr double kBondSolveTolerance = 1e-5;
// Compounds with a network to solve before the solves run in parallel.
constexpr size_t kMinParallelBondSolves = 4;

// One compound's bond network: gathered from the world in a serial pass, solved
// in parallel (the solve touches only this), written to the joints in a serial
// pass.
struct BondJob {
  std::vector<b3dop::BondNode> nodes;
  std::vector<b3dop::BondLink> links;
  std::vector<b3dop::BondSupport> supports;
  std::vector<Box3DJointEntry *> used;
  std::vector<b3dop::BondForce> out;
  std::vector<b3dop::SupportForce> supportsOut;
  // The compound's last displacement in, this one's out.
  std::vector<double> warm;
  Box3DCompound *compound = nullptr;
  double residual = 0.0;
};
} // namespace

void solveCompoundBonds(Box3DSharedState &st, float dt) {
  if (!st.bondsInGraph || !(dt > 0.0f))
    return;
  B3D_PROFILE(kProfileBondSolve);
  b3dop::Box3DWorld &w = st.world;
  // The Bonds inside each compound, and every Bond by its joint (a bridge names
  // the joint it stands in for).
  std::unordered_map<Box3DCompound *, std::vector<Box3DJointEntry *>>
      byCompound;
  std::unordered_map<b3dop::JointHandle, Box3DJointEntry *> byJoint;
  for (Box3DJointEntry *e : st.glueBonds) {
    if (e->type != b3dop::JointType::Bond || !e->handle || e->broken)
      continue;
    byJoint[e->handle] = e;
    if (!e->inCompound)
      continue;
    const auto it = st.compoundOf.find(e->bodyA);
    if (it != st.compoundOf.end())
      byCompound[it->second.first].push_back(e);
  }
  if (byJoint.empty())
    return;
  const auto awakeCompound = [&w](const Box3DCompound &c) {
    return w.typeByHandle(c.body) == b3dop::BodySettings::Type::Dynamic &&
           w.isAwakeByHandle(c.body);
  };
  // A Bond between a piece in a compound and one held out of it: its own weld
  // is parked, the bridge weld carries it. The load on the original's body B is
  // the bridge's, whichever piece is the compound's.
  for (auto &c : st.compounds) {
    if (c->bridges.empty() || !awakeCompound(*c))
      continue;
    for (const Box3DBridge &b : c->bridges) {
      const auto it = byJoint.find(b.original);
      b3dop::JointFeedback fb;
      if (it == byJoint.end() ||
          !w.jointFeedback(b.handle, b3dop::JointType::Bond, fb))
        continue;
      Box3DJointEntry &e = *it->second;
      Box3DJointExtra &x = e.ensureExtra();
      // The bridge's B is the compound: the opposite of a force on the free
      // piece, which is the original's B when the compound holds its A.
      const float sign = e.bodyA == b.member ? -1.0f : 1.0f;
      for (int i = 0; i < 3; ++i) {
        x.bondForce[i] = sign * fb.force[i];
        x.bondTorque[i] = sign * fb.torque[i];
      }
      x.bondLoadTick = st.hitsTick;
    }
  }
  std::vector<BondJob> jobs;
  for (auto &entry : byCompound) {
    Box3DCompound &c = *entry.first;
    if (!awakeCompound(c) || !c.stepPreOk || !c.stepPre.ok)
      continue;
    b3dop::MergedBody post;
    bool awake = false;
    Pose pose;
    if (!bodyMotion(w, c.body, post, awake) || !bodyPose(w, c.body, pose))
      continue;
    const b3dop::MergedBody &pre = c.stepPre;
    const double ref[3] = {post.com[0], post.com[1], post.com[2]};
    double aCm[3], alpha[3], omega[3];
    for (int i = 0; i < 3; ++i) {
      aCm[i] = (static_cast<double>(post.lin[i]) - pre.lin[i]) / dt;
      alpha[i] = (static_cast<double>(post.ang[i]) - pre.ang[i]) / dt;
      omega[i] = 0.5 * (static_cast<double>(pre.ang[i]) + post.ang[i]);
    }
    // The contacts the compound rests on are elastic supports: what they push
    // with is whatever balances the rest, spread the way springs spread it.
    // With none (in the air) what the pieces need but the bridges do not supply
    // is spread over them by mass. A contact with a piece of its own group
    // (one held out of it by a bridge) is no support: the bridge's weld is
    // already taken off the pieces.
    std::vector<b3dop::Box3DWorld::ContactSupport> contacts;
    const int group = compoundGlueGroup(st, c);
    const std::function<bool(BodyHandle)> sameGroup =
        [&st, group](BodyHandle other) {
          return glueSameGroup(st, other, group);
        };
    w.contactSupportsByHandle(c.body, contacts,
                              group >= 0 ? &sameGroup : nullptr);
    const bool useSupports = !contacts.empty();
    const float *g = w.settings().gravity;
    const size_t count = c.members.size();
    std::vector<b3dop::BondNode> nodes(count);
    for (size_t k = 0; k < count; ++k) {
      const Box3DCompoundMember &m = c.members[k];
      b3dop::BondNode &node = nodes[k];
      centreOf(placed(pose, m), m, node.pos);
      const double r[3] = {node.pos[0] - ref[0], node.pos[1] - ref[1],
                           node.pos[2] - ref[2]};
      // What the piece needs to move as the body does, less gravity: its mass
      // times its acceleration (the body's, plus the turning), and its own
      // rotational inertia (a block of its radius) times the angular
      // acceleration.
      double ar[3], wr[3], wwr[3];
      b3dop::cross3(alpha, r, ar);
      b3dop::cross3(omega, r, wr);
      b3dop::cross3(omega, wr, wwr);
      const double inertia = 2.0 / 9.0 * m.mass * m.radius * m.radius;
      for (int i = 0; i < 3; ++i) {
        const double a = aCm[i] + ar[i] + wwr[i];
        node.force[i] = m.mass * (a - g[i] * m.props.gravityScale);
        node.torque[i] = inertia * alpha[i];
      }
    }
    for (const Box3DBridge &b : c.bridges) {
      const auto it = st.compoundOf.find(b.member);
      b3dop::JointFeedback fb;
      b3dop::JointFrame fa, fb2;
      if (it == st.compoundOf.end() || it->second.first != &c ||
          !w.jointFeedback(b.handle, b3dop::JointType::Bond, fb) ||
          !w.jointWorldFrames(b.handle, fa, fb2))
        continue;
      b3dop::BondNode &node = nodes[static_cast<size_t>(it->second.second)];
      const double r[3] = {fb2.pos[0] - node.pos[0], fb2.pos[1] - node.pos[1],
                           fb2.pos[2] - node.pos[2]};
      const double f[3] = {fb.force[0], fb.force[1], fb.force[2]};
      double rxf[3];
      b3dop::cross3(r, f, rxf);
      for (int i = 0; i < 3; ++i) {
        node.force[i] -= f[i];
        node.torque[i] -= static_cast<double>(fb.torque[i]) + rxf[i];
      }
    }
    // Without supports the pieces move as one body, so what they need adds up
    // to nothing. What is left (forces from a Force DOP, rounding) is spread
    // over the pieces by mass.
    if (!useSupports) {
      std::vector<double> weight(count);
      double total = 0.0;
      for (size_t k = 0; k < count; ++k) {
        weight[k] = c.members[k].mass;
        total += weight[k];
      }
      if (!(total > 0.0))
        continue;
      double sumF[3] = {0, 0, 0};
      for (const b3dop::BondNode &n : nodes)
        for (int i = 0; i < 3; ++i)
          sumF[i] += n.force[i];
      for (size_t k = 0; k < count; ++k)
        for (int i = 0; i < 3; ++i)
          nodes[k].force[i] -= sumF[i] * weight[k] / total;
      double sumT[3] = {0, 0, 0};
      for (const b3dop::BondNode &n : nodes) {
        const double r[3] = {n.pos[0] - ref[0], n.pos[1] - ref[1],
                             n.pos[2] - ref[2]};
        double rxf[3];
        b3dop::cross3(r, n.force, rxf);
        for (int i = 0; i < 3; ++i)
          sumT[i] += n.torque[i] + rxf[i];
      }
      for (size_t k = 0; k < count; ++k)
        for (int i = 0; i < 3; ++i)
          nodes[k].torque[i] -= sumT[i] * weight[k] / total;
    }
    // The bonds as springs, a stiffness per bond in proportion to its area.
    std::vector<b3dop::BondLink> links;
    std::vector<Box3DJointEntry *> used;
    for (Box3DJointEntry *e : entry.second) {
      const auto ia = st.compoundOf.find(e->bodyA);
      const auto ib = st.compoundOf.find(e->bodyB);
      b3dop::JointFrame la, lb;
      if (ia == st.compoundOf.end() || ib == st.compoundOf.end() ||
          !bondGeometry(st, *e) || !w.jointLocalFrames(e->handle, la, lb))
        continue;
      b3dop::BondLink l;
      l.a = ia->second.second;
      l.b = ib->second.second;
      // The anchor is where piece A's frame is with the compound where it is
      // now: the parked piece bodies are placed after this, a step behind.
      const Pose pa = placed(pose, c.members[static_cast<size_t>(l.a)]);
      const double local[3] = {la.pos[0], la.pos[1], la.pos[2]};
      double turned[3];
      b3dop::quatRotate(pa.q, local, turned);
      for (int i = 0; i < 3; ++i)
        l.anchor[i] = pa.p[i] + turned[i];
      Box3DJointExtra &px = *e->extra;
      std::copy(pa.p, pa.p + 3, px.bondPosA);
      std::copy(pa.q, pa.q + 4, px.bondRotA);
      std::copy(l.anchor, l.anchor + 3, px.bondAnchor);
      px.bondPoseTick = st.hitsTick;
      const double area = std::max<double>(e->extra->bondAreaUsed, 1e-9);
      l.stiffness = area;
      l.rotStiffness = area * area / 12.0;
      links.push_back(l);
      used.push_back(e);
    }
    // The contacts as supports: the stiffness of a bond is its area, so a
    // support has the mean of them times kSupportStiffness. Every point that
    // carried a real share is as stiff as the others (the share Box3D gave each
    // piece of a rigid compound is arbitrary, so it is not taken as a prior).
    std::vector<b3dop::BondSupport> supports;
    if (useSupports && !links.empty()) {
      double mean = 0.0;
      for (const b3dop::BondLink &l : links)
        mean += l.stiffness;
      mean /= static_cast<double>(links.size());
      std::vector<int> memberOfShape;
      for (size_t k = 0; k < count; ++k) {
        const Box3DCompoundMember &m = c.members[k];
        for (int s = m.shapeFirst; s < m.shapeFirst + m.shapeCount; ++s) {
          if (s < 0)
            continue;
          if (static_cast<size_t>(s) >= memberOfShape.size())
            memberOfShape.resize(static_cast<size_t>(s) + 1, -1);
          memberOfShape[static_cast<size_t>(s)] = static_cast<int>(k);
        }
      }
      double meanImpulse = 0.0;
      for (const auto &contact : contacts)
        meanImpulse += contact.impulse;
      meanImpulse /= static_cast<double>(contacts.size());
      for (const auto &contact : contacts) {
        if (!(meanImpulse > 0.0) ||
            contact.impulse < kSupportMinShare * meanImpulse)
          continue;
        if (contact.shape < 0 ||
            static_cast<size_t>(contact.shape) >= memberOfShape.size() ||
            memberOfShape[static_cast<size_t>(contact.shape)] < 0)
          continue;
        b3dop::BondSupport s;
        s.node = memberOfShape[static_cast<size_t>(contact.shape)];
        std::copy(contact.pos, contact.pos + 3, s.pos);
        std::copy(contact.push, contact.push + 3, s.normal);
        s.normalStiffness = mean * kSupportStiffness;
        s.tangentStiffness = kSupportTangent * s.normalStiffness;
        supports.push_back(s);
      }
    }
    // Contacts that map to no piece leave nothing to balance the rest: no load
    // this step.
    if (useSupports && supports.empty())
      continue;
    BondJob job;
    job.nodes = std::move(nodes);
    job.links = std::move(links);
    job.supports = std::move(supports);
    job.used = std::move(used);
    job.warm = std::move(c.bondWarm);
    job.compound = &c;
    jobs.push_back(std::move(job));
  }
  // The solves are independent: each reads and writes only its own job.
  const auto solve = [&jobs](size_t i) {
    BondJob &j = jobs[i];
    j.residual = b3dop::solveBondNetwork(j.nodes, j.links, j.supports, j.out,
                                         j.supportsOut, kBondSolveIterations,
                                         kBondSolveTolerance,
                                         b3dop::kBondSupportPasses, &j.warm);
  };
  box3dParallelFor(jobs.size(), solve, kMinParallelBondSolves);
  for (BondJob &j : jobs) {
    j.compound->bondWarm = std::move(j.warm);
    // A solve that did not stay finite gives no load this step.
    if (!std::isfinite(j.residual))
      continue;
    for (size_t k = 0; k < j.used.size(); ++k) {
      Box3DJointExtra &x = *j.used[k]->extra;
      for (int i = 0; i < 3; ++i) {
        x.bondForce[i] = static_cast<float>(j.out[k].force[i]);
        x.bondTorque[i] = static_cast<float>(j.out[k].torque[i]);
      }
      x.bondLoadTick = st.hitsTick;
    }
  }
}

bool compoundPieceState(Box3DSharedState &st, const Box3DBodyMirror &piece,
                        b3dop::BodyState &out) {
  const auto it = st.compoundOf.find(piece.handle);
  if (it == st.compoundOf.end())
    return false;
  const Box3DCompound &c = *it->second.first;
  const Box3DCompoundMember &m =
      c.members[static_cast<size_t>(it->second.second)];
  Pose cp;
  b3dop::MergedBody mb;
  bool awake = false;
  b3dop::BodyState own;
  if (!bodyPose(st.world, c.body, cp) ||
      !bodyMotion(st.world, c.body, mb, awake) ||
      !st.world.bodyStateByHandle(piece.handle, own))
    return false;
  const Pose pp = placed(cp, m);
  out.awake = awake;
  out.mass = own.mass;
  centreOf(pp, m, out.com);
  b3dop::pieceVelocity(mb, out.com, out.lin, out.ang);
  return true;
}

bool compoundPieceAwake(Box3DSharedState &st, const Box3DBodyMirror &piece) {
  const auto it = st.compoundOf.find(piece.handle);
  return it != st.compoundOf.end() &&
         st.world.isAwakeByHandle(it->second.first->body);
}

bool compoundPieceForce(Box3DSharedState &st, const Box3DBodyMirror &piece,
                        float force[3], float torque[3], BodyHandle &target) {
  const auto it = st.compoundOf.find(piece.handle);
  if (it == st.compoundOf.end())
    return false;
  const Box3DCompound &c = *it->second.first;
  const Box3DCompoundMember &m =
      c.members[static_cast<size_t>(it->second.second)];
  Pose cp;
  b3dop::BodyState cs;
  if (!bodyPose(st.world, c.body, cp) ||
      !st.world.bodyStateByHandle(c.body, cs))
    return false;
  const Pose pp = placed(cp, m);
  // The torque about the compound's centre of mass: the piece's own plus
  // (where it acts - centre) x force. A force acts at the piece's centre.
  double at[3];
  centreOf(pp, m, at);
  const double r[3] = {at[0] - cs.com[0], at[1] - cs.com[1], at[2] - cs.com[2]};
  const double f[3] = {force[0], force[1], force[2]};
  double rxf[3];
  b3dop::cross3(r, f, rxf);
  for (int i = 0; i < 3; ++i)
    torque[i] += static_cast<float>(rxf[i]);
  target = c.body;
  return true;
}
