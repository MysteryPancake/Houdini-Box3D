// Box3DCompoundSync.cpp
//
// Glued groups as compound bodies: see Box3DCompoundSync.h. The maths is in
// core/Box3DCompound.h; this file keeps the groups in step with the bodies.
#include "Box3DCompoundSync.h"

#include "Box3DJointSync.h"
#include "Box3DState.h"
#include "core/Box3DCompound.h"
#include "core/Box3DMath.h"
#include "core/Box3DShapes.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
  auto same = [](float x, float y) { return x == y || (x != x && y != y); };
  return same(a.linearDamping, b.linearDamping) &&
         same(a.angularDamping, b.angularDamping) &&
         same(a.gravityScale, b.gravityScale) &&
         same(a.sleepThreshold, b.sleepThreshold) &&
         same(a.ccdThreshold, b.ccdThreshold) &&
         a.contactRecycling == b.contactRecycling &&
         a.enableSleep == b.enableSleep && a.fastObject == b.fastObject &&
         a.fastRotation == b.fastRotation && a.locks == b.locks;
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

Reach reachFrom(const Box3DSharedState &st, const std::vector<int> &seeds) {
  Reach r;
  std::vector<int> stack;
  for (const int seed : seeds) {
    if (r.of.count(seed))
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
        if (it != st.glueNodeOf.end() && r.of.emplace(it->second, id).second)
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
// points. `leave` (optional): the motion the pieces take instead of the
// compound's now (its motion before the step that broke it), `moves`
// (optional): gets a move event for each piece, for a split in the middle of a
// frame.
void dissolve(Box3DSharedState &st, Box3DCompound &c,
              const b3dop::MergedBody *leave = nullptr,
              std::vector<b3dop::BodyMove> *moves = nullptr) {
  b3dop::Box3DWorld &w = st.world;
  Pose cp;
  b3dop::MergedBody mb;
  bool awake = false;
  const bool have = bodyPose(w, c.body, cp) && bodyMotion(w, c.body, mb, awake);
  if (leave && have && awake)
    mb = *leave;
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
            int mainId, const b3dop::MergedBody *leave = nullptr,
            std::vector<b3dop::BodyMove> *moves = nullptr) {
  b3dop::Box3DWorld &w = st.world;
  Pose cp;
  b3dop::MergedBody was;
  bool awake = false;
  if (!bodyPose(w, c.body, cp) || !bodyMotion(w, c.body, was, awake))
    return false;

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
    releasePiece(w, m, &cp, awake, leave ? *leave : was, moves);
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
  w.setCcdThresholdByHandle(body, bs.props.ccdThreshold);

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

void syncCompounds(Box3DSharedState &st) {
  if (!st.glueCompounds) {
    if (!st.compounds.empty()) {
      dissolveAll(st);
      markCompoundBonds(st);
    }
    st.groupsRecheck = true;
    return;
  }
  b3dop::Box3DWorld &w = st.world;

  // 1. Compounds whose pieces changed since they were merged.
  bool dissolved = false;
  bool membership = false; // a compound was dissolved or made
  for (size_t i = 0; i < st.compounds.size();) {
    Box3DCompound &c = *st.compounds[i];
    if (resolveMembers(st, c)) {
      dissolve(st, c);
      st.compounds.erase(st.compounds.begin() + static_cast<long>(i));
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
    return;
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
    comp = b3dop::glueComponents(st.glueNodes.size(),
                                 glueBondList(st, false, false));

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
      st.compounds.erase(st.compounds.begin() + static_cast<long>(i));
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
  if (membership)
    markCompoundBonds(st, partial ? &affectedNodes : nullptr);
}

void recordCompoundMotion(Box3DSharedState &st, bool frameStart) {
  const bool wantLoad = st.glueCompoundLoad > 0.0f;
  for (auto &c : st.compounds) {
    // Kept while the last step had hits: the motion from before the impact.
    const bool keep = !frameStart && c->preOk && c->hadHit;
    if (keep && !wantLoad)
      continue;
    b3dop::MergedBody mb;
    bool awake = false;
    const bool ok = st.world.isAwakeByHandle(c->body) &&
                    bodyMotion(st.world, c->body, mb, awake) && awake;
    if (wantLoad) {
      c->stepPre = mb;
      c->stepPreOk = ok;
    }
    if (!keep) {
      c->pre = mb;
      c->preOk = ok;
      c->hadHit = false;
    }
  }
}

void splitBrokenCompounds(Box3DSharedState &st,
                          std::vector<b3dop::BodyMove> &moves,
                          const std::vector<int> &touched) {
  if (st.compounds.empty() || touched.empty())
    return;
  // The components the broken bonds were in, over the bonds still holding (an
  // early break took its bond out of its nodes' partners).
  const Reach reach = reachFrom(st, touched);

  bool membership = false;
  for (size_t i = 0; i < st.compounds.size();) {
    Box3DCompound &c = *st.compounds[i];
    std::vector<int> cid(c.members.size(), -1);
    std::unordered_map<int, size_t> count;
    bool bad = false, anyIn = false, anyOut = false;
    for (size_t k = 0; k < c.members.size(); ++k) {
      const auto n = st.glueNodeOf.find(c.members[k].body);
      if (n == st.glueNodeOf.end() || !c.members[k].entry) {
        bad = true;
        continue;
      }
      cid[k] = reach.componentOf(n->second);
      if (cid[k] >= 0) {
        anyIn = true;
        ++count[cid[k]];
      } else {
        anyOut = true; // part of it is in a component that did not change
      }
    }
    if (!anyIn && !bad) { // not one of the components that can have split
      ++i;
      continue;
    }
    const bool usable = !bad && !anyOut;
    if (usable && count.size() == 1) { // still one group
      ++i;
      continue;
    }
    size_t best = 0;
    const int stay = largestComponent(count, best);
    const b3dop::MergedBody *leave = c.preOk ? &c.pre : nullptr;
    membership = true;
    if (usable && best >= 2 && best * 2 >= c.members.size() &&
        shrink(st, c, cid, stay, leave, &moves)) {
      ++i;
    } else {
      dissolve(st, c, leave, &moves);
      st.compounds.erase(st.compounds.begin() + static_cast<long>(i));
    }
  }
  if (membership) {
    std::vector<int> nodes;
    nodes.reserve(reach.of.size());
    for (const auto &kv : reach.of)
      nodes.push_back(kv.first);
    markCompoundBonds(st, &nodes);
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
      st.world.contactImpulsesByShape(c.body, c.shapeImpulse);
      c.hadHit = std::any_of(c.shapeImpulse.begin(), c.shapeImpulse.end(),
                             [](float v) { return v > 0.0f; });
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
  torque[0] += static_cast<float>(r[1] * f[2] - r[2] * f[1]);
  torque[1] += static_cast<float>(r[2] * f[0] - r[0] * f[2]);
  torque[2] += static_cast<float>(r[0] * f[1] - r[1] * f[0]);
  target = c.body;
  return true;
}
