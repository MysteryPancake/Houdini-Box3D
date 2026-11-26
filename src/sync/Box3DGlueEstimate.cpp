// Box3DGlueEstimate.cpp
//
// Glue's break estimate (core/Box3DGlueEstimate.h) for the contact points of a
// Box3D step: what RBD Bullet Solver breaks glue on before its step, worked out
// here from the contacts the step had. The speed of a contact is its hit
// event's approach speed (the contacts that came together faster than Box3D's
// hit threshold; the slower ones are left to their real impulse, which is what
// a resting body's weight is), plus the step's gravity, as Bullet reads the
// speeds after integrating it. A glued piece is the rigid body of its whole
// group (the pieces merged by mergePieces), whether the group runs as a
// compound or as welds.
#include "core/Box3DGlueEstimate.h"
#include "Box3DCompoundInternal.h"
#include "Box3DJointSync.h"
#include "core/Box3DMath.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

using b3dop::BodyHandle;

// What one side of a contact point is.
struct Side {
  BodyHandle body = 0;
  Box3DBodyMirror *entry = nullptr; // the body's (a glued piece's) entry
  int node = -1;                    // the glue node of the piece, or -1
  bool awake = false;
};

class Estimator {
public:
  explicit Estimator(Box3DSharedState &st) : myState(st) {
    for (const auto &c : st.compounds)
      myCompounds[c->body] = c.get();
    for (int i = 0; i < 3; ++i)
      myGravityStep[i] = st.world.settings().gravity[i] * st.stepDt;
  }

  void run(const std::vector<b3dop::ContactImpulse> &events,
           const std::vector<b3dop::HitContact> &hits);

private:
  Side resolve(BodyHandle body, void *userData, uint64_t shape, bool awake);
  const b3dop::GlueBody *groupBody(int group);
  const b3dop::GlueBody *plainBody(const Side &s);
  float restitutionOf(const Side &s) const;
  // The body of a piece or a plain body as the estimate takes it, from a
  // dynamic body's pose and mass.
  bool rigidPiece(BodyHandle h, b3dop::RigidPiece &out) const;

  Box3DSharedState &myState;
  float myGravityStep[3] = {0, 0, 0};
  std::unordered_map<BodyHandle, const Box3DCompound *> myCompounds;
  // A compound body's shapes (as the events name them) -> the glue node of
  // the piece that owns each.
  std::unordered_map<BodyHandle, std::unordered_map<uint64_t, int>> myShapes;
  // The merged bodies worked out this call; a group with no usable body (a
  // kinematic piece, a piece that is not finite) has none.
  std::unordered_map<int, b3dop::GlueBody> myGroups;
  std::unordered_map<int, bool> myGroupBad;
  // The glue nodes of each group, made on the first group body asked for.
  std::unordered_map<int, std::vector<size_t>> myGroupNodes;
  bool myGroupNodesMade = false;
  std::unordered_map<BodyHandle, b3dop::GlueBody> myBodies;
  // Dynamic bodies that have no usable rigid body (not a glued piece).
  std::unordered_set<BodyHandle> myBodyBad;
  std::vector<uint64_t> myKeys;
};

// A contact is the same whichever way round its shapes are named.
struct PairKey {
  uint64_t lo, hi;
  PairKey(uint64_t a, uint64_t b) : lo(std::min(a, b)), hi(std::max(a, b)) {}
  bool operator==(const PairKey &o) const { return lo == o.lo && hi == o.hi; }
};
struct PairHash {
  size_t operator()(const PairKey &k) const {
    return static_cast<size_t>(b3dop::mixHash(k.lo, k.hi));
  }
};

bool Estimator::rigidPiece(BodyHandle h, b3dop::RigidPiece &out) const {
  const b3dop::Box3DWorld &w = myState.world;
  compoundsync::Pose pose;
  float mass = 0, centre[3] = {}, inertia[3] = {}, off[3] = {};
  if (!compoundsync::bodyPose(w, h, pose) ||
      !w.getMassDataByHandle(h, mass, centre, inertia, off))
    return false;
  const double c3[3] = {centre[0], centre[1], centre[2]};
  double rc[3];
  b3dop::quatRotate(pose.q, c3, rc);
  for (int k = 0; k < 3; ++k) {
    out.pos[k] = pose.p[k] + rc[k];
    out.rot[k] = pose.q[k];
    out.inertia[k] = inertia[k];
    out.inertiaOff[k] = off[k];
  }
  out.rot[3] = pose.q[3];
  out.mass = mass;
  return true;
}

Side Estimator::resolve(BodyHandle body, void *userData, uint64_t shape,
                        bool awake) {
  Side s;
  s.body = body;
  s.awake = awake;
  s.entry = static_cast<Box3DBodyMirror *>(userData);
  const auto n = myState.glueNodeOf.find(body);
  if (n != myState.glueNodeOf.end()) {
    s.node = n->second;
    return s;
  }
  const auto c = myCompounds.find(body);
  if (c == myCompounds.end())
    return s;
  auto shapes = myShapes.find(body);
  if (shapes == myShapes.end()) {
    shapes = myShapes.emplace(body, std::unordered_map<uint64_t, int>()).first;
    myState.world.shapeKeysByHandle(body, myKeys);
    for (const Box3DCompoundMember &m : c->second->members) {
      const auto node = myState.glueNodeOf.find(m.body);
      if (node == myState.glueNodeOf.end())
        continue;
      for (int i = m.shapeFirst; i < m.shapeFirst + m.shapeCount; ++i)
        if (static_cast<size_t>(i) < myKeys.size())
          shapes->second[myKeys[static_cast<size_t>(i)]] = node->second;
    }
  }
  const auto piece = shapes->second.find(shape);
  if (piece != shapes->second.end()) {
    s.node = piece->second;
    s.entry = static_cast<Box3DBodyMirror *>(myState.world.bodyUserDataByHandle(
        myState.glueNodes[static_cast<size_t>(s.node)].body));
  }
  return s;
}

// The rigid body a glue group makes: every piece merged.
const b3dop::GlueBody *Estimator::groupBody(int group) {
  const auto known = myGroups.find(group);
  if (known != myGroups.end())
    return &known->second;
  if (myGroupBad.count(group))
    return nullptr;
  myGroupBad[group] = true;
  if (!myGroupNodesMade) {
    myGroupNodesMade = true;
    for (size_t i = 0; i < myState.glueNodes.size(); ++i)
      myGroupNodes[myState.glueGroup[i]].push_back(i);
  }
  std::vector<b3dop::RigidPiece> pieces;
  for (const size_t i : myGroupNodes[group]) {
    const BodyHandle h = myState.glueNodes[i].body;
    const Box3DBodyMirror *e = static_cast<const Box3DBodyMirror *>(
        myState.world.bodyUserDataByHandle(h));
    b3dop::RigidPiece p;
    if (!e || e->type != b3dop::BodySettings::Type::Dynamic ||
        !rigidPiece(h, p))
      return nullptr;
    pieces.push_back(p);
  }
  const b3dop::MergedBody mb = b3dop::mergePieces(pieces);
  if (pieces.empty() || !mb.ok)
    return nullptr;
  myGroupBad.erase(group);
  return &(myGroups[group] = b3dop::glueBodyOf(mb));
}

// A body that is not glued: its own rigid body if it is dynamic, else one that
// takes no impulse. A dynamic body that is not known as a body (a compound's
// body whose shape belongs to no piece) has none: nullptr, so the contact is
// left to its real impulse.
const b3dop::GlueBody *Estimator::plainBody(const Side &s) {
  const auto known = myBodies.find(s.body);
  if (known != myBodies.end())
    return &known->second;
  if (myBodyBad.count(s.body))
    return nullptr;
  b3dop::GlueBody body;
  if (myState.world.typeByHandle(s.body) ==
      b3dop::BodySettings::Type::Dynamic) {
    b3dop::RigidPiece p;
    if (!s.entry || !rigidPiece(s.body, p)) {
      myBodyBad.insert(s.body);
      return nullptr;
    }
    body = b3dop::glueBodyOf(b3dop::mergePieces({p}));
  }
  return &(myBodies[s.body] = body);
}

float Estimator::restitutionOf(const Side &s) const {
  if (s.entry)
    return s.entry->props.restitution;
  float friction = 0.0f, restitution = 0.0f;
  myState.world.getMaterialByHandle(s.body, friction, restitution);
  return restitution;
}

void Estimator::run(const std::vector<b3dop::ContactImpulse> &events,
                    const std::vector<b3dop::HitContact> &hits) {
  Box3DSharedState &st = myState;
  std::unordered_map<PairKey, float, PairHash> speed;
  speed.reserve(hits.size() * 2);
  for (const b3dop::HitContact &h : hits)
    speed[PairKey(h.shapeA, h.shapeB)] = h.approachSpeed;
  const std::unordered_set<int> earlier = st.glueEstimated;
  std::vector<int> touched;
  for (const b3dop::ContactImpulse &ev : events) {
    if (!(ev.impulse > 0.0f))
      continue;
    const auto hit = speed.find(PairKey(ev.shapeA, ev.shapeB));
    if (hit == speed.end())
      continue;
    Side a = resolve(ev.a, ev.userDataA, ev.shapeA, ev.awakeA);
    Side b = resolve(ev.b, ev.userDataB, ev.shapeB, ev.awakeB);
    if (a.node < 0 && b.node < 0)
      continue;
    int groupA = -1, groupB = -1;
    if (a.node >= 0)
      groupA = st.glueGroup[static_cast<size_t>(a.node)];
    if (b.node >= 0)
      groupB = st.glueGroup[static_cast<size_t>(b.node)];
    if (a.node >= 0 && b.node >= 0 && groupA == groupB)
      continue;
    // A group that has its estimate from an earlier step of the frame has it.
    if ((groupA >= 0 && earlier.count(groupA)) ||
        (groupB >= 0 && earlier.count(groupB)))
      continue;
    // A body of the other side's own group (a compound's body against a held
    // piece of it) is no contact either.
    if ((a.node >= 0 && glueSameGroup(st, b.body, groupA)) ||
        (b.node >= 0 && glueSameGroup(st, a.body, groupB)))
      continue;
    const b3dop::GlueBody *bodyA =
        a.node >= 0 ? groupBody(groupA) : plainBody(a);
    const b3dop::GlueBody *bodyB =
        b.node >= 0 ? groupBody(groupB) : plainBody(b);
    if (!bodyA || !bodyB)
      continue;
    // The step's gravity speeds up what moves, as Bullet's trial step has it.
    float approach = hit->second;
    for (int i = 0; i < 3; ++i)
      approach +=
          static_cast<float>(((bodyA->moves ? myGravityStep[i] : 0.0f) -
                              (bodyB->moves ? myGravityStep[i] : 0.0f)) *
                             ev.normal[i]);
    const float impulse =
        b3dop::glueContactEstimate(*bodyA, *bodyB, ev.pos, ev.normal, approach,
                                   restitutionOf(a) * restitutionOf(b));
    if (!(impulse > 0.0f))
      continue;
    if (a.node >= 0 && a.awake) {
      st.glueNodes[static_cast<size_t>(a.node)].estimateStep += impulse;
      touched.push_back(a.node);
    }
    if (b.node >= 0 && b.awake) {
      st.glueNodes[static_cast<size_t>(b.node)].estimateStep += impulse;
      touched.push_back(b.node);
    }
  }
  // One estimate for the frame, as Bullet makes one from the speeds the frame
  // begins with: a group takes the first step that has one. A later step's
  // contacts are moving because of what the earlier ones did (a row a hit has
  // pushed into the ground), which Bullet's trial does not see. Taking the
  // largest of the steps' instead holds a box dropped on a row 1.7x to 2x
  // looser than Bullet (`glue_break_match`).
  for (const int i : touched) {
    Box3DGlueNode &n = st.glueNodes[static_cast<size_t>(i)];
    n.estimate += n.estimateStep;
    n.estimateStep = 0.0f;
    st.glueEstimated.insert(st.glueGroup[static_cast<size_t>(i)]);
  }
}

} // namespace

void estimateGlueHits(Box3DSharedState &st) {
  if (st.glueNodes.empty() || st.glueGroup.size() != st.glueNodes.size())
    return;
  // No bond that can still break (all broken, or none with a finite Strength):
  // the estimate would be read by nothing.
  const bool canBreak = std::any_of(
      st.glueBonds.begin(), st.glueBonds.end(), [](const Box3DJointEntry *e) {
        const float s = e->extra->glueParams.strength;
        return !e->broken && s >= 0.0f && std::isfinite(s);
      });
  if (!canBreak)
    return;
  // Nothing came together fast this step (a body at rest): nothing to estimate.
  thread_local std::vector<b3dop::HitContact> hits;
  if (st.world.getHitContacts(hits) == 0)
    return;
  thread_local std::vector<b3dop::ContactImpulse> events;
  st.world.getContactImpulseEvents(events);
  Estimator(st).run(events, hits);
}
