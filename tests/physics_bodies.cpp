// physics_bodies.cpp
//
// Physics cases: body settings, filtering, forces, sleep and world settings.
// The case list and main are in core_physics.cpp.
#include "KeyedWorld.h"
#include "PhysicsCases.h"
#include "TestUtil.h"
#include "core/Box3DShapes.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

// Motion locks (Box3D's b3MotionLocks, world axes).
void testMotionLocks() {
  using namespace b3dop;
  KeyedWorld lw;
  CHECK(lw.create(WorldSettings()));
  const float qid[4] = {0, 0, 0, 1};
  ShapeSpec cube;
  // linear Y locked: gravity doesn't move it; X still moves
  BodySettings ly;
  ly.props.locks.linear[1] = true;
  const double p0[3] = {0, 5, 0};
  const BodyHandle a = lw.createBody(ly, p0, qid);
  CHECK(lw.createShape(a, ly, cube));
  CHECK(lw.motionLocksByHandle(a) == ly.props.locks);
  const float vx[3] = {2, 0, 0}, w0[3] = {0, 0, 0};
  CHECK(lw.setVelocityByHandle(a, vx, w0));
  // angular X and Z locked, spun about all three: only Y rotation survives
  BodySettings az;
  az.props.locks.angular[0] = az.props.locks.angular[2] = true;
  az.props.gravityScale = 0.0f;
  const double p1[3] = {10, 5, 0};
  const BodyHandle b = lw.createBody(az, p1, qid);
  CHECK(lw.createShape(b, az, cube));
  const float v0[3] = {0, 0, 0}, spin[3] = {3, 3, 3};
  CHECK(lw.setVelocityByHandle(b, v0, spin));
  for (int i = 0; i < 60; ++i)
    lw.step(1.0f / 60.0f);
  double p[3];
  float q[4], lin[3], ang[3];
  CHECK(lw.getTransformByHandle(a, p, q) && std::fabs(p[1] - 5.0) < 1e-4 &&
        p[0] > 1.5);
  CHECK(lw.getVelocityByHandle(b, lin, ang) && ang[0] == 0.0f &&
        ang[2] == 0.0f && std::fabs(ang[1]) > 1.0f);
  CHECK(lw.getTransformByHandle(b, p, q) && std::fabs(q[0]) < 1e-4 &&
        std::fabs(q[2]) < 1e-4); // rotated about Y only
  // unlock mid-sim: it falls; lock again: the locked velocity is zeroed and
  // it stops
  lw.setMotionLocksByHandle(a, MotionLocks());
  for (int i = 0; i < 30; ++i)
    lw.step(1.0f / 60.0f);
  CHECK(lw.getTransformByHandle(a, p, q) && p[1] < 4.5);
  MotionLocks all;
  for (int i = 0; i < 3; ++i)
    all.linear[i] = all.angular[i] = true;
  lw.setMotionLocksByHandle(a, all);
  CHECK(lw.motionLocksByHandle(a) == all);
  CHECK(lw.getVelocityByHandle(a, lin, ang) && lin[0] == 0.0f &&
        lin[1] == 0.0f && lin[2] == 0.0f);
  double before[3];
  lw.getTransformByHandle(a, before, q);
  for (int i = 0; i < 30; ++i)
    lw.step(1.0f / 60.0f);
  CHECK(lw.getTransformByHandle(a, p, q) &&
        std::fabs(p[0] - before[0]) < 1e-5 &&
        std::fabs(p[1] - before[1]) < 1e-5);
  std::printf("motion lock checks done\n");
}

// Fast rotation (allowFastRotation: no angular speed cap).
void testFastRotation() {
  using namespace b3dop;
  KeyedWorld rw;
  CHECK(rw.create(WorldSettings()));
  const float qid[4] = {0, 0, 0, 1};
  ShapeSpec ball;
  ball.type = ShapeType::Sphere;
  ball.radius = 0.5f;
  BodySettings capped;
  capped.props.gravityScale = 0.0f;
  BodySettings fast = capped;
  fast.props.fastRotation = true;
  const double pa[3] = {0, 0, 0}, pb[3] = {5, 0, 0};
  const BodyHandle a = rw.createBody(capped, pa, qid),
                   b = rw.createBody(fast, pb, qid);
  CHECK(rw.createShape(a, capped, ball) && rw.createShape(b, fast, ball));
  CHECK(!rw.fastRotationByHandle(a) && rw.fastRotationByHandle(b));
  const float v0[3] = {0, 0, 0}, spin[3] = {0, 100, 0}; // rad/s
  CHECK(rw.setVelocityByHandle(a, v0, spin) &&
        rw.setVelocityByHandle(b, v0, spin));
  const float dt = 1.0f / 60.0f,
              cap = 0.25f * 3.14159265f / dt; // B3_MAX_ROTATION per step
  rw.step(dt);
  float lin[3], ang[3];
  CHECK(rw.getVelocityByHandle(a, lin, ang) &&
        std::fabs(ang[1] - cap) < 0.01f * cap);
  CHECK(rw.getVelocityByHandle(b, lin, ang) &&
        std::fabs(ang[1] - 100.0f) < 0.5f);
  // live: turning it on lets a capped body spin up again
  rw.setFastRotationByHandle(a, true);
  CHECK(rw.fastRotationByHandle(a));
  CHECK(rw.setVelocityByHandle(a, v0, spin));
  rw.step(dt);
  CHECK(rw.getVelocityByHandle(a, lin, ang) &&
        std::fabs(ang[1] - 100.0f) < 0.5f);
  std::printf("fast rotation checks done\n");
}

// BodyProps::operator==. The solver detects changes with it, so a field it
// forgets would never be noticed: change each one alone and expect a
// difference.
void testBodyPropsEquality() {
  using namespace b3dop;
  const BodyProps base;
  CHECK(base == BodyProps() && !(base != BodyProps()));
  CHECK(base == BodySettings().props);
  const std::vector<void (*)(BodyProps &)> edits = {
      [](BodyProps &p) { p.mass += 1.0f; },
      [](BodyProps &p) { p.density += 1.0f; },
      [](BodyProps &p) { p.computeMass = !p.computeMass; },
      [](BodyProps &p) { p.inertiaScale += 1.0f; },
      [](BodyProps &p) { p.customInertia = true; },
      [](BodyProps &p) { p.friction += 1.0f; },
      [](BodyProps &p) { p.restitution += 1.0f; },
      [](BodyProps &p) { p.rollingResistance += 1.0f; },
      [](BodyProps &p) { p.tangentVelocity[0] += 1.0f; },
      [](BodyProps &p) { p.tangentVelocity[1] += 1.0f; },
      [](BodyProps &p) { p.tangentVelocity[2] += 1.0f; },
      [](BodyProps &p) { p.linearDamping += 1.0f; },
      [](BodyProps &p) { p.angularDamping += 1.0f; },
      [](BodyProps &p) { p.gravityScale += 1.0f; },
      [](BodyProps &p) { p.contactRecycling = !p.contactRecycling; },
      [](BodyProps &p) { p.enableSleep = !p.enableSleep; },
      [](BodyProps &p) { p.enabled = !p.enabled; },
      [](BodyProps &p) { p.fastObject = !p.fastObject; },
      [](BodyProps &p) { p.fastRotation = !p.fastRotation; },
      [](BodyProps &p) { p.sleepThreshold += 1.0f; },
      [](BodyProps &p) { p.ccdThreshold += 1.0f; },
      [](BodyProps &p) { p.filter.category = 1; },
      [](BodyProps &p) { p.filter.mask = 1; },
      [](BodyProps &p) { p.filter.group = 1; },
      [](BodyProps &p) { p.locks.linear[2] = true; },
      [](BodyProps &p) { p.locks.angular[1] = true; },
  };
  for (size_t i = 0; i < edits.size(); ++i) {
    BodyProps changed = base;
    edits[i](changed);
    if (!(changed != base && !(changed == base)))
      std::printf("BodyProps edit %zu not detected\n", i);
    CHECK(changed != base && !(changed == base));
  }
  // a NaN setting equals itself (an unchanged NaN is no change) but differs
  // from a number
  const float nan = std::nanf("");
  BodyProps na = base, nb = base;
  na.friction = nb.friction = nan;
  na.tangentVelocity[1] = nb.tangentVelocity[1] = nan;
  CHECK(na == nb && !(na != nb));
  nb.friction = 0.6f;
  CHECK(na != nb);
  // the custom inertia tensor counts only while it is on
  BodyProps ca = base, cb = base;
  ca.customInertia = cb.customInertia = true;
  ca.inverseInertia[0] = cb.inverseInertia[0] = 0.1f;
  CHECK(ca == cb);
  cb.inverseInertia[3] = 0.01f;
  CHECK(ca != cb);
  cb.inverseInertia[3] = nan;
  ca.inverseInertia[3] = nan;
  CHECK(ca == cb);
  cb.customInertia = false;
  CHECK(cb == base); // a tensor left behind while off is no difference
  // 26 independent edits: one per scalar, and the filter and lock members one
  // by one
  CHECK(edits.size() == 26);
  std::printf("BodyProps comparison checks done\n");
}

// CCD threshold (Box3D safetyFactor).
void testCcdThreshold() {
  using namespace b3dop;
  CHECK(BodySettings().props.ccdThreshold == 0.5f); // Box3D's default
  // a fast cube at a thin static wall: swept (default) it stops; with a huge
  // threshold it is never "fast", so it tunnels straight through
  auto run = [&](float threshold, bool liveSet) {
    KeyedWorld cw;
    CHECK(cw.create(WorldSettings()));
    const float qid[4] = {0, 0, 0, 1};
    BodySettings wall;
    wall.type = BodySettings::Type::Static;
    ShapeSpec ws;
    ws.halfExtents[0] = 0.05f;
    ws.halfExtents[1] = ws.halfExtents[2] = 5.0f;
    const double wp[3] = {0, 0, 0};
    CHECK(cw.createShape(cw.createBody(wall, wp, qid), wall, ws));
    BodySettings fast;
    fast.props.gravityScale = 0.0f;
    if (!liveSet)
      fast.props.ccdThreshold = threshold;
    ShapeSpec cube;
    cube.halfExtents[0] = cube.halfExtents[1] = cube.halfExtents[2] = 0.1f;
    const double cp[3] = {-2, 0, 0};
    const BodyHandle c = cw.createBody(fast, cp, qid);
    CHECK(cw.createShape(c, fast, cube));
    if (liveSet)
      cw.setCcdThresholdByHandle(c, threshold);
    CHECK(std::fabs(cw.ccdThresholdByHandle(c) - threshold) < 1e-6f);
    const float v[3] = {300, 0, 0}, spin[3] = {0, 0, 0}; // 5 m per 1/60 step
    CHECK(cw.setVelocityByHandle(c, v, spin));
    for (int i = 0; i < 3; ++i)
      cw.step(1.0f / 60.0f);
    double p[3];
    float q[4];
    CHECK(cw.getTransformByHandle(c, p, q));
    return p[0];
  };
  CHECK(run(0.5f, false) < 0.0);    // stopped by the wall
  CHECK(run(1000.0f, false) > 1.0); // tunnelled
  CHECK(run(1000.0f, true) > 1.0);  // the same, set on the live body
  {
    KeyedWorld cw;
    CHECK(cw.create(WorldSettings()));
    const float qid[4] = {0, 0, 0, 1};
    const double p0[3] = {0, 0, 0};
    BodySettings b;
    b.props.ccdThreshold = -1.0f; // invalid: Box3D's default, no assert
    const BodyHandle h = cw.createBody(b, p0, qid);
    CHECK(std::fabs(cw.ccdThresholdByHandle(h) - 0.5f) < 1e-6f);
    cw.setCcdThresholdByHandle(h, 0.1f);
    CHECK(std::fabs(cw.ccdThresholdByHandle(h) - 0.1f) < 1e-6f);
    cw.setCcdThresholdByHandle(h, std::nanf(""));
    CHECK(std::fabs(cw.ccdThresholdByHandle(h) - 0.5f) < 1e-6f);
  }
  std::printf("ccd threshold checks done\n");
}

// Collision filtering (Box3D's b3Filter).
void testCollisionFiltering() {
  using namespace b3dop;
  CollisionFilter def;
  CHECK(def.category == ~static_cast<uint64_t>(0) &&
        def.mask == ~static_cast<uint64_t>(0) && def.group == 0);
  CollisionFilter debris;
  debris.category = 2;
  debris.mask = ~static_cast<uint64_t>(2); // ignores other debris
  CHECK(debris != def && debris == debris);

  // In a world: two debris boxes dropped on each other fall through, a third
  // collides
  KeyedWorld fw;
  CHECK(fw.create(WorldSettings()));
  const float qid[4] = {0, 0, 0, 1};
  BodySettings st;
  st.type = BodySettings::Type::Static; // default filter
  const double gp[3] = {0, -0.5, 0};
  ShapeSpec gs;
  gs.halfExtents[0] = gs.halfExtents[2] = 50.0f;
  gs.halfExtents[1] = 0.5f;
  const BodyHandle gh = fw.createBody(st, gp, qid);
  CHECK(fw.createShape(gh, st, gs));
  ShapeSpec cube;
  BodySettings dd;
  dd.props.filter = debris;
  const double p1[3] = {0, 0.5, 0}, p2[3] = {0, 3, 0};
  const BodyHandle d1 = fw.createBody(dd, p1, qid),
                   d2 = fw.createBody(dd, p2, qid);
  CHECK(fw.createShape(d1, dd, cube) && fw.createShape(d2, dd, cube));
  for (int i = 0; i < 180; ++i)
    fw.step(1.0f / 60.0f);
  double p[3];
  float q[4];
  CHECK(fw.getTransformByHandle(d2, p, q) &&
        std::fabs(p[1] - 0.5) < 0.1); // fell through d1 onto the ground
  // the default filter: d2 now collides with d1, so the two are pushed apart
  // (they overlap exactly, so the direction is Box3D's choice)
  fw.setFilterByHandle(d2, CollisionFilter());
  for (int i = 0; i < 300; ++i)
    fw.step(1.0f / 60.0f);
  double o[3];
  CHECK(fw.getTransformByHandle(d2, p, q) && fw.getTransformByHandle(d1, o, q));
  CHECK(std::fabs(p[0] - o[0]) > 0.9 || std::fabs(p[1] - o[1]) > 0.9 ||
        std::fabs(p[2] - o[2]) > 0.9);

  // groups: the same negative group never collides, whatever the bits; the
  // same positive group always does, even when the masks would say no
  {
    KeyedWorld gw;
    CHECK(gw.create(WorldSettings()));
    const BodyHandle g2 = gw.createBody(st, gp, qid);
    CHECK(gw.createShape(g2, st, gs));
    BodySettings neg;
    neg.props.filter.group = -3;
    BodySettings pos;
    pos.props.filter.group = 4;
    pos.props.filter.mask = 0; // a zero mask hits nothing...
    const double a1[3] = {0, 0.5, 0}, a2[3] = {0, 3, 0}, b1[3] = {5, 0.5, 0},
                 b2[3] = {5, 3, 0};
    const BodyHandle n1 = gw.createBody(neg, a1, qid),
                     n2 = gw.createBody(neg, a2, qid);
    const BodyHandle q1 = gw.createBody(pos, b1, qid),
                     q2 = gw.createBody(pos, b2, qid);
    CHECK(gw.createShape(n1, neg, cube) && gw.createShape(n2, neg, cube));
    CHECK(gw.createShape(q1, pos, cube) && gw.createShape(q2, pos, cube));
    // ...so the positive group's bottom box needs the ground: keep it static
    gw.setTypeByHandle(q1, BodySettings::Type::Static);
    for (int i = 0; i < 180; ++i)
      gw.step(1.0f / 60.0f);
    double pn[3], pq[3];
    CHECK(gw.getTransformByHandle(n2, pn, q) &&
          std::fabs(pn[1] - 0.5) < 0.1); // through n1
    CHECK(gw.getTransformByHandle(q2, pq, q) &&
          std::fabs(pq[1] - 1.5) < 0.1); // stacked on q1
  }

  // contacts: d1 rests on the ground. Asked from the ground's side, the
  // normal points from the ground up into d1, with positive impulse
  std::vector<ContactPoint> cps;
  CHECK(fw.getContactsByHandle(gh, cps) > 0);
  bool sawD1 = false;
  for (const ContactPoint &c : cps)
    if (c.other == d1) {
      sawD1 = true;
      CHECK(c.impulse > 0.0f &&
            c.normal[1] > 0.9f);        // from the ground up into d1
      CHECK(std::fabs(c.pos[1]) < 0.1); // at the ground's surface
    }
  CHECK(sawD1);

  // the lighter call adds up the same impulses, and leaves out the bodies it is
  // told to
  float total = 0.0f, fromD1 = 0.0f;
  for (const ContactPoint &c : cps) {
    total += c.impulse;
    if (c.other == d1)
      fromD1 += c.impulse;
  }
  CHECK(std::fabs(fw.contactImpulseByHandle(gh, nullptr, 0) - total) <=
        1e-3f * total + 1e-6f);
  CHECK(std::fabs(fw.contactImpulseByHandle(gh, &d1, 1) - (total - fromD1)) <=
        1e-3f * total + 1e-6f);

  // live tuning is harmless and applied
  WorldSettings tuned;
  tuned.contactPushSpeed = 1.0f;
  tuned.contactStiffness = 60.0f;
  tuned.contactDampingRatio = 5.0f;
  tuned.restitutionThreshold = 2.0f;
  tuned.maxLinearSpeed = 100.0f;
  tuned.enableContinuous = false;
  tuned.subSteps = 8;
  fw.applyTuning(tuned);
  CHECK(fw.settings().subSteps == 8 && fw.settings().contactStiffness == 60.0f);
  fw.step(1.0f / 60.0f);
  std::printf("collision group checks done\n");
}

// Mass properties, sleep thresholds, kinematic targets.
void testMassSleepKinematic() {
  using namespace b3dop;
  KeyedWorld mw;
  CHECK(mw.create(WorldSettings()));
  const float qid[4] = {0, 0, 0, 1};
  BodySettings dyn;
  const double p0[3] = {0, 10, 0};
  // unit cube offset 1 along x: centre of mass at (1, 0, 0)
  ShapeSpec off;
  off.center[0] = 1.0f;
  const BodyHandle b = mw.createBody(dyn, p0, qid);
  CHECK(mw.createShape(b, dyn, off));
  float m, c[3], I[3];
  mw.applyMassByHandle(b, 2.0f, false, 1.0f);
  CHECK(mw.getMassDataByHandle(b, m, c, I) && std::fabs(m - 2.0f) < 1e-4f &&
        std::fabs(c[0] - 1.0f) < 1e-4f);
  const float Ixx = I[0], Iyy = I[1]; // cube: 2 * (1+1)/12 = 1/3 each
  CHECK(std::fabs(Ixx - 1.0f / 3.0f) < 1e-3f &&
        std::fabs(Iyy - 1.0f / 3.0f) < 1e-3f);
  mw.applyMassByHandle(b, 2.0f, true, 1.0f); // centre moved to the origin
  CHECK(mw.getMassDataByHandle(b, m, c, I) && std::fabs(c[0]) < 1e-5f);
  CHECK(std::fabs(I[0] - Ixx) < 1e-4f &&
        std::fabs(I[1] - (Iyy + 2.0f)) < 1e-3f); // + m d^2 off the x axis
  mw.applyMassByHandle(b, 2.0f, false, 3.0f);    // inertialtensorstiffness 3
  CHECK(mw.getMassDataByHandle(b, m, c, I) &&
        std::fabs(I[0] - 3.0f * Ixx) < 1e-3f && std::fabs(m - 2.0f) < 1e-4f);

  // sleep threshold: set, back to Box3D's default, and from BodySettings at
  // creation
  const float def = mw.sleepThresholdByHandle(b);
  mw.setSleepThresholdByHandle(b, 0.2f);
  CHECK(std::fabs(mw.sleepThresholdByHandle(b) - 0.2f) < 1e-6f);
  mw.setSleepThresholdByHandle(b, -1.0f);
  CHECK(mw.sleepThresholdByHandle(b) == def);
  BodySettings sleepy;
  sleepy.props.sleepThreshold = 0.5f;
  const double ps[3] = {-5, 10, 0};
  const BodyHandle sb = mw.createBody(sleepy, ps, qid);
  CHECK(sb && std::fabs(mw.sleepThresholdByHandle(sb) - 0.5f) < 1e-6f);
  mw.setBulletByHandle(sb, true);

  // kinematic target: moves there in one step, then keeps the velocity until
  // stopped
  BodySettings kin;
  kin.type = BodySettings::Type::Kinematic;
  const double k0[3] = {5, 0, 0};
  const BodyHandle k = mw.createBody(kin, k0, qid);
  ShapeSpec cube;
  CHECK(mw.createShape(k, kin, cube));
  const double k1[3] = {5, 1, 0};
  mw.setTargetTransformByHandle(k, k1, qid, 1.0f / 60.0f);
  mw.step(1.0f / 60.0f);
  double p[3];
  float q[4], lin[3], ang[3];
  CHECK(mw.getTransformByHandle(k, p, q) && std::fabs(p[1] - 1.0) < 1e-3);
  CHECK(mw.getVelocityByHandle(k, lin, ang) &&
        std::fabs(lin[1] - 60.0f) < 0.1f);
  std::printf("mass / sleep / kinematic checks done\n");
}

// External forces (what the solver does with Force DOPs).
void testExternalForces() {
  using namespace b3dop;
  WorldSettings zg;
  zg.gravity[1] = 0.0f; // no gravity: only our forces move things
  KeyedWorld fw;
  CHECK(fw.create(zg));
  const float qid[4] = {0, 0, 0, 1};
  ShapeSpec cube;   // unit cube
  BodySettings dyn; // density 1 -> mass 1
  const double at[3] = {1, 2, 3};
  const BodyHandle b = fw.createBody(dyn, at, qid);
  CHECK(b && fw.createShape(b, dyn, cube));
  const float mass = fw.massByHandle(b);
  CHECK(std::fabs(mass - 1.0f) < 1e-4f);

  // the centre of mass of a centred cube is its origin
  b3dop::BodyState comState;
  CHECK(fw.bodyStateByHandle(b, comState));
  const double *com = comState.com;
  CHECK(std::fabs(com[0] - 1) < 1e-5 && std::fabs(com[1] - 2) < 1e-5 &&
        std::fabs(com[2] - 3) < 1e-5);
  CHECK(!fw.bodyStateByHandle(0, comState));

  // bodyStateByHandle returns what the single getters do
  {
    BodyState bs;
    float l0[3], a0[3];
    CHECK(fw.bodyStateByHandle(b, bs) && fw.getVelocityByHandle(b, l0, a0));
    CHECK(bs.awake == fw.isAwakeByHandle(b) && bs.mass == fw.massByHandle(b));
    CHECK(bs.com[0] == com[0] && bs.com[1] == com[1] && bs.com[2] == com[2]);
    CHECK(bs.lin[0] == l0[0] && bs.lin[1] == l0[1] && bs.lin[2] == l0[2] &&
          bs.ang[1] == a0[1]);
    BodyState untouched;
    untouched.mass = 42.0f;
    CHECK(!fw.bodyStateByHandle(0, untouched) && untouched.mass == 42.0f);
  }

  // a force acts for the coming step only: v = F / m * dt after one step,
  // unchanged after two
  const float dt = 1.0f / 60.0f;
  const float F[3] = {12.0f, 0, 0}, none[3] = {0, 0, 0};
  float lin[3], ang[3];
  fw.applyForceByHandle(b, F, none, false);
  fw.step(dt);
  CHECK(fw.getVelocityByHandle(b, lin, ang));
  CHECK(std::fabs(lin[0] - 12.0f / mass * dt) < 1e-3f);
  CHECK(std::fabs(lin[1]) < 1e-6f && std::fabs(lin[2]) < 1e-6f);
  CHECK(std::fabs(ang[0]) < 1e-6f && std::fabs(ang[1]) < 1e-6f &&
        std::fabs(ang[2]) < 1e-6f);
  const float vAfterOne = lin[0];
  fw.step(dt); // not re-applied: Box3D cleared it
  CHECK(fw.getVelocityByHandle(b, lin, ang));
  CHECK(std::fabs(lin[0] - vAfterOne) < 1e-4f);
  std::printf("force: v after one step %.5f (expected %.5f)\n", vAfterOne,
              12.0f / mass * dt);

  // forces add up across calls in one step
  const float zero[3] = {0, 0, 0};
  fw.setVelocityByHandle(b, zero, zero);
  const float F2[3] = {0, 3.0f, 0};
  fw.applyForceByHandle(b, F2, none, false);
  fw.applyForceByHandle(b, F2, none, false);
  fw.step(dt);
  CHECK(fw.getVelocityByHandle(b, lin, ang));
  CHECK(std::fabs(lin[1] - 6.0f / mass * dt) < 1e-3f);

  // a torque spins the body without moving it
  fw.setVelocityByHandle(b, zero, zero);
  const float T[3] = {0, 0, 1.0f};
  fw.applyForceByHandle(b, none, T, false);
  fw.step(dt);
  CHECK(fw.getVelocityByHandle(b, lin, ang));
  CHECK(ang[2] > 0.0f);
  CHECK(std::fabs(lin[0]) < 1e-5f && std::fabs(lin[1]) < 1e-5f &&
        std::fabs(lin[2]) < 1e-5f);

  // non-finite components count as 0 (no assert, no NaN)
  fw.setVelocityByHandle(b, zero, zero);
  const float bad[3] = {std::nanf(""), 6.0f, INFINITY};
  fw.applyForceByHandle(b, bad, bad, false);
  fw.step(dt);
  double pp[3];
  float pq[4];
  CHECK(fw.getVelocityByHandle(b, lin, ang));
  CHECK(std::isfinite(lin[0]) && std::isfinite(lin[1]) &&
        std::isfinite(lin[2]));
  CHECK(std::fabs(lin[1] - 6.0f / mass * dt) < 1e-3f && lin[0] == 0.0f &&
        lin[2] == 0.0f);
  CHECK(std::isfinite(ang[0]) && std::isfinite(ang[1]) &&
        std::isfinite(ang[2]));
  CHECK(fw.getTransformByHandle(b, pp, pq) && std::isfinite(pp[0]) &&
        std::isfinite(pp[1]));

  // static and kinematic bodies take nothing; handle 0 is harmless
  BodySettings stat;
  stat.type = BodySettings::Type::Static;
  const double sp[3] = {20, 0, 0};
  const BodyHandle sb = fw.createBody(stat, sp, qid);
  CHECK(sb && fw.createShape(sb, stat, cube));
  BodySettings kin;
  kin.type = BodySettings::Type::Kinematic;
  const double kp[3] = {30, 0, 0};
  const BodyHandle kb = fw.createBody(kin, kp, qid);
  CHECK(kb && fw.createShape(kb, kin, cube));
  fw.applyForceByHandle(sb, F, T, true);
  fw.applyForceByHandle(kb, F, T, true);
  fw.applyForceByHandle(0, F, T, true);
  fw.step(dt);
  CHECK(fw.getVelocityByHandle(kb, lin, ang));
  CHECK(lin[0] == 0.0f && ang[2] == 0.0f);
  CHECK(fw.getVelocityByHandle(sb, lin, ang));
  CHECK(lin[0] == 0.0f && ang[2] == 0.0f);

  // sleeping: without wake the force is ignored, with wake it wakes the body
  // and acts
  const double sleeperPos[3] = {-10, 0, 0};
  const BodyHandle sleeper = fw.createBody(dyn, sleeperPos, qid);
  CHECK(sleeper && fw.createShape(sleeper, dyn, cube));
  for (int i = 0; i < 300 && fw.isAwakeByHandle(sleeper); ++i)
    fw.step(dt);
  CHECK(!fw.isAwakeByHandle(sleeper));
  fw.applyForceByHandle(sleeper, F, none, false);
  fw.step(dt);
  CHECK(!fw.isAwakeByHandle(sleeper));
  CHECK(fw.getVelocityByHandle(sleeper, lin, ang) && lin[0] == 0.0f);
  fw.applyForceByHandle(sleeper, F, none, true);
  CHECK(fw.isAwakeByHandle(sleeper));
  fw.step(dt);
  CHECK(fw.getVelocityByHandle(sleeper, lin, ang));
  CHECK(std::fabs(lin[0] - 12.0f / mass * dt) < 1e-3f);

  // gravity as a force (m * g) falls like the world's own gravity
  WorldSettings g1; // standard gravity
  KeyedWorld gw, fw2;
  CHECK(gw.create(g1));
  CHECK(fw2.create(zg));
  const double gp0[3] = {0, 50, 0};
  const BodyHandle ga = gw.createBody(dyn, gp0, qid);
  const BodyHandle gb = fw2.createBody(dyn, gp0, qid);
  CHECK(ga && gb && gw.createShape(ga, dyn, cube) &&
        fw2.createShape(gb, dyn, cube));
  const float gForce[3] = {0, -9.80665f * fw2.massByHandle(gb), 0};
  for (int i = 0; i < 60; ++i) {
    fw2.applyForceByHandle(gb, gForce, none, false);
    gw.step(dt);
    fw2.step(dt);
  }
  double ya[3], yb[3];
  float qa[4], qb[4];
  CHECK(gw.getTransformByHandle(ga, ya, qa) &&
        fw2.getTransformByHandle(gb, yb, qb));
  std::printf("gravity: world y=%.4f  as force y=%.4f\n", ya[1], yb[1]);
  CHECK(std::fabs(ya[1] - yb[1]) < 0.02);
  std::printf("external force checks done\n");
}

// Contact recycling: world distance + per-body toggle.
void testContactRecycling() {
  using namespace b3dop;
  const float qid[4] = {0, 0, 0, 1};
  const float dt = 1.0f / 60.0f;
  WorldSettings ws;
  ws.enableSleep =
      false; // keep the resting box awake so it keeps making contacts
  CHECK(ws.contactRecycleDistance == 0.05f); // Box3D's default

  ShapeSpec slab;
  slab.halfExtents[0] = 20;
  slab.halfExtents[2] = 20;
  ShapeSpec cube; // unit cube
  BodySettings stat;
  stat.type = BodySettings::Type::Static;
  BodySettings dyn;

  // A world with a floor and one resting box; returns the box.
  auto setup = [&](KeyedWorld &rw, const BodySettings &boxBody,
                   BodyHandle &boxOut) {
    const double gp[3] = {0, -0.5, 0}, bp[3] = {0, 0.5, 0};
    const BodyHandle g = rw.createBody(stat, gp, qid);
    boxOut = rw.createBody(boxBody, bp, qid);
    return g && boxOut && rw.createShape(g, stat, slab) &&
           rw.createShape(boxOut, boxBody, cube);
  };
  auto settle = [&](KeyedWorld &rw, int steps) {
    for (int i = 0; i < steps; ++i)
      rw.step(dt);
  };

  // default: a resting box has its contact recycled
  KeyedWorld rw;
  BodyHandle box = 0;
  CHECK(rw.create(ws) && setup(rw, dyn, box));
  CHECK(rw.contactRecyclingByHandle(box));
  settle(rw, 120);
  CHECK(rw.recycledContactCount() > 0);

  // the world distance is live: 0 turns recycling off, a distance turns it
  // back on
  WorldSettings off = ws;
  off.contactRecycleDistance = 0.0f;
  rw.applyTuning(off);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() == 0);
  CHECK(rw.settings().contactRecycleDistance == 0.0f);
  rw.applyTuning(ws);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() > 0);

  // a non-finite distance is ignored (previous value kept); a negative one
  // acts as 0
  WorldSettings bad = ws;
  bad.contactRecycleDistance = std::nanf("");
  rw.applyTuning(bad);
  CHECK(rw.settings().contactRecycleDistance == 0.05f);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() > 0);
  WorldSettings neg = ws;
  neg.contactRecycleDistance = -1.0f;
  rw.applyTuning(neg);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() == 0);

  // per body: off at creation means this body's contacts are never recycled
  // ...
  KeyedWorld pw;
  BodyHandle pbox = 0;
  BodySettings noRec = dyn;
  noRec.props.contactRecycling = false;
  CHECK(pw.create(ws) && setup(pw, noRec, pbox));
  CHECK(!pw.contactRecyclingByHandle(pbox));
  settle(pw, 120);
  CHECK(pw.recycledContactCount() == 0);
  // ... and turning it on later reaches the existing contact (awake dynamic
  // body)
  pw.setContactRecyclingByHandle(pbox, true);
  CHECK(pw.contactRecyclingByHandle(pbox));
  settle(pw, 10);
  CHECK(pw.recycledContactCount() > 0);

  // turning it off on a body already resting on the floor also reaches the
  // existing contact
  rw.applyTuning(ws);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() > 0);
  rw.setContactRecyclingByHandle(box, false);
  CHECK(!rw.contactRecyclingByHandle(box));
  settle(rw, 10);
  CHECK(rw.recycledContactCount() == 0);
  rw.setContactRecyclingByHandle(box, true);
  settle(rw, 10);
  CHECK(rw.recycledContactCount() > 0);

  // changing it leaves the body's velocity and awake state alone
  WorldSettings zg = ws;
  zg.gravity[1] = 0.0f;
  KeyedWorld vw;
  CHECK(vw.create(zg));
  const double vp[3] = {0, 5, 0};
  const BodyHandle vb = vw.createBody(dyn, vp, qid);
  CHECK(vb && vw.createShape(vb, dyn, cube));
  const float lin0[3] = {1.0f, 2.0f, 3.0f}, ang0[3] = {0.0f, 1.0f, 0.0f};
  vw.setVelocityByHandle(vb, lin0, ang0);
  vw.step(dt);
  vw.setContactRecyclingByHandle(vb, false);
  float lin[3], ang[3];
  CHECK(vw.getVelocityByHandle(vb, lin, ang));
  CHECK(std::fabs(lin[0] - 1.0f) < 1e-5f && std::fabs(lin[1] - 2.0f) < 1e-5f &&
        std::fabs(lin[2] - 3.0f) < 1e-5f);
  CHECK(std::fabs(ang[1] - 1.0f) < 1e-5f);
  CHECK(vw.isAwakeByHandle(vb) && vw.isEnabledByHandle(vb));

  // a sleeping body is not woken by the change, and static bodies and handle
  // 0 are harmless
  WorldSettings sleepy; // sleeping allowed
  KeyedWorld sw;
  BodyHandle sbox = 0;
  CHECK(sw.create(sleepy) && setup(sw, dyn, sbox));
  for (int i = 0; i < 400 && sw.isAwakeByHandle(sbox); ++i)
    sw.step(dt);
  CHECK(!sw.isAwakeByHandle(sbox));
  sw.setContactRecyclingByHandle(sbox, false);
  CHECK(!sw.isAwakeByHandle(sbox) && !sw.contactRecyclingByHandle(sbox));
  sw.setContactRecyclingByHandle(0, false);
  CHECK(!sw.contactRecyclingByHandle(0));
  std::printf("contact recycling checks done\n");
}

// Surface material: rolling resistance and conveyor (tangent) velocity.
void testSurfaceMaterial() {
  using namespace b3dop;
  const float qid[4] = {0, 0, 0, 1};
  const float dt = 1.0f / 60.0f;
  WorldSettings ws;
  ws.enableSleep = false;

  ShapeSpec slab;
  slab.halfExtents[0] = 40;
  slab.halfExtents[1] = 0.5f;
  slab.halfExtents[2] = 40;
  ShapeSpec cube;
  ShapeSpec ball;
  ball.type = ShapeType::Sphere;
  ball.radius = 0.5f;
  BodySettings stat;
  stat.type = BodySettings::Type::Static;
  const double gp[3] = {0, -0.5, 0};
  const float noTangent[3] = {0, 0, 0};

  // A ball rolling at 5 m/s: rolling resistance slows it, none does not
  // (much).
  float speedAfter[2] = {0, 0};
  double xAfter[2] = {0, 0};
  for (int k = 0; k < 2; ++k) {
    KeyedWorld rw;
    BodySettings ballBody;
    ballBody.props.friction = 0.6f;
    ballBody.props.rollingResistance = k == 0 ? 0.0f : 0.3f;
    CHECK(rw.create(ws));
    const BodyHandle g = rw.createBody(stat, gp, qid);
    const double bp[3] = {0, 0.5, 0};
    const BodyHandle s = rw.createBody(ballBody, bp, qid);
    CHECK(g && s && rw.createShape(g, stat, slab) &&
          rw.createShape(s, ballBody, ball));
    const float lin[3] = {5, 0, 0},
                ang[3] = {0, 0, -10}; // rolling without slipping: w = v / r
    CHECK(rw.setVelocityByHandle(s, lin, ang));
    float rolling = 0, tv[3];
    CHECK(rw.getSurfaceByHandle(s, rolling, tv));
    CHECK(rolling == ballBody.props.rollingResistance);
    for (int i = 0; i < 180; ++i)
      rw.step(dt);
    float l2[3], a2[3];
    double p2[3];
    float q2[4];
    CHECK(rw.getVelocityByHandle(s, l2, a2) &&
          rw.getTransformByHandle(s, p2, q2));
    speedAfter[k] = l2[0];
    xAfter[k] = p2[0];
  }
  std::printf("rolling resistance: free v=%.3f x=%.2f, 0.3 v=%.3f x=%.2f\n",
              speedAfter[0], xAfter[0], speedAfter[1], xAfter[1]);
  CHECK(speedAfter[0] > 4.0f);                 // barely slowed without it
  CHECK(speedAfter[1] < 0.5f * speedAfter[0]); // clearly slowed with it
  CHECK(xAfter[1] < xAfter[0]);

  // Cleaned input: negative / non-finite values become 0 instead of tripping
  // Box3D's asserts.
  {
    KeyedWorld cw;
    BodySettings bad = stat;
    bad.props.rollingResistance = -1.0f;
    bad.props.tangentVelocity[0] = std::nanf("");
    CHECK(cw.create(ws));
    const BodyHandle h = cw.createBody(bad, gp, qid);
    CHECK(h && cw.createShape(h, bad, slab));
    float rr = 1, tv[3] = {1, 1, 1};
    CHECK(cw.getSurfaceByHandle(h, rr, tv));
    CHECK(rr == 0.0f && tv[0] == 0.0f && tv[1] == 0.0f && tv[2] == 0.0f);
    const float good[3] = {1, 2, 3};
    cw.setSurfaceByHandle(h, 0.25f, good);
    CHECK(cw.getSurfaceByHandle(h, rr, tv));
    CHECK(rr == 0.25f && tv[0] == 1.0f && tv[1] == 2.0f && tv[2] == 3.0f);
    const float nan3[3] = {1, std::nanf(""), 3};
    cw.setSurfaceByHandle(h, std::nanf(""), nan3);
    CHECK(cw.getSurfaceByHandle(h, rr, tv));
    CHECK(rr == 0.0f && tv[0] == 0.0f && tv[1] == 0.0f && tv[2] == 0.0f);
    cw.setSurfaceByHandle(0, 1.0f, good); // null handle: nothing
    CHECK(!cw.getSurfaceByHandle(0, rr, tv));
  }

  // A conveyor: a static slab with a tangent velocity carries a box resting
  // on it.
  auto beltRun = [&](const float beltRot[4], const float tangent[3], int steps,
                     float lin[3]) {
    KeyedWorld cw;
    BodySettings belt = stat;
    for (int i = 0; i < 3; ++i)
      belt.props.tangentVelocity[i] = tangent[i];
    CHECK(cw.create(ws));
    const BodyHandle g = cw.createBody(belt, gp, beltRot);
    const double bp[3] = {0, 0.5, 0};
    BodySettings boxBody;
    const BodyHandle b = cw.createBody(boxBody, bp, qid);
    CHECK(g && b && cw.createShape(g, belt, slab) &&
          cw.createShape(b, boxBody, cube));
    for (int i = 0; i < steps; ++i)
      cw.step(dt);
    float ang[3];
    CHECK(cw.getVelocityByHandle(b, lin, ang));
  };
  {
    float lin[3];
    const float along[3] = {2, 0, 0};
    beltRun(qid, along, 120, lin);
    std::printf("belt +x: box v=(%.3f, %.3f, %.3f)\n", lin[0], lin[1], lin[2]);
    CHECK(std::fabs(lin[0] - 2.0f) < 0.2f); // carried at belt speed
    CHECK(std::fabs(lin[2]) < 0.1f);
    beltRun(qid, noTangent, 120, lin);
    CHECK(std::fabs(lin[0]) < 0.05f &&
          std::fabs(lin[2]) < 0.05f); // a plain slab carries nothing

    // The velocity is local to the belt: turned 90 degrees about Y, local +x
    // is world -z.
    const float s = std::sqrt(0.5f);
    const float turned[4] = {0, s, 0, s};
    beltRun(turned, along, 120, lin);
    std::printf("belt turned: box v=(%.3f, %.3f, %.3f)\n", lin[0], lin[1],
                lin[2]);
    CHECK(std::fabs(lin[0]) < 0.1f);
    CHECK(std::fabs(std::fabs(lin[2]) - 2.0f) < 0.2f);
  }

  // Switched on mid-simulation while the box sleeps on it (recycled contact,
  // sleeping body): the change wakes it and the box is carried.
  {
    WorldSettings sleepy; // sleeping on
    KeyedWorld cw;
    CHECK(cw.create(sleepy));
    const BodyHandle g = cw.createBody(stat, gp, qid);
    const double bp[3] = {0, 0.5, 0};
    BodySettings boxBody;
    const BodyHandle b = cw.createBody(boxBody, bp, qid);
    CHECK(g && b && cw.createShape(g, stat, slab) &&
          cw.createShape(b, boxBody, cube));
    for (int i = 0; i < 400 && cw.isAwakeByHandle(b); ++i)
      cw.step(dt);
    CHECK(!cw.isAwakeByHandle(b));
    const float along[3] = {2, 0, 0};
    cw.setSurfaceByHandle(g, 0.0f, along);
    CHECK(cw.isAwakeByHandle(b));
    for (int i = 0; i < 120; ++i)
      cw.step(dt);
    float lin[3], ang[3];
    CHECK(cw.getVelocityByHandle(b, lin, ang));
    std::printf("belt switched on mid-sim: box v=(%.3f, %.3f, %.3f)\n", lin[0],
                lin[1], lin[2]);
    CHECK(std::fabs(lin[0] - 2.0f) < 0.2f);
    // and off again: it is no longer carried (friction brings it to rest)
    cw.setSurfaceByHandle(g, 0.0f, noTangent);
    for (int i = 0; i < 240; ++i)
      cw.step(dt);
    CHECK(cw.getVelocityByHandle(b, lin, ang));
    CHECK(std::fabs(lin[0]) < 0.1f);
  }
  std::printf("surface material checks done\n");
}

// World settings: mixing rules, hit events, warm starting, restitution.
void testWorldSettings() {
  using namespace b3dop;
  const float qid2[4] = {0, 0, 0, 1};
  const double gpos[3] = {0, -0.5, 0};
  const double spos[3] = {0, 2.0, 0};
  const float gh2[3] = {10, 0.5f, 10};
  // A ball with bounce 0.9 on a ground with bounce 0: Maximum bounces,
  // Minimum does not.
  float peak[2] = {0, 0};
  for (int mode = 0; mode < 2; ++mode) {
    WorldSettings ws;
    ws.restitutionMixing = mode == 0
                               ? WorldSettings::RestitutionMixing::Maximum
                               : WorldSettings::RestitutionMixing::Minimum;
    ws.warmStarting = (mode == 0); // also runs with warm starting off
    ws.restitutionIterations = 4;
    ws.restitutionPropagation = true;
    ws.isotropicInertiaTolerance = 0.5f;
    KeyedWorld sw;
    CHECK(sw.create(ws));
    BodySettings g;
    g.type = BodySettings::Type::Static;
    g.props.restitution = 0.0f;
    BodySettings ball;
    ball.props.restitution = 0.9f;
    CHECK(sw.addBody(1, g, gpos, qid2));
    CHECK(sw.addBoxShape(1, gh2));
    CHECK(sw.addBody(2, ball, spos, qid2));
    CHECK(sw.addSphereShape(2, 0.5f));
    bool hit = false;
    for (int i = 0; i < 180; ++i) {
      sw.step(1.0f / 60.0f);
      double bp[3];
      float bq[4];
      if (sw.getTransform(2, bp, bq) && hit)
        peak[mode] = std::max(peak[mode], static_cast<float>(bp[1]));
      float blin[3], bang[3];
      if (sw.getVelocity(2, blin, bang) && blin[1] > 1.0f)
        hit = true;
    }
  }
  std::printf("mixing: bounce peak max=%.3f min=%.3f\n", peak[0], peak[1]);
  CHECK(peak[0] > peak[1] + 0.3f);

  // live tuning of the new settings, and hit events with a speed threshold
  WorldSettings ws;
  ws.impactMinSpeed = 3.0f;
  KeyedWorld hw;
  CHECK(hw.create(ws));
  BodySettings g;
  g.type = BodySettings::Type::Static;
  BodySettings ball;
  CHECK(hw.addBody(1, g, gpos, qid2));
  CHECK(hw.addBoxShape(1, gh2));
  CHECK(hw.addBody(2, ball, spos, qid2));
  CHECK(hw.addSphereShape(2, 0.5f));
  const BodyHandle hb = hw.bodyHandle(2), hg = hw.bodyHandle(1);
  bool sawHit = false;
  for (int i = 0; i < 120; ++i) {
    hw.step(1.0f / 60.0f);
    hw.updateHitEvents();
    if (hw.hadHitEvent(hb, hg) && hw.hadHitEvent(hg, hb))
      sawHit = true;
  }
  CHECK(sawHit); // fell ~1.5 m: about 5 m/s, above the 3 m/s threshold
  CHECK(!hw.hadHitEvent(hb, hb + 1000) &&
        !hw.hadHitEvent(hb, hb)); // pairs that never hit
  WorldSettings t2 = ws;
  t2.impactMinSpeed = 0.0f;
  t2.frictionMixing = WorldSettings::FrictionMixing::Average;
  t2.restitutionMixing = WorldSettings::RestitutionMixing::Average;
  t2.warmStarting = false;
  t2.restitutionIterations = 8;
  t2.restitutionPropagation = true;
  t2.isotropicInertiaTolerance = 0.1f;
  hw.applyTuning(t2);
  CHECK(hw.settings().frictionMixing ==
            WorldSettings::FrictionMixing::Average &&
        !hw.settings().warmStarting);
  hw.step(1.0f / 60.0f);
  std::printf("world settings checks done\n");
}

// Sleep and enable state: starting asleep / disabled, forced sleep, live
// enableSleep.
void testSleepEnable() {
  using namespace b3dop;
  const float qid[4] = {0, 0, 0, 1};
  const float dt = 1.0f / 60.0f;
  const double hi[3] = {0, 5, 0};
  ShapeSpec cube;
  double p[3];
  float q[4], lin[3], ang[3];
  const float zero[3] = {0, 0, 0};

  KeyedWorld sw; // standard gravity
  CHECK(sw.create(WorldSettings()));

  // starts asleep: it does not fall until woken
  BodySettings asleep;
  asleep.startAwake = false;
  const BodyHandle a = sw.createBody(asleep, hi, qid);
  CHECK(a && sw.createShape(a, asleep, cube));
  CHECK(!sw.isAwakeByHandle(a) && sw.isEnabledByHandle(a) &&
        sw.isSleepEnabledByHandle(a));
  for (int i = 0; i < 30; ++i)
    sw.step(dt);
  CHECK(sw.getTransformByHandle(a, p, q) && std::fabs(p[1] - 5.0) < 1e-6);
  CHECK(!sw.isAwakeByHandle(a));
  sw.wakeByHandle(a);
  CHECK(sw.isAwakeByHandle(a));
  for (int i = 0; i < 30; ++i)
    sw.step(dt);
  CHECK(sw.getTransformByHandle(a, p, q) && p[1] < 4.5);

  // a body that may not sleep starts awake whatever startAwake says (Box3D's
  // rule)
  BodySettings never = asleep;
  never.props.enableSleep = false;
  const BodyHandle n = sw.createBody(never, hi, qid);
  CHECK(n && sw.createShape(n, never, cube));
  CHECK(sw.isAwakeByHandle(n) && !sw.isSleepEnabledByHandle(n));

  // starts disabled: no motion, never awake; enabling makes it a normal awake
  // body
  BodySettings off;
  off.props.enabled = false;
  const BodyHandle d = sw.createBody(off, hi, qid);
  CHECK(d && sw.createShape(d, off, cube));
  CHECK(!sw.isEnabledByHandle(d) && !sw.isAwakeByHandle(d));
  for (int i = 0; i < 30; ++i)
    sw.step(dt);
  CHECK(sw.getTransformByHandle(d, p, q) && std::fabs(p[1] - 5.0) < 1e-6);
  sw.setEnabledByHandle(d, true);
  CHECK(sw.isEnabledByHandle(d) && sw.isAwakeByHandle(d));
  for (int i = 0; i < 30; ++i)
    sw.step(dt);
  CHECK(sw.getTransformByHandle(d, p, q) && p[1] < 4.5);

  // asleep and disabled together: enabling leaves it awake (Enable puts it in
  // the awake set)
  BodySettings both = off;
  both.startAwake = false;
  const BodyHandle bo = sw.createBody(both, hi, qid);
  CHECK(bo && sw.createShape(bo, both, cube));
  CHECK(!sw.isEnabledByHandle(bo) && !sw.isAwakeByHandle(bo));
  sw.setEnabledByHandle(bo, true);
  CHECK(sw.isAwakeByHandle(bo));

  // a velocity wakes a sleeping body (why the solver skips it for a body that
  // starts asleep)
  const BodyHandle sv = sw.createBody(asleep, hi, qid);
  CHECK(sv && sw.createShape(sv, asleep, cube));
  CHECK(!sw.isAwakeByHandle(sv));
  const float push[3] = {1, 0, 0};
  CHECK(sw.setVelocityByHandle(sv, push, zero));
  CHECK(sw.isAwakeByHandle(sv));

  // sleepByHandle: no gravity, so the body keeps still and awake until told
  // to sleep
  WorldSettings zg;
  zg.gravity[0] = zg.gravity[1] = zg.gravity[2] = 0.0f;
  KeyedWorld zw;
  CHECK(zw.create(zg));
  BodySettings dyn;
  const BodyHandle z = zw.createBody(dyn, hi, qid);
  CHECK(z && zw.createShape(z, dyn, cube));
  CHECK(zw.isAwakeByHandle(z));
  zw.sleepByHandle(z);
  CHECK(!zw.isAwakeByHandle(z));
  zw.step(dt);
  CHECK(!zw.isAwakeByHandle(z));
  zw.wakeByHandle(z);
  CHECK(zw.isAwakeByHandle(z));
  // turning sleep off wakes the body and keeps it from sleeping;
  // sleepByHandle then does nothing
  zw.sleepByHandle(z);
  CHECK(!zw.isAwakeByHandle(z));
  zw.setSleepEnabledByHandle(z, false);
  CHECK(zw.isAwakeByHandle(z) && !zw.isSleepEnabledByHandle(z));
  zw.sleepByHandle(z);
  CHECK(zw.isAwakeByHandle(z));
  zw.setSleepEnabledByHandle(z, true);
  zw.sleepByHandle(z);
  CHECK(!zw.isAwakeByHandle(z));
  // static, disabled and null handles are left alone
  BodySettings stat;
  stat.type = BodySettings::Type::Static;
  const BodyHandle st = zw.createBody(stat, hi, qid);
  CHECK(st && zw.createShape(st, stat, cube));
  zw.sleepByHandle(st);
  zw.sleepByHandle(0);
  CHECK(zw.isEnabledByHandle(st));
  zw.setEnabledByHandle(z, false);
  zw.sleepByHandle(z);
  CHECK(!zw.isEnabledByHandle(z));
  // with sleeping off in the world, sleepByHandle does nothing
  WorldSettings noSleep = zg;
  noSleep.enableSleep = false;
  KeyedWorld nw;
  CHECK(nw.create(noSleep));
  const BodyHandle nb = nw.createBody(dyn, hi, qid);
  CHECK(nb && nw.createShape(nb, dyn, cube));
  nw.sleepByHandle(nb);
  CHECK(nw.isAwakeByHandle(nb));
  (void)lin;
  (void)ang;
  std::printf("sleep / enable state checks done\n");
}
