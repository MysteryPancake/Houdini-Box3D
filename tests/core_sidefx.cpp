// SideFX constraint relationships (Glue, Hard, Spring, Bullet Soft, Cone Twist)
// mapped onto Box3D joints: the option table, the mapping per type and
// constraint_type, frame offsets, and that the mapped joints behave like the
// SideFX ones in a world.
#include "TestUtil.h"
#include "core/Box3DSideFX.h"
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace b3dop;

namespace {

const float kQid[4] = {0, 0, 0, 1};
const float kDt = 1.0f / 60.0f;

BodyHandle box(Box3DWorld &w, double x, double y, double z,
               float half = 0.25f) {
  BodySettings b;
  b.type = BodySettings::Type::Dynamic;
  const double p[3] = {x, y, z};
  const BodyHandle h = w.createBody(b, p, kQid);
  ShapeSpec s;
  s.type = ShapeType::Box;
  s.halfExtents[0] = s.halfExtents[1] = s.halfExtents[2] = half;
  w.createShape(h, b, s);
  return h;
}

JointFrame frame(double x, double y, double z, const float q[4] = kQid) {
  JointFrame f;
  f.pos[0] = x;
  f.pos[1] = y;
  f.pos[2] = z;
  for (int i = 0; i < 4; ++i)
    f.rot[i] = q[i];
  return f;
}

void pos(Box3DWorld &w, BodyHandle h, double out[3]) {
  float q[4];
  w.getTransformByHandle(h, out, q);
}

double dist(const double a[3], const double b[3]) {
  const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void steps(Box3DWorld &w, int n) {
  for (int i = 0; i < n; ++i)
    w.step(kDt);
}

SideFXBody bodyOf(const Box3DWorld &w, BodyHandle h) {
  SideFXBody b;
  float c[3], inertia[3];
  if (h && w.getMassDataByHandle(h, b.mass, c, inertia)) {
    b.dynamic = true;
    b.inertia = (inertia[0] + inertia[1] + inertia[2]) / 3.0f;
  }
  return b;
}

// What the Houdini side does: map, offset the frames, create.
JointHandle make(Box3DWorld &w, SideFXType t, const SideFXSettings &s,
                 ConstraintMode mode, BodyHandle a, BodyHandle b,
                 const JointFrame &fa, const JointFrame &fb) {
  SideFXContext c;
  c.timeStep = kDt;
  c.a = bodyOf(w, a);
  c.b = bodyOf(w, b);
  const SideFXJoint j = mapSideFX(t, s, mode, c);
  JointFrame adjA, adjB;
  sideFXFrameOffsets(j, fa, fb, adjA, adjB);
  return w.createJoint(j.type, a, b, applyFrameOffset(fa, adjA),
                       applyFrameOffset(fb, adjB), j.settings);
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

} // namespace

int main() {
  const int assertsBefore = Box3DWorld::assertCount();
  SideFXContext ctx;
  ctx.timeStep = kDt;

  // ---- data types and the option table ----
  {
    for (int i = 0; i < kSideFXTypeCount; ++i) {
      SideFXType t;
      CHECK(sideFXTypeFromDataType(sideFXDataType(SideFXType(i)), t) &&
            t == SideFXType(i));
      std::set<std::string> names;
      for (const SideFXParam &p : sideFXParams())
        if (p.usedBy(SideFXType(i)))
          CHECK(names.insert(p.name).second);
    }
    SideFXType t;
    CHECK(!sideFXTypeFromDataType("SIM_ConRelSoftAttach", t) &&
          !sideFXTypeFromDataType(nullptr, t));
    for (const SideFXParam &p : sideFXParams()) {
      const int members = (p.f ? 1 : 0) + (p.i ? 1 : 0) + (p.v ? 1 : 0);
      CHECK(members == 1 && p.types != 0);
    }
    CHECK(constraintModeFromString("position") == ConstraintMode::Position);
    CHECK(constraintModeFromString("rotation") == ConstraintMode::Rotation);
    CHECK(constraintModeFromString("all") == ConstraintMode::All);
    CHECK(constraintModeFromString("") == ConstraintMode::All &&
          constraintModeFromString(nullptr) == ConstraintMode::All);
    CHECK(defaultSideFXSettings(SideFXType::Glue).strength == 10000.0f);
    CHECK(defaultSideFXSettings(SideFXType::Spring).strength == 1.0f);
    SideFXSettings a, b;
    b.maxTwist = 5.0f;
    CHECK(sameSideFXSettings(SideFXType::Glue, a, b) &&
          !sameSideFXSettings(SideFXType::ConeTwist, a, b));
  }

  // ---- mapping ----
  {
    SideFXSettings s;
    // Glue: rigid weld holding the offset; strength is an impulse per step; -1
    // never breaks.
    SideFXJoint j =
        mapSideFX(SideFXType::Glue, s, ConstraintMode::Position, ctx);
    CHECK(j.type == JointType::Weld && !j.settings.enableLinearSpring &&
          !j.settings.enableAngularSpring);
    CHECK(j.holdSeparation && !j.settings.collideConnected);
    CHECK(near(j.settings.forceThreshold, 10000.0 / kDt, 1.0) &&
          j.settings.torqueThreshold < 0.0f);
    s.strength = -1.0f;
    CHECK(mapSideFX(SideFXType::Glue, s, ConstraintMode::All, ctx)
              .settings.forceThreshold < 0.0f);
    s.strength = 0.0f;
    CHECK(mapSideFX(SideFXType::Glue, s, ConstraintMode::All, ctx)
              .settings.forceThreshold == 0.0f);

    // Hard, by constraint_type.
    s = SideFXSettings();
    j = mapSideFX(SideFXType::Hard, s, ConstraintMode::All, ctx);
    CHECK(j.type == JointType::Weld && j.holdSeparation &&
          j.settings.collideConnected);
    j = mapSideFX(SideFXType::Hard, s, ConstraintMode::Position, ctx);
    CHECK(j.type == JointType::Distance && j.settings.restLength == 1.0f &&
          !j.settings.enableSpring);
    s.restLength = 0.0f;
    s.disableCollisions = 1;
    j = mapSideFX(SideFXType::Hard, s, ConstraintMode::Position, ctx);
    CHECK(j.type == JointType::Spherical && !j.settings.enableConeLimit &&
          !j.settings.collideConnected);
    j = mapSideFX(SideFXType::Hard, s, ConstraintMode::Rotation, ctx);
    CHECK(j.type == JointType::Motor && j.settings.linearStiffness == 0.0f &&
          j.settings.maxSpringForce == 0.0f);
    CHECK(j.settings.angularStiffness > 0.0f &&
          j.settings.maxSpringTorque > 1e30f);
    CHECK(j.settings.maxVelocityForce == 0.0f &&
          j.settings.maxVelocityTorque == 0.0f);

    // Soft: Hz straight through; the angular override.
    s = SideFXSettings();
    s.stiffness = 4.0f;
    s.dampingRatio = 0.5f;
    s.angularStiffness = 7.0f;
    s.angularDampingRatio = 0.25f;
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::All, ctx);
    CHECK(j.type == JointType::Motor && j.separation == 1.0f &&
          !j.holdSeparation);
    CHECK(j.settings.linearStiffness == 4.0f &&
          j.settings.angularStiffness == 4.0f &&
          j.settings.angularDampingRatio == 0.5f);
    s.enableAngular = 1;
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::All, ctx);
    CHECK(j.settings.angularStiffness == 7.0f &&
          j.settings.angularDampingRatio == 0.25f);
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::Position, ctx);
    CHECK(j.type == JointType::Distance && j.settings.enableSpring &&
          j.settings.stiffness == 4.0f && j.settings.restLength == 1.0f);
    s.restLength = 0.0f;
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::Position, ctx);
    CHECK(j.type == JointType::Motor && j.settings.linearStiffness == 4.0f &&
          j.settings.angularStiffness == 0.0f);
    CHECK(j.settings.maxSpringTorque == 0.0f);
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::Rotation, ctx);
    CHECK(j.type == JointType::Motor && j.settings.linearStiffness == 0.0f &&
          j.settings.angularStiffness == 7.0f);
    j = mapSideFX(SideFXType::Soft, s, ConstraintMode::All, ctx);
    CHECK(j.separation < 0.0f); // rest length 0: frames stay on the anchors

    // Spring: k, c -> Hz, damping ratio from the masses (one dynamic body: its
    // own mass; two: reduced).
    s = SideFXSettings();
    s.strength = 100.0f;
    s.damping = 2.0f;
    s.restLength = 0.0f;
    SideFXContext c2 = ctx;
    c2.a.dynamic = false;
    c2.b.dynamic = true;
    c2.b.mass = 1.0f;
    c2.b.inertia = 0.25f;
    j = mapSideFX(SideFXType::Spring, s, ConstraintMode::All, c2);
    CHECK(near(j.settings.linearStiffness, 10.0 / 6.283185307, 1e-4) &&
          near(j.settings.linearDampingRatio, 0.1, 1e-5));
    CHECK(near(j.settings.angularStiffness, 20.0 / 6.283185307, 1e-4) &&
          near(j.settings.angularDampingRatio, 0.2, 1e-5));
    c2.a = c2.b;
    c2.a.mass = c2.b.mass = 2.0f;
    j = mapSideFX(SideFXType::Spring, s, ConstraintMode::Position, c2);
    CHECK(near(j.settings.linearStiffness, 10.0 / 6.283185307,
               1e-4)); // reduced mass 1
    c2.a.dynamic = c2.b.dynamic = false;
    j = mapSideFX(SideFXType::Spring, s, ConstraintMode::Position, c2);
    CHECK(j.settings.linearStiffness == 0.0f); // nothing can move: no spring

    // Cone twist: the smaller cone limit (Box3D's cone is round), 180 = none;
    // twist symmetric.
    s = SideFXSettings();
    j = mapSideFX(SideFXType::ConeTwist, s, ConstraintMode::All, ctx);
    CHECK(j.type == JointType::Spherical && !j.settings.enableConeLimit);
    CHECK(j.settings.enableTwistLimit && j.settings.lowerTwistAngle == 0.0f &&
          j.settings.upperTwistAngle == 0.0f);
    s.maxUpRotation = 40.0f;
    s.maxOutRotation = 25.0f;
    s.maxTwist = 180.0f;
    j = mapSideFX(SideFXType::ConeTwist, s, ConstraintMode::All, ctx);
    CHECK(j.settings.enableConeLimit &&
          near(j.settings.coneAngle, 25.0 * kDegToRad, 1e-6) &&
          !j.settings.enableTwistLimit);
    s.maxTwist = 30.0f;
    j = mapSideFX(SideFXType::ConeTwist, s, ConstraintMode::All, ctx);
    CHECK(near(j.settings.lowerTwistAngle, -30.0 * kDegToRad, 1e-6) &&
          near(j.settings.upperTwistAngle, 30.0 * kDegToRad, 1e-6));
    // Bullet's twist axis (x by default) becomes Box3D's z; the up axis becomes
    // x.
    const float zAxis[3] = {0, 0, 1}, xAxis[3] = {1, 0, 0};
    float tz[3], tx[3];
    quatRotate(j.rotA, zAxis, tz);
    quatRotate(j.rotA, xAxis, tx);
    CHECK(near(tz[0], 1, 1e-5) && near(tz[1], 0, 1e-5) && near(tz[2], 0, 1e-5));
    CHECK(near(tx[0], 0, 1e-5) && near(tx[1], 1, 1e-5) && near(tx[2], 0, 1e-5));
    s.goalTwistAxis[0] = 0;
    s.goalTwistAxis[1] = -2;
    s.goalTwistAxis[2] = 0; // not unit, up not perpendicular
    s.goalUpAxis[0] = 1;
    s.goalUpAxis[1] = 1;
    s.goalUpAxis[2] = 0;
    j = mapSideFX(SideFXType::ConeTwist, s, ConstraintMode::All, ctx);
    quatRotate(j.rotB, zAxis, tz);
    quatRotate(j.rotB, xAxis, tx);
    CHECK(near(tz[1], -1, 1e-5) && near(tx[0], 1, 1e-5) &&
          near(tx[1], 0, 1e-5));
    s.goalUpAxis[0] = 0;
    s.goalUpAxis[1] = 1; // up parallel to twist
    j = mapSideFX(SideFXType::ConeTwist, s, ConstraintMode::All, ctx);
    quatRotate(j.rotB, zAxis, tz);
    CHECK(near(tz[1], -1, 1e-5) && std::isfinite(j.rotB[3]));
  }

  // ---- frame offsets ----
  {
    const float q[4] = {0.2f, 0.5f, -0.1f, 0.8366600f}; // ~unit
    SideFXJoint j;
    j.holdSeparation = true;
    j.rotA[0] = 0.0f;
    j.rotA[1] = 0.7071068f;
    j.rotA[2] = 0.0f;
    j.rotA[3] = 0.7071068f;
    const JointFrame fa = frame(1, 2, 3, q), fb = frame(4, 2, -1, kQid);
    JointFrame adjA, adjB;
    sideFXFrameOffsets(j, fa, fb, adjA, adjB);
    const JointFrame ja = applyFrameOffset(fa, adjA),
                     jb = applyFrameOffset(fb, adjB);
    // Holding the separation: both frames at the midpoint.
    CHECK(near(ja.pos[0], 2.5, 1e-5) && near(ja.pos[2], 1.0, 1e-5) &&
          near(jb.pos[0], 2.5, 1e-5) && near(jb.pos[2], 1.0, 1e-5));
    // Undo gives the anchors back.
    const JointFrame ba = removeFrameOffset(ja, adjA),
                     bb = removeFrameOffset(jb, adjB);
    CHECK(near(ba.pos[0], 1, 1e-5) && near(ba.pos[1], 2, 1e-5) &&
          near(ba.pos[2], 3, 1e-5) && !quatsDiffer(ba.rot, fa.rot, 1e-5));
    CHECK(near(bb.pos[0], 4, 1e-5) && near(bb.pos[2], -1, 1e-5) &&
          !quatsDiffer(bb.rot, fb.rot, 1e-5));
    // A separation: frames meet when the anchors are that far apart (here 5
    // apart, rest 2).
    j = SideFXJoint();
    j.separation = 2.0f;
    sideFXFrameOffsets(j, fa, fb, adjA, adjB);
    const JointFrame ka = applyFrameOffset(fa, adjA),
                     kb = applyFrameOffset(fb, adjB);
    CHECK(near(dist(ka.pos, fa.pos), 1.0, 1e-5) &&
          near(dist(kb.pos, fb.pos), 1.0, 1e-5));
    CHECK(near(dist(ka.pos, kb.pos), 3.0, 1e-5));
    // Nothing to do: identity offsets.
    sideFXFrameOffsets(SideFXJoint(), fa, fb, adjA, adjB);
    CHECK(isIdentityOffset(adjA) && isIdentityOffset(adjB));
    // Anchors on top of each other with a separation: along anchor A's x axis.
    j.separation = 2.0f;
    sideFXFrameOffsets(j, frame(0, 0, 0), frame(0, 0, 0), adjA, adjB);
    CHECK(near(adjA.pos[0], 1, 1e-6) && near(adjB.pos[0], -1, 1e-6));
  }

  // ---- in a world ----
  WorldSettings ws;
  {
    // Glue between two pieces anchored at their centres: they fall together,
    // still 1 apart (a plain weld at those anchors would pull the centres
    // together).
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle a = box(w, 0, 5, 0), b = box(w, 1, 5, 0);
    CHECK(make(w, SideFXType::Glue, SideFXSettings(), ConstraintMode::All, a, b,
               frame(0, 5, 0), frame(1, 5, 0)) != 0);
    steps(w, 60);
    double pa[3], pb[3];
    pos(w, a, pa);
    pos(w, b, pb);
    CHECK(pa[1] < 4.0 && near(dist(pa, pb), 1.0, 2e-3));
    std::printf("glue: centres %.4f apart after falling\n", dist(pa, pb));
  }
  {
    // Glue breaks once the force over a step passes the strength.
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    SideFXSettings s;
    s.strength = 0.001f; // far below the weight times a step
    int marker = 0;
    SideFXContext c;
    c.timeStep = kDt;
    c.b = bodyOf(w, b);
    const SideFXJoint j =
        mapSideFX(SideFXType::Glue, s, ConstraintMode::All, c);
    JointFrame adjA, adjB;
    sideFXFrameOffsets(j, frame(0, 5, 0), frame(0, 5, 0), adjA, adjB);
    CHECK(w.createJoint(j.type, 0, b, frame(0, 5, 0), frame(0, 5, 0),
                        j.settings, &marker) != 0);
    steps(w, 2);
    std::vector<void *> broken;
    w.brokenJoints(broken);
    CHECK(broken.size() == 1 && broken[0] == &marker);
  }
  {
    // Soft "all" with a rest length, no gravity: two pieces 1 apart settle 2
    // apart.
    WorldSettings zg = ws;
    zg.gravity[1] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(zg));
    const BodyHandle a = box(w, 0, 5, 0), b = box(w, 1, 5, 0);
    SideFXSettings s;
    s.restLength = 2.0f;
    s.stiffness = 3.0f;
    s.dampingRatio = 1.0f;
    CHECK(make(w, SideFXType::Soft, s, ConstraintMode::All, a, b,
               frame(0, 5, 0), frame(1, 5, 0)) != 0);
    steps(w, 300);
    double pa[3], pb[3];
    pos(w, a, pa);
    pos(w, b, pb);
    CHECK(near(dist(pa, pb), 2.0, 1e-2));
    std::printf("soft all, rest length 2: %.4f apart\n", dist(pa, pb));
  }
  {
    // Soft "position", no rest length, hanging from the world: sags g / (2 pi
    // f)^2.
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    SideFXSettings s;
    s.restLength = 0.0f;
    s.stiffness = 2.0f;
    s.dampingRatio = 1.0f;
    CHECK(make(w, SideFXType::Soft, s, ConstraintMode::Position, 0, b,
               frame(0, 5, 0), frame(0, 5, 0)) != 0);
    steps(w, 400);
    double p[3];
    pos(w, b, p);
    const double omega = 6.283185307 * 2.0, sag = 9.80665 / (omega * omega);
    CHECK(near(5.0 - p[1], sag, 0.1 * sag));
    std::printf("soft position: sag %.4f (spring theory %.4f)\n", 5.0 - p[1],
                sag);
  }
  {
    // Spring (Hooke's k on the body's mass), hanging: sags m g / k.
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    const SideFXBody body = bodyOf(w, b);
    SideFXSettings s;
    s.restLength = 0.0f;
    s.strength = 50.0f * body.mass;
    s.damping = 2.0f * std::sqrt(s.strength * body.mass); // critical
    CHECK(make(w, SideFXType::Spring, s, ConstraintMode::Position, 0, b,
               frame(0, 5, 0), frame(0, 5, 0)) != 0);
    steps(w, 400);
    double p[3];
    pos(w, b, p);
    const double sag = body.mass * 9.80665 / s.strength;
    CHECK(near(5.0 - p[1], sag, 0.1 * sag));
    std::printf("spring: sag %.4f (m g / k %.4f)\n", 5.0 - p[1], sag);
  }
  {
    // Hard "position" with a rest length: a rigid rod from a world pivot, free
    // to swing.
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 1.5, 5, 0);
    SideFXSettings s;
    s.restLength = 1.5f;
    CHECK(make(w, SideFXType::Hard, s, ConstraintMode::Position, 0, b,
               frame(0, 5, 0), frame(1.5, 5, 0)) != 0);
    steps(w, 45);
    double p[3];
    pos(w, b, p);
    const double pivot[3] = {0, 5, 0};
    CHECK(p[1] < 4.5 && near(dist(p, pivot), 1.5, 1e-2));
    std::printf("hard position rod: %.4f from the pivot\n", dist(p, pivot));
  }
  {
    // Cone twist hanging straight down (twist axis -y), kicked sideways: stops
    // at the smaller limit (20, not 45). Box3D's limits are soft, so a hard
    // kick overshoots by a few degrees.
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 4, 0);
    SideFXSettings s;
    for (float *ax : {s.goalTwistAxis, s.constrainedTwistAxis}) {
      ax[0] = 0;
      ax[1] = -1;
      ax[2] = 0;
    }
    for (float *ax : {s.goalUpAxis, s.constrainedUpAxis}) {
      ax[0] = 1;
      ax[1] = 0;
      ax[2] = 0;
    }
    s.maxUpRotation = 20.0f;
    s.maxOutRotation = 45.0f;
    s.maxTwist = 180.0f;
    CHECK(make(w, SideFXType::ConeTwist, s, ConstraintMode::All, 0, b,
               frame(0, 5, 0), frame(0, 5, 0)) != 0);
    const float kick[3] = {3, 0, 0}, none[3] = {0, 0, 0};
    w.setVelocityByHandle(b, kick, none);
    double worst = 0.0;
    for (int i = 0; i < 120; ++i) {
      w.step(kDt);
      double p[3];
      pos(w, b, p);
      const double dx = p[0], dy = p[1] - 5.0, dz = p[2];
      const double ang =
          std::acos(-dy / std::sqrt(dx * dx + dy * dy + dz * dz)) * kRadToDeg;
      worst = std::max(worst, ang);
    }
    CHECK(worst > 19.0 && worst < 23.0);
    std::printf("cone twist (20 deg cone): swung at most %.2f degrees\n",
                worst);
  }

  CHECK(Box3DWorld::assertCount() == assertsBefore);
  if (fails) {
    std::printf("%d check(s) failed\n", fails);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
