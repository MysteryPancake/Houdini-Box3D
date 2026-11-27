// Box3D joints through Box3DWorld: every type, world anchors, live updates,
// breaking, feedback, and the parameter table the Houdini side is built from.
#include "TestUtil.h"
#include "WorldTestUtil.h"
#include "core/Box3DSideFX.h" // frame offsets, as the sync applies them
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace b3dop;

int main() {
  const int assertsBefore = Box3DWorld::assertCount();

  // ---- the parameter table and names ----
  {
    for (int t = 0; t < kJointTypeCount; ++t) {
      std::set<std::string> names;
      for (const JointParam &p : jointParams())
        if (p.usedBy(JointType(t)))
          CHECK(names.insert(p.name).second); // a name means one thing per type
    }
    for (const JointParam &p : jointParams()) {
      const int members =
          (p.b ? 1 : 0) + (p.f ? 1 : 0) + (p.v ? 1 : 0) + (p.q ? 1 : 0);
      CHECK(members == 1 && p.types != 0 &&
            std::string(p.name).find(' ') == std::string::npos);
    }
    // jointParamIndices is the table filtered by usedBy, in table order, for
    // every type.
    for (int t = 0; t < kJointTypeCount; ++t) {
      std::vector<int> expect;
      for (size_t i = 0; i < jointParams().size(); ++i)
        if (jointParams()[i].usedBy(JointType(t)))
          expect.push_back(static_cast<int>(i));
      CHECK(jointParamIndices(JointType(t)) == expect && !expect.empty());
    }
    CHECK(jointParamIndices(JointType(-1)).empty() &&
          jointParamIndices(JointType(kJointTypeCount)).empty());
    // Enabling toggles: each names a toggle in the table, and the disable
    // string follows the chain and skips toggles the type does not have.
    for (const JointParam &p : jointParams())
      if (p.enabledBy)
        CHECK(findJointParam(p.enabledBy) &&
              findJointParam(p.enabledBy)->kind == JointParam::Toggle);
    auto disableWhen = [](const char *name, JointType t) {
      return jointParamDisableWhen(*findJointParam(name), t);
    };
    CHECK(disableWhen("stiffness", JointType::Revolute) ==
          "{ enable_spring == 0 }");
    CHECK(disableWhen("stiffness", JointType::Parallel).empty());
    CHECK(disableWhen("linear_stiffness", JointType::Weld) ==
          "{ enable_linear_spring == 0 }");
    CHECK(disableWhen("linear_stiffness", JointType::Motor).empty());
    CHECK(disableWhen("motor_speed", JointType::Distance) ==
          "{ enable_motor == 0 }");
    CHECK(disableWhen("lower_steering_limit", JointType::Wheel) ==
          "{ enable_steering_limit == 0 } { enable_steering == 0 }");
    CHECK(disableWhen("enable_spring", JointType::Revolute).empty());
    CHECK(disableWhen("restlength", JointType::Distance) ==
          "{ set_rest_length == 0 }");
    CHECK(disableWhen("max_length", JointType::Distance) ==
          "{ limit_max_length == 0 } { enable_limit == 0 }");
    CHECK(disableWhen("max_tension_force", JointType::Distance) ==
          "{ limit_spring_force == 0 } { enable_spring == 0 }");
    CHECK(disableWhen("force_threshold", JointType::Revolute) ==
          "{ breakable == 0 }");
    CHECK(disableWhen("max_torque", JointType::Parallel) ==
          "{ limit_torque == 0 }");
    // Node toggles: not settings (no member, never in jointParams), each gates
    // at least one setting whose Box3D default is the -1 it writes when off, on
    // every type the toggle is on.
    for (const JointParam &nt : jointNodeToggles()) {
      CHECK(nt.kind == JointParam::Toggle && !nt.b && !nt.f && !nt.v && !nt.q);
      for (const JointParam &p : jointParams())
        CHECK(std::string(p.name) != nt.name);
      unsigned gated = 0;
      for (const JointParam &p : jointParams())
        if (jointNodeToggleOf(p) == &nt) {
          gated |= p.types & nt.types;
          CHECK(p.kind == JointParam::Float && !p.isAngle(JointType::Distance));
          for (int t = 0; t < kJointTypeCount; ++t)
            if (p.usedBy(JointType(t)) && nt.usedBy(JointType(t)))
              CHECK(Box3DWorld::defaultJointSettings(JointType(t)).*p.f ==
                        -1.0f ||
                    Box3DWorld::defaultJointSettings(JointType(t)).*p.f < 0.0f);
        }
      CHECK(gated == nt.types);
    }
    // Node defaults: every toggle's settings do something when turned on, and
    // nothing behind a node toggle starts negative (its slider has a hard 0
    // minimum).
    for (int t = 0; t < kJointTypeCount; ++t) {
      const JointType type = JointType(t);
      const JointSettings n =
          nodeJointDefaults(type, Box3DWorld::defaultJointSettings(type));
      for (const JointParam &p : jointParams()) {
        if (!p.usedBy(type) || !p.f)
          continue;
        float v[3];
        getJointParam(n, p, type, v);
        const JointRange r = jointParamRange(p, type);
        CHECK(r.min < r.max);
        if (r.hardMin)
          CHECK(v[0] >= r.min);
        if (r.hardMax)
          CHECK(v[0] <= r.max);
      }
    }
    {
      const JointSettings rev = nodeJointDefaults(
          JointType::Revolute,
          Box3DWorld::defaultJointSettings(JointType::Revolute));
      CHECK(rev.stiffness > 0.0f && rev.maxMotorTorque > 0.0f &&
            rev.motorSpeed != 0.0f && rev.lowerAngle < rev.upperAngle);
      const JointSettings weld = nodeJointDefaults(
          JointType::Weld, Box3DWorld::defaultJointSettings(JointType::Weld));
      CHECK(weld.linearStiffness > 0.0f && weld.angularStiffness > 0.0f &&
            !weld.enableLinearSpring);
      const JointSettings whl = nodeJointDefaults(
          JointType::Wheel, Box3DWorld::defaultJointSettings(JointType::Wheel));
      CHECK(whl.maxSpinTorque > 0.0f && whl.maxSteeringTorque > 0.0f &&
            whl.lowerSteeringLimit < whl.upperSteeringLimit);
      CHECK(jointParamRange(*findJointParam("motor_speed"), JointType::Revolute)
                .max == 360.0f);
      CHECK(
          jointParamRange(*findJointParam("motor_speed"), JointType::Prismatic)
              .max == 10.0f);
    }
    // Node order: every setting exactly once, and each gated setting follows
    // its toggle with only that toggle's own settings in between, so a toggle
    // only disables a block just below it.
    for (int t = 0; t < kJointTypeCount; ++t) {
      const JointType type = JointType(t);
      const std::vector<const JointParam *> order = jointParamUiOrder(type);
      size_t toggles = 0;
      for (const JointParam &nt : jointNodeToggles())
        toggles += nt.usedBy(type) ? 1 : 0;
      CHECK(order.size() == jointParamIndices(type).size() + toggles);
      std::set<const JointParam *> seen(order.begin(), order.end());
      CHECK(seen.size() == order.size());
      for (size_t i = 0; i < order.size(); ++i) {
        const std::string cond = jointParamDisableWhen(*order[i], type);
        if (cond.empty())
          continue;
        const std::string own =
            "{ " + std::string(order[i]->enabledBy) + " == 0 }";
        size_t j = i;
        while (j > 0 && jointParamDisableWhen(*order[j - 1], type).find(own) !=
                            std::string::npos)
          --j;
        CHECK(j > 0 && std::string(order[j - 1]->name) == order[i]->enabledBy);
      }
    }
    // Box3D's defaults, in our terms.
    const JointSettings wheel =
        Box3DWorld::defaultJointSettings(JointType::Wheel);
    CHECK(wheel.enableSuspensionSpring && wheel.suspensionStiffness == 1.0f);
    CHECK(wheel.constraintStiffness == 60.0f &&
          wheel.constraintDampingRatio == 2.0f);
    CHECK(wheel.forceThreshold < 0.0f &&
          wheel.torqueThreshold < 0.0f); // FLT_MAX: never breaks
    const JointSettings par =
        Box3DWorld::defaultJointSettings(JointType::Parallel);
    CHECK(par.stiffness == 1.0f && par.maxTorque < 0.0f);
    const JointSettings dist =
        Box3DWorld::defaultJointSettings(JointType::Distance);
    CHECK(dist.restLength < 0.0f && dist.maxLength < 0.0f &&
          dist.maxTensionForce < 0.0f);
    const JointSettings weld =
        Box3DWorld::defaultJointSettings(JointType::Weld);
    CHECK(!weld.enableLinearSpring && !weld.enableAngularSpring);
    // Comparison only looks at the type's own settings.
    JointSettings a = weld, b = weld;
    b.motorSpeed = 5.0f; // not a weld setting
    CHECK(sameJointSettings(JointType::Weld, a, b) &&
          !sameJointSettings(JointType::Revolute, a, b));
    // Houdini values: getJointParam then setJointParam gives the same settings
    // (degrees round trip).
    for (int t = 0; t < kJointTypeCount; ++t) {
      const JointType type = JointType(t);
      const JointSettings d = Box3DWorld::defaultJointSettings(type);
      JointSettings r = d;
      for (const JointParam &p : jointParams()) {
        if (!p.usedBy(type) || p.kind == JointParam::Rotation)
          continue;
        float v[3];
        getJointParam(d, p, type, v);
        setJointParam(r, p, type, v, nullptr);
      }
      for (const JointParam &p : jointParams())
        if (p.usedBy(type) &&
            (p.kind == JointParam::Float || p.kind == JointParam::Angle))
          CHECK(std::fabs(r.*p.f - d.*p.f) <=
                1e-6f * std::max(1.0f, std::fabs(d.*p.f)));
      CHECK(jointParamSize(jointParams()[0]) == 1);
    }
    // Float primitive attributes: a factor (the Constraint Network DOP's rule),
    // or with Attributes Replace Values the value itself, in Houdini units.
    {
      const auto near = [](float got, float want) {
        return std::fabs(got - want) <= 1e-6f * std::max(1.0f, std::fabs(want));
      };
      const JointType rev = JointType::Revolute, distance = JointType::Distance;
      const JointParam &target = *findJointParam("target_angle");
      const JointParam &lower = *findJointParam("lower_angle");
      const JointParam &brk = *findJointParam("force_threshold");
      const JointParam &maxLen = *findJointParam("max_length");
      const JointSettings node =
          nodeJointDefaults(rev, Box3DWorld::defaultJointSettings(rev));
      CHECK(node.targetAngle == 0.0f &&
            near(node.lowerAngle, -45.0f * kDegToRad));
      JointSettings mul = node, rep = node;
      applyFloatAttribute(mul, target, rev, 30.0f, false);
      applyFloatAttribute(mul, lower, rev, 0.5f, false);
      applyFloatAttribute(rep, target, rev, 30.0f, true);
      applyFloatAttribute(rep, lower, rev, 0.5f, true);
      CHECK(mul.targetAngle == 0.0f); // 0 times anything
      CHECK(near(mul.lowerAngle, -22.5f * kDegToRad));
      CHECK(near(rep.targetAngle, 30.0f * kDegToRad)); // degrees in
      CHECK(near(rep.lowerAngle, 0.5f * kDegToRad));
      // A cap left unlimited (-1, its node toggle off): a factor flips its
      // sign, the value sets it.
      mul.forceThreshold = rep.forceThreshold = -1.0f;
      applyFloatAttribute(mul, brk, rev, 500.0f, false);
      applyFloatAttribute(rep, brk, rev, 500.0f, true);
      CHECK(mul.forceThreshold == -500.0f && rep.forceThreshold == 500.0f);
      // Not an angle: no unit conversion.
      JointSettings d = Box3DWorld::defaultJointSettings(distance);
      applyFloatAttribute(d, maxLen, distance, 2.5f, true);
      CHECK(d.maxLength == 2.5f);
      // The node parameter is neither a setting nor a node toggle.
      CHECK(findJointParam(kAttributesReplaceName) == nullptr);
    }
    b.linearStiffness = 3.0f;
    CHECK(!sameJointSettings(JointType::Weld, a, b));
  }

  WorldSettings ws;
  ws.workerCount = 1;

  // ---- Hold Separation: a weld between two pieces' centres 2 apart holds
  // them there (frames offset as the sync does), a plain one pulls them
  // together ----
  {
    WorldSettings zeroG = ws;
    zeroG.gravity[0] = zeroG.gravity[1] = zeroG.gravity[2] = 0.0f;
    double gap[2] = {0, 0};
    for (int hold = 0; hold < 2; ++hold) {
      Box3DWorld w;
      CHECK(w.create(zeroG));
      const BodyHandle a = box(w, 0, 5, 0), b = box(w, 2, 5, 0);
      JointFrame fa = frame(0, 5, 0), fb = frame(2, 5, 0);
      if (hold) {
        SideFXJoint m;
        m.holdSeparation = true;
        JointFrame adjA, adjB;
        sideFXFrameOffsets(m, fa, fb, adjA, adjB);
        CHECK(!isIdentityOffset(adjA) && !isIdentityOffset(adjB));
        fa = applyFrameOffset(fa, adjA);
        fb = applyFrameOffset(fb, adjB);
      }
      CHECK(w.createJoint(JointType::Weld, a, b, fa, fb,
                          Box3DWorld::defaultJointSettings(JointType::Weld)) !=
            0);
      steps(w, 60);
      double pa[3], pb[3];
      pos(w, a, pa);
      pos(w, b, pb);
      gap[hold] = std::sqrt((pb[0] - pa[0]) * (pb[0] - pa[0]) +
                            (pb[1] - pa[1]) * (pb[1] - pa[1]) +
                            (pb[2] - pa[2]) * (pb[2] - pa[2]));
    }
    std::printf("weld between centres 2 apart: plain gap %.3f, Hold "
                "Separation gap %.3f\n",
                gap[0], gap[1]);
    CHECK(gap[0] < 0.1 && std::fabs(gap[1] - 2.0) < 1e-3);
  }

  // ---- motion lock to the world: a locked axis holds, the others move ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle slide = box(w, 0, 5, 0);
    JointSettings ys = Box3DWorld::defaultJointSettings(JointType::MotionLock);
    CHECK(!ys.lockLinearX && !ys.lockLinearY && !ys.lockLinearZ &&
          !ys.lockAngularXY);
    ys.lockLinearY = true;
    const JointHandle jy = w.createJoint(JointType::MotionLock, 0, slide,
                                         frame(0, 5, 0), frame(0, 5, 0), ys);
    CHECK(jy != 0);
    const float vx[3] = {2, 0, 0}, none[3] = {0, 0, 0};
    CHECK(w.setVelocityByHandle(slide, vx, none));

    const BodyHandle spin = box(w, 10, 5, 0);
    JointSettings as = Box3DWorld::defaultJointSettings(JointType::MotionLock);
    as.lockLinearX = as.lockLinearY = as.lockLinearZ = true;
    as.lockAngularXY = true;
    const JointHandle j = w.createJoint(JointType::MotionLock, 0, spin,
                                        frame(10, 5, 0), frame(10, 5, 0), as);
    CHECK(j != 0);
    const float turn[3] = {3, 3, 3};
    CHECK(w.setVelocityByHandle(spin, none, turn));
    steps(w, 60);

    double p[3], q3[3];
    pos(w, slide, p);
    CHECK(std::fabs(p[1] - 5.0) < 1e-3 && p[0] > 1.5); // slid along x only
    float rot[4] = {}, lin[3] = {}, ang[3] = {};
    CHECK(w.getVelocityByHandle(spin, lin, ang));
    CHECK(std::fabs(ang[0]) < 0.05f && std::fabs(ang[1]) < 0.05f &&
          std::fabs(ang[2]) > 1.0f); // only the spin about z is left
    w.getTransformByHandle(spin, q3, rot);
    const double at[3] = {10, 5, 0};
    CHECK(dist(q3, at) < 0.02); // a point

    // Settings change on a live joint: turn the Y lock off and it falls.
    ys.lockLinearY = false;
    w.updateJoint(jy, JointType::MotionLock, ys);
    steps(w, 30);
    pos(w, slide, p);
    CHECK(p[1] < 4.5);
  }

  // ---- weld to the world: the body hangs where it is ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    const JointHandle j =
        w.createJoint(JointType::Weld, 0, b, frame(0, 5, 0), frame(0, 5, 0),
                      Box3DWorld::defaultJointSettings(JointType::Weld));
    CHECK(j != 0 && w.isJointValid(j) && w.jointCount() == 1);
    steps(w, 120);
    double p[3];
    pos(w, b, p);
    CHECK(std::fabs(p[1] - 5.0) < 0.02);
    JointFeedback fb;
    CHECK(w.jointFeedback(j, JointType::Weld, fb));
    std::printf("weld: y=%.4f force y=%.2f (weight %.2f)\n", p[1], fb.force[1],
                9.80665f * w.massByHandle(b));
    CHECK(std::fabs(std::fabs(fb.force[1]) - 9.80665f * w.massByHandle(b)) <
          0.1f * 9.80665f * w.massByHandle(b));
    // Refused: both ends the world, the same body twice.
    const JointSettings s = Box3DWorld::defaultJointSettings(JointType::Weld);
    CHECK(w.createJoint(JointType::Weld, 0, 0, frame(0, 0, 0), frame(0, 0, 0),
                        s) == 0);
    CHECK(w.createJoint(JointType::Weld, b, b, frame(0, 0, 0), frame(0, 0, 0),
                        s) == 0);
    // Destroying the body takes the joint with it.
    w.destroyBody(b);
    CHECK(!w.isJointValid(j) && w.jointCount() == 0);
    w.destroyJoint(j); // harmless
  }

  // ---- distance to the world: a pendulum keeps its length ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 2, 5, 0);
    JointSettings s = Box3DWorld::defaultJointSettings(
        JointType::Distance); // length < 0: the start distance
    const JointHandle j = w.createJoint(JointType::Distance, 0, b,
                                        frame(0, 5, 0), frame(2, 5, 0), s);
    CHECK(j != 0);
    double maxErr = 0.0, p[3];
    for (int i = 0; i < 180; ++i) {
      w.step(kDt);
      pos(w, b, p);
      const double r =
          std::sqrt(p[0] * p[0] + (p[1] - 5) * (p[1] - 5) + p[2] * p[2]);
      maxErr = std::max(maxErr, std::fabs(r - 2.0));
    }
    std::printf("distance: max length error %.4f, now at (%.2f, %.2f)\n",
                maxErr, p[0], p[1]);
    CHECK(maxErr < 0.02 && p[1] < 5.0); // swung down, length held
    JointFeedback fb;
    CHECK(w.jointFeedback(j, JointType::Distance, fb) && fb.extraCount == 2);
    CHECK(std::string(fb.extraName[0]) == "current_length" &&
          std::fabs(fb.extra[0] - 2.0f) < 0.02f);
  }

  // ---- revolute with a motor; live update of the motor speed ----
  {
    WorldSettings nog = ws;
    nog.gravity[1] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(nog));
    const BodyHandle b = box(w, 0, 0, 0);
    JointSettings s = Box3DWorld::defaultJointSettings(JointType::Revolute);
    s.enableMotor = true;
    s.motorSpeed = 1.0f; // rad/s about frame z (world z here)
    s.maxMotorTorque = 1000.0f;
    const JointHandle j = w.createJoint(JointType::Revolute, 0, b,
                                        frame(0, 0, 0), frame(0, 0, 0), s);
    CHECK(j != 0);
    steps(w, 60);
    float v[3] = {}, av[3] = {};
    w.getVelocityByHandle(b, v, av);
    CHECK(std::fabs(av[2] - 1.0f) < 0.02f && std::fabs(av[0]) < 1e-3f);
    JointFeedback fb;
    CHECK(w.jointFeedback(j, JointType::Revolute, fb) &&
          std::string(fb.extraName[0]) == "current_angle" &&
          fb.extraIsAngle[0]);
    CHECK(std::fabs(fb.extra[0] - 1.0f) < 0.05f); // ~1 rad after 1 s
    s.motorSpeed = -2.0f;
    w.updateJoint(j, JointType::Revolute, s);
    steps(w, 30);
    w.getVelocityByHandle(b, v, av);
    std::printf("revolute: angular velocity z=%.3f after the update to -2\n",
                av[2]);
    CHECK(std::fabs(av[2] + 2.0f) < 0.02f);
  }

  // ---- prismatic: slides down a rotated axis until its limit ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    // Frame x along world -y: 90 degrees about -z.
    const float down[4] = {0, 0, -0.70710678f, 0.70710678f};
    JointSettings s = Box3DWorld::defaultJointSettings(JointType::Prismatic);
    s.enableLimit = true;
    s.lowerTranslation = 0.0f;
    s.upperTranslation = 1.0f;
    const JointHandle j =
        w.createJoint(JointType::Prismatic, 0, b, frame(0, 5, 0, down),
                      frame(0, 5, 0, down), s);
    CHECK(j != 0);
    steps(w, 120);
    double p[3];
    pos(w, b, p);
    std::printf("prismatic: at (%.3f, %.3f), limit 1 below the start\n", p[0],
                p[1]);
    CHECK(std::fabs(p[1] - 4.0) < 0.02 && std::fabs(p[0]) < 1e-3);
  }

  // ---- spherical with a cone limit, motor joint, parallel, wheel: made,
  // stepped, no asserts ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle a = box(w, 0, 5, 0), b = box(w, 0, 4, 0),
                     c = box(w, 3, 5, 0), d = box(w, 6, 5, 0);
    const BodyHandle e = box(w, 9, 5, 0);
    JointSettings sph = Box3DWorld::defaultJointSettings(JointType::Spherical);
    sph.enableConeLimit = true;
    sph.coneAngle = 2.0f; // > 90 degrees: clamped, no assert
    sph.enableTwistLimit = true;
    sph.lowerTwistAngle = 0.5f;
    sph.upperTwistAngle = -0.5f; // reversed: ordered, no assert
    CHECK(w.createJoint(JointType::Spherical, 0, a, frame(0, 5.5, 0),
                        frame(0, 5.5, 0), sph) != 0);
    CHECK(
        w.createJoint(JointType::Weld, a, b, frame(0, 4.5, 0), frame(0, 4.5, 0),
                      Box3DWorld::defaultJointSettings(JointType::Weld)) != 0);
    JointSettings mot = Box3DWorld::defaultJointSettings(JointType::Motor);
    mot.linearVelocity[0] = 1.0f;
    mot.maxVelocityForce = 1000.0f;
    const JointHandle jm = w.createJoint(JointType::Motor, 0, c, frame(3, 5, 0),
                                         frame(3, 5, 0), mot);
    CHECK(jm != 0);
    CHECK(w.createJoint(
              JointType::Parallel, 0, d, frame(6, 5, 0), frame(6, 5, 0),
              Box3DWorld::defaultJointSettings(JointType::Parallel)) != 0);
    JointSettings wh = Box3DWorld::defaultJointSettings(JointType::Wheel);
    wh.enableSpinMotor = true;
    wh.spinSpeed = 3.0f;
    wh.maxSpinTorque = 100.0f;
    CHECK(w.createJoint(JointType::Wheel, 0, e, frame(9, 5, 0), frame(9, 5, 0),
                        wh) != 0);
    steps(w, 60);
    float v[3] = {}, av[3] = {};
    w.getVelocityByHandle(c, v, av);
    std::printf("motor joint: v=(%.3f, %.3f)\n", v[0], v[1]);
    CHECK(std::fabs(v[0] - 1.0f) < 0.05f); // driven along x
    w.getVelocityByHandle(e, v, av);
    CHECK(std::fabs(av[2] - 3.0f) < 0.1f); // the wheel spins about frame B's z
  }

  // ---- spherical cone shapes: a pendulum hung from the world (frame z down,
  // x along world x, y along world z), kicked toward x, toward y and between.
  // A round cone, an elliptical one, and a box (one angle under a quarter of
  // the other) holding a 0 angle as a hinge; the last made round and changed
  // live. Box3D's limits are soft, so a kick overshoots by a few degrees ----
  {
    struct Case {
      const char *name;
      float x, y; // cone angles, degrees
      bool live;  // made round at x, then given both angles by updateJoint
    };
    const Case cases[] = {{"round", 30, 30, false},
                          {"elliptical", 15, 50, false},
                          {"box", 0, 50, false},
                          {"live", 15, 50, true}};
    const float down[4] = {0.70710678f, 0, 0, 0.70710678f}; // z -> -y
    for (const Case &c : cases) {
      for (int toward = 0; toward < 3; ++toward) {
        Box3DWorld w;
        CHECK(w.create(ws));
        const BodyHandle b = box(w, 0, 4, 0);
        JointSettings sph =
            Box3DWorld::defaultJointSettings(JointType::Spherical);
        sph.enableConeLimit = true;
        sph.coneAngle = c.x * static_cast<float>(kDegToRad);
        sph.ellipticalCone = !c.live && c.x != c.y;
        sph.coneAngleY = c.y * static_cast<float>(kDegToRad);
        const JointHandle j =
            w.createJoint(JointType::Spherical, 0, b, frame(0, 5, 0, down),
                          frame(0, 5, 0, down), sph);
        CHECK(j != 0);
        if (c.live) {
          sph.ellipticalCone = true;
          w.updateJoint(j, JointType::Spherical, sph);
        }
        const float kick[3] = {toward != 1 ? 4.0f : 0.0f, 0,
                               toward != 0 ? 4.0f : 0.0f},
                    none[3] = {0, 0, 0};
        w.setVelocityByHandle(b, kick, none);
        double worstX = 0.0, worstY = 0.0, worstEllipse = 0.0;
        for (int i = 0; i < 120; ++i) {
          w.step(kDt);
          double p[3];
          pos(w, b, p);
          const double dx = p[0], dy = p[1] - 5.0, dz = p[2];
          const double rho = std::sqrt(dx * dx + dz * dz);
          const double swing = std::atan2(rho, -dy) * kRadToDeg;
          const double tx = rho > 1e-9 ? swing * dx / rho : 0.0,
                       ty = rho > 1e-9 ? swing * dz / rho : 0.0;
          worstX = std::max(worstX, std::fabs(tx));
          worstY = std::max(worstY, std::fabs(ty));
          if (c.x > 0 && c.y > 0)
            worstEllipse =
                std::max(worstEllipse, std::sqrt((tx / c.x) * (tx / c.x) +
                                                 (ty / c.y) * (ty / c.y)));
        }
        // Each axis reaches its limit when kicked along it and stays near
        // it; the elliptical cone keeps a diagonal kick inside the ellipse.
        const float slack = 3.0f;
        CHECK(worstX < c.x + slack && worstY < c.y + slack);
        if (toward == 0)
          CHECK(worstX > c.x - 1.0f);
        if (toward == 1)
          CHECK(worstY > c.y - 1.0f);
        // The diagonal kick is the hardest (the round cone overshoots by 11%).
        if (toward == 2 && c.x > 0)
          CHECK(worstEllipse > 0.9 && worstEllipse < 1.2);
        std::printf("spherical %s cone (%.0f, %.0f) kicked toward %s: x %.2f, "
                    "y %.2f degrees, ellipse %.3f\n",
                    c.name, c.x, c.y,
                    toward == 0   ? "x"
                    : toward == 1 ? "y"
                                  : "both",
                    worstX, worstY, worstEllipse);
      }
    }
  }

  // ---- a hinge (box cone 0, 50) that starts leaning 1 degree toward +x and
  // is kicked toward -x: its locked side holds both ways, so the bob crosses
  // to -x by the lean's correction and the soft limit's give under a hard kick
  // (2.8 degrees in the box case above), not by a step of free swing (3.8 more
  // at this speed) ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 4, 0);
    JointSettings sph = Box3DWorld::defaultJointSettings(JointType::Spherical);
    sph.enableConeLimit = true;
    sph.coneAngle = 0.0f;
    sph.ellipticalCone = true;
    sph.coneAngleY = 50.0f * static_cast<float>(kDegToRad);
    const float down[4] = {0.70710678f, 0, 0, 0.70710678f}; // z -> -y
    // Frame B turned 1 degree about world z: its z leans toward +x.
    const float half = 0.5f * static_cast<float>(kDegToRad);
    const float lean[4] = {0, 0, std::sin(half), std::cos(half)};
    const float leaned[4] = {lean[3] * down[0] + lean[0] * down[3] +
                                 lean[1] * down[2] - lean[2] * down[1],
                             lean[3] * down[1] - lean[0] * down[2] +
                                 lean[1] * down[3] + lean[2] * down[0],
                             lean[3] * down[2] + lean[0] * down[1] -
                                 lean[1] * down[0] + lean[2] * down[3],
                             lean[3] * down[3] - lean[0] * down[0] -
                                 lean[1] * down[1] - lean[2] * down[2]};
    CHECK(w.createJoint(JointType::Spherical, 0, b, frame(0, 5, 0, down),
                        frame(0, 5, 0, leaned), sph) != 0);
    const float kick[3] = {-4.0f, 0, 0}, none[3] = {0, 0, 0};
    w.setVelocityByHandle(b, kick, none);
    double worst = 0.0;
    for (int i = 0; i < 60; ++i) {
      w.step(kDt);
      double p[3];
      pos(w, b, p);
      worst = std::max(worst, -std::atan2(p[0], 5.0 - p[1]) * kRadToDeg);
    }
    CHECK(worst < 4.0);
    std::printf("hinge kicked across its locked side: crossed %.2f degrees\n",
                worst);
  }

  // ---- filter: no collision between the pair ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle ground =
        box(w, 0, -0.5, 0, BodySettings::Type::Static, 0.5f);
    const BodyHandle b = box(w, 0, 2, 0);
    CHECK(w.createJoint(
              JointType::Filter, ground, b, frame(0, 0, 0), frame(0, 0, 0),
              Box3DWorld::defaultJointSettings(JointType::Filter)) != 0);
    steps(w, 120);
    double p[3];
    pos(w, b, p);
    CHECK(p[1] < -1.0); // fell through the ground
  }

  // ---- breaking: a force threshold below the body's weight ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle b = box(w, 0, 5, 0);
    JointSettings s = Box3DWorld::defaultJointSettings(JointType::Weld);
    s.forceThreshold = 0.5f * 9.80665f * w.massByHandle(b);
    int marker = 7;
    const JointHandle j = w.createJoint(JointType::Weld, 0, b, frame(0, 5, 0),
                                        frame(0, 5, 0), s, &marker);
    CHECK(j != 0);
    std::vector<void *> broken;
    int stepsToBreak = -1;
    for (int i = 0; i < 30 && stepsToBreak < 0; ++i) {
      w.step(kDt);
      if (w.brokenJoints(broken) > 0)
        stepsToBreak = i + 1;
    }
    CHECK(stepsToBreak > 0 && broken.size() == 1 && broken[0] == &marker);
    w.destroyJoint(j); // what the solver does with a broken joint
    CHECK(!w.isJointValid(j));
    steps(w, 30);
    double p[3];
    pos(w, b, p);
    CHECK(p[1] < 4.9); // falls once released
    // A threshold that is never reached: no event.
    const BodyHandle c = box(w, 3, 5, 0);
    JointSettings strong = s;
    strong.forceThreshold = 1e6f;
    CHECK(w.createJoint(JointType::Weld, 0, c, frame(3, 5, 0), frame(3, 5, 0),
                        strong) != 0);
    steps(w, 10);
    CHECK(w.brokenJoints(broken) == 0);
  }

  // ---- frames are kept relative to the bodies: a joint made on moved bodies
  // starts at rest ----
  {
    WorldSettings nog = ws;
    nog.gravity[1] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(nog));
    const BodyHandle a = box(w, 0, 0, 0), b = box(w, 1, 0, 0);
    const float turned[4] = {0, 0.38268343f, 0,
                             0.92387953f}; // 45 degrees about y
    const double pa[3] = {10, 3, -2};
    w.setTransformByHandle(a, pa, turned);
    const double pb[3] = {11, 3, -2};
    w.setTransformByHandle(b, pb, kQid);
    const JointHandle j = w.createJoint(
        JointType::Weld, a, b, frame(10.5, 3, -2), frame(10.5, 3, -2),
        Box3DWorld::defaultJointSettings(JointType::Weld));
    CHECK(j != 0);
    steps(w, 30);
    double p[3];
    pos(w, b, p);
    CHECK(std::fabs(p[0] - 11) < 1e-3 && std::fabs(p[1] - 3) < 1e-3 &&
          std::fabs(p[2] + 2) < 1e-3);
    JointFeedback fb;
    CHECK(w.jointFeedback(j, JointType::Weld, fb));
    CHECK(fb.linearSeparation < 1e-3f);
    // Box3D's angular separation compares the bodies' rotations, not the joint
    // frames, so it reports their 45 degree difference although the joint holds
    // exactly. The rotation held:
    float qa[4] = {}, qb[4] = {};
    double tmp[3];
    w.getTransformByHandle(a, tmp, qa);
    w.getTransformByHandle(b, tmp, qb);
    CHECK(std::fabs(qa[1] - turned[1]) < 1e-4f &&
          std::fabs(qa[3] - turned[3]) < 1e-4f &&
          std::fabs(qb[3] - 1.0f) < 1e-4f);
    CHECK(std::fabs(fb.angularSeparation - 0.785398f) < 1e-3f);
  }

  // ---- live frames: a joint remade from jointWorldFrames on bodies where they
  // are now is the
  //      same joint (what a world rebuilt mid-simulation does) ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle a = box(w, 0, 5, 0), b = box(w, 0, 4, 0);
    const JointFrame pivotW = frame(0, 5.25, 0), pivotAB = frame(0, 4.5, 0);
    JointSettings rev = Box3DWorld::defaultJointSettings(JointType::Revolute);
    const JointHandle j1 =
        w.createJoint(JointType::Revolute, 0, a, pivotW, pivotW, rev);
    const JointHandle j2 =
        w.createJoint(JointType::Weld, a, b, pivotAB, pivotAB,
                      Box3DWorld::defaultJointSettings(JointType::Weld));
    CHECK(j1 && j2);
    // Swing the chain about the hinge: 3 rad/s about z, each body's velocity =
    // w x r.
    const float spin[3] = {0, 0, 3};
    const float va0[3] = {0.75f, 0, 0}, vb0[3] = {3.75f, 0, 0};
    w.setVelocityByHandle(a, va0, spin);
    w.setVelocityByHandle(b, vb0, spin);
    steps(w, 12);
    JointFrame la, lb, ma, mb;
    CHECK(w.jointWorldFrames(j1, la, lb) && w.jointWorldFrames(j2, ma, mb));
    // The world end stays at the pivot; the body ends follow the bodies and
    // still meet.
    CHECK(std::fabs(la.pos[1] - 5.25) < 1e-6 &&
          std::fabs(lb.pos[0] - la.pos[0]) < 1e-3 &&
          std::fabs(lb.pos[1] - la.pos[1]) < 1e-3);
    CHECK(std::fabs(ma.pos[0] - mb.pos[0]) < 1e-3 &&
          std::fabs(ma.pos[1] - mb.pos[1]) < 1e-3);
    CHECK(std::fabs(ma.pos[0]) > 0.1); // it really swung

    // A fresh world with the bodies where they are now, joints made from the
    // live frames.
    double pa[3], pb[3];
    float qa[4] = {}, qb[4] = {}, va[3] = {}, wa[3] = {}, vb[3] = {},
          wb[3] = {};
    w.getTransformByHandle(a, pa, qa);
    w.getTransformByHandle(b, pb, qb);
    w.getVelocityByHandle(a, va, wa);
    w.getVelocityByHandle(b, vb, wb);
    Box3DWorld r;
    CHECK(r.create(ws));
    const BodyHandle ra = box(r, pa[0], pa[1], pa[2]),
                     rb = box(r, pb[0], pb[1], pb[2]);
    r.setTransformByHandle(ra, pa, qa);
    r.setTransformByHandle(rb, pb, qb);
    r.setVelocityByHandle(ra, va, wa);
    r.setVelocityByHandle(rb, vb, wb);
    const JointHandle r1 =
        r.createJoint(JointType::Revolute, 0, ra, la, lb, rev);
    const JointHandle r2 =
        r.createJoint(JointType::Weld, ra, rb, ma, mb,
                      Box3DWorld::defaultJointSettings(JointType::Weld));
    CHECK(r1 && r2);
    steps(w, 20);
    steps(r, 20);
    double pw[3], pr[3];
    w.getTransformByHandle(b, pw, qa);
    r.getTransformByHandle(rb, pr, qb);
    std::printf("live frames: continued (%.4f, %.4f) vs rebuilt (%.4f, %.4f)\n",
                pw[0], pw[1], pr[0], pr[1]);
    CHECK(std::fabs(pw[0] - pr[0]) < 0.01 && std::fabs(pw[1] - pr[1]) < 0.01);
  }

  // ---- local frames: a joint remade from jointLocalFrames is exactly the same
  // joint, far from the
  //      origin too, and localToWorld puts them where jointWorldFrames does
  //      ----
  {
    WorldSettings nog = ws;
    nog.gravity[1] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(nog));
    const double far = 12000.0; // where float world positions are ~1e-3 apart
    const BodyHandle a = box(w, far, 5, 0), b = box(w, far + 1, 5, 0);
    const float tilt[4] = {0.1f, 0.2f, 0.0f, 0.97467943f};
    const JointHandle j =
        w.createJoint(JointType::Revolute, a, b, frame(far + 0.5, 5, 0, tilt),
                      frame(far + 0.5, 5, 0, tilt),
                      Box3DWorld::defaultJointSettings(JointType::Revolute));
    CHECK(j != 0);
    const float spin[3] = {0.3f, 1.0f, 0.2f}, push[3] = {1, 0, 0};
    w.setVelocityByHandle(a, push, spin);
    steps(w, 30);
    JointFrame la, lb, wa, wb, ta, tb;
    CHECK(w.jointLocalFrames(j, la, lb) && w.jointWorldFrames(j, wa, wb));
    CHECK(w.localToWorld(a, la, ta) && w.localToWorld(b, lb, tb));
    CHECK(std::fabs(ta.pos[0] - wa.pos[0]) < 1e-9 &&
          std::fabs(tb.pos[1] - wb.pos[1]) < 1e-9);
    JointFrame wl;
    CHECK(w.localToWorld(0, la, wl) &&
          wl.pos[0] == la.pos[0]); // the world: unchanged

    double pa[3], pb[3];
    float qa[4] = {}, qb[4] = {};
    w.getTransformByHandle(a, pa, qa);
    w.getTransformByHandle(b, pb, qb);
    Box3DWorld r;
    CHECK(r.create(nog));
    const BodyHandle ra = box(r, pa[0], pa[1], pa[2]),
                     rb = box(r, pb[0], pb[1], pb[2]);
    r.setTransformByHandle(ra, pa, qa);
    r.setTransformByHandle(rb, pb, qb);
    const JointHandle rj = r.createJoint(
        JointType::Revolute, ra, rb, la, lb,
        Box3DWorld::defaultJointSettings(JointType::Revolute), nullptr, true);
    JointFrame ra2, rb2;
    CHECK(rj != 0 && r.jointLocalFrames(rj, ra2, rb2));
    bool exact = true;
    for (int k = 0; k < 3; ++k)
      exact = exact && ra2.pos[k] == la.pos[k] && rb2.pos[k] == lb.pos[k];
    for (int k = 0; k < 4; ++k)
      exact = exact && ra2.rot[k] == la.rot[k] && rb2.rot[k] == lb.rot[k];
    if (!exact)
      std::printf("  la (%.9g %.9g %.9g | %.9g %.9g %.9g %.9g) vs (%.9g %.9g "
                  "%.9g | %.9g %.9g %.9g %.9g)\n",
                  la.pos[0], la.pos[1], la.pos[2], la.rot[0], la.rot[1],
                  la.rot[2], la.rot[3], ra2.pos[0], ra2.pos[1], ra2.pos[2],
                  ra2.rot[0], ra2.rot[1], ra2.rot[2], ra2.rot[3]);
    CHECK(exact); // bit for bit
    // Through world positions instead, the frames come back off by float noise
    // at this distance.
    Box3DWorld v;
    CHECK(v.create(nog));
    const BodyHandle va = box(v, pa[0], pa[1], pa[2]),
                     vb = box(v, pb[0], pb[1], pb[2]);
    v.setTransformByHandle(va, pa, qa);
    v.setTransformByHandle(vb, pb, qb);
    JointFrame fwa = wa, fwb = wb;
    for (int k = 0; k < 3; ++k) {
      fwa.pos[k] = static_cast<double>(static_cast<float>(wa.pos[k]));
      fwb.pos[k] = static_cast<double>(static_cast<float>(wb.pos[k]));
    }
    const JointHandle vj =
        v.createJoint(JointType::Revolute, va, vb, fwa, fwb,
                      Box3DWorld::defaultJointSettings(JointType::Revolute));
    JointFrame va2, vb2;
    CHECK(vj != 0 && v.jointLocalFrames(vj, va2, vb2));
    const double err = std::fabs(va2.pos[0] - la.pos[0]) +
                       std::fabs(va2.pos[1] - la.pos[1]) +
                       std::fabs(va2.pos[2] - la.pos[2]);
    std::printf("local frames at x=%.0f: exact remake; via float world "
                "positions off by %.2e\n",
                far, err);
  }

  // Feedback attribute names never overwrite a setting of the same name.
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    std::set<std::string> settings;
    for (const JointParam &p : jointParams())
      settings.insert(p.name);
    for (const char *n : {"force", "torque", "distance", "angle"})
      CHECK(!settings.count(n));
    for (int t = 0; t < kJointTypeCount; ++t) {
      const BodyHandle b = box(w, 3.0 * t, 5, 0);
      const JointHandle j = w.createJoint(
          JointType(t), 0, b, frame(3.0 * t, 5, 0), frame(3.0 * t, 5, 0),
          Box3DWorld::defaultJointSettings(JointType(t)));
      CHECK(j != 0);
      JointFeedback fb;
      CHECK(w.jointFeedback(j, JointType(t), fb));
      for (int k = 0; k < fb.extraCount; ++k)
        CHECK(!settings.count(fb.extraName[k]));
    }
    steps(w, 5);
  }

  // Settings that hold NaN compare equal to themselves: the sync updates a
  // joint (and wakes its bodies) only when a setting differs, so a NaN that
  // has not changed must not look like a change. A different value still does.
  {
    const float nan = std::nanf("");
    for (const JointParam &p : jointParams()) {
      if (p.kind == JointParam::Toggle)
        continue;
      for (int t = 0; t < kJointTypeCount; ++t) {
        const JointType type = JointType(t);
        if (!p.usedBy(type))
          continue;
        JointSettings a = Box3DWorld::defaultJointSettings(type);
        float *value = p.f   ? &(a.*p.f)
                       : p.v ? &(a.*p.v)[0]
                       : p.q ? &(a.*p.q)[0]
                             : nullptr;
        CHECK(value != nullptr);
        if (!value)
          continue;
        *value = nan;
        JointSettings b = a;
        CHECK(sameJointSettings(type, a, b));
        b = Box3DWorld::defaultJointSettings(type);
        CHECK(!sameJointSettings(type, a, b));
        CHECK(!sameJointSettings(type, b, a));
      }
    }
  }

  { // "not set": only a float at the very bottom of the range counts
    CHECK(isUnsetFloat(kUnsetFloat));
    // What a 32-bit attribute written from the double kUnsetFloat reads back
    // as.
    CHECK(isUnsetFloat(static_cast<float>(static_cast<double>(kUnsetFloat))));
    CHECK(isUnsetFloat(0.95f * kUnsetFloat));
    CHECK(!isUnsetFloat(0.0f) && !isUnsetFloat(-1.0f) && !isUnsetFloat(1.0f));
    CHECK(!isUnsetFloat(0.5f * kUnsetFloat) && !isUnsetFloat(-1e30f));
    CHECK(!isUnsetFloat(std::nanf("")));
    // Applied to a setting the attribute multiplies (a node value of 0 stays 0,
    // -1 flips sign) or, with replace, is the value.
    const JointParam *p = findJointParam("force_threshold");
    CHECK(p && p->f && p->usedBy(JointType::Weld));
    if (p && p->f && p->usedBy(JointType::Weld)) {
      JointSettings s = Box3DWorld::defaultJointSettings(JointType::Weld);
      s.*p->f = 8.0f;
      applyFloatAttribute(s, *p, JointType::Weld, 0.5f, false);
      CHECK(s.*p->f == 4.0f);
      s.*p->f = 0.0f;
      applyFloatAttribute(s, *p, JointType::Weld, 5.0f, false);
      CHECK(s.*p->f == 0.0f);
      s.*p->f = -1.0f;
      applyFloatAttribute(s, *p, JointType::Weld, 3.0f, false);
      CHECK(s.*p->f == -3.0f);
      applyFloatAttribute(s, *p, JointType::Weld, 7.0f, true);
      CHECK(s.*p->f == 7.0f);
    }
  }

  CHECK(Box3DWorld::assertCount() == assertsBefore); // nothing Box3D refused
  if (!fails)
    std::printf("OK\n");
  return fails ? 1 : 0;
}
