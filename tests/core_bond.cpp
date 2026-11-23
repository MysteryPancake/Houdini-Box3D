// Bonds (core/Box3DBond.h): the stress ratio of a load, the tear impulse, the
// bond's area, and a Bond joint under a hanging weight read as tension.
#include "TestUtil.h"
#include "core/Box3DBond.h"
#include "core/Box3DBondNetwork.h"
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>
#include <limits>

using namespace b3dop;

namespace {

const float kQid[4] = {0, 0, 0, 1};
const float kDt = 1.0f / 60.0f;

BodyHandle box(Box3DWorld &w, double x, double y, double z,
               BodySettings::Type type, float half) {
  BodySettings b;
  b.type = type;
  const double p[3] = {x, y, z};
  const BodyHandle h = w.createBody(b, p, kQid);
  ShapeSpec s;
  s.type = ShapeType::Box;
  s.halfExtents[0] = s.halfExtents[1] = s.halfExtents[2] = half;
  w.createShape(h, b, s);
  return h;
}

JointFrame frameAt(double x, double y, double z) {
  JointFrame f;
  f.pos[0] = x;
  f.pos[1] = y;
  f.pos[2] = z;
  return f;
}

// A box of the given half sizes at (x, y, z).
BodyHandle boxOf(Box3DWorld &w, double x, double y, double z, float hx,
                 float hy, float hz, BodySettings::Type type) {
  BodySettings b;
  b.type = type;
  const double p[3] = {x, y, z};
  const BodyHandle h = w.createBody(b, p, kQid);
  ShapeSpec s;
  s.type = ShapeType::Box;
  s.halfExtents[0] = hx;
  s.halfExtents[1] = hy;
  s.halfExtents[2] = hz;
  w.createShape(h, b, s);
  return h;
}

// The load of a Bond joint after a step, along `normal` (from A to B).
BondLoad loadOf(const Box3DWorld &w, JointHandle j, const float normal[3]) {
  JointFeedback fb;
  w.jointFeedback(j, JointType::Bond, fb);
  return bondLoadOf(fb.force, fb.torque, normal);
}

// A box of half size `half` (mass in proportion to its volume) hanging from a
// fixed one by a Bond: the tension, after `steps` of `dt` with `subSteps`.
// `weight` gets the box's weight.
float hangingTension(double dt, int steps, int subSteps, float half,
                     float *weight) {
  WorldSettings ws;
  ws.workerCount = 1;
  ws.subSteps = subSteps;
  Box3DWorld w;
  w.create(ws);
  const BodyHandle a =
      boxOf(w, 0, 5, 0, half, half, half, BodySettings::Type::Static);
  const BodyHandle b = boxOf(w, 0, 5 - 2.0 * half, 0, half, half, half,
                             BodySettings::Type::Dynamic);
  JointFrame f;
  f.pos[1] = 5 - half;
  const JointHandle j =
      w.createJoint(JointType::Bond, a, b, f, f,
                    Box3DWorld::defaultJointSettings(JointType::Bond));
  for (int i = 0; i < steps; ++i)
    w.step(static_cast<float>(dt));
  const float down[3] = {0, -1, 0};
  *weight = 9.80665f * w.massByHandle(b);
  return loadOf(w, j, down).tension;
}

} // namespace

int main() {
  const int assertsBefore = Box3DWorld::assertCount();

  // ---- the ratio: 1 at the strength of each load, alone ----
  {
    BondParams p;
    p.tensile = 2.0e5f;
    p.shear = 1.0e5f;
    p.compressive = 1.0e6f;
    p.area = 0.25f;
    const float modulus = p.area * std::sqrt(p.area) / 6.0f;
    BondLoad l;
    l.tension = p.tensile * p.area;
    CHECK(near(bondRatio(p, l), 1.0, 1e-5));
    l.tension = -p.compressive * p.area;
    CHECK(near(bondRatio(p, l), 1.0, 1e-5));
    l = BondLoad();
    l.shear = p.shear * p.area;
    CHECK(near(bondRatio(p, l), 1.0, 1e-5));
    l = BondLoad();
    l.bending = p.tensile * modulus; // bending strength 0: the tensile one
    CHECK(near(bondRatio(p, l), 1.0, 1e-5));
    l = BondLoad();
    l.twisting = p.shear * modulus;
    CHECK(near(bondRatio(p, l), 1.0, 1e-5));
    // Loads combine: half of tension and half of shear is under 1, 0.8 of both
    // is over.
    l = BondLoad();
    l.tension = 0.5f * p.tensile * p.area;
    l.shear = 0.5f * p.shear * p.area;
    CHECK(bondRatio(p, l) < 1.0f);
    l.tension *= 1.6f;
    l.shear *= 1.6f;
    CHECK(bondRatio(p, l) > 1.0f);
    // A pull is no crush and a push is no pull.
    l = BondLoad();
    l.tension = -p.tensile * p.area * 2.0f;
    CHECK(bondRatio(p, l) < 1.0f);
    // The same stress on a bigger patch takes more force; a bond without an
    // area, or a strength of 0, never fails.
    BondParams big = p;
    big.area = 1.0f;
    l = BondLoad();
    l.tension = p.tensile * p.area * 2.0f;
    CHECK(bondRatio(p, l) > 1.0f && bondRatio(big, l) < 1.0f);
    BondParams none = p;
    none.area = 0.0f;
    CHECK(bondRatio(none, l) == 0.0f);
    BondParams soft = p;
    soft.tensile = 0.0f;
    CHECK(bondRatio(soft, l) == 0.0f);
  }

  // ---- the materials: every one has strengths that mean something ----
  {
    const auto &materials = bondMaterials();
    CHECK(materials.size() >= 5);
    for (const BondMaterial &m : materials) {
      CHECK(m.label != nullptr && m.label[0] != '\0');
      CHECK(m.tensile > 0.0f && m.shear > 0.0f && m.bending >= 0.0f);
      CHECK(m.compressive >= m.tensile); // crushing takes more than pulling
      CHECK(m.fracture > 0.0f);
    }
  }

  // ---- the tear impulse: what the load beyond the capacity must deliver ----
  {
    // sqrt(2 * G * A * m): 100 J/m^2 over 1 m^2 between 1000 kg pieces.
    CHECK(near(bondTearImpulse(100.0f, 1.0f, 1000.0f), 447.2136, 1e-4));
    CHECK(bondTearImpulse(0.0f, 1.0f, 1000.0f) == 0.0f);
    CHECK(bondTearImpulse(100.0f, 0.0f, 1000.0f) == 0.0f);
    CHECK(bondTearImpulse(100.0f, 1.0f, 0.0f) == 0.0f);
    // The two sides' reduced mass, a fixed (infinite) side leaving the other.
    const float fixedSide = std::numeric_limits<float>::infinity();
    CHECK(near(bondReducedMass(1000.0f, 3000.0f), 750.0, 1e-6));
    CHECK(near(bondReducedMass(fixedSide, 1000.0f), 1000.0, 1e-6));
    CHECK(near(bondReducedMass(1000.0f, fixedSide), 1000.0, 1e-6));
    CHECK(bondReducedMass(fixedSide, fixedSide) == 0.0f);
    CHECK(bondReducedMass(0.0f, 0.0f) == 0.0f);
    BondParams p;
    p.tensile = 1000.0f;
    p.area = 1.0f;
    CHECK(near(bondCapacity(p), 1000.0, 1e-6));
    p.tensile = 0.0f;
    p.shear = 2000.0f;
    CHECK(near(bondCapacity(p), 2000.0, 1e-6)); // else shear
    // No fracture energy: at ratio 1, at once. Below 1: never, nothing kept.
    float impulse = 0.0f;
    CHECK(bondAdvance(impulse, 1.0f, 1000.0f, 0.01f, 0.0f));
    impulse = 0.0f;
    CHECK(!bondAdvance(impulse, 0.9f, 1000.0f, 0.01f, 0.0f) && impulse == 0.0f);
    // With it, the excess (ratio - 1) * capacity must deliver the impulse,
    // however the time is cut up: a ratio of 2 over a capacity of 1000 N gives
    // 1000 N, so 447.2 N*s takes 0.447 s at any step.
    const float tear = bondTearImpulse(100.0f, 1.0f, 1000.0f);
    for (const double dt :
         {1.0 / 24.0, 1.0 / 60.0, 1.0 / 240.0, 1.0 / 1000.0}) {
      impulse = 0.0f;
      int steps = 0;
      while (
          !bondAdvance(impulse, 2.0f, 1000.0f, static_cast<float>(dt), tear) &&
          steps < 100000)
        ++steps;
      const double t = (steps + 1) * dt;
      CHECK(t >= 0.4472 - 1e-4 && t <= 0.4472 + dt + 1e-4);
    }
    // A load that drops back keeps what it has done: it does not start again.
    impulse = 0.0f;
    bondAdvance(impulse, 2.0f, 1000.0f, 0.2f, tear);
    const float kept = impulse;
    bondAdvance(impulse, 0.5f, 1000.0f, 0.2f, tear);
    CHECK(impulse == kept && kept > 150.0f);
    CHECK(bondAdvance(impulse, 2.0f, 1000.0f, 0.3f, tear)); // 200 + 300 > 447
    // A weaker overload takes longer, in proportion: 1.1 over takes 10 times.
    impulse = 0.0f;
    int steps = 0;
    while (!bondAdvance(impulse, 1.1f, 1000.0f, 0.001f, tear) && steps < 100000)
      ++steps;
    CHECK(steps > 4000 && steps < 4600);
  }

  // ---- the area and the load's split ----
  {
    const float a[3] = {1, 1, 1}, b[3] = {1, 2, 3};
    const float x[3] = {1, 0, 0}, y[3] = {0, 1, 0};
    CHECK(near(bondAreaOf(a, b, x), 1.0, 1e-5)); // 1x1 and 2x3: the smaller
    CHECK(near(bondAreaOf(b, b, x), 6.0, 1e-5));
    CHECK(near(bondAreaOf(b, b, y), 3.0, 1e-5));
    const float force[3] = {0, -30, 40}, torque[3] = {0, 5, 12};
    const BondLoad l = bondLoadOf(force, torque, y);
    CHECK(near(l.tension, 30.0, 1e-4) && near(l.shear, 40.0, 1e-4));
    CHECK(near(l.twisting, 5.0, 1e-4) && near(l.bending, 12.0, 1e-4));
  }

  // ---- a Bond joint holding a weight: the force on B is up and reads as
  // tension along the normal from A (above) to B (below), about its weight ----
  {
    WorldSettings ws;
    ws.workerCount = 1;
    Box3DWorld w;
    CHECK(w.create(ws));
    const BodyHandle a = box(w, 0, 5, 0, BodySettings::Type::Static, 0.5f);
    const BodyHandle b = box(w, 0, 4, 0, BodySettings::Type::Dynamic, 0.5f);
    const JointHandle j = w.createJoint(
        JointType::Bond, a, b, frameAt(0, 4.5, 0), frameAt(0, 4.5, 0),
        Box3DWorld::defaultJointSettings(JointType::Bond));
    CHECK(j != 0 && w.isJointValid(j));
    for (int i = 0; i < 120; ++i)
      w.step(kDt);
    JointFeedback fb;
    CHECK(w.jointFeedback(j, JointType::Bond, fb));
    const float weight = 9.80665f * w.massByHandle(b);
    const float down[3] = {0, -1, 0};
    const BondLoad l = bondLoadOf(fb.force, fb.torque, down);
    std::printf("bond hanging: force y %.1f, tension %.1f, shear %.2f "
                "(weight %.1f)\n",
                fb.force[1], l.tension, l.shear, weight);
    CHECK(std::fabs(l.tension - weight) < 0.05f * weight);
    CHECK(l.shear < 0.05f * weight);
    float ea[3], eb[3];
    CHECK(w.tightExtentByHandle(a, ea) && w.tightExtentByHandle(b, eb));
    BondParams p;
    p.area = bondAreaOf(ea, eb, down);
    CHECK(near(p.area, 1.0, 1e-3));
    // Held at a strength a little over the weight's stress, broken a little
    // under it.
    p.tensile = 1.2f * weight / p.area;
    CHECK(bondRatio(p, l) < 1.0f);
    p.tensile = 0.8f * weight / p.area;
    CHECK(bondRatio(p, l) > 1.0f);
  }

  // ---- the load of a bond inside a rigid group: a spring network under the
  // net loads, checked against the statics of hand-worked cases (mass 1, g 9.8)
  // ----
  {
    // A hanging chain of 4 pieces, the first held by a force of 4 weights: the
    // bonds carry 3, 2 and 1 weights.
    std::vector<BondNode> nodes(4);
    for (int i = 0; i < 4; ++i) {
      nodes[static_cast<size_t>(i)].pos[1] = 3.0 - i;
      nodes[static_cast<size_t>(i)].force[1] = 9.8; // - m g
    }
    nodes[0].force[1] = 9.8 - 4 * 9.8;
    std::vector<BondLink> links(3);
    for (int i = 0; i < 3; ++i) {
      links[static_cast<size_t>(i)].a = i;
      links[static_cast<size_t>(i)].b = i + 1;
      links[static_cast<size_t>(i)].anchor[1] = 2.5 - i;
    }
    std::vector<BondForce> out;
    const double res = solveBondNetwork(nodes, links, out);
    std::printf("chain: residual %.2e, forces %.3f %.3f %.3f\n", res,
                out[0].force[1], out[1].force[1], out[2].force[1]);
    CHECK(res < 1e-6);
    CHECK(near(out[0].force[1], 29.4, 1e-3) &&
          near(out[1].force[1], 19.6, 1e-3) &&
          near(out[2].force[1], 9.8, 1e-3));
    CHECK(std::fabs(out[0].force[0]) < 1e-6 &&
          std::fabs(out[0].torque[2]) < 1e-6);
  }
  {
    // Two bonds side by side between a held piece and a hanging one share its
    // weight, whichever way they are stiffer.
    std::vector<BondNode> nodes(2);
    nodes[0].pos[1] = 1.0;
    nodes[1].pos[1] = 0.0;
    nodes[0].force[1] = 9.8 - 2 * 9.8;
    nodes[1].force[1] = 9.8;
    std::vector<BondLink> links(2);
    for (int i = 0; i < 2; ++i) {
      links[static_cast<size_t>(i)].a = 0;
      links[static_cast<size_t>(i)].b = 1;
      links[static_cast<size_t>(i)].anchor[0] = i == 0 ? -0.25 : 0.25;
      links[static_cast<size_t>(i)].anchor[1] = 0.5;
    }
    std::vector<BondForce> out;
    CHECK(solveBondNetwork(nodes, links, out) < 1e-6);
    CHECK(near(out[0].force[1], 4.9, 1e-3) && near(out[1].force[1], 4.9, 1e-3));
  }
  {
    // A cantilever: a piece hangs off the side of a held one. The bond carries
    // its weight in shear and a moment of weight times the arm (0.5) at the
    // anchor, the couple on the piece.
    std::vector<BondNode> nodes(2);
    nodes[1].pos[0] = 1.0;
    nodes[1].force[1] = 9.8;
    nodes[0].force[1] = 9.8 - 2 * 9.8;
    nodes[0].torque[2] = -9.8;
    std::vector<BondLink> links(1);
    links[0].a = 0;
    links[0].b = 1;
    links[0].anchor[0] = 0.5;
    std::vector<BondForce> out;
    CHECK(solveBondNetwork(nodes, links, out) < 1e-6);
    std::printf("cantilever: force y %.3f, couple z %.3f\n", out[0].force[1],
                out[0].torque[2]);
    CHECK(near(out[0].force[1], 9.8, 1e-3) &&
          near(out[0].torque[2], 4.9, 1e-3));
    const float f[3] = {static_cast<float>(out[0].force[0]),
                        static_cast<float>(out[0].force[1]),
                        static_cast<float>(out[0].force[2])};
    const float t[3] = {static_cast<float>(out[0].torque[0]),
                        static_cast<float>(out[0].torque[1]),
                        static_cast<float>(out[0].torque[2])};
    const float along[3] = {1, 0, 0}; // the normal from A to B
    const BondLoad l = bondLoadOf(f, t, along);
    CHECK(near(l.shear, 9.8, 1e-3) && near(l.bending, 4.9, 1e-3) &&
          std::fabs(l.tension) < 1e-4);
  }
  {
    // Nothing to carry, or a link to a missing node: zero, no failure.
    std::vector<BondNode> nodes(2);
    std::vector<BondLink> links(2);
    links[0].a = 0;
    links[0].b = 1;
    links[1].a = 0;
    links[1].b = 7;
    std::vector<BondForce> out;
    CHECK(solveBondNetwork(nodes, links, out) == 0.0 && out.size() == 2);
    nodes[0].force[1] = 1.0;
    nodes[1].force[1] = -1.0;
    CHECK(solveBondNetwork(nodes, links, out) < 1e-6);
    CHECK(out[1].force[1] == 0.0 && std::fabs(out[0].force[1]) > 0.5);
  }

  // ---- supports: contacts as elastic springs under the group (mass 1, g 9.8)
  // ----
  {
    // A row of 4 pieces on a support each, bonds between: an even squeeze, so
    // every support pushes with a weight and the bonds carry nothing.
    std::vector<BondNode> nodes(4);
    std::vector<BondSupport> supports(4);
    std::vector<BondLink> links(3);
    for (size_t i = 0; i < 4; ++i) {
      nodes[i].pos[0] = 0.5 + static_cast<double>(i);
      nodes[i].force[1] = 9.8; // - m g
      supports[i].node = static_cast<int>(i);
      supports[i].pos[0] = nodes[i].pos[0];
      supports[i].pos[1] = -0.5;
      supports[i].normalStiffness = 0.01;
      supports[i].tangentStiffness = 0.0002;
    }
    for (size_t i = 0; i < 3; ++i) {
      links[i].a = static_cast<int>(i);
      links[i].b = static_cast<int>(i) + 1;
      links[i].anchor[0] = 1.0 + static_cast<double>(i);
    }
    std::vector<BondForce> out;
    std::vector<SupportForce> sup;
    CHECK(solveBondNetwork(nodes, links, supports, out, sup, 200, 1e-9) < 1e-6);
    for (size_t i = 0; i < 4; ++i)
      CHECK(sup[i].active && near(sup[i].push, 9.8, 1e-4));
    for (const BondForce &b : out)
      CHECK(std::fabs(b.force[1]) < 1e-3 && std::fabs(b.torque[2]) < 1e-3);
  }
  {
    // The same row with a fourth piece hanging past the last support: the bond
    // at the edge carries its weight and the moment of it (the cut is
    // statically determinate, whatever the supports do), and the supports share
    // the weight of the whole row as a rigid beam on a soft bed does:
    // linearly, 3.27, 13.07 and 22.87 for supports 1 m apart under 4 weights
    // of 9.8 with the fourth beyond the last.
    std::vector<BondNode> nodes(4);
    std::vector<BondSupport> supports(3);
    std::vector<BondLink> links(3);
    for (size_t i = 0; i < 4; ++i) {
      nodes[i].pos[0] = 0.5 + static_cast<double>(i);
      nodes[i].force[1] = 9.8;
    }
    for (size_t i = 0; i < 3; ++i) {
      supports[i].node = static_cast<int>(i);
      supports[i].pos[0] = nodes[i].pos[0];
      supports[i].pos[1] = -0.5;
      supports[i].normalStiffness = 0.01;
      supports[i].tangentStiffness = 0.0002;
      links[i].a = static_cast<int>(i);
      links[i].b = static_cast<int>(i) + 1;
      links[i].anchor[0] = 1.0 + static_cast<double>(i);
    }
    std::vector<BondForce> out;
    std::vector<SupportForce> sup;
    CHECK(solveBondNetwork(nodes, links, supports, out, sup, 400, 1e-10) <
          1e-5);
    std::printf(
        "overhang: supports %.2f %.2f %.2f, edge bond force %.3f couple "
        "%.3f\n",
        sup[0].push, sup[1].push, sup[2].push, out[2].force[1],
        out[2].torque[2]);
    CHECK(near(out[2].force[1], 9.8, 1e-3) &&
          near(out[2].torque[2], 4.9, 1e-3));
    CHECK(near(sup[0].push, 3.27, 0.06) && near(sup[1].push, 13.07, 0.06) &&
          near(sup[2].push, 22.87, 0.06));
  }
  {
    // One piece on three supports under an eccentric weight: the elastic split
    // would pull on the one at x = 0, which cannot, so it is left out and the
    // weight goes to the support under it (x = 1.4) alone.
    std::vector<BondNode> nodes(1);
    nodes[0].pos[0] = 1.4;
    nodes[0].pos[1] = 0.5;
    nodes[0].force[1] = 9.8;
    std::vector<BondSupport> supports(3);
    const double at[3] = {0.0, 1.0, 1.4};
    for (size_t i = 0; i < 3; ++i) {
      supports[i].node = 0;
      supports[i].pos[0] = at[i];
      supports[i].normalStiffness = 1.0;
      supports[i].tangentStiffness = 0.02;
    }
    std::vector<BondForce> out;
    std::vector<SupportForce> sup;
    CHECK(solveBondNetwork(nodes, {}, supports, out, sup, 400, 1e-10) < 1e-5);
    CHECK(!sup[0].active && sup[1].active && sup[2].active);
    CHECK(near(sup[2].push, 9.8, 1e-3) && std::fabs(sup[1].push) < 1e-3);
  }

  // ---- benchmarks: the loads a Bond reads against statics ----
  {
    // A hanging box: the tension is its weight, at any step size and number of
    // substeps (a load that moved with them would break bonds by the frame
    // rate).
    float weight = 0.0f;
    float worst = 0.0f;
    for (const double dt : {1.0 / 24.0, 1.0 / 60.0, 1.0 / 240.0})
      for (const int sub : {1, 4, 8}) {
        const int steps = static_cast<int>(2.0 / dt);
        const float t = hangingTension(dt, steps, sub, 0.5f, &weight);
        worst = std::max(worst, std::fabs(t - weight) / weight);
        std::printf("hanging box dt 1/%.0f, %d substeps: tension %.3f of "
                    "weight %.3f\n",
                    1.0 / dt, sub, t, weight);
      }
    CHECK(worst < 0.03);
  }
  {
    // Size: a box twice as big weighs 8 times, over 4 times the area, so its
    // bond is loaded twice as hard (stress ~ size, which is why tall things
    // fall over before small ones).
    float w1 = 0.0f, w2 = 0.0f;
    const float t1 = hangingTension(1.0 / 60.0, 120, 4, 0.5f, &w1);
    const float t2 = hangingTension(1.0 / 60.0, 120, 4, 1.0f, &w2);
    const float s1 = t1 / 1.0f, s2 = t2 / 4.0f; // over each one's area
    std::printf("stress of a 1 m and a 2 m box: %.3f and %.3f (ratio %.3f)\n",
                s1, s2, s2 / s1);
    CHECK(near(s2 / s1, 2.0, 0.05));
  }
  {
    // A box hanging off the side of a fixed one: the bond carries its weight in
    // shear and a moment of weight times the distance from the bond to the
    // box's centre, in proportion to that distance.
    float lastRatio = 0.0f;
    for (const double arm : {0.5, 1.0, 2.0}) {
      WorldSettings ws;
      ws.workerCount = 1;
      Box3DWorld w;
      w.create(ws);
      const BodyHandle a =
          boxOf(w, 0, 0, 0, 0.25f, 0.5f, 0.5f, BodySettings::Type::Static);
      // B's centre is `arm` out from the bond, at the face of A.
      const BodyHandle b = boxOf(w, 0.25 + arm, 0, 0, static_cast<float>(arm),
                                 0.25f, 0.25f, BodySettings::Type::Dynamic);
      JointFrame f;
      f.pos[0] = 0.25;
      const JointHandle j =
          w.createJoint(JointType::Bond, a, b, f, f,
                        Box3DWorld::defaultJointSettings(JointType::Bond));
      for (int i = 0; i < 180; ++i)
        w.step(1.0f / 60.0f);
      const float along[3] = {1, 0, 0};
      const BondLoad l = loadOf(w, j, along);
      const float weight = 9.80665f * w.massByHandle(b);
      const float expected = weight * static_cast<float>(arm);
      std::printf("cantilever arm %.1f: shear %.2f of %.2f, moment %.2f of "
                  "%.2f\n",
                  arm, l.shear, weight, l.bending, expected);
      CHECK(near(l.shear, weight, 0.05));
      CHECK(near(l.bending, expected, 0.05));
      lastRatio = l.bending / expected;
    }
    CHECK(near(lastRatio, 1.0, 0.05));
  }
  {
    // A beam of two halves bonded at the middle, resting on narrow supports at
    // its ends: no shear at the middle, and a bending moment of 0.95 weights of
    // a half (the support's centre 1.95 from the bond, the half's weight 1.0).
    // The supports push the halves along x by friction, which acts 0.25 below
    // the bond and adds that times the push to the moment it carries.
    WorldSettings ws;
    ws.workerCount = 1;
    Box3DWorld w;
    w.create(ws);
    boxOf(w, -1.95, -0.25, 0, 0.05f, 0.25f, 0.25f, BodySettings::Type::Static);
    boxOf(w, 1.95, -0.25, 0, 0.05f, 0.25f, 0.25f, BodySettings::Type::Static);
    const BodyHandle l = boxOf(w, -1.0, 0.25, 0, 1.0f, 0.25f, 0.25f,
                               BodySettings::Type::Dynamic);
    const BodyHandle r =
        boxOf(w, 1.0, 0.25, 0, 1.0f, 0.25f, 0.25f, BodySettings::Type::Dynamic);
    JointFrame f;
    f.pos[1] = 0.25;
    const JointHandle j =
        w.createJoint(JointType::Bond, l, r, f, f,
                      Box3DWorld::defaultJointSettings(JointType::Bond));
    for (int i = 0; i < 300; ++i)
      w.step(1.0f / 60.0f);
    const float along[3] = {1, 0, 0};
    const BondLoad load = loadOf(w, j, along);
    const float half = 9.80665f * w.massByHandle(l);
    const float expected = 0.95f * half;
    std::printf("beam on two supports: moment %.3f of %.3f, shear %.3f, "
                "tension %.3f (half weighs %.3f)\n",
                load.bending, expected, load.shear, load.tension, half);
    CHECK(near(load.bending + std::fabs(load.tension) * 0.25f, expected, 0.04));
    CHECK(load.shear < 0.05f * half);
    CHECK(std::fabs(load.tension) <
          0.3f * half); // friction: 0.6 of a support's push at most
    (void)r;
  }

  {
    // An impact: in zero gravity a ball (restitution about 0) hits a block that
    // a Bond welds to a static wall, so the block cannot move and everything
    // the ball gives up goes through the bond. Box3D's constraint force is the
    // last substep's alone and misses most of it; the impulse over the step
    // (JointFeedback::impulse) adds up to the ball's change of momentum, at any
    // step size.
    for (const double dt : {1.0 / 24.0, 1.0 / 60.0, 1.0 / 240.0}) {
      WorldSettings ws;
      ws.workerCount = 1;
      ws.gravity[0] = ws.gravity[1] = ws.gravity[2] = 0.0f;
      Box3DWorld w;
      w.create(ws);
      const BodyHandle wall =
          boxOf(w, 1.5, 5, 0, 0.5f, 0.5f, 0.5f, BodySettings::Type::Static);
      const BodyHandle block =
          boxOf(w, 0.5, 5, 0, 0.5f, 0.5f, 0.5f, BodySettings::Type::Dynamic);
      const BodyHandle ball =
          boxOf(w, -1.0, 5, 0, 0.5f, 0.5f, 0.5f, BodySettings::Type::Dynamic);
      JointFrame f;
      f.pos[0] = 1.0;
      f.pos[1] = 5.0;
      const JointHandle j =
          w.createJoint(JointType::Bond, wall, block, f, f,
                        Box3DWorld::defaultJointSettings(JointType::Bond));
      const float v = 4.0f, still[3] = {0, 0, 0}, thrown[3] = {v, 0, 0};
      CHECK(w.setVelocityByHandle(ball, thrown, still));
      double total = 0.0, lastOnly = 0.0;
      for (double t = 0.0; t < 1.0; t += dt) {
        w.step(static_cast<float>(dt));
        JointFeedback fb;
        CHECK(w.jointFeedback(j, JointType::Bond, fb));
        total += static_cast<double>(fb.impulse[0]);
        lastOnly += static_cast<double>(fb.force[0]) * dt;
      }
      // What the bond took is what the ball gave up: its change of momentum (it
      // comes back a little, its restitution is not exactly 0).
      float after[3], spin[3];
      CHECK(w.getVelocityByHandle(ball, after, spin));
      const double momentum = static_cast<double>(w.massByHandle(ball)) *
                              (static_cast<double>(v) - after[0]);
      std::printf(
          "impact at dt 1/%.0f: bond impulse %.3f, last substeps' force * "
          "dt %.3f, ball's change of momentum %.3f\n",
          1.0 / dt, std::fabs(total), std::fabs(lastOnly), momentum);
      CHECK(near(std::fabs(total), momentum, 0.03));
    }
  }

  CHECK(Box3DWorld::assertCount() == assertsBefore);
  if (!fails)
    std::printf("OK\n");
  return fails ? 1 : 0;
}
