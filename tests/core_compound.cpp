// A compound body (several shapes on one body) as a stand-in for glued pieces:
// its mass is the pieces' sum, a beam of it does not sag the way a chain of
// weld joints can, and contactImpulsesByShape tells which piece a contact hit.
#include "TestUtil.h"
#include "core/Box3DCompound.h"
#include "core/Box3DMath.h"
#include "core/Box3DWorld.h"
#include <algorithm>
#include <chrono>

#include <cmath>
#include <cstdio>
#include <vector>

using namespace b3dop;

namespace {

const float kQid[4] = {0, 0, 0, 1};
const float kDt = 1.0f / 60.0f;
const float kHalf = 0.25f; // pieces are cubes of 0.5, touching along x

ShapeSpec cube(float x) {
  ShapeSpec s;
  s.type = ShapeType::Box;
  s.halfExtents[0] = s.halfExtents[1] = s.halfExtents[2] = kHalf;
  s.center[0] = x;
  return s;
}

BodyHandle staticBox(Box3DWorld &w, double x, double y, double hx, double hy) {
  BodySettings b;
  b.type = BodySettings::Type::Static;
  const double p[3] = {x, y, 0};
  const BodyHandle h = w.createBody(b, p, kQid);
  ShapeSpec s;
  s.type = ShapeType::Box;
  s.halfExtents[0] = static_cast<float>(hx);
  s.halfExtents[1] = static_cast<float>(hy);
  s.halfExtents[2] = 0.5f;
  w.createShape(h, b, s);
  return h;
}

JointFrame frameAt(double x, double y) {
  JointFrame f;
  f.pos[0] = x;
  f.pos[1] = y;
  f.pos[2] = 0;
  for (int i = 0; i < 4; ++i)
    f.rot[i] = kQid[i];
  return f;
}

double bodyY(const Box3DWorld &w, BodyHandle h) {
  double p[3];
  float q[4];
  w.getTransformByHandle(h, p, q);
  return p[1];
}

} // namespace

int main() {
  WorldSettings ws;
  ws.workerCount = 1;
  const int n = 12;
  const double span = (n - 1) * 2.0 * kHalf; // first to last piece centre

  // ---- mass: the compound weighs what its pieces weigh ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    BodySettings b;
    const double p[3] = {0, 5, 0};
    const BodyHandle one = w.createBody(b, p, kQid);
    CHECK(w.createShape(one, b, cube(0.0f)));
    const BodyHandle all = w.createBody(b, p, kQid);
    for (int i = 0; i < n; ++i)
      CHECK(w.createShape(all, b, cube(static_cast<float>(i) * 2.0f * kHalf)));
    CHECK(w.shapeCountByHandle(all) == n);
    CHECK(near(w.massByHandle(all), n * w.massByHandle(one), 1e-3));
  }

  // ---- a beam resting on two posts: compound against a chain of welds ----
  {
    double sag[2] = {0, 0}; // how far the middle piece sits below the ends
    for (int compound = 0; compound < 2; ++compound) {
      Box3DWorld w;
      CHECK(w.create(ws));
      staticBox(w, 0.0, 0.5, 0.2, 0.5);  // post under the first piece
      staticBox(w, span, 0.5, 0.2, 0.5); // post under the last
      BodySettings b;
      const double y0 = 1.75; // bottom 0.25 above the posts' tops
      BodyHandle mid = 0, first = 0;
      if (compound) {
        const double p[3] = {0, y0, 0};
        const BodyHandle h = w.createBody(b, p, kQid);
        for (int i = 0; i < n; ++i)
          w.createShape(h, b, cube(static_cast<float>(i) * 2.0f * kHalf),
                        false);
        w.applyMassFromShapes(h);
        mid = first = h;
      } else {
        std::vector<BodyHandle> hs;
        for (int i = 0; i < n; ++i) {
          const double p[3] = {i * 2.0 * kHalf, y0, 0};
          const BodyHandle h = w.createBody(b, p, kQid);
          w.createShape(h, b, cube(0.0f));
          hs.push_back(h);
        }
        for (int i = 0; i + 1 < n; ++i) {
          const double x = (i + 0.5) * 2.0 * kHalf;
          CHECK(w.createJoint(
                    JointType::Weld, hs[static_cast<size_t>(i)],
                    hs[static_cast<size_t>(i + 1)], frameAt(x, y0),
                    frameAt(x, y0),
                    Box3DWorld::defaultJointSettings(JointType::Weld)) != 0);
        }
        mid = hs[n / 2];
        first = hs[0];
      }
      for (int i = 0; i < 240; ++i)
        w.step(kDt);
      sag[compound] = bodyY(w, first) - bodyY(w, mid);
    }
    std::printf("beam of %d pieces on two posts, middle below the end: weld "
                "chain %.4f, compound %.4f\n",
                n, sag[0], sag[1]);
    // The compound is one rigid body: the middle cannot sit lower than the end.
    CHECK(std::fabs(sag[1]) < 1e-3);
  }

  // ---- impulses per piece: only the piece over the post is hit ----
  {
    for (int hit = 0; hit < 3; ++hit) {
      Box3DWorld w;
      CHECK(w.create(ws));
      // A post narrower than a piece, under piece `hit` only.
      staticBox(w, hit * 2.0 * kHalf, 0.5, 0.15, 0.5);
      BodySettings b;
      const double p[3] = {0, 1.3, 0}; // bottom 0.05 above the post's top
      const BodyHandle h = w.createBody(b, p, kQid);
      for (int i = 0; i < 3; ++i)
        w.createShape(h, b, cube(static_cast<float>(i) * 2.0f * kHalf), false);
      w.applyMassFromShapes(h);

      std::vector<float> per;
      float total = 0.0f;
      bool found = false;
      for (int s = 0; s < 120 && !found; ++s) {
        w.step(kDt);
        w.contactImpulsesByShape(h, per);
        total = 0.0f;
        for (const float v : per)
          total += v;
        found = total > 0.0f;
      }
      CHECK(found && per.size() == 3);
      if (found && per.size() == 3) {
        for (int i = 0; i < 3; ++i)
          CHECK(i == hit ? per[static_cast<size_t>(i)] > 0.0f
                         : per[static_cast<size_t>(i)] == 0.0f);
        // The per-piece impulses add up to the body's.
        CHECK(near(total, w.contactImpulseByHandle(h, nullptr, 0),
                   1e-4 * total + 1e-6));
      }
    }
  }

  // ---- a hull placed in a compound: the same mass data as the equal box ----
  {
    const float hx = 0.5f, hy = 0.1f, hz = 0.1f;
    std::vector<float> pts;
    for (int i = 0; i < 8; ++i) {
      pts.push_back((i & 1 ? hx : -hx));
      pts.push_back((i & 2 ? hy : -hy));
      pts.push_back((i & 4 ? hz : -hz));
    }
    // Offsets and turns (90 degrees about z, and about an oblique axis).
    struct Place {
      float c[3];
      float q[4];
    };
    const float s = 0.70710678f;
    const Place places[] = {{{0.5f, 0, 0}, {0, 0, 0, 1}},
                            {{0, 0.7f, -0.2f}, {0, 0, s, s}},
                            {{-0.3f, 0.2f, 0.4f}, {0.3f, 0.5f, 0.2f, 0.78f}}};
    for (const Place &pl : places) {
      float q[4] = {pl.q[0], pl.q[1], pl.q[2], pl.q[3]};
      sanitizeQuat(q);
      Box3DWorld w;
      CHECK(w.create(ws));
      BodySettings b;
      const double p[3] = {0, 5, 0};
      const BodyHandle hullBody = w.createBody(b, p, kQid);
      const BodyHandle viaHull = w.createBody(b, p, kQid);
      const BodyHandle viaBox = w.createBody(b, p, kQid);
      const HullId hull = w.createHull(pts.data(), 8);
      CHECK(hull != 0);
      CHECK(w.createHullShape(hullBody, b, hull));
      CHECK(w.copyShapesByHandle(hullBody, viaHull, pl.c, q) == 1);
      w.applyMassFromShapes(viaHull);
      ShapeSpec box;
      box.type = ShapeType::Box;
      box.halfExtents[0] = hx;
      box.halfExtents[1] = hy;
      box.halfExtents[2] = hz;
      for (int i = 0; i < 3; ++i)
        box.center[i] = pl.c[i];
      for (int i = 0; i < 4; ++i)
        box.rot[i] = q[i];
      CHECK(w.createShape(viaBox, b, box));
      float m0 = 0, m1 = 0, c0[3] = {}, c1[3] = {}, i0[3] = {}, i1[3] = {};
      CHECK(w.getMassDataByHandle(viaHull, m0, c0, i0) &&
            w.getMassDataByHandle(viaBox, m1, c1, i1));
      CHECK(near(m0, m1, 1e-4 * m1));
      for (int i = 0; i < 3; ++i) {
        CHECK(near(c0[i], c1[i], 1e-4));
        CHECK(near(i0[i], i1[i], 1e-3 * i1[i] + 1e-6));
      }
    }
  }

  // ---- approachingBodies: who a fast body reaches in the next step ----
  {
    WorldSettings flying = ws;
    flying.gravity[0] = flying.gravity[1] = flying.gravity[2] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(flying));
    BodySettings b;
    const double pBox[3] = {0, 0, 0}, pBall[3] = {-0.6, 0, 0};
    const BodyHandle box = w.createBody(b, pBox, kQid);
    CHECK(w.createShape(box, b, cube(0.0f)));
    const BodyHandle ball = w.createBody(b, pBall, kQid);
    ShapeSpec s;
    s.type = ShapeType::Sphere;
    s.radius = 0.25f;
    CHECK(w.createShape(ball, b, s));
    staticBox(w, -0.5, 0.4, 0.3, 0.3); // in the way too, static
    // The static body is reported like any other (a compound landing on the
    // ground is one that reaches it); the cases below look at the box.
    const auto onlyBox = [box](std::vector<Box3DWorld::Approach> &list) {
      list.erase(std::remove_if(list.begin(), list.end(),
                                [box](const Box3DWorld::Approach &a) {
                                  return a.target != box;
                                }),
                 list.end());
    };
    const float fast[3] = {20, 0, 0}, slow[3] = {1, 0, 0}, none[3] = {0, 0, 0};

    std::vector<Box3DWorld::Approach> out;
    w.setVelocityByHandle(ball, fast, none);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    CHECK(out.size() == 2);
    onlyBox(out);
    CHECK(out.size() == 1);
    if (out.size() == 1) {
      CHECK(out[0].mover == ball && out[0].target == box);
      CHECK(near(out[0].moverMass, w.massByHandle(ball), 1e-4));
      CHECK(near(out[0].moverLin[0], 20.0, 1e-4));
      CHECK(near(out[0].moverCom[0], -0.6, 1e-4));
      // The box is at rest: the relative velocity is the ball's.
      CHECK(near(out[0].relLin[0], 20.0, 1e-4));
    }
    // Too slow to look at, or too far to reach in the time given.
    out.clear();
    w.setVelocityByHandle(ball, slow, none);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    onlyBox(out);
    CHECK(out.empty());
    const double farPos[3] = {-3, 0, 0};
    w.setVelocityByHandle(ball, fast, none);
    w.setTransformByHandle(ball, farPos, kQid);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    onlyBox(out);
    CHECK(out.empty());
    // A longer look gets there.
    w.approachingBodies(&ball, 1, 0.5f, 2.0f, 0.0f, out);
    onlyBox(out);
    CHECK(out.size() == 1);
    // A dynamic body whose own momentum is below the least asked for is not
    // looked at (the cube is about 1.25 kg of density 1000 times 0.5^3).
    out.clear();
    const double nearPos[3] = {-0.6, 0, 0};
    w.setTransformByHandle(ball, nearPos, kQid);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 1e9f, out);
    onlyBox(out);
    CHECK(out.empty());
    // A body that moves with what it overlaps is not coming at it: the box goes
    // along with the ball.
    w.setVelocityByHandle(box, fast, none);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    onlyBox(out);
    CHECK(out.empty());
    // ... nor is one that moves slower than it relative to it: the box moves at
    // 15 and the ball at 20, a relative 5.
    const float fifteen[3] = {15, 0, 0};
    w.setVelocityByHandle(box, fifteen, none);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    onlyBox(out);
    CHECK(out.size() == 1 && near(out[0].relLin[0], 5.0, 1e-3));
    w.setVelocityByHandle(box, none, none);
    // A kinematic mover is looked at too, with mass 0 and no momentum limit.
    out.clear();
    w.setTypeByHandle(ball, BodySettings::Type::Kinematic);
    w.setVelocityByHandle(ball, fast, none);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 1e9f, out);
    onlyBox(out);
    CHECK(out.size() == 1 && out[0].moverMass == 0.0f);
    // A body that is gone gives nothing and does not crash.
    out.clear();
    w.destroyBody(ball);
    w.approachingBodies(&ball, 1, kDt, 2.0f, 0.0f, out);
    CHECK(out.empty());
  }

  // ---- a kinematic mover and a compound: found, hits one piece, is not slowed
  // ----
  {
    WorldSettings flying = ws;
    flying.gravity[0] = flying.gravity[1] = flying.gravity[2] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(flying));
    // Four pieces of a compound in a row along x, at 0, 0.5, 1 and 1.5.
    BodySettings b;
    const double origin[3] = {0, 0, 0};
    const BodyHandle compound = w.createBody(b, origin, kQid);
    for (int i = 0; i < 4; ++i)
      CHECK(w.createShape(compound, b, cube(0.5f * static_cast<float>(i)),
                          false));
    w.applyMassFromShapes(compound);
    BodySettings kb;
    kb.type = BodySettings::Type::Kinematic;
    const double start[3] = {-2, 0, 0};
    const BodyHandle mover = w.createBody(kb, start, kQid);
    CHECK(w.createShape(mover, kb, cube(0.0f)));
    const float v[3] = {6, 0, 0}, none[3] = {0, 0, 0};
    CHECK(w.setVelocityByHandle(mover, v, none));

    // Looked at in a long enough step whatever the compound's momentum limit
    // (a kinematic mover has none), as the compound and not as its pieces.
    std::vector<Box3DWorld::Approach> out;
    w.approachingBodies(&mover, 1, 0.5f, 2.0f, 1e9f, out);
    CHECK(out.size() == 1);
    if (out.size() == 1) {
      CHECK(out[0].mover == mover && out[0].target == compound);
      CHECK(out[0].moverMass == 0.0f);
      CHECK(near(out[0].relLin[0], 6.0, 1e-4));
    }
    // Too close to the compound's motion to come at it: carried along.
    w.setVelocityByHandle(compound, v, none);
    out.clear();
    w.approachingBodies(&mover, 1, 0.5f, 2.0f, 0.0f, out);
    CHECK(out.empty());
    w.setVelocityByHandle(compound, none, none);

    // The hit: the first step with an impulse is on the nearest piece alone
    // (the piece a glue hit is read from), the mover keeps its speed and pose,
    // and the compound is carried off at the mover's speed.
    int steps = 0;
    std::vector<float> impulses;
    bool hit = false;
    for (; steps < 120 && !hit; ++steps) {
      w.step(kDt);
      w.contactImpulsesByShape(compound, impulses);
      for (const float i : impulses)
        hit = hit || i > 0.0f;
    }
    CHECK(hit && impulses.size() == 4);
    if (hit && impulses.size() == 4) {
      std::printf("kinematic hit on step %d: piece impulses %.2f %.2f %.2f "
                  "%.2f\n",
                  steps, impulses[0], impulses[1], impulses[2], impulses[3]);
      CHECK(impulses[0] > 0.0f);
      CHECK(impulses[1] == 0.0f && impulses[2] == 0.0f && impulses[3] == 0.0f);
    }
    for (int i = 0; i < 30; ++i)
      ++steps, w.step(kDt);
    double pm[3], pc[3];
    float qm[4], qc[4], lin[3], spin[3];
    CHECK(w.getTransformByHandle(mover, pm, qm));
    CHECK(w.getTransformByHandle(compound, pc, qc));
    CHECK(w.getVelocityByHandle(mover, lin, spin));
    const double travelled = -2.0 + 6.0 * steps * static_cast<double>(kDt);
    std::printf("kinematic after %d steps: x %.3f (a free run %.3f), speed "
                "%.4f, compound x %.3f\n",
                steps, pm[0], travelled, lin[0], pc[0]);
    CHECK(near(lin[0], 6.0, 1e-4));
    CHECK(near(pm[0], travelled, 1e-3));
    float compLin[3], compSpin[3];
    CHECK(w.getVelocityByHandle(compound, compLin, compSpin));
    CHECK(near(compLin[0], 6.0, 0.3) && pc[0] > 0.5);
    // Pushed on its axis: the contact's corners leave a little sideways speed.
    CHECK(std::fabs(compLin[1]) < 1.0f && std::fabs(compLin[2]) < 1.0f);
    (void)compSpin;
  }

  // ---- contactImpulseByHandle: contacts the caller ignores are left out ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    staticBox(w, 0.0, -0.5, 5.0, 0.5);
    BodySettings b;
    const double pa[3] = {0, 0.26, 0}, pb[3] = {0, 0.77, 0};
    const BodyHandle lower = w.createBody(b, pa, kQid);
    CHECK(w.createShape(lower, b, cube(0.0f)));
    const BodyHandle upper = w.createBody(b, pb, kQid);
    CHECK(w.createShape(upper, b, cube(0.0f)));
    for (int i = 0; i < 60; ++i)
      w.step(kDt);
    // The lower cube presses on the ground and carries the upper one.
    const float all = w.contactImpulseByHandle(lower, nullptr, 0);
    const std::function<bool(BodyHandle)> notUpper = [upper](BodyHandle h) {
      return h == upper;
    };
    const std::function<bool(BodyHandle)> everything = [](BodyHandle) {
      return true;
    };
    const float ground = w.contactImpulseByHandle(lower, nullptr, 0, &notUpper);
    CHECK(all > 0.0f && ground > 0.0f && ground < all);
    // Skipping the upper cube by handle comes to the same.
    CHECK(near(ground, w.contactImpulseByHandle(lower, &upper, 1), 1e-5 * all));
    CHECK(w.contactImpulseByHandle(lower, nullptr, 0, &everything) == 0.0f);
  }

  // ---- the impulse events add up, body by body, to contactImpulseByHandle
  // for the awake bodies (what the Glue's hits read; a body asleep has no
  // events, and the Glue gives it no hits) ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    w.enableContactImpulseEvents(true);
    staticBox(w, 0.0, -0.5, 5.0, 0.5);
    BodySettings b;
    std::vector<BodyHandle> bodies;
    // A leaning stack and a pair side by side: resting, pressed and sliding
    // contacts between the bodies and with the ground.
    const double at[5][3] = {{0, 0.26, 0},
                             {0.1, 0.77, 0},
                             {0.15, 1.28, 0.05},
                             {2, 0.26, 0.4},
                             {2.45, 0.26, 0.4}};
    for (const auto &p : at) {
      const BodyHandle h = w.createBody(b, p, kQid);
      CHECK(w.createShape(h, b, cube(0.0f)));
      bodies.push_back(h);
    }
    for (int i = 0; i < 40; ++i)
      w.step(kDt);
    std::vector<ContactImpulse> events;
    w.getContactImpulseEvents(events);
    CHECK(!events.empty());
    // The same events one record per body pair: fewer records, the same sums.
    std::vector<ContactImpulseSum> sums;
    w.getContactImpulseSums(sums);
    CHECK(!sums.empty() && sums.size() < events.size());
    int awake = 0;
    bool agree = true;
    for (const BodyHandle h : bodies) {
      double fromEvents = 0.0, fromSums = 0.0;
      for (const ContactImpulse &e : events)
        if (e.a == h || e.b == h)
          fromEvents += e.impulse;
      for (const ContactImpulseSum &e : sums)
        if (e.a == h || e.b == h)
          fromSums += e.impulse;
      agree = agree && near(fromSums, fromEvents, 1e-5 * (fromEvents + 1e-9));
      if (w.isAwakeByHandle(h)) {
        ++awake;
        const double perBody = w.contactImpulseByHandle(h, nullptr, 0);
        agree =
            agree && perBody > 0.0 && near(fromEvents, perBody, 1e-4 * perBody);
      } else {
        agree = agree && fromEvents == 0.0;
      }
    }
    CHECK(awake >= 2 && agree);
  }

  // ---- contactImpulsesByShape: contacts the caller ignores are left out ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    staticBox(w, 0.0, -0.5, 5.0, 0.5);
    BodySettings b;
    const double pLow[3] = {0, 0.26, 0}, pHigh[3] = {0, 0.77, 0};
    const BodyHandle lower = w.createBody(b, pLow, kQid);
    CHECK(w.createShape(lower, b, cube(0.0f)));
    const BodyHandle upper = w.createBody(b, pHigh, kQid);
    CHECK(w.createShape(upper, b, cube(0.0f)));
    for (int i = 0; i < 60; ++i)
      w.step(kDt);
    std::vector<float> all, groundOnly, none;
    w.contactImpulsesByShape(lower, all);
    const std::function<bool(BodyHandle)> notUpper = [upper](BodyHandle h) {
      return h == upper;
    };
    const std::function<bool(BodyHandle)> everything = [](BodyHandle) {
      return true;
    };
    w.contactImpulsesByShape(lower, groundOnly, &notUpper);
    w.contactImpulsesByShape(lower, none, &everything);
    CHECK(all.size() == 1 && groundOnly.size() == 1 && none.size() == 1);
    CHECK(all[0] > 0.0f && groundOnly[0] > 0.0f && groundOnly[0] < all[0]);
    CHECK(none[0] == 0.0f);
  }

  // ---- the ledger of held pieces (GlueHolds) ----
  {
    GlueHolds holds;
    CHECK(holds.empty());
    holds.hold(1, 10.0);
    holds.hold(2, 10.0);
    holds.hold(3, 10.0);
    holds.hold(4, 4.0);
    holds.hold(4, 3.0); // never shortens
    CHECK(holds.size() == 4 && holds.held(2) && !holds.held(9));
    // 1 - 2 - 3 are bonded in a row, 4 is on its own.
    const auto neighbours = [](uint64_t piece, const auto &visit) {
      if (piece == 1)
        visit(2);
      if (piece == 2) {
        visit(1);
        visit(3);
      }
      if (piece == 3)
        visit(2);
    };
    // Refreshing from 1 reaches 2 and 3, not 4, and the time moves on.
    holds.refresh(1, 20.0, neighbours);
    // At 5 only piece 4 is due (its 4.0): it goes, the others stay held.
    CHECK(holds.expire(5.0));
    CHECK(!holds.held(4) && holds.held(1) && holds.held(2) && holds.held(3));
    // At 15 nothing is due yet.
    CHECK(!holds.expire(15.0));
    // At 21 all three are due: they go together.
    CHECK(holds.expire(21.0));
    CHECK(holds.empty());
  }

  // ---- the path through a compound: from the body to where it leaves ----
  {
    const double lo[3] = {-5, 0, -5}, hi[3] = {5, 20, 5};
    double a[3], b[3];
    // Coming at the compound from outside: swept to its far side, and its own
    // half size beyond.
    const double com[3] = {-10, 8, 0};
    const float fast[3] = {10, 0, 0};
    CHECK(pathThroughBounds(com, fast, 1.5, lo, hi, a, b));
    CHECK(near(a[0], -11.5, 1e-9) && near(b[0], 5.0 + 1.5, 1e-9));
    CHECK(near(a[1], 6.5, 1e-9) && near(b[1], 9.5, 1e-9));
    // A sideways component leaves through whichever side it reaches first: the
    // far side here (1.5 s away, the top is 2 s).
    const float diagonal[3] = {10, 6, 0};
    CHECK(pathThroughBounds(com, diagonal, 1.0, lo, hi, a, b));
    CHECK(near(b[0], -10 + 1.0 + 15.0, 1e-9) &&
          near(b[1], 8 + 1.0 + 9.0, 1e-9));
    // At rest, or moving away from bounds it is already out of: its own bounds.
    const float still[3] = {0, 0, 0}, away[3] = {-10, 0, 0};
    CHECK(!pathThroughBounds(com, still, 1.5, lo, hi, a, b));
    CHECK(near(a[0], -11.5, 1e-9) && near(b[0], -8.5, 1e-9));
    CHECK(!pathThroughBounds(com, away, 1.5, lo, hi, a, b));
    CHECK(near(a[0], -11.5, 1e-9) && near(b[0], -8.5, 1e-9));
    // Passing the compound's top (centre above it, going up and across) never
    // goes through it.
    const double above[3] = {-10, 25, 0};
    const float risingAcross[3] = {10, 4, 0};
    CHECK(!pathThroughBounds(above, risingAcross, 1.0, lo, hi, a, b));
    // Inside the bounds it goes through, to the side it reaches first.
    const double inside[3] = {0, 8, 0};
    CHECK(pathThroughBounds(inside, fast, 1.0, lo, hi, a, b));
    CHECK(near(b[0], 5.0 + 1.0, 1e-9));
  }

  // ---- a weld from a free body to a compound, at a piece's place in it ----
  {
    WorldSettings flying = ws;
    flying.gravity[0] = flying.gravity[1] = flying.gravity[2] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(flying));
    BodySettings b;
    // Pieces at x 0 and 0.5 (touching) carried by one body at the origin, the
    // second turned a quarter about z, and a free cube touching the second.
    const float quarter[4] = {0, 0, 0.70710678f, 0.70710678f};
    const double pFirst[3] = {0, 0, 0}, pSecond[3] = {0.5, 0, 0},
                 pCube[3] = {1.0, 0, 0};
    const BodyHandle first = w.createBody(b, pFirst, kQid);
    const BodyHandle second = w.createBody(b, pSecond, quarter);
    CHECK(w.createShape(first, b, cube(0.0f)) &&
          w.createShape(second, b, cube(0.0f)));
    const BodyHandle body = w.createBody(b, pFirst, kQid);
    const float none3[3] = {0, 0, 0};
    CHECK(w.copyShapesByHandle(first, body, none3, kQid) == 1);
    const float lp[3] = {0.5f, 0, 0};
    CHECK(w.copyShapesByHandle(second, body, lp, quarter) == 1);
    w.applyMassFromShapes(body);
    w.destroyBody(first);
    w.destroyBody(second);
    const BodyHandle freeCube = w.createBody(b, pCube, kQid);
    CHECK(w.createShape(freeCube, b, cube(0.0f)));

    // The bond's frame in the second piece's own axes: its face towards the
    // free cube, a half along the piece's local y (the piece is turned).
    JointSettings s = Box3DWorld::defaultJointSettings(JointType::Weld);
    s.collideConnected = false;
    JointFrame inPiece, onFree, onBody;
    inPiece.pos[0] = 0.0;
    inPiece.pos[1] = -kHalf; // local -y is world +x for a quarter turn about z
    inPiece.pos[2] = 0.0;
    for (int i = 0; i < 4; ++i)
      inPiece.rot[i] = kQid[i];
    onFree.pos[0] = -kHalf;
    onFree.pos[1] = onFree.pos[2] = 0.0;
    for (int i = 0; i < 4; ++i)
      onFree.rot[i] = quarter[i]; // the bond's axes, as the piece's own
    frameInCompound(lp, quarter, inPiece.pos, inPiece.rot, onBody.pos,
                    onBody.rot);
    CHECK(near(onBody.pos[0], 1.0 - kHalf, 1e-5));
    CHECK(w.createJoint(JointType::Weld, freeCube, body, onFree, onBody, s,
                        nullptr, true) != 0);

    // The compound is pushed and the cube is not: the weld drags it along, and
    // it stays where it was relative to the compound.
    const float push[3] = {0, 3, 0};
    w.setVelocityByHandle(body, push, none3);
    for (int i = 0; i < 90; ++i)
      w.step(kDt);
    double pb[3], pfree[3];
    float q[4];
    CHECK(w.getTransformByHandle(body, pb, q) &&
          w.getTransformByHandle(freeCube, pfree, q));
    CHECK(pb[1] > 2.0 && pfree[1] > 2.0); // both moved: the weld drags it
    // The pair turns about its centre, so the distance between them is what
    // stays.
    CHECK(near(std::hypot(pfree[0] - pb[0], pfree[1] - pb[1]), 1.0, 0.02));
  }

  // ---- copying the shapes of two bodies onto a third (a compound) ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    BodySettings b;
    const double pa[3] = {0, 5, 0}, pb[3] = {0.5, 5, 0};
    const BodyHandle a = w.createBody(b, pa, kQid);
    const BodyHandle bb = w.createBody(b, pb, kQid);
    CHECK(w.createShape(a, b, cube(0.0f)));
    // The second piece: a box and a sphere on it, denser than the first.
    BodySettings dense = b;
    dense.props.density = 3.0f;
    CHECK(w.createShape(bb, dense, cube(0.0f)));
    ShapeSpec ball;
    ball.type = ShapeType::Sphere;
    ball.radius = 0.2f;
    ball.center[1] = 0.45f;
    CHECK(w.createShape(bb, dense, ball));

    const BodyHandle all = w.createBody(b, pa, kQid);
    const float none[3] = {0, 0, 0}, shift[3] = {0.5f, 0, 0};
    CHECK(w.copyShapesByHandle(a, all, none, kQid) == 1);
    CHECK(w.copyShapesByHandle(bb, all, shift, kQid) == 2);
    CHECK(w.shapeCountByHandle(all) == 3);
    w.applyMassFromShapes(all);
    // Masses add, and the centre is between the pieces' centres.
    CHECK(near(w.massByHandle(all), w.massByHandle(a) + w.massByHandle(bb),
               1e-4 * w.massByHandle(all)));
    float m = 0, c[3] = {}, in[3] = {}, off[3] = {};
    CHECK(w.getMassDataByHandle(all, m, c, in, off));

    // Box3D's inertia of the compound agrees with merging the two pieces.
    auto piece = [&](BodyHandle h, double x, double y) {
      RigidPiece p;
      float mm = 0, cc[3] = {}, ii[3] = {}, oo[3] = {};
      CHECK(w.getMassDataByHandle(h, mm, cc, ii, oo));
      p.mass = mm;
      p.pos[0] = x + cc[0];
      p.pos[1] = y + cc[1];
      for (int i = 0; i < 3; ++i) {
        p.inertia[i] = ii[i];
        p.inertiaOff[i] = oo[i];
      }
      return p;
    };
    const MergedBody merged =
        mergePieces({piece(a, 0.0, 5.0), piece(bb, 0.5, 5.0)});
    CHECK(merged.ok && near(merged.mass, m, 1e-4 * m));
    CHECK(near(merged.com[0], c[0], 1e-4) &&
          near(merged.com[1] - 5.0, c[1], 1e-4));
    CHECK(near(merged.inertia[0], in[0], 1e-3 * in[0]) &&
          near(merged.inertia[4], in[1], 1e-3 * in[1]) &&
          near(merged.inertia[8], in[2], 1e-3 * in[2]));

    // Shapes of another kind refuse the copy: a static body's mesh.
    BodySettings st;
    st.type = BodySettings::Type::Static;
    const BodyHandle ground = w.createBody(st, pa, kQid);
    const float tri[9] = {0, 0, 0, 1, 0, 0, 0, 0, 1};
    const int32_t idx[3] = {0, 1, 2};
    CHECK(w.createMeshShape(ground, st, tri, 3, idx, 1));
    CHECK(w.copyShapesByHandle(ground, all, none, kQid) == -1);
    CHECK(w.copyShapesByHandle(a, a, none, kQid) == -1);
  }

  // ---- taking pieces out of a compound: the shapes that stay keep their
  // order, and the rest moves on as one rigid body ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    BodySettings b;
    const double p[3] = {0, 5, 0};
    const BodyHandle h = w.createBody(b, p, kQid);
    for (int i = 0; i < 3; ++i)
      CHECK(w.createShape(h, b, cube(static_cast<float>(i) * 2.0f * kHalf)));
    float m3 = 0, c[3] = {}, in[3] = {};
    CHECK(w.getMassDataByHandle(h, m3, c, in));
    // The middle one goes: two pieces left, centre between 0 and 1.
    CHECK(w.destroyShapeRangesByHandle(h, {{1, 1}}) == 1);
    w.applyMassFromShapes(h);
    float m2 = 0;
    CHECK(w.shapeCountByHandle(h) == 2 && w.getMassDataByHandle(h, m2, c, in));
    CHECK(near(m2, m3 * 2.0f / 3.0f, 1e-4 * m3) && near(c[0], 0.5, 1e-4));
    // Then the first of those (position 0): what stays is the piece at x = 1.
    CHECK(w.destroyShapeRangesByHandle(h, {{0, 1}}) == 1);
    w.applyMassFromShapes(h);
    CHECK(w.shapeCountByHandle(h) == 1 && w.getMassDataByHandle(h, m2, c, in));
    CHECK(near(c[0], 1.0, 1e-4));
    // Out of range and an empty list destroy nothing.
    CHECK(w.destroyShapeRangesByHandle(h, {{5, 2}, {-3, 2}}) == 0);
    CHECK(w.destroyShapeRangesByHandle(h, {}) == 0 &&
          w.destroyShapeRangesByHandle(0, {{0, 1}}) == 0);
  }
  {
    // Three cubes moving as one rigid body; one leaves. The others still move
    // as that body, with v + w x (new centre - old centre).
    const float m = 2.0f, I = m * 0.5f / 12.0f;
    std::vector<RigidPiece> three(3);
    for (size_t i = 0; i < 3; ++i) {
      three[i].pos[0] = (static_cast<double>(i) - 1.0) * 0.5;
      three[i].mass = m;
      three[i].inertia[0] = three[i].inertia[1] = three[i].inertia[2] = I;
    }
    three[2].lin[1] = 3.0f;
    three[0].lin[2] = -1.0f;
    const MergedBody whole = mergePieces(three);
    CHECK(whole.ok);
    for (RigidPiece &p : three)
      pieceVelocity(whole, p.pos, p.lin, p.ang);
    const MergedBody rest = mergePieces({three[0], three[2]});
    CHECK(rest.ok);
    float lin[3], ang[3];
    pieceVelocity(whole, rest.com, lin, ang);
    for (int i = 0; i < 3; ++i) {
      CHECK(near(rest.lin[i], lin[i], 1e-4));
      CHECK(near(rest.ang[i], whole.ang[i], 1e-4));
    }
  }

  // ---- the load a piece takes from the rest of a rigid body ----
  {
    // A body whose motion does not change loads nothing, however it moves.
    MergedBody steady;
    steady.ok = true;
    steady.mass = 4.0f;
    steady.lin[1] = -3.0f;
    steady.ang[2] = 1.0f;
    const double at[3] = {1.0, 0.0, 0.0};
    CHECK(near(pieceLoad(steady, steady, at, 2.0f), 0.0, 1e-6));

    // Stopped dead without turning: every piece takes mass * speed.
    MergedBody falling = steady;
    falling.ang[2] = 0.0f;
    MergedBody stopped = falling;
    stopped.lin[1] = 0.0f;
    const double near_[3] = {0.0, 0.0, 0.0}, far_[3] = {3.0, 0.0, 0.0};
    CHECK(near(pieceLoad(falling, stopped, near_, 2.0f), 6.0, 1e-4));
    CHECK(near(pieceLoad(falling, stopped, far_, 2.0f), 6.0, 1e-4));

    // Stopped at one end, so it also turns (w = 2 about z): the piece 3 away
    // from the centre takes more than the one at the centre, by m * w * r.
    MergedBody pivoted = stopped;
    pivoted.ang[2] = 2.0f;
    CHECK(near(pieceLoad(falling, pivoted, near_, 2.0f), 6.0, 1e-4));
    CHECK(pieceLoad(falling, pivoted, far_, 2.0f) > 6.0f);
  }

  // ---- components of the glue graph ----
  {
    auto bond = [](int a, int b) {
      GlueBond g;
      g.a = a;
      g.b = b;
      return g;
    };
    std::vector<GlueBond> bonds = {bond(0, 1), bond(1, 2), bond(4, 5),
                                   bond(2, 99)}; // the last is out of range
    GlueComponents c = glueComponents(6, bonds);
    CHECK(c.count == 3 && c.of.size() == 6);
    CHECK(c.of[0] == 0 && c.of[1] == 0 && c.of[2] == 0 && c.of[3] == 1 &&
          c.of[4] == 2 && c.of[5] == 2);
    CHECK(c.size == std::vector<int>({3, 1, 2}));
    // The same groups and numbers whatever the order of the bonds.
    std::vector<GlueBond> reversed(bonds.rbegin(), bonds.rend());
    const GlueComponents d = glueComponents(6, reversed);
    CHECK(d.of == c.of && d.count == c.count && d.size == c.size);
    CHECK(glueComponents(0, {}).count == 0);
  }

  // ---- many groups: 2500 chains of 4 pieces, numbered interleaved so that a
  // group's nodes are far apart, bonds in reverse order ----
  {
    const int groups = 2500, per = 4;
    std::vector<GlueBond> bonds;
    for (int k = per - 2; k >= 0; --k)
      for (int g = groups - 1; g >= 0; --g) {
        GlueBond bond;
        bond.a = g + (k + 1) * groups;
        bond.b = g + k * groups;
        bonds.push_back(bond);
      }
    const auto t0 = std::chrono::steady_clock::now();
    const GlueComponents c = glueComponents(
        static_cast<size_t>(groups) * static_cast<size_t>(per), bonds);
    const double componentsMs = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - t0)
                                    .count();
    CHECK(c.count == groups);
    bool sizesOk = c.size.size() == static_cast<size_t>(groups);
    for (const int nodes : c.size)
      sizesOk = sizesOk && nodes == per;
    CHECK(sizesOk);
    // The members of a group share its number (counted in order of each
    // group's first node, which is node g), and no other group has it.
    bool membersOk = true;
    for (int g = 0; g < groups; ++g)
      for (int k = 0; k < per; ++k)
        membersOk = membersOk && c.of[static_cast<size_t>(g + k * groups)] == g;
    CHECK(membersOk);

    // Each group merged into one body: the mass is the sum, the centre the
    // mean, and every group is its own (here on a row y = g).
    const auto t1 = std::chrono::steady_clock::now();
    bool mergedOk = true;
    for (int g = 0; g < groups; ++g) {
      std::vector<RigidPiece> pieces(per);
      for (int k = 0; k < per; ++k) {
        pieces[static_cast<size_t>(k)].pos[0] = k;
        pieces[static_cast<size_t>(k)].pos[1] = g;
        pieces[static_cast<size_t>(k)].mass = 2.0f;
        for (float &moment : pieces[static_cast<size_t>(k)].inertia)
          moment = 1.0f;
      }
      const MergedBody body = mergePieces(pieces);
      mergedOk = mergedOk && body.ok && near(body.mass, 8.0, 1e-4) &&
                 near(body.com[0], 1.5, 1e-6) && near(body.com[1], g, 1e-6);
    }
    const double mergeMs = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t1)
                               .count();
    CHECK(mergedOk);
    std::printf("%d groups of %d: components %.3f ms, merging every group "
                "%.3f ms\n",
                groups, per, componentsMs, mergeMs);
    // Far inside what a frame can spare (these were about a millisecond or less
    // each): a quadratic search over the groups would be seconds.
    CHECK(componentsMs < 250.0 && mergeMs < 250.0);
  }

  // ---- merging pieces keeps their momentum ----
  {
    // Two cubes of side 0.5 and mass 2 side by side; the right one moves up at
    // 2. About the centre (0, 0, 0): v = (0, 1, 0), and L = 0.5 m about z over
    // I = 0.2083 m gives w = 2.4, so the pieces end up at 0.4 and 1.6.
    const float m = 2.0f, I = m * 0.5f / 12.0f;
    std::vector<RigidPiece> pieces(2);
    pieces[0].pos[0] = -0.25;
    pieces[1].pos[0] = 0.25;
    for (RigidPiece &p : pieces) {
      p.mass = m;
      p.inertia[0] = p.inertia[1] = p.inertia[2] = I;
    }
    pieces[1].lin[1] = 2.0f;
    const MergedBody body = mergePieces(pieces);
    CHECK(body.ok && near(body.mass, 4.0, 1e-5));
    CHECK(near(body.com[0], 0.0) && near(body.lin[1], 1.0, 1e-5) &&
          near(body.ang[2], 2.4, 1e-4) && near(body.ang[0], 0.0, 1e-6));
    float lin[3], ang[3];
    pieceVelocity(body, pieces[0].pos, lin, ang);
    CHECK(near(lin[1], 0.4, 1e-4) && near(ang[2], 2.4, 1e-4));
    pieceVelocity(body, pieces[1].pos, lin, ang);
    CHECK(near(lin[1], 1.6, 1e-4));
    // Merging the pieces again as they now move gives the same body.
    for (RigidPiece &p : pieces)
      pieceVelocity(body, p.pos, p.lin, p.ang);
    const MergedBody again = mergePieces(pieces);
    CHECK(again.ok && near(again.lin[1], 1.0, 1e-5) &&
          near(again.ang[2], 2.4, 1e-4));

    // Pieces that already move as one rigid body keep that motion: v =
    // (1, 2, 3), w = (0, 0, 1) about the centre.
    std::vector<RigidPiece> rigid = pieces;
    rigid[0].lin[0] = rigid[1].lin[0] = 1.0f;
    rigid[0].lin[1] = 1.75f;
    rigid[1].lin[1] = 2.25f;
    rigid[0].lin[2] = rigid[1].lin[2] = 3.0f;
    for (RigidPiece &p : rigid)
      p.ang[0] = p.ang[1] = 0.0f, p.ang[2] = 1.0f;
    const MergedBody same = mergePieces(rigid);
    CHECK(same.ok && near(same.lin[0], 1.0, 1e-5) &&
          near(same.lin[1], 2.0, 1e-5) && near(same.lin[2], 3.0, 1e-5) &&
          near(same.ang[2], 1.0, 1e-5));

    // Rotated pieces: the same two cubes turned 90 degrees about x give the
    // same inertia for cubes, so the answer does not change.
    std::vector<RigidPiece> turned(2);
    for (size_t i = 0; i < 2; ++i) {
      turned[i].pos[0] = pieces[i].pos[0];
      turned[i].mass = pieces[i].mass;
      turned[i].inertia[0] = turned[i].inertia[1] = turned[i].inertia[2] =
          pieces[i].inertia[0];
      turned[i].rot[0] = turned[i].rot[3] = 0.70710678f;
    }
    turned[1].lin[1] = 2.0f;
    const MergedBody t = mergePieces(turned);
    CHECK(t.ok && near(t.ang[2], 2.4, 1e-4));

    // Unusable input: no pieces, no mass.
    CHECK(!mergePieces({}).ok);
    pieces[0].mass = 0.0f;
    CHECK(!mergePieces(pieces).ok);
  }

  // ---- how a compound reads and takes velocity and force (no gravity) ----
  {
    WorldSettings zero = ws;
    zero.gravity[0] = zero.gravity[1] = zero.gravity[2] = 0.0f;
    Box3DWorld w;
    CHECK(w.create(zero));
    BodySettings b;
    // Two cubes side by side: the centre of mass is half a cube from the
    // origin.
    auto pair = [&](double y) {
      const double p[3] = {0, y, 0};
      const BodyHandle h = w.createBody(b, p, kQid);
      CHECK(w.createShape(h, b, cube(0.0f), false));
      CHECK(w.createShape(h, b, cube(2.0f * kHalf), false));
      w.applyMassFromShapes(h);
      return h;
    };

    // Angular velocity turns the body about its centre of mass: the origin,
    // half a cube from it, swings round it, and the centre stays where it was.
    const BodyHandle spun = pair(0.0);
    const float none[3] = {0, 0, 0}, spin[3] = {0, 0, 2.0f};
    CHECK(w.setVelocityByHandle(spun, none, spin));
    w.step(kDt);
    double pos[3];
    float rot[4], lin[3], ang[3];
    CHECK(w.getTransformByHandle(spun, pos, rot));
    const double theta = 2.0 * kDt;
    CHECK(near(pos[1], -kHalf * std::sin(theta), 2e-4));
    CHECK(near(pos[0], kHalf - kHalf * std::cos(theta), 2e-4));
    CHECK(w.getVelocityByHandle(spun, lin, ang));
    CHECK(near(lin[0], 0.0, 1e-3) && near(lin[1], 0.0, 1e-3));

    // The linear velocity is the centre's, so a spin-free body moves its origin
    // at the same speed.
    const BodyHandle slid = pair(10.0);
    const float push[3] = {1.0f, 0, 0};
    CHECK(w.setVelocityByHandle(slid, push, none));
    w.step(kDt);
    CHECK(w.getTransformByHandle(slid, pos, rot));
    CHECK(near(pos[0], kDt, 1e-4));

    // Forces add up within a step: two calls move a body twice as fast as one.
    const BodyHandle once = pair(20.0), twice = pair(30.0);
    const float up[3] = {0, 50.0f, 0};
    w.applyForceByHandle(once, up, none, false);
    w.applyForceByHandle(twice, up, none, false);
    w.applyForceByHandle(twice, up, none, false);
    w.step(kDt);
    float v1[3], v2[3];
    CHECK(w.getVelocityByHandle(once, v1, ang) &&
          w.getVelocityByHandle(twice, v2, ang));
    CHECK(v1[1] > 1e-3f && near(v2[1], 2.0 * v1[1], 1e-3 * v1[1]));
  }

  // ---- a disabled (parked) body's joints: enabling one piece leaves it free
  // ----
  {
    Box3DWorld w;
    CHECK(w.create(ws));
    BodySettings b;
    const double pa[3] = {0, 5, 0}, pb[3] = {1, 5, 0};
    const BodyHandle a = w.createBody(b, pa, kQid);
    const BodyHandle bb = w.createBody(b, pb, kQid);
    CHECK(w.createShape(a, b, cube(0.0f)));
    CHECK(w.createShape(bb, b, cube(0.0f)));
    CHECK(
        w.createJoint(JointType::Weld, a, bb, frameAt(0.5, 5), frameAt(0.5, 5),
                      Box3DWorld::defaultJointSettings(JointType::Weld)) != 0);
    w.setEnabledByHandle(a, false);
    w.setEnabledByHandle(bb, false);
    for (int i = 0; i < 30; ++i)
      w.step(kDt);
    // Parked: nothing moves.
    CHECK(near(bodyY(w, a), 5.0, 1e-4) && near(bodyY(w, bb), 5.0, 1e-4));
    // One enabled again: it falls as a free body, not held by the weld.
    w.setEnabledByHandle(a, true);
    for (int i = 0; i < 30; ++i)
      w.step(kDt);
    const double fall = 0.5 * 9.80665 * 0.5 * 0.5; // after half a second
    CHECK(near(5.0 - bodyY(w, a), fall, 0.1 * fall));
    CHECK(near(bodyY(w, bb), 5.0, 1e-4));
  }

  if (fails) {
    std::printf("%d check(s) failed\n", fails);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
