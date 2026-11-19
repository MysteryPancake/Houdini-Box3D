// A compound body (several shapes on one body) as a stand-in for glued pieces:
// its mass is the pieces' sum, a beam of it does not sag the way a chain of
// weld joints can, and contactImpulsesByShape tells which piece a contact hit.
#include "TestUtil.h"
#include "core/Box3DCompound.h"
#include "core/Box3DMath.h"
#include "core/Box3DWorld.h"

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
