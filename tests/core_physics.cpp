// Exercises the real Box3D calls in the wrapper: a box falls onto a static
// ground box, comes to rest, and velocities/transforms round-trip.
#include "Box3DShapes.h"
#include "Box3DWorld.h"
#include <cmath>
#include <cstdio>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)

int main()
{
    b3dop::Box3DWorld w;
    CHECK(w.create(b3dop::WorldSettings()));

    const float qid[4] = { 0, 0, 0, 1 };
    const double groundPos[3] = { 0, -0.5, 0 };
    const double boxPos[3]    = { 0, 3.0, 0 };

    b3dop::BodySettings ground;  ground.type = b3dop::BodySettings::Type::Static;
    b3dop::BodySettings box;     // dynamic

    CHECK(w.addBody(1, ground, groundPos, qid));
    const float gh[3] = { 10, 0.5f, 10 };
    CHECK(w.addBoxShape(1, gh));
    CHECK(w.addBody(2, box, boxPos, qid));
    const float bh[3] = { 0.5f, 0.5f, 0.5f };
    CHECK(w.addBoxShape(2, bh));
    CHECK(w.addBody(3, box, boxPos, qid));
    CHECK(w.addSphereShape(3, 0.5f));
    CHECK(!w.addBody(2, box, boxPos, qid));      // duplicate id rejected
    CHECK(w.bodyCount() == 3);

    for (int i = 0; i < 240; ++i) w.step(1.0f / 60.0f);

    double p[3]; float q[4], lin[3], ang[3];
    CHECK(w.getTransform(2, p, q));
    CHECK(w.getVelocity(2, lin, ang));
    std::printf("box    rest y=%.4f  |v|=%.5f\n", p[1], std::fabs(lin[1]));
    CHECK(std::fabs(p[1] - 0.5) < 0.05);          // resting on ground top (y=0) + half extent
    CHECK(std::fabs(lin[1]) < 0.1);
    CHECK(w.getTransform(3, p, q));
    std::printf("sphere rest y=%.4f\n", p[1]);
    CHECK(std::fabs(p[1] - 0.5) < 0.05);

    // push state in and read it back (what the solver does for external edits)
    const double np[3] = { 5, 7, -2 };
    CHECK(w.setTransform(2, np, qid));
    const float v[3] = { 1, 2, 3 }, a[3] = { 0.5f, 0, 0 };
    CHECK(w.setVelocity(2, v, a));
    CHECK(w.getTransform(2, p, q));
    CHECK(std::fabs(p[0] - 5) < 1e-5 && std::fabs(p[1] - 7) < 1e-5 && std::fabs(p[2] + 2) < 1e-5);
    CHECK(w.getVelocity(2, lin, ang));
    CHECK(std::fabs(lin[0] - 1) < 1e-5 && std::fabs(lin[2] - 3) < 1e-5 && std::fabs(ang[0] - 0.5f) < 1e-5);

    // handle access gives the same answers as keyed access
    {
        const b3dop::BodyHandle h = w.bodyHandle(2);
        CHECK(h != 0);
        CHECK(w.bodyHandle(9999) == 0);
        double hp[3]; float hq[4], hl[3], ha[3];
        CHECK(w.getTransformByHandle(h, hp, hq));
        CHECK(w.getTransform(2, p, q));
        CHECK(hp[0] == p[0] && hp[1] == p[1] && hp[2] == p[2] && hq[3] == q[3]);
        CHECK(w.getVelocityByHandle(h, hl, ha));
        CHECK(hl[0] == lin[0] && hl[2] == lin[2] && ha[0] == ang[0]);
        const double np2[3] = { -4, 8, 1 };
        const float  v2[3] = { 0, 1, 0 }, a2[3] = { 0, 0, 2 };
        CHECK(w.setTransformByHandle(h, np2, qid));
        CHECK(w.setVelocityByHandle(h, v2, a2));
        CHECK(w.getTransform(2, p, q));
        CHECK(std::fabs(p[0] + 4) < 1e-5 && std::fabs(p[1] - 8) < 1e-5);
        CHECK(w.getVelocity(2, lin, ang));
        CHECK(std::fabs(lin[1] - 1) < 1e-5 && std::fabs(ang[2] - 2) < 1e-5);
        CHECK(!w.getTransformByHandle(0, hp, hq));
        // distinct bodies get distinct handles
        CHECK(w.bodyHandle(3) != h);
    }

    w.removeBody(2);
    CHECK(!w.hasBody(2) && w.bodyCount() == 2);
    CHECK(!w.getTransform(2, p, q));

    // ---- point-attribute mapping (Box3DShapes.h) + addShape/setBodyType ----
    {
        using namespace b3dop;
        ShapeType t = ShapeType::Box;
        CHECK(parseShapeType("sphere", t) && t == ShapeType::Sphere);
        CHECK(parseShapeType("box", t) && t == ShapeType::Box);
        t = ShapeType::Sphere;
        CHECK(!parseShapeType("Sphere", t) && !parseShapeType("BOX", t) && t == ShapeType::Sphere);   // case-sensitive
        t = ShapeType::Sphere;
        CHECK(!parseShapeType("proxy", t) && t == ShapeType::Sphere);        // unknown: unchanged
        CHECK(!parseShapeType("", t) && !parseShapeType(nullptr, t));

        const float one[3] = { 1, 1, 1 }, aniso[3] = { 1, 2, 4 };
        const ShapeSpec b1 = makeShapeSpec(ShapeType::Box, 2.0f, one);
        CHECK(std::fabs(b1.halfExtents[0] - 1.0f) < 1e-6f);             // unit box * pscale 2
        const ShapeSpec b2 = makeShapeSpec(ShapeType::Box, 1.0f, aniso);
        CHECK(std::fabs(b2.halfExtents[2] - 2.0f) < 1e-6f);
        const ShapeSpec s1 = makeShapeSpec(ShapeType::Sphere, 1.0f, aniso);
        CHECK(std::fabs(s1.radius - 2.0f) < 1e-6f);                      // largest component
        const float zero[3] = { 0, 0, 0 };
        CHECK(makeShapeSpec(ShapeType::Box, 1.0f, zero).halfExtents[0] > 0.0f);   // clamped

        CHECK(std::fabs(shapeVolume(b2) - 8.0f) < 1e-4f);               // 1 x 2 x 4
        CHECK(makeBodyKey(1, 5) != makeBodyKey(2, 5) && makeBodyKey(1, 5) != makeBodyKey(1, 6));

        // Two "points" with different mass, dropped from the same height, plus
        // a static ground point: mass -> density -> real Box3D mass.
        Box3DWorld pw;
        CHECK(pw.create(WorldSettings()));
        BodySettings stat; stat.type = BodySettings::Type::Static;
        const double gp[3] = { 0, -0.5, 0 };
        const float  gsc[3] = { 20, 1, 20 };
        CHECK(pw.addBody(makeBodyKey(1, 0), stat, gp, qid));
        CHECK(pw.addShape(makeBodyKey(1, 0), makeShapeSpec(ShapeType::Box, 1.0f, gsc)));

        BodySettings dyn;
        const ShapeSpec heavy = makeShapeSpec(ShapeType::Box, 1.0f, one);
        dyn.density = 50.0f / shapeVolume(heavy);
        const double hp[3] = { -2, 3, 0 }, sp[3] = { 2, 3, 0 };
        CHECK(pw.addBody(makeBodyKey(1, 1), dyn, hp, qid));
        CHECK(pw.addShape(makeBodyKey(1, 1), heavy));
        CHECK(pw.addBody(makeBodyKey(1, 2), dyn, sp, qid));
        CHECK(pw.addShape(makeBodyKey(1, 2), makeShapeSpec(ShapeType::Sphere, 1.0f, one)));
        const float v0[3] = { 1, 0, 0 }, w0[3] = { 0, 0, 0 };
        CHECK(pw.setVelocity(makeBodyKey(1, 1), v0, w0));                // initial `v`

        pw.step(1.0f / 60.0f);
        CHECK(pw.getVelocity(makeBodyKey(1, 1), lin, ang));
        CHECK(lin[0] > 0.9f);                                            // v attribute honoured

        for (int i = 0; i < 300; ++i) pw.step(1.0f / 60.0f);
        CHECK(pw.getTransform(makeBodyKey(1, 1), p, q));
        CHECK(std::fabs(p[1] - 0.5) < 0.05);                             // rests on ground
        CHECK(pw.getTransform(makeBodyKey(1, 2), p, q));
        CHECK(std::fabs(p[1] - 0.5) < 0.05);

        // active 1 -> 0: becomes static and stops moving.
        BodySettings::Type ty;
        CHECK(pw.getBodyType(makeBodyKey(1, 2), ty) && ty == BodySettings::Type::Dynamic);
        CHECK(pw.setBodyType(makeBodyKey(1, 2), BodySettings::Type::Static));
        CHECK(pw.getBodyType(makeBodyKey(1, 2), ty) && ty == BodySettings::Type::Static);
        const double lift[3] = { 2, 5, 0 };
        CHECK(pw.setTransform(makeBodyKey(1, 2), lift, qid));
        pw.step(1.0f / 60.0f);
        CHECK(pw.getTransform(makeBodyKey(1, 2), p, q));
        CHECK(std::fabs(p[1] - 5.0) < 1e-4);                             // static: gravity ignored

        // Point removed -> body removed, others untouched.
        pw.removeBody(makeBodyKey(1, 2));
        CHECK(!pw.hasBody(makeBodyKey(1, 2)) && pw.hasBody(makeBodyKey(1, 1)));

        // a capsule dropped upright on the ground (its top face is y = 0)
        // comes to rest at half its height.
        const float stand[3] = { 1, 3, 1 };
        const double capP[3] = { -6, 4, 0 };
        CHECK(pw.addBody(makeBodyKey(1, 10), dyn, capP, qid));
        CHECK(pw.addShape(makeBodyKey(1, 10), makeShapeSpec(ShapeType::Capsule, 1.0f, stand)));
        for (int i = 0; i < 300; ++i) pw.step(1.0f / 60.0f);
        CHECK(pw.getTransform(makeBodyKey(1, 10), p, q));
        CHECK(std::fabs(p[1] - 1.5) < 0.06);                              // capsule: 3 tall
        CHECK(pw.bodyMass(makeBodyKey(1, 10)) > 0.0f);

        // a capsule placed off-centre via ShapeSpec::center moves the shape, not the body
        {
            Box3DWorld w3;
            CHECK(w3.create(WorldSettings()));
            const double o[3] = { 0, 0, 0 };
            CHECK(w3.addBody(makeBodyKey(1, 1), dyn, o, qid));
            ShapeSpec sp = makeShapeSpec(ShapeType::Capsule, 1.0f, stand);
            sp.center[0] = 5.0f;
            CHECK(w3.addShape(makeBodyKey(1, 1), sp));
            CHECK(w3.shapeCount(makeBodyKey(1, 1)) == 1);
        }

        std::printf("point-mapping checks done\n");
    }


    // ---- convex shapes: names, packed-prim math, hulls, compounds -----------
    {
        using namespace b3dop;
        ShapeType t = ShapeType::Box;
        // every bullet_georep value Houdini defines
        CHECK(parseShapeType("convexhull", t) && t == ShapeType::Convex);
        CHECK(parseShapeType("concave", t)    && t == ShapeType::Concave);
        CHECK(parseShapeType("box", t)        && t == ShapeType::Box);
        CHECK(parseShapeType("capsule", t)    && t == ShapeType::Capsule);
        CHECK(parseShapeType("compound", t)   && t == ShapeType::Concave);
        CHECK(parseShapeType("sphere", t)     && t == ShapeType::Sphere);
        CHECK(parseShapeType("none", t)       && t == ShapeType::None);
        CHECK(!parseShapeType("Convexhull", t) && !parseShapeType("CONCAVE", t));   // case-sensitive
        // the old `shape` attribute names are gone
        CHECK(!parseShapeType("convex", t) && !parseShapeType("hull", t));
        CHECK(!parseShapeType("cube", t) && !parseShapeType("ball", t));
        CHECK(!parseShapeType("proxy", t));
        // Houdini values Box3D cannot do are treated as unknown (caller uses its default)
        t = ShapeType::Box;
        CHECK(!parseShapeType("cylinder", t) && !parseShapeType("plane", t) && t == ShapeType::Box);

        // quaternion <-> row matrix round trip, scale split
        const float s45 = std::sin(0.5f * 0.7f), c45 = std::cos(0.5f * 0.7f);
        const float qa[4] = { 0.0f, s45, 0.0f, c45 };               // 0.7 rad about Y
        double m[9];
        quatToMatrix3(qa, m);
        float  qb[4];  double sc[3];
        CHECK(matrix3ToQuatScale(m, qb, sc));
        CHECK(std::fabs(qb[1] - qa[1]) < 1e-5f && std::fabs(qb[3] - qa[3]) < 1e-5f);
        CHECK(std::fabs(sc[0] - 1) < 1e-5 && std::fabs(sc[2] - 1) < 1e-5);
        // Y rotation of +0.7 rad takes +X (row vector [1 0 0]) to (cos, 0, -sin).
        CHECK(std::fabs(m[0] - std::cos(0.7)) < 1e-5 && std::fabs(m[2] + std::sin(0.7)) < 1e-5);

        // scaled rotation: rows scaled by 2, 3, 4 (M = S * R)
        double ms[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                ms[r * 3 + c] = (2.0 + r) * m[r * 3 + c];
        CHECK(matrix3ToQuatScale(ms, qb, sc));
        CHECK(std::fabs(sc[0] - 2) < 1e-6 && std::fabs(sc[1] - 3) < 1e-6 && std::fabs(sc[2] - 4) < 1e-6);
        CHECK(std::fabs(qb[1] - qa[1]) < 1e-5f);
        // a degenerate matrix reports failure and gives identity
        const double zero9[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        CHECK(!matrix3ToQuatScale(zero9, qb, sc) && qb[3] == 1.0f);

        // composeLocal3: replace the rotation, keep the per-row scale. Run over
        // rotations that hit every branch of the matrix -> quaternion conversion
        // (w large, and x / y / z dominant, incl. near 180 degrees).
        {
            const float axes[][3] = { {0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {1, 2, 3}, {-2, 1, 0.5f} };
            const float angs[]    = { 0.3f, 1.2f, 2.9f, 3.1f, -2.0f };
            for (const auto &ax : axes)
                for (float an : angs)
                {
                    const float al = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
                    const float sh = std::sin(0.5f * an) / al;
                    const float qn[4] = { ax[0] * sh, ax[1] * sh, ax[2] * sh, std::cos(0.5f * an) };

                    double out[9];
                    composeLocal3(qn, ms, out);                       // ms = S(2,3,4) * R(0.7 about Y)
                    float qo[4];  double so[3];
                    CHECK(matrix3ToQuatScale(out, qo, so));
                    const float d = qo[0] * qn[0] + qo[1] * qn[1] + qo[2] * qn[2] + qo[3] * qn[3];
                    CHECK(std::fabs(std::fabs(d) - 1.0f) < 1e-5f);    // same rotation (q and -q are equal)
                    CHECK(std::fabs(so[0] - 2) < 1e-6 && std::fabs(so[1] - 3) < 1e-6 && std::fabs(so[2] - 4) < 1e-6);

                    // and it really is the row-vector rotation: row i of the result is scale[i] * rotated axis i
                    double r[9];  quatToMatrix3(qn, r);
                    float  q0[4];  double s0[3];
                    matrix3ToQuatScale(ms, q0, s0);                   // the scale compose must keep (about 2, 3, 4)
                    for (int i = 0; i < 9; ++i)
                        CHECK(std::fabs(out[i] - s0[i / 3] * r[i]) < 1e-12);
                }
            // degenerate old matrix: scale falls back to 1, the rotation is still applied
            double out0[9];
            composeLocal3(qa, zero9, out0);
            double ref0[9];  quatToMatrix3(qa, ref0);
            for (int i = 0; i < 9; ++i)
                CHECK(std::fabs(out0[i] - ref0[i]) < 1e-12);
        }

        // connected sets: prims 0-1-2 chain through shared points, 3 is alone, 4 joins 3
        {
            const std::vector<std::vector<int>> prims = { { 10, 11 }, { 11, 12 }, { 12, 13 },
                                                          { 20, 21 }, { 21, 22 }, { 30 } };
            const std::vector<int> sets = connectedSets(prims);
            CHECK(sets.size() == 6);
            CHECK(sets[0] == 0 && sets[1] == 0 && sets[2] == 0);
            CHECK(sets[3] == 1 && sets[4] == 1);
            CHECK(sets[5] == 2);
            // a late primitive that bridges two earlier sets merges them
            const std::vector<std::vector<int>> bridge = { { 1, 2 }, { 3, 4 }, { 2, 3 } };
            const std::vector<int> bs = connectedSets(bridge);
            CHECK(bs[0] == 0 && bs[1] == 0 && bs[2] == 0);
            CHECK(connectedSets(std::vector<std::vector<int>>()).empty());
        }

        // Packed prim: inner points around a pivot, M = S*R, translation t = P - pivot*S*R.
        // Body-local points must come out as (p - pivot) * S, rotation removed.
        const double pivot[3] = { 1.0, 2.0, 3.0 };
        const double P[3]     = { 10.0, -4.0, 7.0 };
        double m4[16] = { 0 };
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                m4[r * 4 + c] = ms[r * 3 + c];
        for (int c = 0; c < 3; ++c)
            m4[3 * 4 + c] = P[c] - (pivot[0] * ms[0 * 3 + c] + pivot[1] * ms[1 * 3 + c] + pivot[2] * ms[2 * 3 + c]);
        m4[15] = 1;
        const std::vector<float> inner = { 1, 2, 3,   2, 2, 3,   1, 4, 3,   1, 2, 6 };
        std::vector<float> loc;
        packedPointsToBodyLocal(inner.data(), 4, m4, pivot, loc);
        CHECK(std::fabs(loc[0]) < 1e-4f && std::fabs(loc[1]) < 1e-4f && std::fabs(loc[2]) < 1e-4f);  // the pivot is the origin
        CHECK(std::fabs(loc[3] - 2.0f) < 1e-4f && std::fabs(loc[4]) < 1e-4f);                      // +1 in x, S = 2
        CHECK(std::fabs(loc[7] - 6.0f) < 1e-4f);                                                    // +2 in y, S = 3
        CHECK(std::fabs(loc[11] - 12.0f) < 1e-4f);                                                  // +3 in z, S = 4

        // Same source, 3 different rotations / positions (float noise in m4 and P):
        // the body-local points must come out byte-identical so the hull cache hits.
        {
            Box3DWorld iw;
            CHECK(iw.create(WorldSettings()));
            std::vector<float> src;                                   // an off-centre cube around the pivot
            for (int i = 0; i < 8; ++i)
            {
                src.push_back(float(pivot[0]) + ((i & 1) ? 0.37f : -0.61f));
                src.push_back(float(pivot[1]) + ((i & 2) ? 0.52f : -0.23f));
                src.push_back(float(pivot[2]) + ((i & 4) ? 0.91f : -0.44f));
            }
            HullId first = 0;
            for (int k = 0; k < 3; ++k)
            {
                const float ang = 0.9f + 1.7f * k;
                const float qk[4] = { 0.3f * std::sin(0.5f * ang), std::sin(0.5f * ang) * 0.8f, 0.1f * std::sin(0.5f * ang), std::cos(0.5f * ang) };
                float qn2[4]; const float ln = std::sqrt(qk[0]*qk[0] + qk[1]*qk[1] + qk[2]*qk[2] + qk[3]*qk[3]);
                for (int c = 0; c < 4; ++c) qn2[c] = qk[c] / ln;
                double rk[9];  quatToMatrix3(qn2, rk);
                const double Pk[3] = { 3.1 * k + 0.7, -2.2 * k, 100.0 + 13.37 * k };
                const double sk = 1.5;                                // uniform scale
                double mk[16] = { 0 };
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        mk[r * 4 + c] = sk * rk[r * 3 + c];
                for (int c = 0; c < 3; ++c)
                    mk[12 + c] = Pk[c] - (pivot[0] * mk[0 * 4 + c] + pivot[1] * mk[1 * 4 + c] + pivot[2] * mk[2 * 4 + c]);
                mk[15] = 1;
                std::vector<float> lk;
                packedPointsToBodyLocal(src.data(), 8, mk, pivot, lk);
                const HullId hk = iw.createHull(lk.data(), 8);
                CHECK(hk != 0);
                if (k == 0) first = hk;
                CHECK(hk == first);                                   // identical bytes -> one cache entry
            }
            CHECK(iw.hullCacheSize() == 1);
        }

        // Keyed hulls: Houdini says the instances share one source, so the source is
        // hashed once and an instance only adds pivot + scale. Same source at 3
        // rotations -> one key, built once, and the points are only needed on a miss.
        {
            Box3DWorld kw;
            CHECK(kw.create(WorldSettings()));
            std::vector<float> src;
            for (int i = 0; i < 8; ++i)
            {
                src.push_back(float(pivot[0]) + ((i & 1) ? 0.37f : -0.61f));
                src.push_back(float(pivot[1]) + ((i & 2) ? 0.52f : -0.23f));
                src.push_back(float(pivot[2]) + ((i & 4) ? 0.91f : -0.44f));
            }
            const uint64_t sh = hashFloats(src.data(), src.size());
            auto instanceM4 = [&](int k, double sk, double *m) {
                const float ang = 0.9f + 1.7f * k;
                float q[4] = { 0.3f * std::sin(0.5f * ang), std::sin(0.5f * ang) * 0.8f, 0.1f * std::sin(0.5f * ang), std::cos(0.5f * ang) };
                const float ln = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
                for (int c = 0; c < 4; ++c) q[c] /= ln;
                double r[9];  quatToMatrix3(q, r);
                for (int i = 0; i < 16; ++i) m[i] = 0;
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b)
                        m[a * 4 + b] = sk * r[a * 3 + b];
                m[15] = 1;
            };
            uint64_t key0 = 0;
            int builds = 0;
            for (int k = 0; k < 3; ++k)
            {
                double m[16];  instanceM4(k, 1.5, m);
                double S[3];
                CHECK(packedScale(m, S));
                const uint64_t key = packedHullKey(sh, pivot, S, 1);
                if (k == 0) key0 = key;
                CHECK(key == key0);                                   // rotation / noise does not change the key
                float vol = 0.0f;
                if (!kw.findHull(key, &vol))
                {
                    std::vector<float> loc;
                    packedPointsToBodyLocal(src.data(), 8, m, pivot, loc);
                    CHECK(kw.createHullWithKey(key, loc.data(), 8, &vol) == key);
                    ++builds;
                }
                CHECK(vol > 0.0f);
            }
            CHECK(builds == 1 && kw.hullCacheSize() == 1);
            double m2[16];  instanceM4(0, 2.0, m2);
            double S2[3];
            CHECK(packedScale(m2, S2));
            CHECK(packedHullKey(sh, pivot, S2, 1) != key0);           // other scale: other hull
            double S1[3];
            double m1[16];  instanceM4(0, 1.5, m1);
            CHECK(packedScale(m1, S1));
            CHECK(packedHullKey(sh, pivot, S1, 2) != key0);           // per-piece vs merged: other key
            CHECK(kw.createHullWithKey(0, src.data(), 8) == 0);       // 0 is not a key
        }

        // fit a box / sphere to an off-centre point set
        ShapeSpec fit;
        const std::vector<float> off = { 2, 0, 0,  4, 0, 0,  2, 2, 0,  2, 0, 6 };
        CHECK(fitShapeToPoints(ShapeType::Box, off.data(), 4, fit));
        CHECK(fit.type == ShapeType::Box && std::fabs(fit.center[0] - 3) < 1e-6f && std::fabs(fit.halfExtents[2] - 3) < 1e-6f);
        CHECK(fitShapeToPoints(ShapeType::Sphere, off.data(), 4, fit) && std::fabs(fit.radius - 3) < 1e-6f);
        const std::vector<float> flat = { 0, 0, 0,  1, 0, 0,  0, 0, 1,  1, 0, 1 };
        CHECK(fitShapeToPoints(ShapeType::Box, flat.data(), 4, fit) && fit.halfExtents[1] > 0.0f);   // clamped
        CHECK(!fitShapeToPoints(ShapeType::Box, off.data(), 0, fit));

        // capsule fits: bounds are x 2..4, y 0..2, z 0..6
        CHECK(fitShapeToPoints(ShapeType::Capsule, off.data(), 4, fit));
        CHECK(fit.type == ShapeType::Capsule && std::fabs(fit.radius - 3.0f) < 1e-6f   // wider of x / z
              && std::fabs(fit.center[0] - 3.0f) < 1e-6f);
        CHECK(fit.halfLength == 0.0f);                                            // too short: a sphere
        const std::vector<float> tall = { -1, 0, -1,  1, 0, -1,  -1, 10, 1,  1, 0, 1 };
        CHECK(fitShapeToPoints(ShapeType::Capsule, tall.data(), 4, fit));
        CHECK(std::fabs(fit.radius - 1.0f) < 1e-6f && std::fabs(fit.halfLength - 4.0f) < 1e-6f);   // 10 tall = 4 + 4 + 2 caps

        // makeShapeSpec for the capsule, and its volume
        const float tallScale[3] = { 1, 4, 2 };
        const ShapeSpec cap = makeShapeSpec(ShapeType::Capsule, 1.0f, tallScale);
        CHECK(cap.type == ShapeType::Capsule && std::fabs(cap.radius - 1.0f) < 1e-6f && std::fabs(cap.halfLength - 1.0f) < 1e-6f);
        CHECK(std::fabs(shapeVolume(cap) - (3.14159265f * 2.0f + (4.0f / 3.0f) * 3.14159265f)) < 1e-3f);
        const float oneS[3] = { 1, 1, 1 };
        CHECK(makeShapeSpec(ShapeType::Capsule, 1.0f, oneS).halfLength == 0.0f);          // unit capsule is a sphere

        // ---- hulls ------------------------------------------------------------
        Box3DWorld hw;
        CHECK(hw.create(WorldSettings()));
        auto cube = [](float cx, float cy, float cz, float h) {
            std::vector<float> v;
            for (int i = 0; i < 8; ++i)
            {
                v.push_back(cx + ((i & 1) ? h : -h));
                v.push_back(cy + ((i & 2) ? h : -h));
                v.push_back(cz + ((i & 4) ? h : -h));
            }
            return v;
        };
        float vol = 0.0f;
        const std::vector<float> unitCube = cube(0, 0, 0, 0.5f);
        const HullId h1 = hw.createHull(unitCube.data(), 8, &vol);
        CHECK(h1 != 0 && std::fabs(vol - 1.0f) < 1e-3f);
        CHECK(hw.createHull(unitCube.data(), 8) == h1 && hw.hullCacheSize() == 1);     // cached by content
        CHECK(hw.createHull(unitCube.data(), 3) == 0);                                 // < 4 points
        const std::vector<float> flatQuad = { 0, 0, 0,  1, 0, 0,  0, 0, 1,  1, 0, 1 };
        CHECK(hw.createHull(flatQuad.data(), 4) == 0);                                 // flat: no volume
        std::vector<float> bad = unitCube;  bad[4] = std::nanf("");
        CHECK(hw.createHull(bad.data(), 8) == 0);                                      // non-finite
        // a dense cloud (500 points on a sphere) still gives a valid hull
        std::vector<float> cloud;
        for (int i = 0; i < 500; ++i)
        {
            const float zz = 1.0f - 2.0f * (i + 0.5f) / 500.0f, rr = std::sqrt(1.0f - zz * zz), a = 2.39996323f * i;
            cloud.push_back(rr * std::cos(a));  cloud.push_back(rr * std::sin(a));  cloud.push_back(zz);
        }
        float cvol = 0.0f;
        CHECK(hw.createHull(cloud.data(), 500, &cvol) != 0 && cvol > 3.3f && cvol < 4.2f);   // unit sphere is 4.19; the hull is cut down to fit the limits
        hw.pruneHulls();                                                               // nothing used them yet
        CHECK(hw.hullCacheSize() == 0);

        // Ground, then: (a) a hull whose pivot is NOT its centre, (b) a two-piece
        // dynamic compound, (c) 50 instances of one hull.
        BodySettings stat; stat.type = BodySettings::Type::Static;
        const double gp[3] = { 0, -0.5, 0 };
        const float  gh[3] = { 30, 0.5f, 30 };
        CHECK(hw.addBody(1, stat, gp, qid) && hw.addBoxShape(1, gh));

        // (a) cube centred at x = +2 from the body origin, dropped with the origin at x = 0.
        const std::vector<float> offCube = cube(2, 0, 0, 0.5f);
        float offVol = 0.0f;
        const HullId ho = hw.createHull(offCube.data(), 8, &offVol);
        CHECK(ho != 0);
        BodySettings db;  db.density = 4.0f;
        const double sp[3] = { 0, 3, 0 };
        CHECK(hw.addBody(10, db, sp, qid));
        CHECK(hw.setBodyDensity(10, 40.0f / offVol));            // mass attribute 40 -> density
        CHECK(hw.addHullShape(10, ho));
        CHECK(std::fabs(hw.bodyMass(10) - 40.0f) < 0.05f);
        CHECK(hw.shapeCount(10) == 1);

        // (b) L shape from two cubes, as two pieces; total mass 10.
        const std::vector<float> pA = cube(0, 0, 0, 0.5f), pB = cube(1, 0, 0, 0.5f);
        float vA = 0, vB = 0;
        const HullId hA = hw.createHull(pA.data(), 8, &vA), hB = hw.createHull(pB.data(), 8, &vB);
        CHECK(hA != 0 && hB != 0 && hA != hB);
        const double lp[3] = { 6, 3, 0 };
        CHECK(hw.addBody(11, db, lp, qid));
        CHECK(hw.setBodyDensity(11, 10.0f / (vA + vB)));
        CHECK(hw.addHullShape(11, hA) && hw.addHullShape(11, hB));
        CHECK(hw.shapeCount(11) == 2 && std::fabs(hw.bodyMass(11) - 10.0f) < 0.05f);

        // (c) instancing: 50 bodies, one hull build
        for (int i = 0; i < 50; ++i)
        {
            const double ip[3] = { -20.0 + i * 0.8, 3, 8 };
            CHECK(hw.addBody(100 + i, db, ip, qid) && hw.addHullShape(100 + i, hw.createHull(unitCube.data(), 8)));
        }
        hw.pruneHulls();
        CHECK(hw.hullCacheSize() == 3);          // unit cube (also piece A), offset cube, piece B
        for (int i = 0; i < 400; ++i) hw.step(1.0f / 60.0f);

        double hp[3];  float hq[4];
        CHECK(hw.getTransform(10, hp, hq));
        std::printf("offset hull  origin y=%.4f x=%.4f\n", hp[1], hp[0]);
        CHECK(std::fabs(hp[1] - 0.5) < 0.06);                    // cube centre is on the origin's height
        CHECK(hw.getTransform(11, hp, hq));
        std::printf("L compound   origin y=%.4f\n", hp[1]);
        CHECK(std::fabs(hp[1] - 0.5) < 0.06);                    // lies flat on the ground
        CHECK(hw.getTransform(120, hp, hq));
        CHECK(std::fabs(hp[1] - 0.5) < 0.06);

        // Removing bodies releases their hulls.
        for (int i = 0; i < 50; ++i) hw.removeBody(100 + i);
        hw.removeBody(10);
        hw.removeBody(11);
        hw.pruneHulls();
        CHECK(hw.hullCacheSize() == 0);

        // box / sphere with a centre offset (fitted to packed bounds)
        CHECK(hw.addBody(20, db, sp, qid));
        ShapeSpec offBox;  offBox.type = ShapeType::Box;  offBox.center[0] = 3.0f;
        CHECK(hw.addShape(20, offBox));
        CHECK(hw.addBody(21, db, sp, qid));
        ShapeSpec offSph;  offSph.type = ShapeType::Sphere;  offSph.center[1] = 2.0f;  offSph.radius = 0.5f;
        CHECK(hw.addShape(21, offSph));
        std::printf("convex-shape checks done\n");
    }

    // ---- explicit primitive overrides (bullet_primS / primR / primT / radius / length) ----
    {
        using namespace b3dop;
        auto near3 = [](const float a[3], float x, float y, float z) {
            return std::fabs(a[0] - x) < 1e-5f && std::fabs(a[1] - y) < 1e-5f && std::fabs(a[2] - z) < 1e-5f;
        };

        // quatRotate: 90 degrees about X turns Y into Z (the Euler -> quaternion step is
        // the HDK's now, so only the rotation itself is checked here).
        {
            const float h = 0.70710678f;
            const float qx90[4] = { h, 0, 0, h };
            const float up[3] = { 0, 1, 0 };
            float r[3];
            quatRotate(qx90, up, r);
            CHECK(near3(r, 0, 0, 1));
        }

        // applyPrimOverrides
        {
            const float one[3] = { 1, 1, 1 };
            PrimOverrides o;
            CHECK(!o.any());
            ShapeSpec box = makeShapeSpec(ShapeType::Box, 1.0f, one);
            o.hasSize = true;  o.size[0] = 2;  o.size[1] = 0.5f;  o.size[2] = 1;       // HALF extents
            o.hasPos  = true;  o.pos[0] = 3;
            applyPrimOverrides(box, o);
            CHECK(near3(box.halfExtents, 2, 0.5f, 1) && box.center[0] == 3.0f);
            CHECK(std::fabs(shapeVolume(box) - 8.0f * 2 * 0.5f * 1) < 1e-5f);

            ShapeSpec cap = makeShapeSpec(ShapeType::Capsule, 1.0f, one);
            PrimOverrides oc;
            oc.hasRadius = true;  oc.radius = 0.25f;
            oc.hasLength = true;  oc.length = 3.0f;                                      // straight section
            applyPrimOverrides(cap, oc);
            CHECK(std::fabs(cap.radius - 0.25f) < 1e-6f && std::fabs(cap.halfLength - 1.5f) < 1e-6f);

            // size does not touch a sphere, radius does; only what is given changes
            ShapeSpec sph = makeShapeSpec(ShapeType::Sphere, 1.0f, one);
            applyPrimOverrides(sph, o);
            CHECK(std::fabs(sph.radius - 0.5f) < 1e-6f && sph.center[0] == 3.0f);
            applyPrimOverrides(sph, oc);
            CHECK(std::fabs(sph.radius - 0.25f) < 1e-6f);

            // a rotation override is carried through; a non-finite one is ignored
            ShapeSpec rb = makeShapeSpec(ShapeType::Box, 1.0f, one);
            PrimOverrides orr;
            orr.hasRot = true;  orr.rot[2] = 0.70710678f;  orr.rot[3] = 0.70710678f;
            applyPrimOverrides(rb, orr);
            CHECK(std::fabs(rb.rot[2] - 0.70710678f) < 1e-6f && std::fabs(rb.rot[3] - 0.70710678f) < 1e-6f);
            orr.rot[0] = NAN;
            ShapeSpec rb2 = makeShapeSpec(ShapeType::Box, 1.0f, one);
            applyPrimOverrides(rb2, orr);
            CHECK(rb2.rot[3] == 1.0f);

            // bad values are ignored / clamped
            ShapeSpec bad = makeShapeSpec(ShapeType::Box, 1.0f, one);
            PrimOverrides ob;
            ob.hasSize = true;  ob.size[0] = NAN;  ob.size[1] = 1;  ob.size[2] = 1;
            applyPrimOverrides(bad, ob);
            CHECK(near3(bad.halfExtents, 0.5f, 0.5f, 0.5f));
            ob.size[0] = 0;
            applyPrimOverrides(bad, ob);
            CHECK(bad.halfExtents[0] > 0.0f);
            ShapeSpec flat = makeShapeSpec(ShapeType::Capsule, 1.0f, one);
            PrimOverrides ol;  ol.hasLength = true;  ol.length = -2.0f;
            applyPrimOverrides(flat, ol);
            CHECK(flat.halfLength == 0.0f);
        }

        // Rotated shapes under real physics, resting on a ground plane (top at y = 0).
        {
            Box3DWorld rw;
            CHECK(rw.create(WorldSettings()));
            const float qid[4] = { 0, 0, 0, 1 };
            BodySettings st;  st.type = BodySettings::Type::Static;
            BodySettings dyn;
            const double g[3] = { 0, -0.5, 0 };
            CHECK(rw.addBody(1, st, g, qid));
            const float gh[3] = { 20, 0.5f, 20 };
            CHECK(rw.addBoxShape(1, gh));

            // capsule radius 0.5, straight section 2, lying on its side (turned 90 about Z):
            // rests at y = radius.
            const float rz90[4] = { 0, 0, 0.70710678f, 0.70710678f };   // 90 degrees about Z
            ShapeSpec cap;  cap.type = ShapeType::Capsule;  cap.radius = 0.5f;  cap.halfLength = 1.0f;
            for (int i = 0; i < 4; ++i) cap.rot[i] = rz90[i];
            const double cp[3] = { 0, 3, 0 };
            CHECK(rw.addBody(2, dyn, cp, qid));
            CHECK(rw.addShape(2, cap));

            // box 4 x 1 x 1 turned 90 about Z: stands 4 tall, rests at y = 2.
            ShapeSpec box;  box.type = ShapeType::Box;  box.halfExtents[0] = 2.0f;
            box.halfExtents[1] = 0.5f;  box.halfExtents[2] = 0.5f;
            for (int i = 0; i < 4; ++i) box.rot[i] = rz90[i];
            const double bp[3] = { 8, 4, 0 };
            CHECK(rw.addBody(3, dyn, bp, qid));
            CHECK(rw.addShape(3, box));
            CHECK(rw.shapeCount(3) == 1);
            CHECK(std::fabs(rw.bodyMass(3) - shapeVolume(box)) < 0.05f);   // density 1: mass = volume

            // an unrotated box with the same offset, placed 3 to the side, keeps its body origin
            ShapeSpec off;  off.type = ShapeType::Box;  off.center[0] = 3.0f;
            const double op[3] = { -8, 3, 0 };
            CHECK(rw.addBody(4, dyn, op, qid));
            CHECK(rw.addShape(4, off));

            for (int i = 0; i < 420; ++i) rw.step(1.0f / 60.0f);
            double p[3];  float q[4];
            CHECK(rw.getTransform(2, p, q));
            std::printf("rotated capsule rest y=%.4f\n", p[1]);
            CHECK(std::fabs(p[1] - 0.5) < 0.06);
            CHECK(rw.getTransform(3, p, q));
            std::printf("rotated box     rest y=%.4f\n", p[1]);
            CHECK(std::fabs(p[1] - 2.0) < 0.08);
        }
        std::printf("primitive-override checks done\n");
    }

    // ---- move events: only awake bodies, with the transform, user data and sleep flag ----
    {
        b3dop::Box3DWorld ew;
        b3dop::WorldSettings ws;
        CHECK(ew.create(ws));
        const float q0[4] = { 0, 0, 0, 1 };
        b3dop::BodySettings st;  st.type = b3dop::BodySettings::Type::Static;
        const double gp[3] = { 0, -0.5, 0 };
        const float  gh[3] = { 20, 0.5f, 20 };
        CHECK(ew.addBody(1, st, gp, q0));
        CHECK(ew.addBoxShape(1, gh));
        const double bp[3] = { 0, 0.5, 0 };
        CHECK(ew.addBody(2, b3dop::BodySettings(), bp, q0));
        const float bh[3] = { 0.5f, 0.5f, 0.5f };
        CHECK(ew.addBoxShape(2, bh));
        int tag = 7;
        ew.setBodyUserData(ew.bodyHandle(2), &tag);

        CHECK(ew.moveEventCount() == 0);                    // nothing stepped yet
        ew.step(1.0f / 60.0f);
        CHECK(ew.moveEventCount() == 1);                    // the static ground never moves
        b3dop::BodyMove mv;
        CHECK(ew.getMoveEvent(0, mv));
        CHECK(mv.userData == &tag && mv.handle == ew.bodyHandle(2));
        double p[3];  float q[4];
        CHECK(ew.getTransformByHandle(mv.handle, p, q));
        CHECK(mv.pos[0] == p[0] && mv.pos[1] == p[1] && mv.pos[2] == p[2]);
        CHECK(mv.rot[0] == q[0] && mv.rot[3] == q[3]);
        CHECK(!ew.getMoveEvent(1, mv) && !ew.getMoveEvent(-1, mv));

        // the bulk accessor agrees with the single one, and reuses the vector
        std::vector<b3dop::BodyMove> all;
        CHECK(ew.getMoveEvents(all) == 1 && all.size() == 1);
        CHECK(all[0].userData == &tag && all[0].handle == mv.handle && all[0].pos[1] == mv.pos[1]);

        // releasing the build settings keeps type queries and changes working (Box3D answers)
        b3dop::BodySettings::Type ty;
        CHECK(ew.getBodyType(2, ty) && ty == b3dop::BodySettings::Type::Dynamic);
        ew.releaseBuildState(2);
        CHECK(ew.getBodyType(2, ty) && ty == b3dop::BodySettings::Type::Dynamic);
        CHECK(ew.setBodyType(2, b3dop::BodySettings::Type::Kinematic));
        CHECK(ew.getBodyType(2, ty) && ty == b3dop::BodySettings::Type::Kinematic);
        CHECK(ew.setBodyType(2, b3dop::BodySettings::Type::Dynamic));
        CHECK(!ew.getBodyType(99, ty));

        // resting on the ground, it falls asleep: one last event with the flag, then none
        bool sawSleep = false;
        int  lastCount = 1;
        for (int i = 0; i < 600 && lastCount > 0; ++i)
        {
            ew.step(1.0f / 60.0f);
            lastCount = ew.moveEventCount();
            if (lastCount && ew.getMoveEvent(0, mv) && mv.fellAsleep)
                sawSleep = true;
        }
        CHECK(sawSleep);
        CHECK(ew.moveEventCount() == 0);
        // a transform set by the caller is not a move event
        const double np[3] = { 5, 0.5, 0 };
        CHECK(ew.setTransformByHandle(ew.bodyHandle(2), np, q0));
        CHECK(ew.moveEventCount() == 0);
        std::printf("move-event checks done\n");
    }

    {
        // sanitizeQuat / storedPosition: the write-back predicts what the geometry holds
        // with these instead of reading it back, so their behaviour is pinned here.
        float z[4] = { 0, 0, 0, 0 };
        b3dop::sanitizeQuat(z);
        CHECK(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 1);           // zero -> identity
        float nan4[4] = { NAN, 0, 0, 1 };
        b3dop::sanitizeQuat(nan4);
        CHECK(nan4[3] == 1 && nan4[0] == 0);                                // non-finite -> identity
        float off[4] = { 0.1f, 0.2f, 0.3f, 1.7f };
        b3dop::sanitizeQuat(off);
        double l2 = 0; for (int i = 0; i < 4; ++i) l2 += double(off[i]) * off[i];
        CHECK(std::fabs(l2 - 1.0) < 1e-6);                                  // normalised
        float again[4] = { off[0], off[1], off[2], off[3] };
        b3dop::sanitizeQuat(again);
        double drift = 0; for (int i = 0; i < 4; ++i) drift += std::fabs(double(again[i]) - off[i]);
        CHECK(drift < 1e-6);                                                // idempotent to float noise

        const double in[3] = { 0.1, -12345.678901234, 1e-9 };
        double out[3];
        CHECK(b3dop::storedPosition(in, out));
        for (int i = 0; i < 3; ++i) CHECK(out[i] == double(float(in[i])));
        double out2[3];
        CHECK(b3dop::storedPosition(out, out2));                            // idempotent: no drift on re-store
        for (int i = 0; i < 3; ++i) CHECK(out2[i] == out[i]);
        const double huge[3] = { 1e300, 0, 0 };
        CHECK(!b3dop::storedPosition(huge, out));                           // overflows float -> not finite
        std::printf("stored-pose checks done\n");
    }

    // ---- handle-based building: createBody / createShape / createHullShape / applyMassFromShapes ----
    {
        using namespace b3dop;

        // ShapeTypeCache gives parseShapeType's answers, and leaves `out` alone on a miss
        {
            ShapeTypeCache cache;
            const char *names[] = { "box", "sphere", "convexhull", "concave", "compound", "capsule", "none",
                                    "cylinder", "", nullptr, "Box", "box", "box", "sphere", "box" };
            for (const char *n : names)
            {
                ShapeType a = ShapeType::Capsule, b = ShapeType::Capsule;    // sentinel: unchanged on failure
                const bool ra = parseShapeType(n, a);
                const bool rb = cache.parse(n, b);
                CHECK(ra == rb && a == b);
            }
            CHECK(cache.size() == 9);        // distinct non-empty strings seen (case-sensitive)
        }

    }

    {
        using namespace b3dop;
        Box3DWorld hw;
        CHECK(hw.create(WorldSettings()));
        hw.reserve(100);                         // a hint only

        BodySettings stat; stat.type = BodySettings::Type::Static;
        const double gp[3] = { 0, -0.5, 0 };
        const float  gh[3] = { 30, 0.5f, 30 };
        ShapeSpec groundBox;  groundBox.type = ShapeType::Box;
        groundBox.halfExtents[0] = gh[0];  groundBox.halfExtents[1] = gh[1];  groundBox.halfExtents[2] = gh[2];
        const BodyHandle ground = hw.createBody(stat, gp, qid);
        CHECK(ground != 0 && hw.createShape(ground, stat, groundBox));
        CHECK(hw.bodyCount() == 1);

        // user data comes back in the move events (the solver maps an event to its mirror entry with it)
        int tag = 7;
        BodySettings dyn;  dyn.density = 2.0f;
        const double dp[3] = { 0, 3, 0 };
        const BodyHandle falling = hw.createBody(dyn, dp, qid, &tag);
        ShapeSpec cubeSpec;  cubeSpec.type = ShapeType::Box;
        cubeSpec.halfExtents[0] = cubeSpec.halfExtents[1] = cubeSpec.halfExtents[2] = 0.5f;
        CHECK(falling != 0 && hw.createShape(falling, dyn, cubeSpec));
        CHECK(hw.bodyCount() == 2);
        const float lin[3] = { 1, 0, 0 }, ang[3] = { 0, 0, 0 };
        CHECK(hw.setVelocityByHandle(falling, lin, ang));
        hw.step(1.0f / 60.0f);
        bool sawTag = false;
        BodyMove mv;
        for (int i = 0; i < hw.moveEventCount(); ++i)
            if (hw.getMoveEvent(i, mv) && mv.handle == falling && mv.userData == &tag)
                sawTag = true;
        CHECK(sawTag);

        // a handle-made body is invisible to the keyed API (no maps are filled)
        CHECK(!hw.hasBody(1) && hw.bodyHandle(1) == 0);

        // Mass is the same whether it is recomputed per shape (keyed API) or applied once
        // (handle API, updateMass = false): an L of two cubes at density 5, then three pieces.
        auto cube = [](float cx, float cy, float cz, float h) {
            std::vector<float> v;
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2)
                    {
                        v.push_back(cx + sx * h);  v.push_back(cy + sy * h);  v.push_back(cz + sz * h);
                    }
            return v;
        };
        const std::vector<float> pA = cube(0, 0, 0, 0.5f), pB = cube(1, 0, 0, 0.5f), pC = cube(0, 1, 0, 0.5f);
        float vA = 0, vB = 0, vC = 0;
        const HullId hA = hw.createHull(pA.data(), 8, &vA);
        const HullId hB = hw.createHull(pB.data(), 8, &vB);
        const HullId hC = hw.createHull(pC.data(), 8, &vC);
        CHECK(hA && hB && hC);
        BodySettings dense;  dense.density = 5.0f;
        const double lp[3] = { 6, 3, 0 };

        CHECK(hw.addBody(500, dense, lp, qid) && hw.addHullShape(500, hA) && hw.addHullShape(500, hB) &&
              hw.addHullShape(500, hC));
        const BodyHandle once = hw.createBody(dense, lp, qid);
        CHECK(once != 0);
        CHECK(hw.createHullShape(once, dense, hA, false) && hw.createHullShape(once, dense, hB, false) &&
              hw.createHullShape(once, dense, hC, false));
        hw.applyMassFromShapes(once);
        CHECK(std::fabs(hw.bodyMass(500) - 5.0f * (vA + vB + vC)) < 1e-3f);

        // run both for a while: identical behaviour (they sit apart, on the ground)
        const double lp2[3] = { -6, 3, 0 };
        const BodyHandle once2 = hw.createBody(dense, lp2, qid);    // same body, other place
        CHECK(hw.createHullShape(once2, dense, hA, false) && hw.createHullShape(once2, dense, hB, false) &&
              hw.createHullShape(once2, dense, hC, false));
        hw.applyMassFromShapes(once2);
        const double lp3[3] = { -6, 3, 20 };
        CHECK(hw.addBody(501, dense, lp3, qid) && hw.addHullShape(501, hA) && hw.addHullShape(501, hB) &&
              hw.addHullShape(501, hC));
        for (int i = 0; i < 240; ++i) hw.step(1.0f / 60.0f);
        double p1[3], p2[3];  float q1[4], q2[4];
        CHECK(hw.getTransformByHandle(once2, p1, q1) && hw.getTransform(501, p2, q2));
        CHECK(std::fabs(p1[1] - p2[1]) < 1e-3);                      // same resting height
        for (int i = 0; i < 4; ++i) CHECK(std::fabs(q1[i] - q2[i]) < 1e-3f);

        // setTypeByHandle: a dynamic body made static stops falling
        const double fp[3] = { 20, 5, 0 };
        const BodyHandle hover = hw.createBody(dyn, fp, qid);
        CHECK(hover != 0 && hw.createShape(hover, dyn, cubeSpec));
        CHECK(hw.setTypeByHandle(hover, BodySettings::Type::Static));
        for (int i = 0; i < 30; ++i) hw.step(1.0f / 60.0f);
        double hp[3];  float hq[4];
        CHECK(hw.getTransformByHandle(hover, hp, hq) && std::fabs(hp[1] - 5.0) < 1e-6);

        // destroyBody releases the hulls the body used; the count drops by one per body
        const int before = hw.bodyCount();
        hw.destroyBody(once);
        hw.destroyBody(once2);
        CHECK(hw.bodyCount() == before - 2);
        hw.removeBody(500);
        hw.removeBody(501);
        hw.pruneHulls();
        CHECK(hw.hullCacheSize() == 0);

        // destroying a body twice (or handle 0) is harmless
        hw.destroyBody(once);
        hw.destroyBody(0);
        CHECK(hw.bodyCount() == before - 4);
        std::printf("handle-build checks done\n");
    }

    // ---- mid-simulation changes (what the solver does when an attribute changes) ----
    {
        using namespace b3dop;
        Box3DWorld dw;
        CHECK(dw.create(WorldSettings()));
        const float qid[4] = { 0, 0, 0, 1 };
        BodySettings stat;  stat.type = BodySettings::Type::Static;
        const double gp[3] = { 0, -0.5, 0 };
        const BodyHandle ground = dw.createBody(stat, gp, qid);
        ShapeSpec groundSpec;  groundSpec.halfExtents[0] = groundSpec.halfExtents[2] = 100.0f;
        groundSpec.halfExtents[1] = 0.5f;
        CHECK(ground && dw.createShape(ground, stat, groundSpec));

        ShapeSpec cube;        // unit cube, volume 1
        BodySettings dyn;
        const double p0[3] = { 0, 0.5, 0 };
        const BodyHandle b = dw.createBody(dyn, p0, qid);
        CHECK(b && dw.createShape(b, dyn, cube));
        CHECK(std::fabs(dw.massByHandle(b) - 1.0f) < 1e-4f);

        // friction / restitution reach the shapes
        dw.setMaterialByHandle(b, 0.25f, 0.75f);
        float f = 0, r = 0;
        CHECK(dw.getMaterialByHandle(b, f, r) && f == 0.25f && r == 0.75f);

        // damping, gravity scale
        dw.setDampingByHandle(b, 0.5f, 2.0f);
        float ld = 0, ad = 0;
        CHECK(dw.getDampingByHandle(b, ld, ad) && ld == 0.5f && ad == 2.0f);
        dw.setGravityScaleByHandle(b, -1.0f);
        CHECK(dw.gravityScaleByHandle(b) == -1.0f);

        // mass via density (volume 1 -> mass = density); inertia follows the shape
        dw.setDensityByHandle(b, 7.0f);
        CHECK(std::fabs(dw.massByHandle(b) - 7.0f) < 1e-4f);

        // gravity scale -1 floats the body up
        for (int i = 0; i < 30; ++i) dw.step(1.0f / 60.0f);
        double p[3];  float q[4];
        CHECK(dw.getTransformByHandle(b, p, q) && p[1] > 0.6);

        // sleep permission: a body that may not sleep is awake and stays so
        dw.setGravityScaleByHandle(b, 0.0f);
        const float zero[3] = { 0, 0, 0 };
        dw.setVelocityByHandle(b, zero, zero);
        dw.setSleepEnabledByHandle(b, false);
        CHECK(!dw.isSleepEnabledByHandle(b));
        for (int i = 0; i < 200; ++i) dw.step(1.0f / 60.0f);
        CHECK(dw.isAwakeByHandle(b));
        dw.setSleepEnabledByHandle(b, true);
        CHECK(dw.isSleepEnabledByHandle(b));
        for (int i = 0; i < 200; ++i) dw.step(1.0f / 60.0f);
        CHECK(!dw.isAwakeByHandle(b));                  // still body, allowed to sleep: asleep

        // disabled: does not move, keeps its shapes; enabled again: moves
        dw.setGravityScaleByHandle(b, 1.0f);
        dw.wakeByHandle(b);
        CHECK(dw.isAwakeByHandle(b));
        dw.setEnabledByHandle(b, false);
        CHECK(!dw.isEnabledByHandle(b));
        double before[3];
        CHECK(dw.getTransformByHandle(b, before, q));
        for (int i = 0; i < 10; ++i) dw.step(1.0f / 60.0f);
        CHECK(dw.getTransformByHandle(b, p, q) && p[1] == before[1]);
        CHECK(dw.shapeCountByHandle(b) == 1);
        dw.setEnabledByHandle(b, true);
        CHECK(dw.isEnabledByHandle(b));
        for (int i = 0; i < 10; ++i) dw.step(1.0f / 60.0f);
        CHECK(dw.getTransformByHandle(b, p, q) && p[1] < before[1]);

        // rebuild: drop the shapes, add a bigger cube; mass recomputed from the new shape
        float pts[24];
        int k = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    pts[k++] = float(sx);  pts[k++] = float(sy);  pts[k++] = float(sz);
                }
        float hv = 0;
        const HullId big = dw.createHull(pts, 8, &hv);      // 2x2x2, volume 8
        CHECK(big && std::fabs(hv - 8.0f) < 1e-3f);
        dw.destroyShapesByHandle(b);
        CHECK(dw.shapeCountByHandle(b) == 0);
        BodySettings dense = dyn;  dense.density = 7.0f / 8.0f;      // keep mass 7
        CHECK(dw.createHullShape(b, dense, big, false));
        dw.applyMassFromShapes(b);
        CHECK(dw.shapeCountByHandle(b) == 1);
        CHECK(std::fabs(dw.massByHandle(b) - 7.0f) < 1e-3f);
        // rebuilding again releases the hull it used: prune empties the cache
        dw.destroyShapesByHandle(b);
        CHECK(dw.createShape(b, dyn, cube));
        dw.pruneHulls();
        CHECK(dw.hullCacheSize() == 0);
        // and the body comes to rest on the ground at its new size
        for (int i = 0; i < 240; ++i) dw.step(1.0f / 60.0f);
        CHECK(dw.getTransformByHandle(b, p, q) && std::fabs(p[1] - 0.5) < 0.05);

        // handle 0 is ignored everywhere
        dw.setMaterialByHandle(0, 1, 1);  dw.setDampingByHandle(0, 1, 1);  dw.setGravityScaleByHandle(0, 1);
        dw.setSleepEnabledByHandle(0, true);  dw.setDensityByHandle(0, 1);  dw.setEnabledByHandle(0, false);
        dw.destroyShapesByHandle(0);  dw.wakeByHandle(0);
        CHECK(!dw.isEnabledByHandle(0) && dw.massByHandle(0) == 0.0f && dw.shapeCountByHandle(0) == 0);

        // the world's sleep switch wins: with sleep off in the world a body never sleeps
        Box3DWorld nw;
        WorldSettings noSleep;  noSleep.enableSleep = false;
        CHECK(nw.create(noSleep));
        const BodyHandle nb = nw.createBody(dyn, p0, qid);
        CHECK(nb && nw.createShape(nb, dyn, cube));
        nw.setSleepEnabledByHandle(nb, true);
        CHECK(!nw.isSleepEnabledByHandle(nb));
        std::printf("mid-sim update checks done\n");
    }

    w.destroy();
    CHECK(!w.isValid());
    std::printf(fails ? "FAILED\n" : "OK\n");
    return fails ? 1 : 0;
}
