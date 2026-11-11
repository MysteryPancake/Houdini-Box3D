// Exercises the real Box3D calls in the wrapper: a box falls onto a static
// ground box, comes to rest, and velocities/transforms round-trip.
#include "Box3DWorld.h"
#include <cmath>
#include <cstdio>

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

    w.removeBody(2);
    CHECK(!w.hasBody(2) && w.bodyCount() == 2);
    CHECK(!w.getTransform(2, p, q));

    w.destroy();
    CHECK(!w.isValid());
    std::printf(fails ? "FAILED\n" : "OK\n");
    return fails ? 1 : 0;
}
