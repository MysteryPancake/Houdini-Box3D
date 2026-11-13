// An RBD Object's own settings (RBD State, PhysicalParms, Bullet Data) as a whole-object body's
// settings when it has no Box3D Data: rbdObjectSetup().
#include "core/Box3DRbdObject.h"
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)

using namespace b3dop;

namespace {

bool
near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol;
}

// Houdini 22.0's RBD Object defaults, as the probe read them.
RbdObjectSettings
houdiniDefaults()
{
    RbdObjectSettings r;
    r.hasComputeMass = r.hasDensity = r.hasMass = r.hasInertia = true;
    r.hasFriction = r.hasBounce = true;
    r.hasGeorep = r.hasAutofit = r.hasGroupConnected = true;
    return r;
}

} // namespace

int
main()
{
    // ---- nothing read: Box3D Data's defaults ----
    {
        const WholeObjectSetup s = rbdObjectSetup(RbdObjectSettings());
        const BodySettings d;
        CHECK(s.node.computeMass == d.computeMass && s.node.mass == d.mass && s.node.density == d.density);
        CHECK(s.node.friction == d.friction && s.node.restitution == d.restitution);
        CHECK(s.node.defaultShape == d.defaultShape && s.node.autoFit == d.autoFit);
        CHECK(s.inertiaScale == 1.0f && !s.explicitShape);
    }

    // ---- an RBD Object as Houdini makes it ----
    {
        const WholeObjectSetup s = rbdObjectSetup(houdiniDefaults());
        CHECK(s.node.computeMass && s.node.density == 1000.0f && s.node.mass == 1.0f);
        CHECK(s.node.friction == 1.0f && s.node.restitution == 0.5f);
        CHECK(s.node.defaultShape == ShapeType::Convex && s.node.autoFit && !s.node.hullPerConnectedSet);
        CHECK(!s.explicitShape);
    }

    // ---- values Box3D cannot use are ignored ----
    {
        RbdObjectSettings r = houdiniDefaults();
        r.density = -5.0f;  r.mass = 0.0f;  r.inertiaStiffness = NAN;
        r.friction = -1.0f; r.bounce = INFINITY;
        const WholeObjectSetup s = rbdObjectSetup(r);
        const BodySettings d;
        CHECK(s.node.density == d.density && s.node.mass == d.mass && s.inertiaScale == 1.0f);
        CHECK(s.node.friction == d.friction && s.node.restitution == d.restitution);
        r = houdiniDefaults();
        r.inertiaStiffness = 2.5f;  r.computeMass = false;  r.mass = 40.0f;
        const WholeObjectSetup t = rbdObjectSetup(r);
        CHECK(t.inertiaScale == 2.5f && !t.node.computeMass && t.node.mass == 40.0f);
    }

    // ---- shapes from Bullet Data ----
    {
        RbdObjectSettings r = houdiniDefaults();
        r.georep = "cylinder";                      // no Box3D shape: the default (box)
        CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::Box);
        r.georep = "none";
        CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::None);
        r.georep = "compound";
        CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::Concave);
        r.georep = "convexhull";  r.groupConnected = true;
        CHECK(rbdObjectSetup(r).node.hullPerConnectedSet);
        r.autofit = false;                          // convex ignores AutoFit
        CHECK(!rbdObjectSetup(r).explicitShape);

        // AutoFit off: the hand-set box / sphere / capsule.
        r = houdiniDefaults();
        r.georep = "box";  r.autofit = false;
        r.primS[0] = 0.5f;  r.primS[1] = 1.0f;  r.primS[2] = 2.0f;
        r.primT[1] = 0.25f;
        r.primRot[0] = 0.0f;  r.primRot[1] = 0.0f;  r.primRot[2] = 0.7071068f;  r.primRot[3] = 0.7071068f;
        WholeObjectSetup s = rbdObjectSetup(r);
        CHECK(s.explicitShape && s.spec.type == ShapeType::Box);
        CHECK(s.spec.halfExtents[0] == 0.5f && s.spec.halfExtents[1] == 1.0f && s.spec.halfExtents[2] == 2.0f);
        CHECK(s.spec.center[1] == 0.25f && near(s.spec.rot[2], 0.7071068, 1e-6));
        r.georep = "sphere";  r.radius = 0.3f;
        s = rbdObjectSetup(r);
        CHECK(s.explicitShape && s.spec.type == ShapeType::Sphere && s.spec.radius == 0.3f);
        r.georep = "capsule";  r.radius = 0.2f;  r.length = 1.5f;
        s = rbdObjectSetup(r);
        CHECK(s.explicitShape && s.spec.type == ShapeType::Capsule && s.spec.radius == 0.2f && s.spec.halfLength == 0.75f);
        r.primRot[0] = r.primRot[1] = r.primRot[2] = r.primRot[3] = 0.0f;     // unusable rotation: identity
        s = rbdObjectSetup(r);
        CHECK(s.spec.rot[3] == 1.0f);
    }

    // ---- in a world: density 1000 on a 1 m box is 1000 kg, as in Houdini ----
    {
        Box3DWorld w;
        CHECK(w.create(WorldSettings()));
        RbdObjectSettings r = houdiniDefaults();
        r.georep = "box";  r.autofit = false;
        r.primS[0] = r.primS[1] = r.primS[2] = 0.5f;
        const WholeObjectSetup s = rbdObjectSetup(r);
        BodySettings b = s.node;
        b.density = s.node.density;               // computemass: density as is
        const double p[3] = { 0, 5, 0 };
        const float  q[4] = { 0, 0, 0, 1 };
        const BodyHandle h = w.createBody(b, p, q);
        CHECK(h != 0 && w.createShape(h, b, s.spec));
        float mass = 0.0f, c[3], inertia[3];
        CHECK(w.getMassDataByHandle(h, mass, c, inertia) && near(mass, 1000.0, 1e-2));
        std::printf("1 m box at density 1000: %.2f kg\n", mass);
    }

    if (fails)
    {
        std::printf("%d check(s) failed\n", fails);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
