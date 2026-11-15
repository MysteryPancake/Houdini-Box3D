// An RBD Object's own settings (RBD State, PhysicalParms, Bullet Data) as a
// whole-object body's settings when it has no Box3D Data: rbdObjectSetup().
#include "TestUtil.h"
#include "core/Box3DRbdObject.h"
#include "core/Box3DWorld.h"

#include <cmath>
#include <cstdio>

using namespace b3dop;

namespace {

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Houdini 22.0's RBD Object defaults, as the probe read them.
RbdObjectSettings houdiniDefaults() {
  RbdObjectSettings r;
  r.hasComputeMass = r.hasDensity = r.hasMass = r.hasInertia = true;
  r.hasFriction = r.hasBounce = true;
  r.hasGeorep = r.hasAutofit = r.hasGroupConnected = true;
  return r;
}

} // namespace

int main() {
  // ---- nothing read: Box3D Data's defaults ----
  {
    const WholeObjectSetup s = rbdObjectSetup(RbdObjectSettings());
    const BodySettings d = nodeBodyDefaults();
    CHECK(s.node.props.computeMass == d.props.computeMass &&
          s.node.props.mass == d.props.mass &&
          s.node.props.density == d.props.density);
    CHECK(s.node.props.friction == d.props.friction &&
          s.node.props.restitution == d.props.restitution);
    CHECK(s.node.defaultShape == d.defaultShape && s.node.autoFit == d.autoFit);
    CHECK(s.node.props.inertiaScale == 1.0f && !s.explicitShape);
  }

  // ---- an RBD Object as Houdini makes it ----
  {
    const WholeObjectSetup s = rbdObjectSetup(houdiniDefaults());
    CHECK(s.node.props.computeMass && s.node.props.density == 1000.0f &&
          s.node.props.mass == 1.0f);
    CHECK(s.node.props.friction == 1.0f && s.node.props.restitution == 0.5f);
    CHECK(s.node.defaultShape == ShapeType::Convex && s.node.autoFit &&
          !s.node.hullPerConnectedSet);
    CHECK(!s.explicitShape);
  }

  // ---- values Box3D cannot use are ignored ----
  {
    RbdObjectSettings r = houdiniDefaults();
    r.density = -5.0f;
    r.mass = 0.0f;
    r.inertiaStiffness = NAN;
    r.friction = -1.0f;
    r.bounce = INFINITY;
    const WholeObjectSetup s = rbdObjectSetup(r);
    const BodySettings d = nodeBodyDefaults();
    CHECK(s.node.props.density == d.props.density &&
          s.node.props.mass == d.props.mass &&
          s.node.props.inertiaScale == 1.0f);
    CHECK(s.node.props.friction == d.props.friction &&
          s.node.props.restitution == d.props.restitution);
    r = houdiniDefaults();
    r.inertiaStiffness = 2.5f;
    r.computeMass = false;
    r.mass = 40.0f;
    const WholeObjectSetup t = rbdObjectSetup(r);
    CHECK(t.node.props.inertiaScale == 2.5f && !t.node.props.computeMass &&
          t.node.props.mass == 40.0f);
  }

  // ---- shapes from Bullet Data ----
  {
    RbdObjectSettings r = houdiniDefaults();
    r.georep = "cylinder"; // no Box3D shape: the default (convex hull)
    CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::Convex);
    r.georep = "none";
    CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::None);
    r.georep = "compound";
    CHECK(rbdObjectSetup(r).node.defaultShape == ShapeType::Concave);
    r.georep = "convexhull";
    r.groupConnected = true;
    CHECK(rbdObjectSetup(r).node.hullPerConnectedSet);
    r.autofit = false; // convex ignores AutoFit
    CHECK(!rbdObjectSetup(r).explicitShape);

    // AutoFit off: the hand-set box / sphere / capsule.
    r = houdiniDefaults();
    r.georep = "box";
    r.autofit = false;
    r.primS[0] = 0.5f;
    r.primS[1] = 1.0f;
    r.primS[2] = 2.0f;
    r.primT[1] = 0.25f;
    r.primRot[0] = 0.0f;
    r.primRot[1] = 0.0f;
    r.primRot[2] = 0.7071068f;
    r.primRot[3] = 0.7071068f;
    WholeObjectSetup s = rbdObjectSetup(r);
    CHECK(s.explicitShape && s.spec.type == ShapeType::Box);
    CHECK(s.spec.halfExtents[0] == 0.5f && s.spec.halfExtents[1] == 1.0f &&
          s.spec.halfExtents[2] == 2.0f);
    CHECK(s.spec.center[1] == 0.25f && near(s.spec.rot[2], 0.7071068, 1e-6));
    r.georep = "sphere";
    r.radius = 0.3f;
    s = rbdObjectSetup(r);
    CHECK(s.explicitShape && s.spec.type == ShapeType::Sphere &&
          s.spec.radius == 0.3f);
    r.georep = "capsule";
    r.radius = 0.2f;
    r.length = 1.5f;
    s = rbdObjectSetup(r);
    CHECK(s.explicitShape && s.spec.type == ShapeType::Capsule &&
          s.spec.radius == 0.2f && s.spec.halfLength == 0.75f);
    r.primRot[0] = r.primRot[1] = r.primRot[2] = r.primRot[3] =
        0.0f; // unusable rotation: identity
    s = rbdObjectSetup(r);
    CHECK(s.spec.rot[3] == 1.0f);
  }

  // ---- in a world: density 1000 on a 1 m box is 1000 kg, as in Houdini ----
  {
    Box3DWorld w;
    CHECK(w.create(WorldSettings()));
    RbdObjectSettings r = houdiniDefaults();
    r.georep = "box";
    r.autofit = false;
    r.primS[0] = r.primS[1] = r.primS[2] = 0.5f;
    const WholeObjectSetup s = rbdObjectSetup(r);
    BodySettings b = s.node;
    b.props.density = s.node.props.density; // computemass: density as is
    const double p[3] = {0, 5, 0};
    const float q[4] = {0, 0, 0, 1};
    const BodyHandle h = w.createBody(b, p, q);
    CHECK(h != 0 && w.createShape(h, b, s.spec));
    float mass = 0.0f, c[3], inertia[3];
    CHECK(w.getMassDataByHandle(h, mass, c, inertia) &&
          near(mass, 1000.0, 1e-2));
    std::printf("1 m box at density 1000: %.2f kg\n", mass);
  }

  // ---- Create Active Object (SolverParms ActiveValue) ----
  {
    RbdObjectSettings r = houdiniDefaults();
    CHECK(rbdObjectSetup(r).node.type == BodySettings::Type::Dynamic);
    r.hasActive = true;
    r.active = true;
    CHECK(rbdObjectSetup(r).node.type == BodySettings::Type::Dynamic);
    r.active = false;
    CHECK(rbdObjectSetup(r).node.type == BodySettings::Type::Static);
    r.hasActive = false; // option not there: active
    CHECK(rbdObjectSetup(r).node.type == BodySettings::Type::Dynamic);
  }

  // ---- inertia type: bounding box only with Compute Inertial Tensor on ----
  {
    RbdObjectSettings r = houdiniDefaults();
    CHECK(!rbdObjectSetup(r).boundsInertia); // type not read: computed
    r.hasInertiaType = true;
    CHECK(!rbdObjectSetup(r).boundsInertia); // Use Volume
    r.boundsInertiaType = true;
    CHECK(rbdObjectSetup(r).boundsInertia);
    r.hasComputeInertia = true;
    r.computeInertia = true;
    CHECK(rbdObjectSetup(r).boundsInertia);
    r.computeInertia = false; // toggle off: the computed inertia stays
    CHECK(!rbdObjectSetup(r).boundsInertia);
  }

  // ---- boundsInverseInertia ----
  {
    float inv[6];
    const float ext[3] = {1.0f, 2.0f, 3.0f};
    CHECK(boundsInverseInertia(ext, 12.0f, inv));
    // I = m (b^2 + c^2) / 12: xx 13, yy 10, zz 5
    CHECK(near(inv[0], 1.0 / 13.0, 1e-6) && near(inv[1], 1.0 / 10.0, 1e-6) &&
          near(inv[2], 1.0 / 5.0, 1e-6));
    CHECK(inv[3] == 0.0f && inv[4] == 0.0f && inv[5] == 0.0f);
    CHECK(!boundsInverseInertia(ext, 0.0f, inv));
    const float flat[3] = {1.0f, 0.0f, 3.0f};
    CHECK(!boundsInverseInertia(flat, 12.0f, inv));
    const float bad[3] = {1.0f, NAN, 3.0f};
    CHECK(!boundsInverseInertia(bad, 12.0f, inv));
  }

  // ---- invertSymmetric3 ----
  {
    const float m[6] = {2, 3, 4, 0.5f, 0.25f, -0.5f};
    float inv[6], back[6];
    CHECK(invertSymmetric3(m, inv) && invertSymmetric3(inv, back));
    for (int i = 0; i < 6; ++i)
      CHECK(near(back[i], m[i], 1e-5));
    const float bad[6] = {1, 1, 1, 1, 0, 0}; // singular
    CHECK(!invertSymmetric3(bad, inv));
    const float neg[6] = {-1, 1, 1, 0, 0, 0};
    CHECK(!invertSymmetric3(neg, inv));
  }

  // ---- a custom inertia in a world ----
  {
    Box3DWorld w;
    CHECK(w.create(WorldSettings()));
    RbdObjectSettings r = houdiniDefaults();
    r.georep = "box";
    r.autofit = false;
    r.primS[0] = r.primS[1] = r.primS[2] = 0.5f;
    const WholeObjectSetup s = rbdObjectSetup(r);
    const double p[3] = {0, 5, 0};
    const float q[4] = {0, 0, 0, 1};
    auto makeBody = [&](float inertiaScale, const float *inverse) {
      BodySettings b = s.node;
      const BodyHandle h = w.createBody(b, p, q);
      CHECK(h != 0 && w.createShape(h, b, s.spec, false));
      CHECK(w.applyMassByHandle(h, b.props.density, true, inertiaScale, true,
                                inverse));
      return h;
    };
    float mass = 0.0f, c[3], I[3], off[3];

    // computed: a 1000 kg cube is 1000 / 6 on every axis
    BodyHandle h = makeBody(1.0f, nullptr);
    CHECK(w.getMassDataByHandle(h, mass, c, I, off) && near(mass, 1000.0, 1e-2));
    CHECK(near(I[0], 1000.0 / 6.0, 0.05) && near(I[1], 1000.0 / 6.0, 0.05) &&
          near(I[2], 1000.0 / 6.0, 0.05));

    // custom diagonal: inverse 1/10, 1/20, 1/30 is inertia 10, 20, 30; the
    // mass stays computed
    const float diag[6] = {0.1f, 0.05f, 1.0f / 30.0f, 0, 0, 0};
    h = makeBody(1.0f, diag);
    CHECK(w.getMassDataByHandle(h, mass, c, I, off));
    CHECK(near(mass, 1000.0, 1e-2));
    CHECK(near(I[0], 10.0, 1e-3) && near(I[1], 20.0, 1e-3) &&
          near(I[2], 30.0, 1e-3));
    CHECK(near(off[0], 0.0, 1e-4) && near(off[1], 0.0, 1e-4) &&
          near(off[2], 0.0, 1e-4));
    CHECK(near(c[0], 0.0, 1e-6) && near(c[1], 0.0, 1e-6) &&
          near(c[2], 0.0, 1e-6));

    // inertialtensorstiffness scales it
    h = makeBody(2.0f, diag);
    CHECK(w.getMassDataByHandle(h, mass, c, I, off) && near(I[0], 20.0, 1e-3) &&
          near(I[1], 40.0, 1e-3) && near(I[2], 60.0, 1e-3));

    // off-diagonal terms: the inverse of the inverse comes back
    const float full[6] = {0.1f, 0.1f, 0.2f, 0.02f, 0.01f, -0.03f};
    float expect[6];
    CHECK(invertSymmetric3(full, expect));
    h = makeBody(1.0f, full);
    CHECK(w.getMassDataByHandle(h, mass, c, I, off));
    CHECK(near(I[0], expect[0], 1e-3) && near(I[1], expect[1], 1e-3) &&
          near(I[2], expect[2], 1e-3));
    CHECK(near(off[0], expect[3], 1e-3) && near(off[1], expect[4], 1e-3) &&
          near(off[2], expect[5], 1e-3));

    // unusable: false, and the computed inertia stays
    const float zero[6] = {0, 0, 0, 0, 0, 0};
    BodySettings b = s.node;
    h = w.createBody(b, p, q);
    CHECK(h != 0 && w.createShape(h, b, s.spec, false));
    CHECK(!w.applyMassByHandle(h, b.props.density, true, 1.0f, true, zero));
    CHECK(w.getMassDataByHandle(h, mass, c, I, off) &&
          near(I[0], 1000.0 / 6.0, 0.05));
    std::printf("custom inertia: %.2f %.2f %.2f\n", 10.0, 20.0, 30.0);
  }

  // ---- custom inertia moves the centre of mass to the origin ----
  {
    Box3DWorld w;
    CHECK(w.create(WorldSettings()));
    BodySettings b = nodeBodyDefaults();
    ShapeSpec spec;
    const float one[3] = {1.0f, 1.0f, 1.0f}; // a 1 m box
    spec = makeShapeSpec(ShapeType::Box, 1.0f, one);
    spec.center[0] = 2.0f; // the shape is off the body origin
    const double p[3] = {0, 5, 0};
    const float q[4] = {0, 0, 0, 1};
    const BodyHandle h = w.createBody(b, p, q);
    CHECK(h != 0 && w.createShape(h, b, spec, false));
    const float diag[6] = {0.1f, 0.1f, 0.1f, 0, 0, 0};
    CHECK(w.applyMassByHandle(h, b.props.density, true, 1.0f, true, diag));
    float mass = 0.0f, c[3], I[3];
    CHECK(w.getMassDataByHandle(h, mass, c, I) && near(c[0], 0.0, 1e-6) &&
          near(I[0], 10.0, 1e-3) && near(mass, 1000.0, 1e-2));
  }

  if (fails) {
    std::printf("%d check(s) failed\n", fails);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
