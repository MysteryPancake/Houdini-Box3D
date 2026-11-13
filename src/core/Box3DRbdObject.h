// Box3DRbdObject.h
//
// An RBD Object's own settings, for a whole-object body without Box3D Data:
// mass from its RBD State (the Position data), friction and bounce from
// PhysicalParms, the collision shape from Geometry/BulletData. The Houdini side
// reads whichever options exist (each has a flag) and rbdObjectSetup() turns
// them into the settings the whole-object sync uses in place of Box3D Data's.
// Houdini-free, covered by tests/core_rbdobject.cpp.
#pragma once

#include <cmath>
#include <string>

#include "Box3DShapes.h"
#include "Box3DWorld.h"

namespace b3dop {

// Option names on the three data (Houdini 22.0 probe). They match the point
// attributes.
struct RbdObjectSettings {
  // RBD State (`Position`, an RBD_State)
  bool hasComputeMass = false;
  bool computeMass = true; // computemass
  bool hasDensity = false;
  float density = 1000.0f; // density
  bool hasMass = false;
  float mass = 1.0f; // mass
  bool hasInertia = false;
  float inertiaStiffness = 1.0f; // inertialtensorstiffness

  // PhysicalParms (SIM_PhysicalParms)
  bool hasFriction = false;
  float friction = 1.0f; // friction
  bool hasBounce = false;
  float bounce = 0.5f; // bounce

  // Geometry/BulletData (SIM_BulletData)
  bool hasGeorep = false;
  std::string georep = "convexhull"; // bullet_georep
  bool hasAutofit = false;
  bool autofit = true; // bullet_autofit
  bool hasGroupConnected = false;
  bool groupConnected = false; // bullet_groupconnected
  // With AutoFit off (each read if present; Bullet Data's defaults otherwise):
  float primS[3] = {1, 1, 1}; // bullet_primS: box half extents, as for points
  float primT[3] = {0, 0,
                    0}; // bullet_primT: offset from the body origin (the pivot)
  float primRot[4] = {
      0, 0, 0,
      1}; // bullet_primR, already a quaternion (Euler XYZ degrees in Houdini)
  float radius = 1.0f; // bullet_radius
  float length = 1.0f; // bullet_length: capsule straight section
};

// What the whole-object sync uses instead of Box3D Data's settings.
struct WholeObjectSetup {
  BodySettings
      node; // Box3D Data's defaults with the RBD Object's values on top
  bool explicitShape =
      false; // AutoFit off on a box / sphere / capsule: `spec`, not fitted
  ShapeSpec spec;
};

inline WholeObjectSetup rbdObjectSetup(const RbdObjectSettings &r) {
  WholeObjectSetup out;
  BodySettings &n = out.node;
  n = nodeBodyDefaults();
  auto isPositive = [](float v) { return v > 0.0f && std::isfinite(v); };
  auto isNonNeg = [](float v) { return v >= 0.0f && std::isfinite(v); };

  BodyProps &p = n.props;
  if (r.hasComputeMass)
    p.computeMass = r.computeMass;
  if (r.hasDensity && isPositive(r.density))
    p.density = r.density;
  if (r.hasMass && isPositive(r.mass))
    p.mass = r.mass;
  if (r.hasInertia && isPositive(r.inertiaStiffness))
    p.inertiaScale = r.inertiaStiffness;

  if (r.hasFriction && isNonNeg(r.friction))
    p.friction = r.friction;
  if (r.hasBounce && isNonNeg(r.bounce))
    p.restitution = r.bounce;

  // Unknown names (cylinder, plane) keep the default shape, as for points.
  ShapeType shape = n.defaultShape;
  if (r.hasGeorep && parseShapeType(r.georep.c_str(), shape))
    n.defaultShape = shape;
  if (r.hasGroupConnected)
    n.hullPerConnectedSet = r.groupConnected;
  if (r.hasAutofit)
    n.autoFit = r.autofit;

  const bool primitive = n.defaultShape == ShapeType::Box ||
                         n.defaultShape == ShapeType::Sphere ||
                         n.defaultShape == ShapeType::Capsule;
  if (primitive && !n.autoFit) {
    const float one[3] = {1, 1, 1};
    out.spec = makeShapeSpec(n.defaultShape, 1.0f, one);
    PrimOverrides ov;
    ov.hasSize = ov.hasPos = ov.hasRadius = ov.hasLength = true;
    for (int i = 0; i < 3; ++i) {
      ov.size[i] = r.primS[i];
      ov.pos[i] = r.primT[i];
    }
    ov.hasRot = true;
    copy4(ov.rot, r.primRot);
    sanitizeQuat(ov.rot);
    ov.radius = r.radius;
    ov.length = r.length;
    applyPrimOverrides(out.spec, ov);
    out.explicitShape = true;
  }
  return out;
}

} // namespace b3dop
