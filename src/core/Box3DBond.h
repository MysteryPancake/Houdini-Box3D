// Box3DBond.h
//
// Houdini-free model of a Bond: a weld that breaks when the stress it carries
// passes the material's strength, in real units. The load is the joint's
// constraint force and torque, split in the bond's own frame (the normal is the
// direction from piece A to piece B), turned into stresses over the contact
// area, and combined into one ratio. Past 1 the load beyond the bond's capacity
// pushes its two sides apart, and the bond is torn when the energy that gives
// them reaches its fracture energy (bondTearImpulse): a bond with none goes at
// its strength.
//
// Stresses use the contact patch's section moduli (Box3DPatch.h), or without
// one a square patch of side sqrt(area), whose modulus is area * sqrt(area)
// / 6.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace b3dop {

// A material's strengths in Pa, as typical values (the order of magnitude of
// the kind of joint, not a datasheet). A bending strength of 0 is the tensile
// one. Index 0 of the node's menu is Custom (the parameters' own values), so a
// material's menu item is its index here plus 1.
struct BondMaterial {
  const char *label;
  float tensile, shear, bending, compressive;
  float fracture; // J/m^2 to tear it apart
};

inline const std::vector<BondMaterial> &bondMaterials() {
  static const std::vector<BondMaterial> kMaterials = {
      {"Gypsum Plaster", 0.8e6f, 1.2e6f, 0.0f, 8.0e6f, 10.0f},
      {"Brick Mortar", 0.2e6f, 0.4e6f, 0.0f, 5.0e6f, 20.0f},
      {"Concrete", 3.0e6f, 5.0e6f, 0.0f, 30.0e6f, 100.0f},
      {"Wood Glue", 8.0e6f, 10.0e6f, 0.0f, 30.0e6f, 300.0f},
      {"Epoxy", 30.0e6f, 25.0e6f, 0.0f, 90.0e6f, 1000.0f},
      {"Dried Mud", 0.05e6f, 0.1e6f, 0.0f, 1.0e6f, 5.0f},
      {"Ice", 1.5e6f, 1.0e6f, 0.0f, 10.0e6f, 2.0f},
  };
  return kMaterials;
}

struct BondParams {
  float tensile = 1.0e6f;     // Pa: pull per area
  float shear = 1.0e6f;       // Pa: sliding per area
  float bending = 0.0f;       // Pa: bending and twisting; <= 0: the tensile one
  float compressive = 1.0e7f; // Pa: crushing; <= 0: never
  float area = 0.0f;          // m^2 of the contact patch
  float fracture = 0.0f;      // J/m^2 to tear it; 0: it goes at its strength
  // The patch's section moduli (m^3): bending about its two in-plane axes and
  // twisting about the normal. 0: a square patch of the area, for all.
  float modulusU = 0.0f;
  float modulusV = 0.0f;
  float modulusTwist = 0.0f;
};

// The constraint force and torque in the bond's frame.
struct BondLoad {
  float tension = 0.0f;  // N along the normal, positive pulling A and B apart
  float shear = 0.0f;    // N across it
  float bending = 0.0f;  // N*m about axes across the normal
  float twisting = 0.0f; // N*m about the normal
  // The bending split about the patch's two in-plane axes (0 unless given).
  float bendingU = 0.0f;
  float bendingV = 0.0f;
};

// `force` and `torque` are the joint's constraint force and torque on body B
// (what Box3D reports), `normal` the unit direction from A to B, all in the
// same frame. The joint pulls B back toward A when they are being pulled
// apart, so the force on B along the normal is negative for tension.
inline BondLoad bondLoadOf(const float force[3], const float torque[3],
                           const float normal[3]) {
  BondLoad l;
  const float fn =
      force[0] * normal[0] + force[1] * normal[1] + force[2] * normal[2];
  const float tn =
      torque[0] * normal[0] + torque[1] * normal[1] + torque[2] * normal[2];
  float fs[3], ts[3];
  for (int i = 0; i < 3; ++i) {
    fs[i] = force[i] - fn * normal[i];
    ts[i] = torque[i] - tn * normal[i];
  }
  l.tension = -fn;
  l.shear = std::sqrt(fs[0] * fs[0] + fs[1] * fs[1] + fs[2] * fs[2]);
  l.bending = std::sqrt(ts[0] * ts[0] + ts[1] * ts[1] + ts[2] * ts[2]);
  l.twisting = std::fabs(tn);
  return l;
}

// The same with the bending split about the patch's in-plane axes `u` and `v`
// (unit vectors across the normal).
inline BondLoad bondLoadInPatch(const float force[3], const float torque[3],
                                const float normal[3], const float u[3],
                                const float v[3]) {
  BondLoad l = bondLoadOf(force, torque, normal);
  l.bendingU =
      std::fabs(torque[0] * u[0] + torque[1] * u[1] + torque[2] * u[2]);
  l.bendingV =
      std::fabs(torque[0] * v[0] + torque[1] * v[1] + torque[2] * v[2]);
  return l;
}

// The stress ratio: 1 is the strength. 0 for a bond without an area or load.
// A strength <= 0 never fails by that load.
inline float bondRatio(const BondParams &p, const BondLoad &l) {
  if (!(p.area > 0.0f) || !std::isfinite(p.area))
    return 0.0f;
  const float modulus = p.area * std::sqrt(p.area) / 6.0f;
  const float bendingStrength = p.bending > 0.0f ? p.bending : p.tensile;
  auto term = [](float load, float capacity) {
    return capacity > 0.0f ? (load / capacity) * (load / capacity) : 0.0f;
  };
  float sum = 0.0f;
  sum += term(std::max(l.tension, 0.0f), p.tensile * p.area);
  sum += term(std::max(-l.tension, 0.0f), p.compressive * p.area);
  sum += term(l.shear, p.shear * p.area);
  if (p.modulusU > 0.0f && p.modulusV > 0.0f) {
    sum += term(l.bendingU, bendingStrength * p.modulusU);
    sum += term(l.bendingV, bendingStrength * p.modulusV);
  } else
    sum += term(l.bending, bendingStrength * modulus);
  sum += term(l.twisting,
              p.shear * (p.modulusTwist > 0.0f ? p.modulusTwist : modulus));
  return std::sqrt(sum);
}

// The force the bond holds along its main mode: its tensile (else shear)
// strength over the area.
inline float bondCapacity(const BondParams &p) {
  const float strength = p.tensile > 0.0f ? p.tensile : p.shear;
  return strength > 0.0f && p.area > 0.0f ? strength * p.area : 0.0f;
}

// The reduced mass m1 * m2 / (m1 + m2) of two sides, an infinite one (a fixed
// body) leaving the other; 0 when both are fixed.
inline float bondReducedMass(float a, float b) {
  const bool aFixed = std::isinf(a), bFixed = std::isinf(b);
  if (aFixed && bFixed)
    return 0.0f;
  if (aFixed)
    return b;
  if (bFixed)
    return a;
  return a + b > 0.0f ? a * b / (a + b) : 0.0f;
}

// The impulse (N*s) the load beyond a bond's capacity must deliver to tear it.
// Past its strength the excess load pushes the two sides apart, and the bond is
// torn when the energy that gives them reaches the fracture energy over its
// area: I^2 / (2 m) = G * A, with `reducedMass` that of the two sides the bond
// holds together (bondReducedMass). Independent of the step size. 0 without a
// fracture energy: the bond goes at its strength.
inline float bondTearImpulse(float fracture, float area, float reducedMass) {
  if (!(fracture > 0.0f) || !(area > 0.0f) || !(reducedMass > 0.0f))
    return 0.0f;
  return std::sqrt(2.0f * fracture * area * reducedMass);
}

// One step of `dt` seconds. `impulse` is what the load has delivered beyond the
// capacity so far (kept when the load drops: the damage is not undone); true
// when it has reached `tear` (0 breaks at once at ratio 1).
inline bool bondAdvance(float &impulse, float ratio, float capacity, float dt,
                        float tear) {
  if (!(ratio >= 1.0f))
    return false;
  impulse += (ratio - 1.0f) * capacity * dt;
  return impulse >= tear;
}

// The area of the patch two bodies of bounding box sizes `ea` and `eb` share
// across `normal`: for each, its box's volume over its length along the normal
// (a face of the box when the normal is an axis), the smaller of the two.
inline float bondAreaOf(const float ea[3], const float eb[3],
                        const float normal[3]) {
  auto face = [&](const float e[3]) {
    const float along = std::fabs(normal[0]) * e[0] +
                        std::fabs(normal[1]) * e[1] +
                        std::fabs(normal[2]) * e[2];
    return along > 1e-9f ? e[0] * e[1] * e[2] / along : 0.0f;
  };
  return std::min(face(ea), face(eb));
}

} // namespace b3dop
