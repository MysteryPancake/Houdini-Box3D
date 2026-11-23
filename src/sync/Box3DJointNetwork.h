// Box3DJointNetwork.h
//
// What Box3DJointSync.cpp (reads the networks, makes the joints) and
// Box3DJointBreak.cpp (after the step) share: the network's attribute names and
// the helpers that find and fingerprint a network's geometry.
#pragma once

#include <cstdint>
#include <string>

#include "core/Box3DJoints.h"

class GU_Detail;
class SIM_Geometry;
class SIM_GeometryCopy;
class SIM_Object;

// The Constraint Network DOP's attribute and group names.
inline constexpr const char *kConstraintName = "constraint_name";
inline constexpr const char *kNextConstraintName = "next_constraint_name";
inline constexpr const char *kBrokenGroup = "broken";
// Ours: the length a distance joint made with a negative Rest Length started
// at.
inline constexpr const char *kAutoRestLength = "auto_restlength";
// Where each live joint's anchors are after the last step (point attributes).
// P / orient stay the authored anchors: Houdini reads them as the anchors'
// initial world frames and moves its constraint guide with the pieces itself,
// so writing the live frames there drew every moving anchor twice as far.
inline constexpr const char *kCurrentP = "current_P";
inline constexpr const char *kCurrentOrient = "current_orient";
// Glue's feedback: the impulse its hits have accumulated
// (docs/agents/joints.md).
inline constexpr const char *kImpact = "impact";
// A Bond's feedback: the impulse its load has delivered beyond its capacity so
// far (core/Box3DBond.h), written so a rebuilt world carries on from it.
inline constexpr const char *kBondImpulse = "bond_impulse";
// Detail attribute of a Constraint Network: Glue rounds for a bond with a
// negative count.
inline constexpr const char *kPropagateIteration = "propagate_iteration";

// Where a network was found: a relationship of `object` (by name), or, with an
// empty name, the object's ConstraintGeometry.
struct NetworkSource {
  bool inGroup = true;
  std::string relName;
};

const SIM_Geometry *networkGeometry(const SIM_Object &obj,
                                    const NetworkSource &src);
SIM_GeometryCopy *writableNetworkGeometry(SIM_Object &obj,
                                          const NetworkSource &src);
// Everything the joints are read from: equal fingerprints mean nothing to
// re-read.
uint64_t networkFingerprint(const GU_Detail *gdp);
// A joint frame and its anchor frame (identity offsets pass through bit for
// bit), and a frame as the geometry stores it.
b3dop::JointFrame anchorFrameOf(const b3dop::JointFrame &joint,
                                const b3dop::JointFrame &adj);
b3dop::JointFrame asStored(const b3dop::JointFrame &f);
