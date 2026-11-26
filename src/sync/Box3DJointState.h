// Box3DJointState.h
//
// The solver's joint state: one entry per joint made from a constraint network
// primitive, the Glue nodes and kinds, and the networks. Box3DSharedState holds
// them (Box3DState.h).
#pragma once

#include <GA/GA_Types.h>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/Box3DCompound.h"
#include "core/Box3DGlue.h"
#include "core/Box3DShapes.h"
#include "core/Box3DSideFX.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DAttribNames.h"

// The parts of a joint entry that few joints use (Box3DJointEntry::extra).
struct Box3DJointExtra {
  // A point / vertex anchor (anchor_id) on a whole object or packed piece:
  // where that point was the first time this joint saw it, so the body moving
  // is not an edit. pin*Id is the id it was taken for (-1: none), pin*Vertex
  // whether a vertex.
  b3dop::JointFrame pinA, pinB;
  int pinIdA = -1, pinIdB = -1;
  bool pinVertexA = false, pinVertexB = false;
  // SideFX Glue: the bond's options, and its accumulated `impact` (the
  // attribute of the same name), kept from step to step.
  b3dop::GlueParams glueParams;
  float glueImpact = 0.0f;
  // SideFX Spring: the masses its Hz and damping ratio were converted with.
  b3dop::SideFXBody massA, massB;
  // A joint that broke: its local frames at the break, so a next constraint can
  // start from them (switchFrames).
  b3dop::JointFrame brokeLocalA, brokeLocalB;
  // A world end whose authored anchor moves (animated constraint geometry): the
  // shapeless kinematic body the joint holds instead of the world, which end it
  // is, and where the body goes this frame (driveWorldAnchors). 0: none.
  b3dop::BodyHandle anchorBody = 0;
  bool anchorOnA = false;
  b3dop::JointFrame anchorTarget;
};

// One joint made from a primitive of a constraint network (Box3DJointSync.cpp).
struct Box3DJointEntry {
  b3dop::JointHandle handle =
      0; // 0: none (broken, or its anchors did not resolve)
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::BodyHandle bodyA = 0; // 0: the world
  b3dop::BodyHandle bodyB = 0;
  uint64_t sig = 0; // type, bodies, frames: a change recreates the joint
  b3dop::JointSettings settings; // as last given to Box3D
  uint64_t network = 0;          // key of its Box3DJointNetwork
  GA_Offset prim = GA_INVALID_OFFSET;
  uint64_t stamp = 0;  // its network's liveStamp when last seen
  bool broken = false; // broke (force / torque threshold): never remade

  // Live frames: after each step the anchors' current_P / current_orient are
  // set to where the joint's frames are (written networks only). What was
  // written, to tell our writes from edits upstream.
  b3dop::JointFrame wroteA, wroteB;
  bool wrote = false;
  // The authored anchors (P / orient, as stored) the joint was made from: an
  // upstream edit of them remakes the joint.
  b3dop::JointFrame authA, authB;
  bool made = false;
  // Both bodies asleep (or fixed) when the write-back last visited the joint,
  // and then already written: its live frames and feedback are not written
  // again until a body wakes. `quiet` is this step's decision.
  bool asleepWritten = false;
  bool quiet = false;
  // distance made with restlength < 0: written to `auto_restlength` once
  float madeRestLength = -1.0f;
  // made from P / orient: write local_P / local_orient after the step
  bool writeLocal = false;

  // next_constraint_name: the broken joint's local frames, so the anchors can
  // be put where its frames were at the break (switchFrames) and the next
  // constraint made exactly from there.
  // (the frames themselves are in `extra`)
  bool brokeLocalKnown = false;
  bool switchFrames = false;

  // SideFX data only: the joint's frames relative to its anchors
  // (b3dop::sideFXFrameOffsets), so the anchors written back are the anchors,
  // not the frames. Identity for Box3D nodes.
  b3dop::JointFrame adjA, adjB;
  // SideFXJoint::reaim: adjA's position is aimed again every frame
  // (reaimJoints) along the anchors' current line, `reaimRest` long.
  bool reaim = false;
  bool reaimListed = false; // in Box3DSharedState::reaimEntries
  float reaimRest = 0.0f;

  // Each body's rotation at rest (its restxform's, else its rotation when the
  // joint was made; identity for the world), for the `angle` reading: the
  // bodies' relative rotation since rest, as Bullet.
  float restRotA[4] = {0, 0, 0, 1};
  float restRotB[4] = {0, 0, 0, 1};

  // SideFX Glue: breaks on the impulse of the hits its bodies take, not on a
  // load (core/Box3DGlue.h, docs/agents/joints-sidefx.md); its bond and
  // `impact` are in `extra`.
  bool glue = false;
  // Its place in Box3DSharedState::glueBonds, -1 when it is not a live bond.
  int glueIndex = -1;
  // A Glue bond between two pieces of one compound (Box3DCompoundSync.cpp):
  // the weld carries nothing, so its force, torque, distance and angle are
  // written as zero once and only `impact` after that. feedbackZeroed /
  // impactWritten say what was written.
  bool inCompound = false;
  bool feedbackZeroed = false;
  float impactWritten = -1.0f;
  // SideFX Spring: its Hz and damping ratio come from the bodies' masses
  // (Hooke's constant and absolute damping converted, b3dop::mapSideFX), so the
  // masses they were made from are kept (in `extra`) and the joint is remade
  // when they change.
  bool massDependent = false;

  // What only some joints keep, made on first use so the others do not carry
  // it (about a quarter of an entry).
  std::unique_ptr<Box3DJointExtra> extra;
  Box3DJointExtra &ensureExtra() {
    if (!extra)
      extra = std::make_unique<Box3DJointExtra>();
    return *extra;
  }
};

// Whether the joint is an edge of the glue graph (the groups that run as one
// body, the contacts inside a group that are no hits): a SideFX Glue bond.
inline bool inGlueGraph(const Box3DJointEntry &e) { return e.glue; }

// A body in a Glue network this frame and the impulse its contacts put on it
// over the frame's Box3D steps.
struct Box3DGlueNode {
  b3dop::BodyHandle body = 0;
  float hits = 0.0f;
  // The impulse estimate of the frame's contacts (estimateGlueHits): its
  // group's first step that has one (`estimateStep` is the step being worked
  // out).
  float estimate = 0.0f;
  float estimateStep = 0.0f;
  std::vector<b3dop::BodyHandle> partners; // bodies it is glued to
  std::vector<Box3DJointEntry *> bonds;    // the live bonds it is in
};

// The constraint data a `constraint_name` refers to (a Box3D constraint
// relationship node, or SideFX constraint data mapped onto a Box3D joint per
// primitive), as last used.
struct Box3DJointKind {
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::JointSettings settings;
  // Attributes Replace Values on the node: float primitive attributes are the
  // value instead of a factor (b3dop::applyFloatAttribute).
  bool attributesReplace = false;
  bool sideFX =
      false; // SideFX data: type / settings come from mapSideFX per primitive
  b3dop::SideFXType sideFXType = b3dop::SideFXType::Glue;
  b3dop::SideFXSettings sideFXSettings;
};

// A constraint network: the two-point primitives of a Constraint Network DOP's
// relationship (or an object's ConstraintGeometry, with Attach Internal
// Constraints to Object).
struct Box3DJointNetwork {
  uint64_t fingerprint = 0; // geometry identity + data ids of everything read
  // Box3DSharedState::nameVersion when last processed
  uint64_t nameVersion = 0;
  // Some primitive's anchors did not resolve (or resolve by an object's name):
  // bodies coming or going can change what it makes. Else only the removal of
  // a body one of its joints is on, or a rename, can.
  bool needsBodies = false;
  uint64_t liveStamp = 0; // stamp its current entries carry
  uint64_t visited = 0;   // last sync that saw it
  int objectId = -1; // an object it was found on (and is written back through)
  // the relationship's name; empty: the object's ConstraintGeometry
  std::string relName;
  bool inGroup =
      true; // found among the object's group (not affector) relationships
  bool writable = true; // the last write-back reached the geometry
  std::unordered_map<std::string, Box3DJointKind>
      kinds; // by data name, as last used
  bool kindsKnown = false;
};

// What one bond adds to the signature of the bond list (kGlueCountSalt for the
// bond itself): the same whichever way round its two bodies are.
constexpr uint64_t kGlueCountSalt = 0x9E3779B97F4A7C15ull;

inline uint64_t glueBondHash(b3dop::BodyHandle bodyA, b3dop::BodyHandle bodyB) {
  uint64_t a = static_cast<uint64_t>(bodyA), b = static_cast<uint64_t>(bodyB);
  if (a > b)
    std::swap(a, b);
  return b3dop::mixHash(b3dop::mixHash(0x5bd1e995ull, a), b);
}
