// Box3DCompoundState.h
//
// The solver's compound state: glued groups run as one body, their members and
// bridges.
#pragma once

#include "core/Box3DCompound.h"
#include "core/Box3DGlue.h"
#include "core/Box3DMotion.h"
#include "core/Box3DShapes.h"
#include "core/Box3DSideFX.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DAttribNames.h"
#include "sync/Box3DBodyState.h"

#include <GA/GA_Types.h>
#include <cstdint>
#include <vector>

// The point attribute of a glued piece: its compound (Write Compound IDs).
inline constexpr const char* kCompoundIdName = "compound_id";

// One piece of a compound: its own body (disabled while merged) and where it
// sits in the compound body's space. `entry` is refreshed by every
// syncCompounds before use; key / body / props / shapeSig are what the piece
// was merged with, so a change is seen.
struct Box3DCompoundMember
{
	uint64_t key = 0; // makeBodyKey of the entry
	b3dop::BodyHandle body = 0;
	Box3DBodyMirror* entry = nullptr;
	float lp[3] = { 0, 0, 0 };	   // position in the compound body's space
	float lq[4] = { 0, 0, 0, 1 };  // rotation in the compound body's space
	float centre[3] = { 0, 0, 0 }; // the piece's centre of mass, in its own space
	float mass = 0.0f;			   // the piece's own mass
	float radius = 0.0f;		   // half the diagonal of its bounds
	int shapeFirst = 0;			   // its shapes are [shapeFirst, +shapeCount) in
	int shapeCount = 0;			   // the compound's, in creation order
	b3dop::BodyProps props;		   // as merged
	uint64_t shapeSig = 0;		   // as merged
};

// A Glue bond between a piece of a compound and a piece that is not in it (one
// taken out while a fast body comes through, `Box3DSharedState::glueHold`). The
// bond's own weld is parked with the piece that is in the compound, so this
// weld joins the free piece to the compound's body at that piece's place in
// it. `original` is the bond's joint, which wakes by itself when its piece
// leaves the compound.
struct Box3DBridge
{
	b3dop::JointHandle original = 0;
	b3dop::JointHandle handle = 0;
	b3dop::BodyHandle member = 0; // the piece in the compound
};

// A glued group run as one Box3D body (core/Box3DCompound.h).
struct Box3DCompound
{
	b3dop::BodyHandle body = 0;
	std::vector<Box3DCompoundMember> members;
	std::vector<Box3DBridge> bridges;
	// The impulse on each of its shapes in the last Box3D step read for the glue
	// (contactImpulsesByShape), the collectGlueHits call it belongs to, and
	// whether the body was awake and dynamic then (a sleeping one has no hits).
	std::vector<float> shapeImpulse;
	// Where its body is after the last step, read once before the live frames
	// are written (refreshLivePoses).
	b3dop::BodyPose livePose;
	bool livePoseOk = false;
	uint64_t hitsTick = 0;
	bool hitsActive = false;
	// For the inertial load (glueCompoundLoad): its motion before and after the
	// last Box3D step, and its pose after it, read once per step
	// (loadTick: the collectGlueHits call they belong to).
	b3dop::MergedBody stepPre, stepPost;
	bool stepPreOk = false;
	b3dop::BodyPose postPose;
	uint64_t loadTick = 0;
	// Drag and speed limits (refreshCompoundMotion): its pieces' combined, as
	// Bullet's glue cluster takes them, and whether there are any. pieceForce /
	// pieceTorque: the forces its pieces put on it in this Box3D step (about its
	// centre), which its drag works on (applyExternalForces).
	b3dop::MotionParams motion;
	bool hasMotion = false;
	float pieceForce[3] = { 0, 0, 0 };
	float pieceTorque[3] = { 0, 0, 0 };
};
