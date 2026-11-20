// Box3DJointSync.h
//
// Constraint networks: the geometry of a Constraint Network DOP's relationship
// (or an object's ConstraintGeometry, with Attach Internal Constraints to
// Object). Each two-point primitive is one Box3D joint:
//   prim  constraint_name   the Data Name of a Box3D constraint relationship
//   wired into the
//                           Constraint Network: it gives the joint type and
//                           settings. SideFX's Glue / Hard / Spring / Bullet
//                           Soft / Cone Twist data are mapped onto Box3D joints
//                           (b3dop::mapSideFX)
//   prim  constraint_type   position / rotation / all (missing: position),
//                           SideFX data only
//   prim  <setting>         modifies that node's value (b3dop::jointParams
//   names, or
//                           b3dop::sideFXParams for SideFX data): floats
//                           multiply, toggles / vectors / Euler angles
//                           (degrees) replace
//   prim  next_constraint_name   switched to when the joint breaks
//   point name              the body: a piece's `name` in any solved object,
//   "object/piece", an
//                           object with one body, or an affector object; empty:
//                           the world
//   point P, orient         that body's joint frame in world space when the
//   joint is made
// The solver calls syncJoints() once per step after the bodies are synced, and
// jointsAfterStep() after the step (breaking, feedback). Both run with
// Box3DSharedState::mutex held.
#pragma once

#include "core/Box3DGlue.h"
#include "core/Box3DWorld.h"

#include <vector>

class SIM_ObjectArray;
struct Box3DSharedState;

struct Box3DJointCounts {
  int networks = 0; // networks seen
  int created = 0;  // joints made (new, or remade after a change)
  int updated = 0;  // live setting changes
  int removed = 0;  // joints whose primitive (or network) went away
  // primitives whose constraint_name names no Box3D or supported SideFX
  // constraint relationship
  int unknown = 0;
  int unresolved = 0; // an anchor name that matches no body
  // a constraint_type of position or rotation on one of our own joint nodes
  // (it is read for SideFX data only)
  int typeIgnored = 0;
  int badAnchorId = 0;  // an anchor_id that is no point / vertex of its object
  int agentAnchors = 0; // anchor_type agent (not simulated here)
  int anchorNoGeometry = 0; // anchor_id on a body whose geometry was not read
  int notLines = 0;         // primitives that are not two-point lines
  int failed =
      0; // Box3D refused (both ends the world, the same body twice, ...)
};

// True if any object has a constraint network (cheap: no geometry is read). The
// sync keeps per-object name lookups only while this is true.
bool anyJointNetwork(SIM_ObjectArray &objects);

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects,
                Box3DJointCounts &counts);

// SideFX Glue (core/Box3DGlue.h): syncJoints lists the live bonds and their
// bodies each step; collectGlueHits after every Box3D step adds the impulse of
// the glued bodies' contacts; advanceGlue (dt: the whole frame's time) at the
// end of the frame spreads those over the bonds, adds them to each `impact` and
// queues the bonds over their strength for jointsAfterStep.
void collectGlueHits(Box3DSharedState &st);
// The glue group of each node (st.glueGroup), worked out again if the bonds
// changed since: pieces of one group are what contacts inside it (weight) are
// told from real hits by.
void refreshGlueGroups(Box3DSharedState &st);
// The live bonds as glueAdvance / glueComponents take them, in the order of
// `glueBonds`. `withParams` adds each bond's strength settings and impact;
// `blankBroken` turns a bond broken mid-frame into one that joins nothing.
std::vector<b3dop::GlueBond> glueBondList(const Box3DSharedState &st,
                                          bool withParams, bool blankBroken);
// After collectGlueHits, for glued groups run as compounds (so one body takes
// the whole hit): bonds already over their strength on the hits so far break
// now and their compounds split (`elapsed`: seconds of the frame done). The
// pieces that left get a move event in `moves`.
void breakGlueEarly(Box3DSharedState &st, float elapsed,
                    std::vector<b3dop::BodyMove> &moves);
void advanceGlue(Box3DSharedState &st, float dt);

// After the step: joints over their force / torque threshold are destroyed and
// marked broken (`broken` primitive group, or switched to their
// next_constraint_name), and the live frames and feedback attributes are
// written onto the networks. Returns the number of joints that broke.
int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects);
