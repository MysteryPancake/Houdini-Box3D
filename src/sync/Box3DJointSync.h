// Box3DJointSync.h
//
// Constraint networks: the geometry of a Constraint Network DOP's relationship (or an object's
// ConstraintGeometry, with Attach Internal Constraints to Object). Each two-point primitive is
// one Box3D joint:
//   prim  constraint_name   the Data Name of a Box3D constraint relationship wired into the
//                           Constraint Network: it gives the joint type and settings
//   prim  <setting>         modifies that node's value (b3dop::jointParams names): floats
//                           multiply, toggles / vectors / Euler angles (degrees) replace
//   prim  next_constraint_name   switched to when the joint breaks
//   point name              the body: a piece's `name` in any solved object, "object/piece", an
//                           object with one body, or an affector object; empty: the world
//   point P, orient         that body's joint frame in world space when the joint is made
// The solver calls syncJoints() once per step after the bodies are synced, and jointsAfterStep()
// after the step (breaking, feedback). Both run with Box3DSharedState::mutex held.
#pragma once

class SIM_ObjectArray;
struct Box3DSharedState;

struct Box3DJointCounts
{
    int networks   = 0;     // networks seen
    int created    = 0;     // joints made (new, or remade after a change)
    int updated    = 0;     // live setting changes
    int removed    = 0;     // joints whose primitive (or network) went away
    int unknown    = 0;     // primitives whose constraint_name names no Box3D constraint relationship
    int unresolved = 0;     // an anchor name that matches no body
    int notLines   = 0;     // primitives that are not two-point lines
    int failed     = 0;     // Box3D refused (both ends the world, the same body twice, ...)
};

// True if any object has a constraint network (cheap: no geometry is read). The sync keeps
// per-object name lookups only while this is true.
bool anyJointNetwork(SIM_ObjectArray &objects);

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects, Box3DJointCounts &counts);

// After the step: joints over their force / torque threshold are destroyed and marked broken
// (`broken` primitive group, or switched to their next_constraint_name), and the live frames and
// feedback attributes are written onto the networks. Returns the number of joints that broke.
int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects);
