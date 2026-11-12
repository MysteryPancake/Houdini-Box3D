// Box3DJointSync.h
//
// Joint networks: an object's geometry data named "Joints" (or the Box3D Joints node's Geometry
// Data), usually attached with a SOP Geometry DOP. Each two-point primitive is one Box3D joint:
//   prim  joint_type   weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter
//   prim  <setting>    overrides the Box3D Joints default (b3dop::jointParams names; angles in degrees)
//   point name         the body: a piece's `name` in any solved object, "object/piece", an object
//                      with one body, or an affector object; empty: the world
//   point P, orient    that body's joint frame in world space when the joint is made
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
    int unknown    = 0;     // primitives with a missing or unknown joint_type
    int unresolved = 0;     // an anchor name that matches no body
    int notLines   = 0;     // primitives that are not two-point lines
    int failed     = 0;     // Box3D refused (both ends the world, the same body twice, ...)
    int shared     = 0;     // networks on several objects: read once, no write-back
};

// True if any object carries joint network geometry (cheap: no geometry is read). The sync keeps
// per-object name lookups only while this is true.
bool anyJointNetwork(SIM_ObjectArray &objects);

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects, Box3DJointCounts &counts);

// After the step: joints over their force / torque threshold are destroyed and marked broken
// (`broken` primitive group), and feedback attributes are written onto the networks that have one
// carrier object. Returns the number of joints that broke.
int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects);
