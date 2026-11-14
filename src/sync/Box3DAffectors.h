// Box3DAffectors.h
//
// Affector objects as static colliders. An object we do not solve (for example
// a Static Object DOP) takes part in the simulation only through a collision
// relationship: it is an "affector" of the objects we do solve
// (SIM_Object::getConstAffectors). The solver calls syncAffectorBodies() once
// per step, after the point bodies are synced. Each affector's polygons become
// one static Box3D triangle mesh, posed by the object's Houdini transform
// (SIMgetGeometryTransform); an affector with a height field (a flat float
// volume named "height") becomes a static Box3D height field instead.
#pragma once

#include <cstdint>

class SIM_ObjectArray;
struct Box3DSharedState;

struct Box3DAffectorCounts {
  int total = 0;   // affector objects seen
  int added = 0;   // bodies built (first time, or geometry / material changed)
  int moved = 0;   // poses updated (animated Static Object)
  int removed = 0; // bodies destroyed (no longer an affector)
  int failed = 0; // could not be built (no polygons, bad points, Box3D refused)
  int noGeo = 0;  // no Geometry data: skipped
  int baked =
      0; // built with a scaled / sheared transform baked into the vertices
  int heightFields = 0; // of `added`: built from a height field
};

// `objects` are the objects being solved (their affectors are gathered here).
// Objects that are themselves in `objects` are never affectors: they are
// simulated from their points. Called with Box3DSharedState::mutex held.
void syncAffectorBodies(Box3DSharedState &st, SIM_ObjectArray &objects,
                        Box3DAffectorCounts &counts);
