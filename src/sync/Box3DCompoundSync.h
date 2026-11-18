// Box3DCompoundSync.h
//
// Glued groups run as one Box3D body (core/Box3DCompound.h). The solver calls
// syncCompounds() each step after syncJoints(), and expandCompoundMoves()
// after the step. Both run with Box3DSharedState::mutex held.
//
// A group of pieces joined by Glue bonds becomes a compound when every piece is
// a plain dynamic body with the same body-level settings, the default inertia
// and no other joint. Each piece keeps its own Box3D body, disabled (Box3D
// parks a disabled body's joints, so the welds sleep with it), and the
// compound body carries copies of the pieces' shapes. The per-point sync,
// mirror entries and joints stay as they are. A group that changes is
// dissolved: its pieces are enabled where the compound left them.
#pragma once

#include "core/Box3DWorld.h"

struct Box3DBodyMirror;
struct Box3DSharedState;

// Merges the groups that qualify and dissolves the ones that changed (a bond
// broke, a piece was removed, rebuilt or edited). With the solver's switch
// (Box3DSharedState::glueCompounds) off, dissolves every compound.
void syncCompounds(Box3DSharedState &st);

// After the step, before the write-back: every moved compound adds a move
// event for each of its pieces (pose = the compound's pose * the piece's
// offset), with the piece's velocity kept on its mirror entry (glueLin /
// glueAng), and the parked bodies are put where the pieces are.
void expandCompoundMoves(Box3DSharedState &st);

// Before every Box3D step: each compound's motion, which the pieces that leave
// it take (they were not slowed by the hit that broke it). It is kept while the
// compound's last step had hits, and taken again at the start of a frame.
void recordCompoundMotion(Box3DSharedState &st, bool frameStart);

// After bonds broke in the middle of a frame (breakGlueEarly): the compounds
// that lost pieces shrink or dissolve at once. Each piece that left gets a move
// event in `moves`.
void splitBrokenCompounds(Box3DSharedState &st,
                          std::vector<b3dop::BodyMove> &moves);

// The glue's hits on a piece of a compound: the impulse on its shapes in the
// last Box3D step. False if `body` is not a piece of a compound.
bool compoundGlueHits(Box3DSharedState &st, b3dop::BodyHandle body,
                      float &hits);

// What a force evaluation reads of a piece of a compound (awake, mass, its
// centre, its velocity from the compound). False if the entry is not one.
bool compoundPieceState(Box3DSharedState &st, const Box3DBodyMirror &piece,
                        b3dop::BodyState &out);

// Whether a piece's compound is awake (a parked piece's own body never is).
bool compoundPieceAwake(Box3DSharedState &st, const Box3DBodyMirror &piece);

// A piece's force and torque as the force and torque on its compound (about
// its centre of mass). `target` becomes the compound's body. False if the
// entry is not a piece of a compound (nothing changed).
bool compoundPieceForce(Box3DSharedState &st, const Box3DBodyMirror &piece,
                        float force[3], float torque[3],
                        b3dop::BodyHandle &target);
