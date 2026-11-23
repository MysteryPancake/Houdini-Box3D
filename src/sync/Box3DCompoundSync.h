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
struct Box3DCompound;
struct Box3DSharedState;

// Merges the groups that qualify and dissolves the ones that changed (a bond
// broke, a piece was removed, rebuilt or edited). With the solver's switch
// (Box3DSharedState::glueCompounds) off, dissolves every compound.
void syncCompounds(Box3DSharedState &st);

// With the solver's Write Bond Attributes on, queues each compound piece's
// `compound_id` (the lowest point id in its compound; -1 once it is out of one)
// as a point write, for the write-back.
void queueCompoundIds(Box3DSharedState &st);

// After the step, before the write-back: every moved compound adds a move
// event for each of its pieces (pose = the compound's pose * the piece's
// offset), with the piece's velocity kept on its mirror entry (glueLin /
// glueAng), and the parked bodies are put where the pieces are.
void expandCompoundMoves(Box3DSharedState &st);

// The glue group (st.glueGroup: the pieces joined by bonds) of a compound's
// pieces, -1 if none of them is a glue node.
int compoundGlueGroup(const Box3DSharedState &st, const Box3DCompound &c);

// Whether `body` is a glue node of glue group `group` or the body of a compound
// of it (st.compoundGroup, made by collectGlueHits).
bool glueSameGroup(const Box3DSharedState &st, b3dop::BodyHandle body,
                   int group);

// Destroys the bridges whose bond's joint is gone (broke mid-frame).
void pruneCompoundBridges(Box3DSharedState &st);

// At the start of a frame, before syncCompounds: advances the clock holds are
// measured on by the frame's time and lets the groups merge again once their
// hold has run out.
void releaseExpiredHolds(Box3DSharedState &st, float frameDt);

// Before a Box3D step: a compound is one rigid body, so a fast heavy body that
// reaches it takes the whole building's mass and rebounds in that step, where
// glued pieces on their weld joints give way. So a body of `seen` (the move
// events of the last step: faster than kGlueHoldMinSpeed, not glued itself,
// able to deliver more than the weakest bond's break impulse) whose motion over
// the next kGlueHoldLeadSeconds (two frames at least) reaches a compound
// (`Box3DWorld::approachingBodies`) takes the pieces in its way out of the
// compound, all the way through it (pathThroughBounds), for kGlueHoldSeconds:
// they are enabled where the compound has them (a move event each in `moves`,
// to go before the step's own) and stay unmerged, as with Glue as One Body off,
// while a body keeps coming near them.
void holdCompoundsForApproach(Box3DSharedState &st,
                              const std::vector<b3dop::BodyMove> &seen,
                              std::vector<b3dop::BodyMove> &moves);

// Before every Box3D step with Glue Inertial Load on: each compound's motion,
// which the load of its pieces is worked out from (`pieceLoad`).
void recordCompoundMotion(Box3DSharedState &st);

// The glue's hits on a piece of a compound: the impulse on its shapes in the
// last Box3D step. False if `body` is not a piece of a compound.
bool compoundGlueHits(Box3DSharedState &st, b3dop::BodyHandle body,
                      float &hits);

// After a Box3D step (breakBonds calls it): the load of each Bond that has no
// joint to read, in `extra->bondForce` / `bondTorque` at `st.hitsTick`. A Bond
// inside an awake compound gets it from the compound's spring network
// (core/Box3DBondNetwork.h) under what each piece needs to move as the body
// does; a Bond held by a bridge gets the bridge's. `dt`: the step.
void solveCompoundBonds(Box3DSharedState &st, float dt);

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
