// Box3DCompoundInternal.h
//
// What the Box3DCompound*.cpp files share (the sync, the splits, the holds and
// bridges, the per-step queries): helpers that used to be local to one file.
// Only those files include it.
#pragma once

#include "Box3DCompoundSync.h"
#include "Box3DState.h"
#include "core/Box3DCompound.h"

#include <unordered_map>
#include <vector>

namespace compoundsync {

using b3dop::BodyHandle;
using Pose = b3dop::BodyPose;

bool bodyPose(const b3dop::Box3DWorld &w, BodyHandle h, Pose &out);

// Where a piece is when its compound body is at `body`.
Pose placed(const Pose &body, const Box3DCompoundMember &m);

// The centre of mass of a piece placed at `pp`.
void centreOf(const Pose &pp, const Box3DCompoundMember &m, double out[3]);

// A body's centre of mass and velocity, as the rigid body pieceVelocity needs.
bool bodyMotion(const b3dop::Box3DWorld &w, BodyHandle h, b3dop::MergedBody &mb,
                bool &awake);

// The settings that belong to a body, not a shape: pieces that differ here
// cannot share one.
bool sameBodyLevel(const b3dop::BodyProps &a, const b3dop::BodyProps &b);

// Whether the piece of a glue node is held out of its compound
// (holdCompoundsForApproach): held pieces are not part of any group, they and
// their bonds are left out of the grouping.
bool heldNode(const Box3DSharedState &st, int node);

// A piece the compound can carry: a plain dynamic body whose mass and inertia
// come from its shapes.
bool eligible(const Box3DBodyMirror &e);

uint64_t keyOf(const Box3DBodyMirror &e);

// A move event for a piece of `entry` put at `pp`.
b3dop::BodyMove moveAt(Box3DBodyMirror &entry, BodyHandle body, const Pose &pp,
                       bool asleep, const float lin[3] = nullptr,
                       const float ang[3] = nullptr);

// Destroys the bridges at the compound's pieces in `members` (null: all of
// them) and forgets them. A bridge to a piece that leaves, or all of them when
// the compound goes.
void dropBridges(Box3DSharedState &st, Box3DCompound &c,
                 const std::unordered_set<BodyHandle> *members = nullptr);

// Enables the pieces where the compound left them and destroys the compound
// body. Pieces whose entry is gone (`entry` null) were destroyed with their
// points. `moves` (optional): gets a move event for each piece, for a release
// in the middle of a frame.
void dissolve(Box3DSharedState &st, Box3DCompound &c,
              std::vector<b3dop::BodyMove> *moves = nullptr);

// Points each piece at its entry now (null: its point, and body, are gone).
// True if some piece is gone, was edited, or differs from what was merged.
bool resolveMembers(Box3DSharedState &st, Box3DCompound &c);

// Which Glue bonds join two pieces of one compound. `nodes`: only the bonds of
// those glue nodes (the components a change was in); null: all.
void markCompoundBonds(Box3DSharedState &st,
                       const std::vector<int> *nodes = nullptr);

// Takes the compound out of the list (its body and pieces are dissolve's).
void eraseCompound(Box3DSharedState &st, const Box3DCompound *c);

// Bridges whose bond's joint is gone (the bond broke, its network went) are
// destroyed and forgotten.
void pruneBridges(Box3DSharedState &st);

// A bridge for each Glue bond between a piece of the compound and a piece that
// is not in a compound (held out). All the compound's pieces are looked at:
// for after the groups were worked out again.
void bridgeBoundaries(Box3DSharedState &st, Box3DCompound &c);

// Takes the pieces whose component is not `mainId` out of the compound and
// keeps the body (and the contacts of the pieces that stay): they are enabled
// where the compound has them, their shapes destroyed, and the body keeps the
// momentum of the pieces that stay, which still move as one rigid body. False,
// with nothing changed, if the compound's body cannot be read.
// `leave` and `moves` as for dissolve: the pieces that leave take `leave`'s
// motion, the ones that stay keep the compound's now.
bool shrink(Box3DSharedState &st, Box3DCompound &c, const std::vector<int> &cid,
            int mainId, std::vector<b3dop::BodyMove> *moves = nullptr);

// One compound from `group` (sorted by key, the first is the leader, the
// compound's origin). False, with nothing changed, if Box3D refuses a shape.
bool merge(Box3DSharedState &st, const std::vector<Box3DBodyMirror *> &group);

// Bonds broke inside compounds (st.glueTouched: their nodes). For each compound
// that loses pieces, the pieces leave it (shrink) and groups of two or more
// become compounds of their own, with work in proportion to the pieces that
// left and not to the compounds. What it cannot do that way is left in
// glueTouched for the full regrouping.
void splitOffSmallPieces(Box3DSharedState &st);

} // namespace compoundsync
