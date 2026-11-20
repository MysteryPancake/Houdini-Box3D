# Glue as One Body

`gluecompound`: a glued group runs as one Box3D body. The Glue model itself: [joints-sidefx.md](joints-sidefx.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

* **Glue as One Body** (`gluecompound`, on by default; `Box3DCompoundSync.cpp`): after `syncJoints`, a group of bonded
  pieces runs as one Box3D body that carries copies of the pieces' shapes (`copyShapesByHandle`, mass from the
  shapes, velocity from `mergePieces`). Each piece keeps its own body, disabled: Box3D parks a disabled body's joints,
  so the welds sleep and the per-point sync, mirror and joints stay as they are. A group merges only when every
  piece is a plain dynamic body with the default inertia and the same body-level settings (`sameBodyLevel`) and no
  other joint touches it. After the step `expandCompoundMoves` adds a move event per piece (compound pose * offset,
  velocity `pieceVelocity` kept on the entry as `glueLin` / `glueAng`), so the write-back is the usual one, and
  puts each parked body where its piece is. `collectGlueHits` reads a piece's hits from `contactImpulsesByShape` of
  its compound. With `gluecompoundload` above 0 a piece's hits are at least `gluecompoundload * pieceLoad`
  (`core/Box3DCompound.h`): mass times the change of the piece's velocity over the step, from the compound's motion
  before it (`stepPre`, taken by `recordCompoundMotion`) and after, so a body stopped at one point loads the pieces
  away from it too (rotation), and a steady load such as weight loads nothing. The break test is the same. One rigid body takes a whole hit, so after every Box3D step `breakGlueEarly` runs the break test on the hits so far
  (`glueAdvance` on copies) for bonds inside compounds, destroys the ones over (`earlyBroken`, written by `jointsAfterStep`;
  `advanceGlue` skips them) and `splitBrokenCompounds` splits their compounds at once. The pieces that leave take the
  compound's motion from before the hit (`recordCompoundMotion`, `Box3DCompound::pre`, kept while its last step had hits,
  taken again each frame), so a corner landing crumbles; the pieces that stay keep the motion after it. The compound keeps its body for the largest
  group of pieces left (`shrink`: the pieces that leave are enabled where the compound has them, their shapes are
  destroyed with `destroyShapeRangesByHandle`, and the body takes the velocity of the rest, `v + w x (new centre -
  old centre)`, so the contacts of the pieces that stay survive); the pieces that left may form new compounds. A
  group is dissolved whole when a piece is removed, rebuilt, changed (`shapeSig`, `props`) or edited by another node
  (`glueEdited`, set where the sync sees the edit), or when less than half of it stays together; its pieces are
  enabled where the compound left them and the groups are worked out again.
  * Hit by a fast body: a compound is rigid, so it would throw back a heavy fast body in the step it is hit (Box3D breaks
    bonds after a step, so `breakGlueEarly` cannot help). `holdCompoundsForApproach` (before each Box3D step) takes the
    pieces in the body's way out of the compound instead.
  * Which bodies: the last step's move events faster than `kGlueHoldMinSpeed`. `Box3DWorld::approachingBodies` keeps those
    with the momentum to break the weakest bond (`weakestBreak`; a kinematic body has no limit; nothing is held when no
    bond can break) whose bounds, swept for `kGlueHoldLeadSeconds` (two frames at least), overlap a dynamic body. The pair
    must be closing (`Approach::relLin`: relative to the target's motion), and the body must not be of the target's glue
    group (`glueSameGroup`). The momentum is compared with `compoundWeakest` or, for a held piece, `nodeWeakest`.
  * Which pieces: `releaseNear` takes those within `Box3DCompoundMember::radius` of the body's bounds swept to where its
    centre leaves the compound (`pathThroughBounds`), grown by `kGlueHoldHaloScale` times its half size. It is the whole
    way through, because a rigid piece left behind the freed ones throws the body back. They leave with `shrink` (a move
    event each ahead of the step's own), or the compound is dissolved when fewer than two would stay.
  * Holds: the pieces go into `glueHold` (`b3dop::GlueHolds`, `tests/core_compound.cpp`) until `glueClock` passes
    `kGlueHoldSeconds` from the last body near them. A body near held pieces extends the ones bonded to them
    (`GlueHolds::refresh`). `syncCompounds` leaves held pieces and their bonds out of the glue graph.
  * Bridges: a bond between a held piece and a piece still in the compound has its own weld parked with the compound's
    piece (Box3D leaves a joint disabled while either body is), so a `Box3DBridge` welds the free piece to the compound's
    body at that piece's place (`bridgeBond`, `frameInCompound`). `bridgeReleased` and `bridgeBoundaries` make them,
    `dropBridges` and `pruneBridges` remove them, `bridgesDirty` asks `syncCompounds` to make them again.
  * Merging back: `releaseExpiredHolds` (start of the frame) advances the clock. Expired pieces are let go only when
    their velocities are within `kGlueMergeSpread` of each other (`GlueHolds::expire`, else `kGlueHoldRetrySeconds`
    more), then the compound is one group larger than its members and is dissolved and merged whole.
  * Contacts inside a glue group are no hits, so the weight of held pieces resting on the compound breaks nothing
    (`glueSameGroup`, `compoundGlueHits`, `collectGlueHits`).
  Bonds that break (`jointsAfterStep`, `breakGlueEarly`) are taken out of the glue lists in place (`removeGlueBonds`: `glueRemoved`
  -> swapped out of `glueBonds`, the nodes' `partners` / `bonds` and `glueSignature`), so a frame with a few breaks costs a few
  bonds, not the scene. Nodes left without bonds stay until a quarter of them have, then the lists are rebuilt
  (`glueFull`, also set when a joint that is not a bond breaks). `syncCompounds` then works out again only the components the
  broken bonds were in (`reachFrom` over the nodes' partners, from `glueTouched`), using `tiedBodies` and the body's user data
  in place of the whole-scene tables; a rebuilt graph, a changed body set or a dissolved compound regroups everything.
  `splitBrokenCompounds` does the same for its components. A piece's force
  and torque go to the compound about its centre of mass (`compoundPieceForce`); a Force DOP reads its state from
  the compound (`compoundPieceState`). A bond inside a compound (`Box3DJointEntry::inCompound`, set by `markCompoundBonds`) writes
  force, torque, distance and angle as zero once and `impact` when it changed. Live frames are written in parallel
  (`writeLiveFrames`, above `kMinParallelFrames` bonds). Impacts data and the collision guide are not made for pieces.
