# Joint live frames

What `writeLiveFrames` writes onto a written network after each step (`current_*`, `local_*`, `auto_restlength`), how
an upstream edit of the anchors is told from it, and moving world anchors. Networks, nodes and the rest of the
after-step write-back: [joint-networks.md](joint-networks.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

**Houdini** (`Box3DJointSync.cpp`, `Box3DJointBreak.cpp`):
* **Live frames** (written networks): after every step `writeLiveFrames` sets each live joint's anchor points'
  `current_P` / `current_orient` to where its frames are now (`Box3DWorld::jointWorldFrames`: body pose times local
  frame; a world end does not move). `P` / `orient` are never written: the Constraint Network DOP documents `P` as the
  anchor's initial world position, and Houdini's constraint guide moves each anchor with its piece itself, so a live
  `P` would be drawn twice as far (`anchor_positions` checks both). `NetworkReader::frame` reads `current_*` when
  written (a zero `current_orient`, the value of a point added later, means not written), else `P` / `orient`. The
  network data is cached with each timestep (SIM_GeometryCopy), so a world rebuilt mid-simulation makes its joints
  from the cached `current_*` against the cached body poses, which gives exactly the same local frames as continuing
  (`tests/core_joints.cpp` compares a rebuilt world with the original).
  * **Local frames** (`local_P` / `local_orient`, point attributes, the names SideFX's Constraint Network uses for an
    anchor in its transform's local space): a joint made from `P` / `orient` saves the frames Box3D keeps
    (`jointLocalFrames`: relative to the piece, position in its axes, in scene units) after the step. A joint made
    later (a world rebuilt mid-simulation) is made from them (`createJoint(..., framesAreLocal)`) when they still
    agree with the live values: `localToWorld(body, local)` must land on the point's `current_P` / `current_orient`
    (1e-5 relative, quaternion dot 1 - 1e-6), which holds when all three were written after the same step and fails
    for a renamed, moved or new anchor (a zero `local_orient` means none). This makes rebuilds exact (float world
    positions lose about a float step at the coordinates' size per rebuild; `asLocalFrame` only renormalises a
    quaternion). The live `current_*` are still read: to make new joints and to see edits.
  * Edits: the entry remembers the authored anchors it was made from (`authA` / `authB`) and what it wrote (`wroteA` /
    `wroteB`), as stored in float. A reprocessed network whose `P` / `orient` differ from the first, or whose
    `current_*` differ from the second (1e-5 relative; q and -q equal), was edited upstream (a SOP Solver): the joint
    is remade from the authored `P` / `orient` (an edit of the authored anchors made just before a world rebuild is
    not seen: the rebuild trusts `current_*`). The signature therefore leaves the frames out for written networks; a
    network that could not be written (`writable` false) keeps the frames in its signature, so editing them remakes
    the joint. An edit of only a world end's `P` / `orient` (an animated world anchor, written networks) is not a
    remake: `followWorldAnchor` moves the joint once onto a shapeless kinematic body of its own
    (`Box3DJointExtra::anchorBody`), and `driveWorldAnchors` gives that body the anchor as its target every frame, so
    it moves there over the frame's steps with that velocity (`moving_world_anchor`). Bullet gives its anchor no
    velocity, so a joint dragged along by one trails less here. `jointLocalFramesOf` keeps that end's saved local
    frames relative to the world.
  * A distance joint made with a negative rest length gets the anchors' distance at that moment written to the
    `auto_restlength` primitive attribute (created with default -1), so a rebuild keeps the rest length instead of
    the current, stretched distance. Not `restlength`: that attribute multiplies the node's value, and -1 times the
    distance would be negative again. It is only read while the joint's rest length is negative.
  * Our writes bump `current_P` / `current_orient` (in the network fingerprint), so the fingerprint is re-recorded after
    the write and an unchanged network still skips. Writing happens every step except for a network whose joints are all
    quiet (`markQuietJoints`: bodies at rest and already written, or broken joints with their last values written); with
    nothing broken and no Enable Aging it is left alone (no write lock, new data ids or fingerprint).
  * A Constraint Network with Overwrite with SOP re-imports the SOP every step, dropping `current_*`: the joints
    carry on (their signature leaves the frames out), but a world rebuilt from a cached frame other than the first
    makes them from the authored anchors there.
