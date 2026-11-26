# SideFX constraint data and Glue

Glue, Hard, Spring, Bullet Soft and Cone Twist mapped onto Box3D joints. Joint nodes, networks and live frames:
[joints.md](joints.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

**SideFX constraint data** (`core/Box3DSideFX.*`, tested in `tests/core_sidefx.cpp`; read in `Box3DJointRead.cpp`):
* Found by data type among the same subdata as our nodes (`networkKinds`): `SIM_ConRelGlue`, `SIM_ConRelHard`,
  `SIM_ConRelSpring`, `SIM_ConRelBulletSoft`, `SIM_ConRelConeTwist` (names from a Houdini 22.0 probe; their default
  Data Names are Glue, Hard, ConRelSpring, Soft, ConeTwist). Only Hard and Spring have HDK headers, so all five are read
  the same way, by name (`box3dDataOption` in `Box3DHoudiniUtil.h`: `dynamic_cast` to `SIM_OptionsUser`, polymorphic
  through its virtual destructor, and `importOption`, else `SIM_Query::getFieldFloat("Options", 0, name)`, scalars
  only), per option in `sideFXParams()`. Option names are the node
  parameter names, from the same probe. `Box3DJointKind` holds the SideFX type and options instead of a joint type.
* Per primitive: matching primitive attributes modify the options (floats multiply, ints / vectors replace, except
  Glue's `strength`, which follows Bullet's `getStrength`: a negative node Strength takes the attribute as it is, a
  negative attribute wins, `combineGlueStrength`; our `jointParams` overrides are not applied), `constraint_type` is read (missing, empty or unknown: position, as
  SideFX's help says and Bullet does: `sidefx_hard_untyped`), then `mapSideFX` picks the joint:
  * Glue: rigid weld holding the pieces where they are, never colliding, never broken by force or torque. It breaks on
    the impulse of the hits it takes, as the Glue help describes Bullet's (`core/Box3DGlue.h`, Houdini-free,
    `tests/core_glue.cpp`): after every Box3D step `collectGlueHits` adds the normal impulse of each glued body's
    contacts (not its glued neighbours', nor its own glue group's: see `glueSameGroup`,
    [glue-compound.md](glue-compound.md)) to its hits; at the end of the frame `advanceGlue` spreads them over the
    bonds (`impulse` shares: itself 1, each bond its Propagation Rate, `propagationiterations` rounds), adds the two
    bodies' results to the bond's `impact` after it decayed by `impulse_halflife`, and queues a bond over its Strength
    (< 0 never) for `jointsAfterStep`, which breaks it with the force ones (the `broken` group,
    `next_constraint_name`). A bond also breaks when its decayed `impact` plus the spread break estimate passes the
    Strength (below), and keeps that as its `impact`. `advanceGlue` spreads over a copy of the bonds (`glueMirror`, with a flag
    that none carries an impact, which is what makes a quiet frame idle) kept between frames: it is made again only
    when `glueEpoch` moved, so anything that adds, removes, breaks or remakes a bond or sets its Glue settings or
    impact must call `touchGlueBonds`. `impact` is written to the primitives and read back when a joint is remade. The
    bonds are listed by `syncJoints`'s removal loop (no pass of their own); the hits of single bodies come from the
    step's impulse events in one pass (`glueHitsFromEvents`, on while there are glued bodies), read as one record per
    body pair (`getContactImpulseSums`) with both bodies' node and group looked up once per pair;
    `contactImpulseByHandle` agrees with them for awake bodies in `core_compound`, a compound's pieces from
    `contactImpulsesByShape`, and a decayed `impact` under `kGlueImpactFloor` is 0, so a quiet frame is idle. Every
    contact counts, a resting body's weight included, as in Bullet (a deliberate match: only the contacts inside a
    glue group are left out). A hanging piece has no contacts and holds at any Strength (`sidefx_glue`); a landing is
    a hit (`glue_impact`). `impact` matches Bullet's own (peak within ~20%, the same decay: `glue_impact_trace`).
    * The break estimate (`estimateGlueHits`, `Box3DGlueEstimate.cpp`; the maths in `core/Box3DGlueEstimate.h`, checked
      in `tests/core_glue.cpp`): Bullet breaks glue before its step, on an impulse estimate at every contact point of
      the glued body (the pieces of a group merged into one rigid body), (1 + e v) v / d, summed per piece and spread
      like hits. The speed v of a contact is its hit event's approach speed (`getHitContacts`: only contacts that came
      together faster than Box3D's hit threshold, the slower ones being a body at rest, left to their real impulse)
      plus the step's gravity; e is the two restitutions multiplied; d is the group's impulse denominator at the point
      (`GlueBody`, from `mergePieces` of the pieces' poses and mass data, so a compound and welds are the same) plus the
      other body's. A contact is looked up by its shape pair (an exact key: a hashed one collided). A compound's
      body has no entry, so its shapes' owners come from `shapeKeysByHandle` and the members' shape ranges; a contact
      with a dynamic body that is neither a piece nor has an entry is left to its real impulse. A group takes its
      estimate from the first step of the frame that has one (`glueEstimated`), as Bullet makes one from the speeds the
      frame begins with; later steps' contacts move because of the earlier ones (the largest of the steps' held a box
      dropped on a row 1.7x to 2x looser than Bullet). Nothing is estimated when no bond can still break. Landings match Bullet's
      hold Strength within about 15%, and a box dropped on a group within 1.0x to 1.3x (`glue_break_match`); an edge
      landing of a fractured block holds 4 to 7 times better than Bullet's, whose trial moves it a step forward into the
      ground, so more pieces have points (`glue_fracture_vs_bullet`).
    * A negative iteration count is the global one (the `propagate_iteration`
    detail attribute of the network, read by `NetworkReader::glueIterations`; one round without it); the most
    iterations of any bond set the rounds.
    * Material: Bullet runs a glue group as one body with the pieces' friction and bounce weighted by their mass, so
      `applyGlueMaterials` (`Box3DGlueMaterial.cpp`, after `syncCompounds`) gives every dynamic piece of a group and
      each compound of it that average, in both modes; a piece that leaves every group gets its own back. It runs only
      when the bonds, the compounds or the bodies changed (`bullet_glue_materials`).
  * Glue as One Body (`gluecompound`): [glue-compound.md](glue-compound.md).
  * Hard: all -> rigid weld with the anchors kept the rest length apart (as Bullet does; held where they are when there
    is no rest length); position -> spherical joint (Bullet's pin: see "Position with a rest length" below), or a rigid
    distance joint at the rest length when the rest anchors coincide; rotation -> motor joint, angular spring only at 60 Hz / ratio 2 with
    unlimited torque (Box3D has no rigid rotation-only joint). CFM, ERP, iterations and the angular motors are ignored.
  * Bullet Soft: stiffness is already Hz. all -> motor joint with linear and angular springs (enableangular picks the
    angular stiffness / damping), frames offset by the rest length; position -> linear-only motor joint (offset as below; a sprung distance joint
    when the rest anchors coincide, and a plain linear motor joint when the rest length is ~0); rotation -> angular-only motor joint. Plasticity is ignored.
  * Spring: as Soft, after converting Hooke's constant and absolute damping per joint: Hz = sqrt(k / m) / 2 pi,
    ratio = c / (2 sqrt(k m)), with m the reduced mass of the dynamic bodies (one dynamic body: its mass) for the
    linear spring, and the mean principal inertia the same way for the angular one. The masses it was made from are kept
    (`massDependent`, `massA` / `massB` on the entry); `syncJoints` compares them with the bodies' now each step and
    reprocesses a network whose Spring's masses changed, so its Hz and ratio follow (`sidefx_spring_mass`).
  * Position with a rest length (Hard, Soft, Spring): Bullet shifts anchor A by the rest length along the direction
    between the anchors at rest, fixed in A's frame (`simSetRestLength`), and pins (Hard) or springs (Soft, Spring) anchor
    B to that point in all three axes, so it is not a distance: a box beside the fixed one with the rest length their
    distance stays where it is (`bullet_position_rest_length`). `SideFXJoint::offsetOnA` shifts anchor A's frame by all
    of the separation (the all mode splits it between both frames, which is the same while the pieces are locked to each
    other, but a position-only joint lets B turn about its anchor). The rest anchors on top of each other have no
    direction, and Bullet takes the current one, each frame, with its three rows holding the anchors' relative velocity: a
    box hung from the shared point opens a gap under its weight and ends the rest length along it, not swinging
    about the point. Those joints (`SideFXJoint::reaim`, from `SideFXContext::anchorDistance`, the authored anchors'
    distance, so the choice does not change as the pieces move) are the same Spherical or linear Motor joint, with no shift
    while the anchors are within `kSideFXAimMinDistance`, and `reaimJoints` (`Box3DJointSync.cpp`, before the steps;
    `Box3DWorld::reaimJointFrameA`) aims frame A's shift along the anchors' line again each frame, updating the entry's
    `adjA` so the written anchors stay the anchors. A joint made later (a rebuilt world) starts from the anchors as
    they are (`bullet_soft_coincident_anchors`, `bullet_position_rest_length`, `position_rest_rebuild`). Which way that
    gap opens comes from the first frame's sag, which differs between the solvers, so a swinging piece then moves
    differently in each; a pin, or a rest offset fixed at rest, swings as Bullet's (`bullet_hard_pendulum`).
  * Cone Twist: spherical joint. Box3D's cone is round: the smaller of Max Up / Max Out, clamped to Box3D's 90 degree
    maximum; 180 or more turns it off (Houdini's default). Twist limit +-Max Twist (180 or more: none). Bullet twists
    about x, Box3D about z, so each frame is turned so z = the twist axis and x = the up axis (constrained axes on
    point 0, goal axes on point 1). Softness, the soft-constraint options, translation ranges, rest length and the
    motor are ignored.
  * Hard / Spring / Soft / Cone Twist: Collide Connected = !disablecollisions.
* **Frame offsets**: a SideFX joint's frames need not sit on its anchors. `sideFXFrameOffsets` gives each anchor an
  offset in its own axes (`adjA` / `adjB` on `Box3DJointEntry`): a rotation (cone twist) and/or a shift along the
  anchor line so the frames meet when the anchors are `separation` apart (Soft / Spring / Hard all with a rest
  length), or are as far apart as now (`holdSeparation`: Glue, Hard all without one; anchors from Connect Adjacent
  Pieces sit at the centroids, and a plain weld there would pull the pieces together). The joint is made from
  `applyFrameOffset(anchor, adj)`; `writeLiveFrames` and the `next_constraint_name` switch write
  `removeFrameOffset(frame, adj)` back as `current_P` / `current_orient`, so the anchors stay anchors, and `local_P` /
  `local_orient` hold the joint's own frames (compared with the offset anchors in `localFrameAgrees`). Offsets are
  fixed when the joint is made; their inputs (rotations, separation, the hold flag, not the anchor positions) are in
  the joint signature. Our Weld's Hold Separation (`JointSettings::holdSeparation`, `hold_separation`, not a Box3D
  setting) gets Glue's hold offsets in `specFor`. Identity offsets (our other nodes) bypass the math.
