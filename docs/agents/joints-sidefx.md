# SideFX constraint data and Glue

Glue, Hard, Spring, Bullet Soft and Cone Twist mapped onto Box3D joints. Joint nodes, networks and live frames: [joints.md](joints.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

**SideFX constraint data** (`core/Box3DSideFX.*`, tested in `tests/core_sidefx.cpp`; read in `Box3DJointSync.cpp`):
* Found by data type among the same subdata as our nodes (`networkKinds`): `SIM_ConRelGlue`, `SIM_ConRelHard`,
  `SIM_ConRelSpring`, `SIM_ConRelBulletSoft`, `SIM_ConRelConeTwist` (names from a Houdini 22.0 probe; their default
  Data Names are Glue, Hard, ConRelSpring, Soft, ConeTwist). Only Hard and Spring have HDK headers, so all five are read
  the same way, by name (`box3dDataOption` in `Box3DHoudiniUtil.h`: `dynamic_cast` to `SIM_OptionsUser`, polymorphic
  through its virtual destructor, and `importOption`, else `SIM_Query::getFieldFloat("Options", 0, name)`, scalars
  only), per option in `sideFXParams()`. Option names are the node
  parameter names, from the same probe. `Box3DJointKind` holds the SideFX type and options instead of a joint type.
* Per primitive: matching primitive attributes modify the options (floats multiply, ints / vectors replace; our
  `jointParams` overrides are not applied), `constraint_type` is read (missing, empty or unknown: position, as SideFX's help says and Bullet does: `sidefx_hard_untyped`), then `mapSideFX`
  picks the joint:
  * Glue: rigid weld holding the pieces where they are, never colliding, never broken by force or torque. It breaks
    on the impulse of the hits it takes, as the Glue help describes Bullet's (`core/Box3DGlue.h`, Houdini-free,
    `tests/core_glue.cpp`): after every Box3D step `collectGlueHits` adds the normal impulse of each glued body's contacts
    (not its glued neighbours') to its hits; at the end of the frame `advanceGlue` spreads them over the bonds
    (`impulse` shares: itself 1, each bond its Propagation Rate, `propagationiterations` rounds), adds the two bodies'
    results to the bond's `impact` after it decayed by `impulse_halflife`, and queues a bond over its Strength
    divided by `kGlueStrengthScale` (< 0 never) for `jointsAfterStep`, which breaks it with the force ones (the `broken` group, `next_constraint_name`).
    `impact` is written to the primitives and read back when a joint is remade. The bonds are listed by `syncJoints`'s
    removal loop (no pass of their own); `contactImpulseByHandle` sums the contacts' impulses without building records. Weight is no hit, so a hanging piece
    holds at any Strength (`sidefx_glue`); a landing is (`glue_impact`). `impact` matches Bullet's own (peak within
    ~20%, the same decay: `glue_impact_trace`); a Bullet glue holds from about 5 to 6 times its peak impact, so
    the break test uses `kGlueStrengthScale` (measured from one scene: `glue_impact_scale`). A negative iteration count is the global one
    (the `propagate_iteration` detail attribute of the network, read by `NetworkReader::glueIterations`; one round without it);
    the most iterations of any bond set the rounds.
  * Glue as One Body (`gluecompound`): [glue-compound.md](glue-compound.md).
  * Hard: all -> rigid weld with the anchors kept the rest length apart (as Bullet does; held where they are when there
    is no rest length); position -> rigid distance joint at the
    rest length, or spherical when it is ~0; rotation -> motor joint, angular spring only at 60 Hz / ratio 2 with
    unlimited torque (Box3D has no rigid rotation-only joint). CFM, ERP, iterations and the angular motors are ignored.
  * Bullet Soft: stiffness is already Hz. all -> motor joint with linear and angular springs (enableangular picks the
    angular stiffness / damping), frames offset by the rest length; position -> sprung distance joint at the rest
    length, or a linear-only motor joint when it is ~0; rotation -> angular-only motor joint. Plasticity is ignored.
  * Spring: as Soft, after converting Hooke's constant and absolute damping per joint: Hz = sqrt(k / m) / 2 pi,
    ratio = c / (2 sqrt(k m)), with m the reduced mass of the dynamic bodies (one dynamic body: its mass) for the
    linear spring, and the mean principal inertia the same way for the angular one. The masses it was made from are kept
    (`massDependent`, `massA` / `massB` on the entry); `syncJoints` compares them with the bodies' now each step and
    reprocesses a network whose Spring's masses changed, so its Hz and ratio follow (`sidefx_spring_mass`).
  * Cone Twist: spherical joint. Box3D's cone is round: the smaller of Max Up / Max Out, clamped to Box3D's 90 degree
    maximum; 180 or more turns it off (Houdini's default). Twist limit +-Max Twist (180 or more: none). Bullet twists
    about x, Box3D about z, so each frame is turned so z = the twist axis and x = the up axis (constrained axes on
    point 0, goal axes on point 1). Softness, the soft-constraint options, translation ranges, rest length and the
    motor are ignored.
  * Hard / Spring / Soft / Cone Twist: Collide Connected = !disablecollisions.
* **Frame offsets**: a SideFX joint's frames need not sit on its anchors. `sideFXFrameOffsets` gives each anchor an
  offset in its own axes (`adjA` / `adjB` on `Box3DJointEntry`): a rotation (cone twist) and/or a shift along the anchor
  line so the frames meet when the anchors are `separation` apart (Soft / Spring / Hard all with a rest length), or are
  as far apart as now (`holdSeparation`: Glue, Hard all without one; anchors from Connect Adjacent Pieces sit at the
  centroids, and
  a plain weld there would pull the pieces together). The joint is made from `applyFrameOffset(anchor, adj)`;
  `writeLiveFrames` and the `next_constraint_name` switch write `removeFrameOffset(frame, adj)` back as `current_P` / `current_orient`,
  so the anchors stay anchors, and `local_P` / `local_orient` hold the joint's own frames (compared with the
  offset anchors in `localFrameAgrees`). Offsets are fixed when the joint is made; their inputs (rotations,
  separation, the hold flag, not the anchor positions) are in the joint signature. Our Weld's Hold Separation
  (`JointSettings::holdSeparation`, `hold_separation`, not a Box3D setting) gets Glue's hold offsets in `specFor`.
  Identity offsets (our other nodes) bypass the math.
