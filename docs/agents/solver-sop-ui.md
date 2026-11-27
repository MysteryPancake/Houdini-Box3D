# Box3D Solver SOP asset: parameters, thresholds, viewer

The Solver SOP's parameter tabs, joint settings, Break Thresholds, constraint attribute updates, Output tab, viewer
state and messages. Its network: [solver-sop.md](solver-sop.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

`tools/make_solver_sop.py` and the `b3d_solver_*.py` modules build these.

* Advanced > Constraints (`b3d_solver_constraints.py`, as RBD Bullet Solver's joint settings): every relationship in
  `RELATIONSHIPS` gets a closed folder with its settings (those in the node's parameter folders, else all but
  `DATA_PARMS`, that have a `parmop_*` menu: SideFX's colour, radius, link and `guide_*` display settings have none
  and stay off), named `<node>_<setting>`, linked to the inner node and Set Initial (`finish_constraints`).
  `scale_joint_settings` wraps the existing channel reference instead of replacing it, so World Scale multiplies the
  promoted value. Each folder leads with Enable (`use<name>`, read by a Switch `switch_use_<name>` in front of the
  relationship node), Data Name (the node's `dataname`, which `constraint_name` matches) and Default Operation
  (`<name>_defaultparmop`, linked to the node's and driving each promoted setting's `parmop_*` menu: 0 initial, 1
  always, 2 never, so a Set Always one acts every step). `SOP_DEFAULTS` gives Glue, Hard, Soft and Spring settings RBD
  Bullet Solver's defaults (its SOP uses 1 where SideFX's nodes have their own: Glue Strength and Half-life 1, Soft
  stiffness and thresholds 1, Spring damping 1, Hard CFM and ERP 1), `SOP_DATA_NAMES` and `SOP_ENABLED` its Spring
  (named `Spring`, off), `PIECE_DEFAULTS` the Pieces Bounce (0.25): a `strength` attribute multiplies the setting in
  both solvers, so the same scene breaks at the same impulse (`glue_fracture_vs_bullet`). Cone Twist keeps the node's
  defaults: Bullet's own do not hold a piece (`bullet_cone_twist_goal`), so there is nothing to match.
* Break Thresholds (`b3d_solver_breaks.py`, first on the Constraints tab, then Export Broken Constraints): RBD Bullet
  Solver's `Constraint_Solver` for the Angle, Distance, Impact and Torque Thresholds and At Frame. Data Defaults are
  RBD Bullet Solver's (enabled, Constraint Names `Soft Hard`, only the Distance Threshold on at 1, no Scale
  Attribute), so a Soft or Hard constraint stretched past its Rest Length by 1 breaks in both. A SOP Solver DOP
  (`constraint_breaks`, Name `ConstraintGeometry`) loops over the entries as Bullet's does: `break_begin` (Fetch
  Feedback), `break_metadata` (the `iteration`), `break_end` (Count = `breaks`), inside Block Begin / End Compile
  nodes (`break_compile_begin`, `break_compile_live`, `break_compile_end`; compiled it costs a third of what it does
  uncompiled, 1.9 against 6.5 ms a frame on the `fracture` bench scene with `breaknames=Glue`, so every constraint is
  looked at). The primitive wrangle `break_constraints` (`BREAK_VEX`) deletes the primitives whose `impact` passes an
  entry's threshold, or switches them to `next_constraint_name`. The entries run in order, so a constraint one
  switches to another name can be taken by a later one, as Bullet's. The entry's settings are spare parameters of the
  wrangle (`break_spare_parms`: `constraint_<name>` is the asset's `constraint_<name>#` of `iteration + 1`), read
  through its spare input, the metadata block: `detail(-1, "iteration", 0)` (a node path is an error inside a compiled
  block). Its Group (`GROUP_EXPRESSION`, a Python expression) is the entry's names as one `@constraint_name=` pattern
  each, so the match is exact, as Bullet's, and happens natively (string work in VEX per primitive is slow: `tolower`
  alone cost 340 ms a frame on 45,000 primitives). The parameters are Bullet's names (`enable_constraintbreaks`, the
  `breaks` multiparm, `constraint_names#`, `constraint_mode#`, `constraint_usefromframe#`, `constraint_useatframe#`,
  and per key in `THRESHOLDS` `constraint_use<key>#`, `constraint_<key>threshold#`, `constraint_scale<key>#`,
  `constraint_use<key>var#`) and rules (Distance and Impact thresholds x World Scale, x the `strength` attribute with
  Scale by Attribute (with Update from Input, `constraint_<key>attribupdate#`, the same constraint's value in the live
  Constraints input: `live_constraints`, an Object Merge of the DOP network's constraints input, through
  `break_compile_live` and `break_live` as the wrangle's third input, found by `__primid`, which `store_primid` writes
  on the input), scattered by Variance with `__primid` else the primitive number; Distance compares against threshold
  + `restlength`; At Frame's window is 0.5 / Global Substeps). The SOP Solver is the Multiple Solver's last input,
  behind an Enable Solver DOP (`enable_constraint_breaks`, on while Break Thresholds are on and the Constraints input
  has points: the compiled loop costs about a millisecond a step over nothing): off, the constraint geometry is not
  imported, because a rewrite changes its data ids and the solver then reads the whole network again each step. Test
  `solver_sop_break_impact_bullet`, `solver_sop_break_atframe_bullet`, `solver_sop_break_distance_bullet`,
  `solver_sop_break_attrib_update_bullet`, `solver_sop_break_variance_bullet`, `solver_sop_break_scale_bullet` (against
  the RBD Bullet Solver SOP), `solver_sop_break_next_settings` and
  the INFO `solver_sop_break_angle_torque_bullet`.
* Constraint attribute updates (`tools/b3d_solver_updates.py`, as RBD Bullet Solver's): the Constraints tab's Override
  Attributes folder (`enable_constraintupdates` with its label hidden, joined to `constraint_attributes`, then
  `constraint_attribgroup`) runs a SOP Solver DOP (`constraint_updates`) over the ConstraintGeometry behind an Enable
  Solver DOP, before the breaks: an Attribute Copy SOP copies the listed attributes from `live_constraints` (an Object
  Merge of the DOP network's constraints input) matched by `__primid`, onto the Group's constraints. Overwrite with
  SOP (`constraint_reloadfromsops`, Advanced > Constraints) drives the Constraint Network's Reload from SOPs (read the
  whole input again every step) and greys the update out. Test `solver_sop_constraint_updates_bullet`.
* Help card: `HELP_CARD` (`b3d_solver_help.py`, Houdini wiki markup) as the definition's `Help` section.
* The Solver and Pieces tabs are labelled Setup and Properties (`TAB_LABELS`), as RBD Bullet Solver's; the code and
  these docs call them Solver and Pieces.
* Output tab, as RBD Bullet Solver's: Impact Data, a closed collapsible folder whose header toggle is the pieces' Add
  Impact Data (`sidefx::header_toggle`), with the solver's Impact Min Speed and Fetch Impacts; Transfer Attributes,
  with Transfer to Geometry / Proxy Geometry choosing each Transform Pieces' Attributes to Copy as RBD Bullet
  Solver's: the list while on, else `w` (`copy_attributes`, `TRANSFER_TO`); Transform Pieces' Point Velocities stay at
  their defaults. Fetch Impacts makes (or reuses) an Object Merge of the asset's `IMPACTS` null beside the asset, fed
  by a DOP Import Records of the pieces' `Impacts` data (its parameters set by name).
* Parameters: Reset Simulation and Start Frame above the tabs, then the solver's tabs (folders found under its Data
  Options, a tab of simple folders kept whole; Guide Options skipped). Tab order, as RBD Bullet Solver's: Solver,
  Pieces, Collision, Forces, Constraints, any other solver tabs, Visualization (`visualization_tab`: a Visualize
  Geometries folder (header checkbox `VISUALIZE`, on: it gates every guide's Switch and greys out the folders, added
  to their own Disable When) with Geometry (Show Collision Shape, Shape Color), Ground and Collision Geometry (Show
  Geometry, Show Collision Shape, Color), Show Constraints (`SHOW_CONSTRAINTS`, off), Constraint Geometry (False Color
  Mode `FALSE_COLOR_MODE`, Attribute, Show Guides `SHOW_GUIDES`, Guide Radius), Impact Data (Show Impacts, off), then
  Inspector, as RBD Bullet Solver's; its Display Visualization Geometry master toggle, Show Active / Sleeping
  colouring and the constraint guide folders have no counterpart), then Advanced and Output always last; the solver's
  parameters are regrouped by name (`SOLVER_FOLDERS`, `IMPACT_SPEED`, `SHAPES_TOGGLE`, pulled with `pull`), whichever
  DOP tab holds them: the DOP's tabs and folders follow this layout (Contacts, Sleeping and Aging folders on the
  Solver tab, Force DOPs, Bounce / Friction and Glue on Forces, Stability, Mass & Inertia and Performance on Advanced), so only
  Contacts and Sleeping are rebuilt (Sleeping closed), Aging is pulled (`AGING_PARMS`) into Output's Attributes
  folder, Forces keeps the DOP's folders as they are, Advanced is flattened and regrouped by `TAB_GROUPS`, and DOP
  tabs left empty (Visualization, Output) are dropped. Solver: a General folder (World Scale, Time Scale), a
  Simulation folder (the solver's own settings, Solve on Creation Frame, Emit RBDs joined to Replicate RBDs, Cache
  Simulation with its label hidden, joined to Cache Memory (MB), which it disables when off), then Contacts and
  Sleeping. Cache Memory is the inner DOP network's value with the parameters on its row (`row_of`, `CACHE_MEMORY`).
  Advanced is Stability (warm starting, continuous, speed limit), Mass & Inertia (bounding box inertia, near-sphere tolerance), Performance (threads), Collision (the collider's Solve on Creation Frame) and
  Constraints (a closed folder per joint type). Collision: one Collision Geometry folder whose header checkbox is Use
  External Collisions (`usecollisions`, placed just before the folder, as RBD Bullet Solver's), holding Collision Type
  and the collision object's Shape, Material and CCD & Contacts folders (`COLLIDER_FOLDERS`, all named `collision_*`),
  the last cut to Contact Recycling as Contacts (`COLLIDER_DROPPED`: the rest does nothing for a Static / Kinematic
  body). Pieces: the pieces' Box3D Data folders (Shape, Body, Material, CCD & Contacts, Sleep) without their Allow
  Sleeping (`PIECES_DROPPED`; `drop_templates`), then an Override Attributes simple folder holding Override Attributes
  from SOP (`overwritefromsop`, label hidden, joined to Attributes, off, linked to the pieces' Packed Object, which
  then reads the SOP every step; the collision's is driven by Collision Type), the solver's Allow Sleeping is in the
  Solver tab's Sleeping folder instead (one switch for every body); names the solver also uses would get `piece_`.
  Conditions are rewritten with the names (the pieces' Start Asleep and Sleep Threshold grey out with the solver's
  Allow Sleeping). Inner solver `parmop_*` set to Set Always (the solver reads its settings every step, so an animated
  one acts mid-simulation; test `solver_sop_animated_setting`). The pieces' and collider's Box3D Data, the
  relationships and the ground stay Set Initial. Inside each tab the settings sit in collapsible folders, as RBD
  Bullet Solver's (`TAB_GROUPS`, `group_tab`) and with its tags (`group_type`, `group_default`,
  `sidefx::header_toggle`), every one open except Output's Impact Data, the Solver tab's Sleeping and the joint
  folders; Collision is one Collision Geometry folder with the collider's folders simple inside it (each disabled with
  Enable Collision Geometry off), Pieces is Shape, Body, Material, CCD & Contacts and Sleep (the Packed Object's Start
  Asleep, taken out of its Sleep tab and put first, `moved_settings`), and Add Impact Data (taken out of its Shape
  tab) is on Output. Test `solver_sop_layout`.
* Viewer state (`tools/b3d_solver_state.py`, `STATE_NAME` / `STATE_FILE`, installed by `b3d_hda.add_viewer_state`),
  RBD Bullet Solver's trimmed: an Option Menu with Output for View Flags (Geometry / Constraint / Proxy,
  `setOutputForViewFlag`) and an inspector, hotkeys 1 to 5 and the HUD lines for them. The inspector prints the
  attributes of the primitive under the cursor (`inspectorattribs`, `inspectorfontsize`, `inspectorprecision` in the
  Visualization tab's Inspector folder) from the viewed output's geometry. RBD's false-colour constraint info,
  histogram and guide capture are not copied. The state name has no dot: its hotkey context is built from it.
  `b3d_hda.show_in_toolbar` puts Reset Simulation, Show Geometry, Show Collision Shape and Show Ground in the
  viewport's toolbar (export level 3, as RBD's toggles), and an `OnCreated` section sets Start Frame to `$FSTART` on a
  new node.
* Message Nodes (Node tab of the Type Properties, the `MessageNodes` section: relative paths, space separated) are
  set by `b3d_hda.set_message_nodes` so a warning from inside shows on the asset: the Solver SOP's `dopnet/box3d_solver`
  and Box3D Constraint Properties' `warn_if_no_constraints` (`MESSAGE_NODES`). Box3D Configure has none: only its own
  packing and cleanup nodes warn ("no match for name", "'__packed' is not a valid group", ...), on every cook, and
  listing them would show that noise. Only the node's own messages show, not its children's, so a subnet is no use.
  A group delete warns when its group is missing; `removeprimgroup` in a wrangle does not.
* Input / output labels are written into the DialogScript after the last `updateFromNode` (no hou setter). Wire
  colours come from the subnet's indirect input nodes (`INPUT_COLORS`: the constraints input pink, and its IN /
  OUT nodes).
