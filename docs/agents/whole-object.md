# Whole-object bodies

RBD Object style bodies: one body per DOP object. Index and rules: [AGENTS.md](../../AGENTS.md).

## Whole-object bodies (`syncWholeObject`)

The older RBD workflow: each DOP object (RBD Object, RBD Point Object, RBD Fractured Object's pieces) is one rigid body,
its state in the object's `Position` data instead of point attributes.

* Which objects (`isWholeObject`): the object has a `SIM_ColliderLabel` at `SIM_COLLIDERS_DATANAME` ("Colliders") whose
  label is not `SIM_COLLIDERLABEL_PACKEDOBJECT` ("PackedObject"). RBD Packed Object and RBD Object both attach the label
  (checked in Houdini); an Empty Object + SOP Geometry has none and stays per point. No user switch, as in Bullet.
* Pose: RBD Object's RBD State (an `RBD_State`, a `SIM_Motion`) is attached as `Position` (checked), and the object's
  Geometry data has `positionpath = ../Position`. `readWholePose` reads any `SIM_Position` there: pivot (object
  space), the world position of the pivot through `getTransform` (so the translation convention does not matter on
  read), orientation, and velocity / angular velocity (radians; `SIM_Motion::getAngularVelocity`) of a `SIM_Motion`.
  The body origin is the pivot, and `applyBodyMass` moves the centre of mass there, as for points. No Position data
  (only until the first write-back): the object where its geometry is, about the centre of its object-space bounding
  box, zero velocity.
* Object space = the geometry through its own transform (`SIM_Geometry::getTransform`; scale / shear baked into the
  shape points). Shapes (`buildWholeObjectShapes`) from Box3D Data's Default Shape / Create Convex Hull per Set of
  Connected Primitives: box / sphere / capsule fitted to all points, convex one hull (or per connected set), concave
  one hull per primitive `name`; unkeyed hulls (by content); `none` or no points: shapeless static. The shape signature
  (`wholeObjectShapeSig`: geometry identity, P / primitive list data ids, counts, own transform, pivot, shape settings)
  changing rebuilds the shapes on the same body.
* Settings (`readDynamicProps` with unbound point attributes; changes go through `applyPropChanges`, read every step):
  Box3D Data whenever it is attached. Without it, the RBD Object's own data (`readRbdObjectSettings`, then
  `b3dop::rbdObjectSetup` over Box3D Data's defaults; option names from a Houdini 22.0 probe, read by name with
  `box3dDataOption` since none of the classes is in the HDK):
  * RBD State (`Position`, `RBD_State`): `computemass`, `density`, `mass` (the parameter, not a computed mass: Box3D
    computes density x volume from its shapes as for points), `inertialtensorstiffness` (the inertia scale). Its
    custom inertia tensor (`computeinertialtensortype`, `itensorlocalinv`) and glue options are not read. `p` /
    `computecom` is the pivot, already read through `getPivot`.
  * `PhysicalParms` (`SIM_PhysicalParms`): `friction`, `bounce`. `dynamicfriction`, `bounceforward`, `temperature`
    have no Box3D equivalent.
  * `Geometry/BulletData` (`SIM_BulletData`): `bullet_georep` (unknown names such as cylinder / plane: box),
    `bullet_groupconnected`, `bullet_autofit`; with AutoFit off a box / sphere / capsule is built from `bullet_primS`
    (half extents, as for points), `bullet_primT` (offset from the pivot), `bullet_primR` (Euler XYZ degrees),
    `bullet_radius`, `bullet_length` instead of being fitted (`wholeObjectShapes`; the spec is in the shape signature).
    Padding / shrink and Bullet's sleep options stay unread (see "Not supported on purpose" in [reference.md](reference.md)); `bullet_add_impact` is
    not read either, so Impacts stay opt-in through Box3D Data.
  Values Box3D cannot use (non-positive mass / density / inertia, negative friction / bounce, non-finite) keep Box3D
  Data's defaults. Houdini's RBD Object defaults differ from Box3D Data's: density 1000 with Compute Mass (a 1 m box is
  1000 kg), friction 1, bounce 0.5, convex hull. Body type is the node's (no `active` / `animated`).
* Mirror: one entry, key `makeBodyKey(objectId, kWholeObjectId = -1)`, `wholeObject = true`, `pivot`, no point
  (`pointOff` -1, `pointIndex` -1 in Impacts). The object scans every step (one body: no fingerprint).
  `Box3DObjectSync::wholeObject` makes aging and point `force` / `torque` skip the object; Force DOPs still apply.
  Joint anchors name the object (Anchors: an object with one body).
* Edits: Position data differing from what the solver last wrote (`poseDiffers`) teleports the body, as an edited point
  does.
* Write-back (`writeWholeObject`, from `writeResultsBack`'s move events, serial): the `SIM_PositionSimple` at
  `Position` (created as a `SIM_Motion` if missing; other `SIM_Position` kinds are left alone with a warning): pivot,
  orientation, translation, then velocities when it is a `SIM_Motion` (RBD State stays an RBD State). The translation
  is first written as "pivot displacement" (rotate about the pivot, then translate); if `getTransform` does not then
  put the pivot on the body, it is rewritten as the pivot's world position. The mirror takes the read-back
  (`readWholePose`) so float storage is not an edit. Double-precision setters (`setPosition(UT_Vector3D)`).
* HDK, from the reference: `SIM_ColliderLabel::getColliderLabel` (`GETSET_DATA_FUNCS_S`), `SIM_COLLIDERS_DATANAME`,
  `SIM_COLLIDERLABEL_PACKEDOBJECT`, `SIM_POSITION_DATANAME`; `SIM_Position::getPivot` / `getOrientation` /
  `getTransform`; `SIM_PositionSimple` `Position` / `Pivot` (V3) and `Orientation` (Q) GETSET; `SIM_Motion` (a
  `SIM_PositionSimple`) `Velocity` GETSET, `setAngularVelocity` / `getAngularVelocity` in radians;
  `SIM_Geometry::getTransform`; the GETSET float / double overloads from `SIM_OptionsUser.h`.
