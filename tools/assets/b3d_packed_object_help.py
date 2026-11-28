"""Box3D Packed Object: the help card (F1 on the node), the asset's "Help" section in Houdini's wiki markup."""

HELP_CARD = '''\
= Box3D Packed Object =

#type: node
#context: dop
#internal: box3d_packedobject
#icon: DOP/box3d_packedobject
#tags: dynamics, rbd, box3d

"""Creates a DOP object from packed pieces for the Box3D Solver, like RBD Packed Object."""

Box3D Packed Object makes one DOP object holding packed pieces, each piece a rigid body for the Box3D Solver. It works
like RBD Packed Object: the pieces come from a SOP, a point per piece, and every setting can be overridden per piece
by a point attribute of the same name. Settings left at Box3D Data's defaults write nothing, so the point attributes
of Box3D Configure, RBD Configure or RBD Packed Object win.

The Box3D Solver SOP makes its own, so this node is for DOP networks you build yourself.

It works with the Box3D Solver only. In RBD Bullet Solver its pieces don't move: use RBD Packed Object there.

@parameters

== Object ==

Object Name:
    #id: objname
    The name of the DOP object. Pieces without a `name` attribute are named after it.

Creation Frame:
    #id: createframe
    The frame the object is created on. Creation Frame Specifies Simulation Frame counts it from the simulation's
    start instead of the scene's.

Solve on Creation Frame:
    #id: solvefirstframe
    Simulates the pieces on the frame they are created. Off: they start moving one frame later.

Geometry Source:
    #id: geosource
    Where the pieces come from: the SOP Path, or the geometry wired into the DOP network's first to fourth input.
    Use Object Transform moves them by the SOP's object transform.

Initial Object Type:
    #id: initialstate
    Whether the pieces start active, static, animated static or deforming, as RBD Packed Object's. Custom sets Body
    Type and Deforming yourself. A point's own `active`, `animated` or `deforming` still wins.

Overwrite Attributes from SOP:
    #id: overwritefromsop
    Copies the listed point attributes from the SOP onto the pieces every step, matched by name, so ones that change
    over time (such as `active`) reach the simulation. Off: read once, when the object is created.

Deforming:
    #id: deforming
    The pieces' shapes follow the SOP: it is read again every step and the shapes are rebuilt.

Display Geometry:
    #id: displaygeo
    Render Geometry draws the Render SOP Path's hi-res pieces moved with the simulated ones, matched by name.

== Initial State ==

Position, Rotation and Pivot move and turn every piece when the object is created. Velocity and Angular Velocity
(degrees per second) set the pieces' starting motion. Inherit Velocity from Point Velocity keeps the points' own `v`
and `w` instead, such as those Box3D Configure writes.

== Shape, Body, Material, CCD & Contacts and Sleep ==

Box3D Data's settings for every piece: the collision shape (Default Shape, Auto Fit, the hand-set box, sphere or
capsule), Body Type, mass and density, centre of mass, bounce, friction, damping, continuous collision and sleep. Each
is overridden per piece by the point attribute its tooltip names. Add Impact Data records the pieces' collisions in
Impacts data on the object.

@related

- [Box3D Solver|Node:sop/box3d_solver]
- [Box3D Configure|Node:sop/box3d_configure]
- [RBD Packed Object|Node:dop/rbdpackedobject]
'''
