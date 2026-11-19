"""Box3D Solver SOP: the inner SOP-level DOP network (build_dops, FORCES subnet)."""

import hou

import b3d_hda
from b3d_hda import labels
from b3d_solver_activeage import build_active_age_dop
from b3d_solver_breaks import build_breaks
from b3d_solver_updates import build_constraint_updates
from b3d_solver_ground import build_ground_dops, setp
from b3d_solver_common import (
    COLLISION_SOURCE,
    COLLISION_TYPE,
    CONSTRAINT_RELOAD,
    CONTEXT_GEOMETRY,
    create,
    DOP_COLLISION,
    DOP_CONSTRAINTS,
    DOP_PIECES,
    DOP_STATIC,
    FORCES,
    FORCES_NOTES,
    FORCES_OUTPUTS,
    OVERWRITE_FROM_SOP,
    PACKED_OBJECT,
    SHAPE_CONVEX,
    RELATIONSHIPS,
    BODY_KINEMATIC,
    BODY_STATIC,
    set_menu,
    set_named,
    STATIC_DATA,
    STATIC_NULL,
    switch_on_context_geo,
    USE_COLLISIONS,
    warn,
)


def packed_object(dopnet, name, context_input, object_name):
    obj = create(dopnet, PACKED_OBJECT, name)
    set_named(obj, "objname", object_name)
    set_menu(obj, "geosource", CONTEXT_GEOMETRY[context_input])  # the SOP Path stays empty, as RBD Bullet Solver's
    set_named(obj, "usetransform", 0)  # SOP space, like RBD Bullet Solver
    # One hull per connected set, as RBD Bullet Solver sets bullet_groupconnected on its objects:
    # pieces sharing a `name` are packed together and must not merge into one hull.
    set_named(obj, "hullperconnected", 1)
    return obj


def animate_collision(collision):
    """Collision Type on the asset drives the collision object: Animated or Deforming make it Kinematic (the solver
    moves each body to its new pose), Static leaves it Static. Animated bodies follow the source SOP in the solver
    (the Packed Object's SourceObject data, followSourceSops); Deforming reads the whole SOP every step (Overwrite
    Attributes from SOP and Read SOP Every Step), as the shapes change. All are the Packed Object's own parameters:
    the nested asset stays locked."""
    kind = collision.parm("initialstate")
    if kind is None:
        warn(f"{collision.path()} has no Initial Object Type: its Body Type may be ignored")
    else:
        kind.set("custom")  # Body Type below is ours
    p = collision.parm("bodytype")
    if p is None:
        warn(f"{collision.path()} has no Body Type; its parameters: {labels(collision)}")
    else:
        b3d_hda.menu_token_expr(p, f'ch("../../{COLLISION_TYPE}") > 0', BODY_KINEMATIC, BODY_STATIC, warn)
    deforming = f'ch("../../{COLLISION_TYPE}") == 2'
    for name in (OVERWRITE_FROM_SOP, "reimportsop"):
        parm = collision.parm(name)
        if parm is None:
            warn(f"{collision.path()} has no {name}: a deforming collision cannot follow its SOP")
        else:
            parm.setExpression(deforming, hou.exprLanguage.Hscript)


def build_dops(dopnet):
    pieces = packed_object(dopnet, "pieces", DOP_PIECES, "pieces")

    # Collision: packed colliders as convex hulls unless a point sets bullet_georep. Collision
    # Type Animated / Deforming: Kinematic (the solver moves each body to its new pose) and the SOP re-imported
    # every step; Static: Static, imported once.
    collision = packed_object(dopnet, "collision", DOP_COLLISION, "collision")
    set_menu(collision, "defaultshape", SHAPE_CONVEX)
    animate_collision(collision)

    # Collide As Static Object: the Collision input, unpacked, as one Static Object (polygons become one mesh,
    # a height field a height field), static. Only Deforming rebuilds it every step. A Box3D Data after it takes
    # the collider's Material settings.
    static = create(dopnet, "staticobject", "collision_static")
    setp(static, "object_name", "collision")
    setp(static, "soppath", f"../../{STATIC_NULL}")
    setp(static, "animategeo", expr=f'ch("../../{COLLISION_TYPE}") == 2')
    static_data = create(dopnet, "box3d_data", STATIC_DATA)
    static_data.setInput(0, static)
    b3d_hda.set_operations(static_data, "initial", warn)
    collider = create(dopnet, "switch", "collision_source")
    collider.setInput(0, collision)
    collider.setInput(1, static_data)
    setp(collider, "input", expr=f'ch("../../{COLLISION_SOURCE}")')

    # Constraints: every supported relationship, the network from the second context geometry,
    # attached to the pieces object as its ConstraintGeometry (as RBD Bullet Solver does), so the
    # constraints output can DOP Import it. Only the pieces go through it: the collision object
    # would get its own copy and every joint would be made twice. Anchors naming collision pieces
    # still resolve (piece names are looked up in every object).
    # Each one behind a Switch on its Enable toggle (use<name> on the asset): off, nothing is merged for it.
    rels = []
    # A Switch's inputs close up when input 0 is empty, so the "off" input is a Null: input 1 is the relationship.
    no_relationship = create(dopnet, "null", "no_relationship")
    for type_names, name in RELATIONSHIPS:
        try:
            rel = create(dopnet, type_names, name)
        except RuntimeError as e:
            warn(f"{e}; constraints of that type will not work in the Box3D Solver SOP")
            continue
        enable = create(dopnet, "switch", f"switch_use_{name}")
        enable.setInput(0, no_relationship)
        enable.setInput(1, rel)
        enable.parm("input").setExpression(f'ch("../../use{name}")', hou.exprLanguage.Hscript)
        rels.append(enable)
    rel_merge = create(dopnet, "merge", "relationships")
    for i, r in enumerate(rels):
        rel_merge.setInput(i, r)
    network = create(dopnet, "constraintnetwork", "constraintnetwork")
    network.setInput(0, pieces)
    network.setInput(1, rel_merge)
    set_menu(network, "geosource", CONTEXT_GEOMETRY[DOP_CONSTRAINTS])
    set_named(network, "attachinternalconstraints", 1)
    # Reload from SOPs is Overwrite with SOP, off by default, as RBD Bullet Solver's (the node's own default is an
    # expression that reloads while the input changes over time): the constraints are read once, so an attribute that
    # changes on the input afterwards only reaches the simulation through Update from Input or Enable Attribute
    # Update from SOPs.
    reload = network.parm("reloadfromsops")
    if reload is None:
        warn(f"{network.path()} has no Reload from SOPs: the constraints may be read again every step")
    else:
        reload.deleteAllKeyframes()
        reload.setExpression(f'ch("{reload.node().relativePathTo(dopnet.parent())}/{CONSTRAINT_RELOAD}")',
                             hou.exprLanguage.Hscript)
    constrained = create(dopnet, "switch", "use_constraints")
    constrained.setInput(0, pieces)
    constrained.setInput(1, network)
    switch_on_context_geo(constrained, DOP_CONSTRAINTS)

    both = create(dopnet, "merge", "pieces_and_collision")
    both.setInput(0, constrained)
    both.setInput(1, collider)
    objects = create(dopnet, "switch", "use_collision")
    objects.setInput(0, constrained)
    objects.setInput(1, both)
    p = objects.parm("input")
    if p is None:
        warn(f"{objects.path()} has no Select Input parameter")
    else:  # Enable Collision Geometry on, and the collision input has points (as RBD Bullet Solver), or, as a
        # Static Object, primitives (a height field has no points)
        p.setExpression(
            f'ch("../../{USE_COLLISIONS}") && if(ch("../../{COLLISION_SOURCE}"), '
            f'nprims(opinputpath("..", {DOP_STATIC})) > 0, npoints(opinputpath("..", {DOP_COLLISION})) > 0)',
            hou.exprLanguage.Hscript)

    # Forces: a FORCES subnet (the asset's dive target) with PRESOLVE and POSTSOLVE outputs, run by
    # a Multiple Solver around the Box3D Solver: PRESOLVE, Box3D Solver, POSTSOLVE. POP forces
    # wired to PRESOLVE write `force`, which the solver applies and clears in the same step.
    forces = build_forces(dopnet)
    solver = create(dopnet, "box3d_solver", "box3d_solver")
    multi = create(dopnet, "multisolver", "multisolver")
    # Ground Collision: None, a Ground Plane or a Static Object, merged with the pieces (mutual affectors).
    with_ground = create(dopnet, "merge", "objects_and_ground")
    with_ground.setInput(0, objects)
    with_ground.setInput(1, build_ground_dops(dopnet))
    multi.setInput(0, with_ground)
    multi.setInput(1, forces, FORCES_OUTPUTS.index("PRESOLVE"))
    multi.setInput(2, solver)
    multi.setInput(3, forces, FORCES_OUTPUTS.index("POSTSOLVE"))
    multi.setInput(4, build_active_age_dop(dopnet, "pieces"))  # Add Active Age Attribute, Just Activated
    multi.setInput(5, build_constraint_updates(dopnet))  # Enable Attribute Update from SOPs, before the breaks
    multi.setInput(6, build_breaks(dopnet))  # Break Thresholds, after everything else
    out = create(dopnet, "output", "output")
    out.setInput(0, multi)
    out.setDisplayFlag(True)
    return solver


# The DOP node marking a subnet output (DOP_SubNetOutput). Not "output": in DOPs that is the Output
# DOP that saves the simulation to disk.
SUBNET_OUTPUT_TYPE = "subnetoutput"


def build_forces(dopnet):
    """The FORCES DOP subnet: one subnet-output node per FORCES_OUTPUTS entry, named after it, its
    Output Index its position, with nothing wired in. The user wires POP forces and other
    microsolvers into them."""
    forces = create(dopnet, "subnet", FORCES)
    for child in forces.children():
        child.destroy()
    outs = []
    for i, name in enumerate(FORCES_OUTPUTS):
        out = create(forces, SUBNET_OUTPUT_TYPE, name)
        outs.append(out)
        if out.parm("outputidx") is None:
            warn(f"{out.path()} has no Output Index: FORCES outputs are not set")
            continue
        out.parm("outputidx").set(i)
    layout_forces(forces, outs)
    return forces


def layout_forces(forces, outs):
    """PRESOLVE on the left, POSTSOLVE on the right, each with its sticky note on its outer side;
    the subnet's inputs moved up out of the way."""
    # Node names are drawn to the right of a node, so the right-hand note sits further out.
    gap_left, gap_right, spacing = 1.0, 3.0, 6.0
    for i, out in enumerate(outs):
        out.setPosition(hou.Vector2(i * spacing, 0.0))
    for i, item in enumerate(forces.indirectInputs()):
        item.setPosition(hou.Vector2(i * 1.5, 4.0))
    for i, (out, text) in enumerate(zip(outs, FORCES_NOTES)):
        note = forces.createStickyNote(f"{out.name().lower()}_note")
        note.setText(text)
        note.setSize(hou.Vector2(4.5, 1.6))
        w, h = note.size()[0], note.size()[1]
        nw, nh = out.size()[0], out.size()[1]
        x = out.position()[0] - gap_left - w if i == 0 else out.position()[0] + nw + gap_right
        note.setPosition(hou.Vector2(x, out.position()[1] + nh / 2 - h / 2))
