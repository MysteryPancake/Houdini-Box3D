"""Builds example scenes that show what the Box3D solver can do, for new users to open and play.

Run from the repository root with the plugin built and installed (build.bat):

    hython tools/examples/make_examples.py              # every scene
    hython tools/examples/make_examples.py building     # some scenes (names below)
    hython tools/examples/make_examples.py --check      # also simulate each scene and print timings
    hython tools/examples/make_examples.py --samples    # Box3D's own samples, ported (box3d_samples/)
    hython tools/examples/make_examples.py --samples stacking restitution/varying   # some of them

On Windows, make_examples.bat (in the repository root) finds Houdini and runs this.

Each scene is saved to examples/<number>_<name>.hip (the folder is made when needed). Open one and press play: a sticky
note in /obj says what it shows and what to try. Every scene is built from ordinary SOPs (Box,
Copy to Points, Attribute Wrangle; the building also RBD Material Fracture) and one Box3D Packed
Object (built with the plugin; RBD Packed Object if it is missing), so it doubles as a template.

Box3D's samples go to examples/<category>/<sample>.hip instead, one scene per sample
of Box3D's sample app that ports (docs/agents/sample-port-plan.md says which), the Box3D Solver set as
that app runs (60 fps).

Scenes:
  shapes          the collision shapes: box, sphere, capsule and convex hull
  many_bodies     40,000 boxes poured into a bin; Rows / Columns / Layers on the geometry change the count
  building        breaking constraints: a concrete building (RBD Material Fracture) glued together, hit by a ball
  bridge_chain    constraints: a hinged rope bridge (revolute) and a swinging chain (spherical)
  wrecking_ball   a ball on a rigid rope (distance joint) swinging into a brick wall
  paddle_mixer    a motor: a paddle driven by a revolute motor churning a tank of balls
  forces          a Fan Force DOP and a POP Force (in a Multiple Solver) pushing a field of boxes
  terrain         a height-field collider: bodies tumbling down hills made with a HeightField SOP
  car_park        every joint type at once: a car towing a trailer through a car park entrance
"""

import argparse
import hashlib
import math
import os
import sys
import time

sys.dont_write_bytecode = True
# This folder, for scene_kit and box3d_samples also when this file is loaded by path (tests/houdini/checks_glue.py).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hou  # noqa: E402

from scene_kit import (  # noqa: E402  (the helpers the scenes share with box3d_samples)
    Pieces, SetupError, build_dops, cooked_geometry, create, create_any, display, fix_constraint_names, ground, mk,
    network_sop, num, packed_pieces, quat_about, rainbow, set_parm, tidy, vec)

# examples/ at the repository root holds only what this builds (the scenes, and cache/), and is git-ignored.
OUT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "examples"))


# ---------------------------------------------------------------------------
# Scenes. Each builds into an empty scene and returns (dopnet, frames).
# ---------------------------------------------------------------------------


def scene_shapes():
    p = Pieces("shape_pieces")
    ground(p)
    kinds = [("boxes", "box"), ("spheres", "sphere"), ("capsules", "capsule"),
             ("dodecahedra", "convexhull"), ("teapots", "convexhull")]
    shapes = [p.shape_box("boxes", (0.8, 0.8, 0.8)), p.shape_sphere("spheres", 0.45),
              p.shape_tube("capsules", 0.25, 1.0), p.shape_platonic("dodecahedra", "4", 0.5),
              p.shape_platonic("teapots", "6", 0.45)]
    for k, ((tag, rep), shape) in enumerate(zip(kinds, shapes)):
        cd = rainbow(k / len(kinds))
        calls = []
        for i in range(12):
            for j in range(12):
                for layer in range(3):
                    x, z = (i - 5.5) * 1.6, (j - 5.5) * 1.6
                    y = 1.0 + (k * 3 + layer) * 1.4
                    spin = quat_about((0.6, 0.0, 0.8), 0.3 * (i + j + layer))  # tumble a little
                    calls.append(mk((x + 0.1 * layer, y, z), f"{tag}{i}_{j}_{layer}", spin, cd, georep=rep))
        p.stream(tag, shape, "\n".join(calls), f"{tag}: bullet_georep '{rep}', fitted to the shape (bullet_autofit 1)")
    path = p.finish()
    dop, _, _ = build_dops("shapes_sim", path)
    note = f"""SHAPES: {p.count - 1} bodies in five shapes fall onto a static floor.

Each piece's bullet_georep point attribute picks its collision shape, as with Bullet:
box, sphere, capsule, convexhull (the dodecahedra and teapots). bullet_autofit 1 fits it
to the piece. The default for a piece without bullet_georep is convexhull.

To see what the solver collides, turn on Show Collision Shapes in the Box3D Solver's
Guide Options: cyan outlines, dimmed when a body is asleep.

Try: change the shapes in shape_pieces, or add a Box3D Data node after 'pieces' in the
DOP network to set friction, bounce or the default shape for pieces without bullet_georep."""
    tidy(240, note, dop)
    return dop, 240


def scene_many_bodies():
    p = Pieces("many_pieces")
    ground(p, 80.0)
    for tag, size, pos in (("wall_left", (1, 6, 62), (-30.5, 3, 0)), ("wall_right", (1, 6, 62), (30.5, 3, 0)),
                           ("wall_back", (62, 6, 1), (0, 3, -30.5)), ("wall_front", (62, 6, 1), (0, 3, 30.5))):
        p.boxes(tag, size, [mk(pos, tag, cd=(0.45, 0.47, 0.5), active=0)], "a static wall of the bin")
    geo = p.geo
    # A "Count" tab on the object, read by the boxes' wrangle with chi("../rows") etc.
    folder = hou.FolderParmTemplate("count", "Count")
    for n, label, default in (("rows", "Rows (x)", 40), ("columns", "Columns (z)", 40), ("layers", "Layers (y)", 25)):
        folder.addParmTemplate(hou.IntParmTemplate(n, label, 1, default_value=(default,), min=1, max=80))
    group = geo.parmTemplateGroup()
    group.append(folder)
    geo.setParmTemplateGroup(group)
    vex = """\
// A lattice of Rows x Columns x Layers boxes, from the spare parameters on this object.
int nx = chi("../rows"), nz = chi("../columns"), ny = chi("../layers");
float pitch = 1.1;
int n = 0;
for (int k = 0; k < ny; k++) for (int j = 0; j < nz; j++) for (int i = 0; i < nx; i++) {
    vector jitter = 0.1 * (vector(rand(set(i, j, k) + 0.5)) - 0.5);
    vector p = set((i - (nx - 1) * 0.5) * pitch, 1.0 + k * pitch, (j - (nz - 1) * 0.5) * pitch) + jitter;
    vector4 q = quaternion(radians(20.0 * rand(n + 0.1)), normalize(vector(rand(n + 0.2)) - 0.5));
    vector cd = hsvtorgb(set(0.55 + 0.35 * k / max(ny - 1, 1), 0.6, 0.95));
    mk(p, sprintf("box%d", n), q, cd, 1, 1000.0, "box");
    n++;
}
"""
    p.stream("boxes", p.shape_box("boxes", (0.8, 0.8, 0.8)), vex, "the boxes")
    path = p.finish()
    dop, _, _ = build_dops("many_bodies_sim", path)
    note = f"""MANY BODIES: {p.count - 5} boxes poured into a bin.

Box3D is built for scenes like this: it runs the contact solver on all your cores and
puts resting bodies to sleep. The first frame builds the bodies; after that each frame
only steps the world.

Try: select many_pieces and change Rows / Columns / Layers (up to 50 x 50 fits the bin)
and see how the time per frame grows. Windows > Performance Monitor shows where the time
goes, stage by stage. On the Box3D Solver: Substeps trades speed for stiffer stacks;
Allow Sleeping is on Box3D Data. With Bullet the same scene is usually much slower."""
    tidy(360, note, dop)
    return dop, 360


def cached_fracture(geo, source, snippet):
    """A File SOP named `fracture` that reads the fractured `source`. The Material Fracture is slow on the
    building, so its output is written once to examples/cache (keyed by what it fractures) and read from then on:
    every build after the first, and the checks that rebuild the scene, skip it. Delete the folder to refracture."""
    key = hashlib.md5(snippet.encode("utf-8")).hexdigest()[:12]
    path = os.path.join(OUT_DIR, "cache", f"building_fracture_{key}.bgeo.sc").replace("\\", "/")
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        build = create(geo, "rbdmaterialfracture", "fracture_build")
        build.setInput(0, source)
        tmp = path + ".tmp.bgeo.sc"
        build.geometry().saveToFile(tmp)
        os.replace(tmp, path)  # a cook that was interrupted leaves no half-written cache
        build.destroy()
    reader = create(geo, "file", "fracture")
    set_parm(reader, ["file"], path)
    return reader


def scene_building(start_x=-32.0):
    p = Pieces("building_pieces")
    ground(p, 80.0)
    geo = p.geo

    # --- The building, from Box SOPs only: per floor a slab and the walls, split into panels.
    # Parts touch but never overlap (overlapping convex hulls would push apart on frame 1).
    floors, width, storey, slab, wall, bays = 8, 12.0, 3.0, 0.3, 0.4, 3
    half = width / 2
    parts = []  # (centre, size)
    for f in range(floors):
        base = f * storey
        parts.append(((0.0, base + slab / 2, 0.0), (width, slab, width)))
        h = storey - slab
        y = base + slab + h / 2
        for b in range(bays):  # walls along x (front / back), full width, split into bays
            length = width / bays
            x = -half + length * (b + 0.5)
            for z in (-half + wall / 2, half - wall / 2):
                parts.append(((x, y, z), (length, h, wall)))
        inner = width - 2 * wall
        for b in range(bays):  # walls along z (sides), between the front and back walls
            length = inner / bays
            z = -half + wall + length * (b + 0.5)
            for x in (-half + wall / 2, half - wall / 2):
                parts.append(((x, y, z), (wall, h, length)))
    parts.append(((0.0, floors * storey + slab / 2, 0.0), (width, slab, width)))  # the roof
    unit = create(geo, "box", "building_unit_box")  # a 1 m polygon box, scaled per part
    set_parm(unit, "size", (1.0, 1.0, 1.0))
    layout = create(geo, "attribwrangle", "building_parts")
    set_parm(layout, "class", 0)
    lines = ["// One point per building part (slab or wall panel): its centre, and its size as 'scale',",
             "// which Copy to Points applies to the unit box."]
    for c, s in parts:
        lines.append(f'{{ int pt = addpoint(0, {vec(c)}); setpointattrib(0, "scale", pt, {vec(s)}); }}')
    set_parm(layout, "snippet", "\n".join(lines))
    copies = create(geo, "copytopoints", "building_copy")
    copies.setInput(0, unit)
    copies.setInput(1, layout)
    set_parm(copies, ["pack", "packgeo"], 0)
    named = create(geo, "attribwrangle", "name_parts")
    named.setInput(0, copies)
    set_parm(named, "class", 1)  # primitives
    set_parm(named, "snippet", "// Each copy is a 6-face box: name every part, so each is fractured on its own.\n"
                               's@name = sprintf("part%d", @primnum / 6);')
    fracture = cached_fracture(geo, named, layout.parm("snippet").eval())
    # Concrete (the default material) breaks every named part into chunks.
    pack = create(geo, "pack", "pack_pieces")
    pack.setInput(0, fracture, 0)
    set_parm(pack, ["packbyname"], 1)
    set_parm(pack, ["nameattribute"], "name")
    # Pack drops every attribute it is not told to transfer, the piece names included.
    set_parm(pack, ["transfer_attributes", "transferattributes", "transferattribs", "transfer"], "name")
    body = create(geo, "attribwrangle", "piece_attributes")
    body.setInput(0, pack)
    set_parm(body, "snippet", """\
// The body attributes every stream has: the packed piece's name on its point, dynamic, concrete.
int prims[] = pointprims(0, @ptnum);
s@name = prim(0, "name", prims[0]);
i@active = 1;
f@density = 2400.0;
s@bullet_georep = "convexhull";
float shade = 0.62 + 0.12 * rand(@ptnum + 0.5);
v@Cd = set(shade, shade * 0.98, shade * 0.93);
""")
    p.merge.setInput(len(p.merge.inputs()), body)

    # --- Glue between touching pieces, across cracks and between parts.
    connect = create(geo, "connectadjacentpieces", "connect_pieces")
    connect.setInput(0, fracture, 0)
    glue_lines = create(geo, "attribwrangle", "glue_name")
    glue_lines.setInput(0, connect)
    set_parm(glue_lines, "class", 1)  # primitives
    constraints = create(geo, "null", "OUT_constraints")
    constraints.setInput(0, glue_lines)

    # --- A heavy ball thrown at the third floor.
    radius, speed, target_y = 1.5, 30.0, 7.5
    flight = (half - start_x) / speed
    start_y = target_y + 0.5 * 9.80665 * flight * flight  # falls to the third floor by the time it arrives
    p.stream("ball", p.shape_sphere("ball", radius, freq=4),
             mk((start_x, start_y, 0.0), "ball", cd=(0.2, 0.2, 0.22), density=8000.0, georep="sphere"),
             "the ball: steel density, so about 110 tonnes")
    path = p.finish(
        "// The ball is thrown at the building (Box3D Packed Object keeps the points' v).\n"
        f'v@v = s@name == "ball" ? {vec((speed, 0, 0))} : {{0, 0, 0}};')

    strength = 3.0e5
    dop, _, names_used = build_dops(
        "building_sim", path, network_path=constraints.path(), keep_velocity=True,
        joints=[("glueconrel", "glue", {"strength": strength})])
    glue_name = names_used.get("glue") or "Glue"
    set_parm(glue_lines, "snippet", "// Every line is a glue bond: constraint_name picks the Glue relationship by "
                                    f'its Data Name.\ns@constraint_name = "{glue_name}";')
    pieces = p.count - 2  # the ground and the ball
    bonds = len(cooked_geometry(constraints).prims())
    note = f"""BUILDING: an {floors}-storey concrete building in {pieces} pieces, held by {bonds}
glue bonds, hit by a 110-tonne ball.

building_pieces makes the floors and wall panels from Box SOPs, RBD Material Fracture
(Concrete) breaks each one into chunks, and Connect Adjacent Pieces joins every pair of
touching chunks with a line: one glue bond each.

Breaking: the Glue Constraint Relationship's Strength ({strength:g}) sets how much impulse
from hits a bond takes before it breaks. At this Strength the building
stops the ball; a lower one lets it plow in. Broken bonds go into the constraint geometry's
'broken' group, so a SOP Solver can react to them.

Try: lower Strength on 'glue' for a weaker building, raise it for one that only dents;
change the ball's speed or height in building_pieces; on 'fracture', change the number of
chunks."""
    tidy(240, note, dop)
    return dop, 240


def bridge_hinges(span=14.0, links=16, length=1.0, height=5.0):
    """Hinge points of a sagging bridge: links+1 points from x = -span/2 to span/2, each
    `length` apart, on a parabola deep enough to take up the slack."""

    def walk(sag):
        pts, x = [(-span / 2, height)], -span / 2
        curve = lambda u: height - sag * (1 - (2 * u / span) ** 2)  # noqa: E731
        for _ in range(links):
            lo, hi = x, x + length
            for _ in range(60):  # the next x one chord length away
                mid = (lo + hi) / 2
                if math.hypot(mid - x, curve(mid) - curve(x)) < length:
                    lo = mid
                else:
                    hi = mid
            x = (lo + hi) / 2
            pts.append((x, curve(x)))
        return pts

    lo, hi = 0.0, span
    for _ in range(60):  # the sag at which the last hinge lands on the far post
        mid = (lo + hi) / 2
        if walk(mid)[-1][0] > span / 2:
            lo = mid
        else:
            hi = mid
    return walk((lo + hi) / 2)


def scene_bridge_chain():
    p = Pieces("bridge_chain_pieces")
    ground(p)
    lines = []
    # Rope bridge: planks hinged to each other and to two static posts (revolute joints turn
    # about the joint's z axis, which runs across the bridge here).
    hinges = bridge_hinges()
    height = hinges[0][1]
    p.boxes("posts", (1.0, height, 2.0),
            [mk((-7.5, height / 2, 0), "post_left", cd=(0.5, 0.35, 0.25), active=0),
             mk((7.5, height / 2, 0), "post_right", cd=(0.5, 0.35, 0.25), active=0)], "static posts")
    planks = []
    for i in range(len(hinges) - 1):
        (x0, y0), (x1, y1) = hinges[i], hinges[i + 1]
        q = quat_about((0, 0, 1), math.atan2(y1 - y0, x1 - x0))
        planks.append(mk(((x0 + x1) / 2, (y0 + y1) / 2, 0), f"plank{i}", q, (0.75, 0.6, 0.4)))
    p.boxes("planks", (0.92, 0.1, 1.6), planks, "bridge planks, laid along the sagging curve")
    names = ["post_left"] + [f"plank{i}" for i in range(len(hinges) - 1)] + ["post_right"]
    for i, (x, y) in enumerate(hinges):
        lines.append((names[i], (x, y, 0), names[i + 1], (x, y, 0), "revolute"))
    # 20 layers of 12 crates (6 across, 2 deep), each layer 0.8 higher and shifted a little
    ncrates = 240
    crates = [mk((-5 + 2 * (i % 6) + 0.1 * ((i * 0.618) % 1), 8 + (i // 12) * 0.8, -0.35 + 0.7 * ((i // 6) % 2)
                  + 0.05 * ((i * 0.37) % 1)), f"crate{i}", quat_about((0, 1, 0), i), rainbow(i / ncrates))
              for i in range(ncrates)]
    p.boxes("crates", (0.6, 0.6, 0.6), crates, "crates dropped onto the bridge")

    # Chain: links joined end to end by spherical joints, pinned to the world at one end,
    # starting straight out so it swings down into a small tower.
    chains, n, pitch = 4, 24, 0.5
    links, tower = [], []
    for c in range(chains):
        pivot = (-6.0, 13.0, -6.0 - 3.0 * c)
        links += [mk((pivot[0] + (i + 0.5) * pitch, pivot[1], pivot[2]), f"link{c}_{i}", cd=(0.8, 0.8, 0.85))
                  for i in range(n)]
        lines.append(("", pivot, f"link{c}_0", pivot, "spherical"))
        for i in range(1, n):
            at = (pivot[0] + i * pitch, pivot[1], pivot[2])
            lines.append((f"link{c}_{i - 1}", at, f"link{c}_{i}", at, "spherical"))
        tower += [mk((-7.5, 0.5 + k * 1.001, pivot[2]), f"tower{c}_{k}", cd=rainbow(0.6 + k * 0.1)) for k in range(3)]
    p.boxes("links", (0.44, 0.12, 0.12), links, "chain links, four chains side by side")
    p.boxes("tower", (1, 1, 1), tower, "a small tower in each chain's path")

    path = p.finish()
    network = network_sop("bridge_chain_network", lines)
    dop, _, names_used = build_dops("bridge_chain_sim", path, network_path=network,
                                    joints=[("box3d_revoluteconrel", "hinges", {}),
                                            ("box3d_sphericalconrel", "ball_joints", {})])
    fix_constraint_names(network, names_used, {"revolute": "hinges", "spherical": "ball_joints"})
    note = f"""BRIDGE AND CHAINS: {p.count - 1} bodies, {len(lines)} of them joined by constraints.

A constraint network (bridge_chain_network) is a polyline per joint: each end names the
piece it holds and sits where the joint is. An empty name pins the piece to the world.
The polyline's constraint_name picks the relationship node in the DOP network:
  - the bridge planks use 'Box3D Revolute' (hinges turning about the joint's z axis);
  - the chain links use 'Box3D Spherical' (ball joints), in four chains side by side.
The crates dropped onto the bridge are free bodies.

Try: on the hinges node, turn on Enable Spring or limits; turn on Breakable on the
ball joints with a low Break Force and the chain snaps when it hits the tower."""
    tidy(240, note, dop)
    return dop, 240


def scene_wrecking_ball():
    p = Pieces("wrecking_pieces")
    ground(p)
    bricks, rows, per_row, depth = [], 30, 24, 3
    for d in range(depth):
        for row in range(rows):
            offset = 0.5 if row % 2 else 0.0
            count = per_row - (1 if row % 2 else 0)
            for i in range(count):
                z = (i - (per_row - 1) / 2) * 1.002 + offset
                y = 0.25 + row * 0.501
                shade = 0.85 + 0.15 * ((i * 7 + row * 3 + d * 2) % 5) / 4
                bricks.append(mk((1.5 + d * 0.501, y, z), f"brick{d}_{row}_{i}",
                                 cd=(0.7 * shade, 0.32 * shade, 0.22 * shade)))
    p.boxes("bricks", (0.5, 0.5, 1.0), bricks, "a running-bond brick wall, three bricks thick")
    pivot, ball_start = (0.0, 14.0, 0.0), (-9.0, 14.0, 0.0)
    p.stream("ball", p.shape_sphere("ball", 1.0, freq=4),
             mk(ball_start, "ball", cd=(0.2, 0.2, 0.22), density=8000.0, georep="sphere"),
             "the wrecking ball: steel density, so about 33 tonnes")
    path = p.finish()
    network = network_sop("wrecking_network", [("", pivot, "ball", ball_start, "distance")])
    dop, _, names_used = build_dops("wrecking_ball_sim", path, network_path=network,
                                    joints=[("box3d_distanceconrel", "rope", {})])
    fix_constraint_names(network, names_used, {"distance": "rope"})
    note = f"""WRECKING BALL: a {len(bricks)}-brick wall and a heavy ball on a rigid rope.

The rope is one 'Box3D Distance' joint between the world (empty name) and the ball:
it keeps the starting distance between its two ends (9 m) and lets the ball swing freely.
The ball's density point attribute (8000) makes it much heavier than the bricks.

Try: on the rope node, turn on Enable Spring for a bungee; change the ball's start
height in wrecking_pieces; add a Box3D Weld relationship and weld the bricks together."""
    tidy(240, note, dop)
    return dop, 240


def scene_paddle_mixer():
    p = Pieces("mixer_pieces")
    ground(p, 60.0)
    # Tank inside: 2 * half_x wide (x), 2 * half_z deep (z), `height` tall. The paddle turns about z
    # through the hub, so it must clear the floor and the side walls: hub height > half its length.
    half_x, half_z, height, wall = 8.0, 5.75, 22.0, 0.3
    paddle = (2 * half_x - 3.0, 0.6, 2 * half_z - 0.5)  # length (x), thickness, depth (z)
    hub = (0.0, paddle[0] / 2 + 0.3, 0.0)
    outer_x = half_x + wall
    tank = [("tank_left", (wall, height, 2 * half_z), (-half_x - wall / 2, height / 2, 0)),
            ("tank_right", (wall, height, 2 * half_z), (half_x + wall / 2, height / 2, 0)),
            ("tank_back", (2 * outer_x, height, wall), (0, height / 2, -half_z - wall / 2)),
            ("tank_front", (2 * outer_x, height, wall), (0, height / 2, half_z + wall / 2))]
    for tag, size, pos in tank:
        p.boxes(tag, size, [mk(pos, tag, cd=(0.55, 0.65, 0.75), active=0)], "a static wall of the tank")
    p.boxes("paddle", paddle, [mk(hub, "paddle", cd=(0.9, 0.5, 0.15))],
            "the paddle, hinged to the world at its middle")
    radius, pitch = 0.17, 0.4
    nx = int((2 * half_x - 0.4) / pitch)
    nz = int((2 * half_z - 0.4) / pitch)
    y0 = hub[1] + paddle[1] / 2 + 0.3
    ny = int((height - 1.0 - y0) / pitch)
    vex = f"""\
// Balls in a lattice above the paddle, filling the tank.
int nx = {nx}, nz = {nz}, ny = {ny};
int n = 0;
for (int k = 0; k < ny; k++) for (int j = 0; j < nz; j++) for (int i = 0; i < nx; i++) {{
    vector p = set((i - (nx - 1) * 0.5) * {num(pitch)}, {num(y0)} + k * {num(pitch)}, (j - (nz - 1) * 0.5) * {num(pitch)});
    p += 0.02 * (vector(rand(n)) - 0.5);
    mk(p, sprintf("ball%d", n), identity, hsvtorgb(set(rand(n + 0.3), 0.6, 0.95)), 1, 1000.0, "sphere");
    n++;
}}
"""
    p.stream("balls", p.shape_sphere("balls", radius, freq=2), vex, "the balls")
    path = p.finish(
        "// See-through front wall (packed primitives draw with their point's Alpha).\n"
        'f@Alpha = s@name == "tank_front" ? 0.15 : 1.0;')
    network = network_sop("mixer_network", [("", hub, "paddle", hub, "revolute")])
    speed = 60.0
    dop, _, names_used = build_dops(
        "paddle_mixer_sim", path, network_path=network,
        joints=[("box3d_revoluteconrel", "motor_hinge",
                 {"enable_motor": 1, "motor_speed": speed, "max_motor_torque": 1.0e9})])
    fix_constraint_names(network, names_used, {"revolute": "motor_hinge"})
    note = f"""PADDLE MIXER: a motor-driven paddle churning {p.count - 6} balls.

The paddle ({paddle[0]:g} m long) is hinged to the world by a 'Box3D Revolute' joint (axis
along z, out of the tank) with Enable Motor on: Motor Speed {speed:g} degrees/second, and Max
Motor Torque high enough to push through hundreds of tonnes of balls. The front wall is
drawn see-through.

Try: on motor_hinge, lower Max Motor Torque until the balls stall the paddle, reverse
Motor Speed, or turn on Enable Limit to make it rock between two angles. In this script,
half_x / half_z / height size the tank and the ball count follows."""
    tidy(480, note, dop)
    return dop, 480


def scene_forces():
    p = Pieces("forces_pieces")
    ground(p, 80.0)
    # Light boxes (density 50, so about 1.4 kg each) on the ground, in a square field.
    rows, size, pitch = 40, 0.3, 0.55
    vex = f"""\
// A {rows} x {rows} field of light boxes resting on the ground, coloured by position.
int n = 0;
for (int j = 0; j < {rows}; j++) for (int i = 0; i < {rows}; i++) {{
    vector p = set((i - ({rows} - 1) * 0.5) * {num(pitch)}, {num(size / 2 + 0.001)}, (j - ({rows} - 1) * 0.5) * {num(pitch)});
    vector cd = hsvtorgb(set(0.08 + 0.5 * j / ({rows} - 1.0), 0.7, 0.95));
    mk(p, sprintf("box%d", n), identity, cd, 1, 50.0, "box");
    n++;
}}
"""
    p.stream("boxes", p.shape_box("boxes", (size, size, size)), vex, "the boxes")
    path = p.finish()

    dop = create(hou.node("/obj"), "dopnet", "forces_sim")
    obj = packed_pieces(dop, path)

    # DOP force: a Fan Force DOP attaches force data to the objects; the Box3D Solver evaluates it
    # at every body's centre of mass, every step.
    half = (rows - 1) * pitch / 2
    fan = create_any(dop, ["fan", "fanforce"], "fan_force")
    fan.setInput(0, obj)
    set_parm(fan, "t", (-half - 4.0, 0.5, 0.0))  # Position
    set_parm(fan, "direction", (1.0, 0.35, 0.0))

    # POP force: a POP Force adds turbulence to each piece's `force` point attribute. In a Multiple
    # Solver it runs first; the Box3D Solver then applies `force` for one step and clears it.
    popforce = create(dop, "popforce", "pop_turbulence")
    set_parm(popforce, "amp", 25.0)
    set_parm(popforce, "swirlsize", 2.0)
    solver = create(dop, "box3d_solver", "box3d_solver")
    set_parm(solver, "forceswake", 1)  # resting boxes wake up when a force reaches them
    multi = create(dop, "multisolver", "pop_then_box3d")
    multi.setInput(0, fan)
    multi.setInput(1, popforce)
    multi.setInput(2, solver)
    out = create(dop, "output", "output")
    out.setInput(0, multi)
    display(out)

    note = f"""FORCES: {p.count - 1} light boxes pushed by a DOP force and a POP force.

fan_force (Fan Force DOP): blows from the left edge of the field. Force DOPs attach force
data to the objects; the Box3D Solver evaluates each at every body's centre of mass, every
step (Use Force DOPs, on the solver's Forces tab).

pop_turbulence (POP Force): adds noise to each box's 'force' point attribute. It runs before
the Box3D Solver in the Multiple Solver; the solver applies 'force' (and 'torque') for one
step and then clears it, so POP forces don't pile up.

Forces Wake Bodies is on, so boxes that have fallen asleep wake when a force reaches them.

Try: change the fan's Direction or strength; raise the POP Force's Amplitude, or set its
Force for a steady push; add a Wind, Vortex or Gravity Force DOP after fan_force; turn off
Use Force DOPs on the solver to see only the POP force."""
    tidy(240, note, dop)
    return dop, 240

def scene_terrain():
    # The terrain: a HeightField SOP whose height layer a Volume Wrangle shapes into hills
    # sloping down towards +x.
    terrain = create(hou.node("/obj"), "geo", "terrain")
    for child in terrain.children():
        child.destroy()
    hf = create(terrain, "heightfield", "heightfield")
    set_parm(hf, "size", (60.0, 60.0))
    set_parm(hf, "gridspacing", 0.4)
    hills = create(terrain, "volumewrangle", "hills")
    hills.setInput(0, hf)
    set_parm(hills, "snippet", """\
// Rolling hills on a slope that falls towards +x (the height layer, in metres).
@height = 3.0 * sin(@P.x * 0.16) * cos(@P.z * 0.13)
        + 1.2 * sin(@P.x * 0.45 + @P.z * 0.3)
        - 0.12 * @P.x;""")
    out_terrain = create(terrain, "null", "OUT_terrain")
    out_terrain.setInput(0, hills)
    display(out_terrain)
    cooked_geometry(out_terrain)

    p = Pieces("terrain_pieces")
    kinds = [("boxes", "box"), ("spheres", "sphere"), ("capsules", "capsule")]
    shapes = [p.shape_box("boxes", (0.8, 0.8, 0.8)), p.shape_sphere("spheres", 0.45),
              p.shape_tube("capsules", 0.25, 1.0)]
    for k, ((tag, rep), shape) in enumerate(zip(kinds, shapes)):
        cd = rainbow(k / len(kinds))
        calls = []
        for i in range(10):
            for j in range(10):
                x, z = -18.0 + (i - 4.5) * 2.0, (j - 4.5) * 2.0
                y = 12.0 + k * 2.5 + 0.3 * ((i + j) % 3)
                spin = quat_about((0.6, 0.0, 0.8), 0.4 * (i + 2 * j + k))
                calls.append(mk((x + 0.6 * k, y, z), f"{tag}{i}_{j}", spin, cd, georep=rep))
        p.stream(tag, shape, "\n".join(calls), f"{tag}: bullet_georep '{rep}'")
    path = p.finish()

    dop = create(hou.node("/obj"), "dopnet", "terrain_sim")
    obj = packed_pieces(dop, path)
    # A Static Object whose geometry is a height field: the Box3D Solver collides the
    # pieces with a Box3D height field made from its height layer.
    static = create(dop, "staticobject", "terrain")
    set_parm(static, "soppath", out_terrain.path())
    merge = create(dop, "merge", "merge")
    merge.setInput(0, obj)
    merge.setInput(1, static)
    solver = create(dop, "box3d_solver", "box3d_solver")
    solver.setInput(0, merge)
    out = create(dop, "output", "output")
    out.setInput(0, solver)
    display(out)

    note = f"""TERRAIN: {p.count} bodies tumble down hills made with a HeightField SOP.

The Static Object 'terrain' reads /obj/terrain, whose geometry is a height field (a 2D
volume named 'height'). The Box3D Solver turns it into a Box3D height field: the same
grid and heights, no polygons needed. A Static Object with polygons instead becomes a
triangle mesh.

The object's transform poses the height field; moving it with an animated OBJ Path moves
the collider without rebuilding it. Shear falls back to the field's triangles. To change
the terrain while the simulation runs, turn on Use Deforming Geometry on 'terrain'.

Try: change the hills wrangle, or add a HeightField Noise or HeightField Erode SOP before
OUT_terrain; add a Box3D Data node after 'terrain' in the DOP network to set its friction
or bounce."""
    tidy(240, note, dop)
    return dop, 240


DATA_NAME = ("dataname", "data_name", "DataName")  # a relationship node's Data Name parameter


def scene_car_park():
    """Every Box3D joint type, each doing the job it is made for. The car drives along +x."""
    p = Pieces("car_park_pieces")
    ground(p, 80.0)
    up = quat_about((1, 0, 0), -math.pi / 2)  # a joint frame whose z axis points up (world y)
    wheel_axes = quat_about((0, 0, 1), math.pi / 2)  # x up (suspension, steering), z along the axle
    backward = quat_about((0, 1, 0), -math.pi / 2)  # z along -x, down the trailer's drawbar
    lines = []  # (piece A, anchor A, piece B, anchor B, constraint_name[, orient])

    # --- The car: a body (one convex hull around chassis and cabin) on four wheels.
    x0 = -14.0  # the car's centre
    body_y, wheel_r, wheel_w = 0.75, 0.34, 0.2
    wheel_y = math.hypot(wheel_r, wheel_w / 2) + 0.002  # the fitted sphere just encloses the tube
    p.stream("car_body", p.shape_parts("car_body", [((4.0, 0.55, 1.7), (0, 0, 0)),
                                                    ((2.2, 0.55, 1.5), (-0.2, 0.55, 0))]),
             mk((x0, body_y, 0), "car_body", cd=(0.75, 0.12, 0.1), density=150.0, georep="convexhull"),
             "the car's body: chassis and cabin, about 900 kg")
    car_wheels = []
    for tag, dx in (("f", 1.35), ("r", -1.35)):
        for side, z in (("l", 0.95), ("r", -0.95)):
            name = f"wheel_{tag}{side}"
            centre = (x0 + dx, wheel_y, z)
            car_wheels.append(mk(centre, name, cd=(0.12, 0.12, 0.13), density=300.0, georep="sphere"))
            lines.append(("car_body", centre, name, centre, "front_wheels" if tag == "f" else "rear_wheels",
                          wheel_axes))
    p.stream("car_wheels", p.shape_wheel("car_wheels", wheel_r, wheel_w), "\n".join(car_wheels),
             "the car's wheels (sphere shapes: they roll smoothly)")

    # Tow bar under the rear bumper, bolted (welded) to the car; the hitch ball is on its end.
    towbar = (x0 - 1.75, 0.42, 0.0)
    hitch = (x0 - 1.9, 0.5, 0.0)
    p.boxes("tow_bar", (0.5, 0.1, 0.1), [mk(towbar, "tow_bar", cd=(0.3, 0.3, 0.32), density=7800.0)],
            "the tow bar, a separate steel part bolted under the car")
    lines.append(("car_body", towbar, "tow_bar", towbar, "bolted"))

    # --- The trailer: bed, side rails and a drawbar whose coupler sits on the hitch ball, tucked
    # under the car's bumper (so the car's and the trailer's shapes overlap there).
    tx = x0 - 4.7  # the bed's centre
    p.stream("trailer", p.shape_parts("trailer", [((2.6, 0.15, 1.6), (0, 0, 0)),
                                                  ((2.6, 0.25, 0.06), (0, 0.2, 0.77)),
                                                  ((2.6, 0.25, 0.06), (0, 0.2, -0.77)),
                                                  ((1.6, 0.12, 0.12), (2.1, -0.22, 0))]),
             mk((tx, 0.75, 0), "trailer", cd=(0.35, 0.45, 0.6), density=120.0, georep="convexhull"),
             "the trailer: bed, rails and drawbar in one convex hull")
    lines.append(("tow_bar", hitch, "trailer", hitch, "hitch_ball", backward))
    lines.append(("car_body", hitch, "trailer", hitch, "no_collide"))
    trailer_r, trailer_w = 0.29, 0.16
    trailer_y = math.hypot(trailer_r, trailer_w / 2) + 0.002
    trailer_wheels = []
    for side, z in (("l", 0.95), ("r", -0.95)):
        centre = (tx - 0.2, trailer_y, z)
        trailer_wheels.append(mk(centre, f"trailer_wheel_{side}", cd=(0.12, 0.12, 0.13), density=300.0,
                                 georep="sphere"))
        lines.append(("trailer", centre, f"trailer_wheel_{side}", centre, "trailer_wheels", wheel_axes))
    p.stream("trailer_wheels", p.shape_wheel("trailer_wheels", trailer_r, trailer_w), "\n".join(trailer_wheels),
             "the trailer's wheels")
    # A tall crate strapped down on the bed (welded): too tall for the height bar.
    crate_size = (1.3, 1.8, 1.3)
    bed_top = 0.825
    p.boxes("crate", crate_size, [mk((tx, bed_top + crate_size[1] / 2 + 0.002, 0), "crate", cd=(0.7, 0.52, 0.3),
                                     density=100.0)], "the load, strapped to the trailer")
    lines.append(("trailer", (tx, bed_top, 0), "crate", (tx, bed_top, 0), "bolted"))

    # --- A camera drone following the car: a motor joint to the car body holds it above the
    # trailer with capped force (its thrust); a parallel joint to the world keeps it level (its
    # flight controller) and leaves it free to turn about the vertical.
    drone = (x0 - 4.0, 5.0, 0.0)
    p.stream("drone", p.shape_parts("drone", [((0.6, 0.12, 0.6), (0, 0, 0))]
                                    + [((0.28, 0.03, 0.28), (sx * 0.3, 0.08, sz * 0.3))
                                       for sx in (-1, 1) for sz in (-1, 1)]),
             mk(drone, "drone", cd=(0.15, 0.15, 0.17), density=70.0, georep="box"),
             "a camera drone, about 10 kg")
    lines.append(("car_body", drone, "drone", drone, "drone_follow"))
    lines.append(("", drone, "drone", drone, "drone_level", up))

    # --- The entrance: two spring-loaded swing gates on static posts (revolute, axis up).
    xg, post_z = -8.0, 1.7
    p.boxes("gate_posts", (0.2, 1.5, 0.2),
            [mk((xg, 0.75, s * post_z), f"gate_post_{n}", cd=(0.5, 0.5, 0.52), active=0)
             for n, s in (("l", 1), ("r", -1))], "static gate posts")
    p.boxes("gates", (0.12, 1.0, 1.55),
            [mk((xg, 0.75, s * 0.85), f"gate_{n}", cd=(0.92, 0.92, 0.9), density=150.0)
             for n, s in (("l", 1), ("r", -1))], "the gate leaves")
    for n, s in (("l", 1), ("r", -1)):
        hinge = (xg, 0.75, s * 1.62)
        lines.append((f"gate_post_{n}", hinge, f"gate_{n}", hinge, "gate_hinges", up))

    # --- A height bar hung on two chains from a static gantry (distance joints used as chains).
    xc, beam_y, bar_y = 0.0, 3.5, 2.35
    p.boxes("gantry_posts", (0.25, beam_y - 0.15, 0.25),
            [mk((xc, (beam_y - 0.15) / 2, s * 2.3), f"gantry_post_{n}", cd=(0.5, 0.5, 0.52), active=0)
             for n, s in (("l", 1), ("r", -1))], "static gantry posts")
    p.boxes("gantry_beam", (0.3, 0.3, 4.85), [mk((xc, beam_y, 0), "gantry", cd=(0.5, 0.5, 0.52), active=0)],
            "the static gantry beam")
    p.boxes("height_bar", (0.12, 0.12, 3.2), [mk((xc, bar_y, 0), "height_bar", cd=(0.95, 0.8, 0.1), density=300.0)],
            "the hanging height bar: the car fits under it, the crate does not")
    chain = (beam_y - 0.15) - (bar_y + 0.06)
    for z in (1.3, -1.3):
        lines.append(("gantry", (xc, beam_y - 0.15, z), "height_bar", (xc, bar_y + 0.06, z), "chains"))

    # --- Flexible lane bollards on the centre line: a sprung ball joint at the base (spherical)
    # lets each bend any way and stands it back up; the twist limit keeps it from spinning on its
    # base, as a bolted-down one cannot (twisted far round, the spring's way back is ambiguous).
    bollards = []
    for i, xb in enumerate((5.0, 6.5, 8.0)):
        bollards.append(mk((xb, 0.46, 0), f"bollard{i}", cd=(1.0, 0.45, 0.05), density=400.0, georep="capsule"))
        base = (xb, 0.09, 0)  # the centre of the capsule's bottom cap: it pivots without digging in
        lines.append(("", base, f"bollard{i}", base, "bollard_base", up))
    p.stream("bollards", p.shape_tube("bollards", 0.08, 0.9), "\n".join(bollards), "flexible bollards")

    # --- A spring buffer stop at the end of the road: a plate sliding along x (prismatic).
    xs = 16.15
    p.boxes("buffer_block", (0.6, 1.4, 3.0), [mk((xs + 1.45, 0.7, 0), "buffer_block", cd=(0.45, 0.45, 0.47),
                                                 active=0)], "the static concrete block behind the buffer")
    p.boxes("buffer_plate", (0.3, 1.0, 2.6), [mk((xs, 0.7, 0), "buffer_plate", cd=(0.95, 0.75, 0.1),
                                                 density=300.0)], "the buffer plate")
    lines.append(("buffer_block", (xs, 0.7, 0), "buffer_plate", (xs, 0.7, 0), "buffer_slide"))

    path = p.finish("""\
// Grip: rubber wheels on a rough road, slippery plastic bollards, everything else in between.
// Little bounce (Box3D Data's default is 0.5).
f@friction = 0.6;
if (s@name == "ground") f@friction = 1.0;
if (match("*wheel*", s@name)) f@friction = 1.2;
if (match("bollard*", s@name)) f@friction = 0.2;
f@bounce = 0.1;""")
    network = network_sop("car_park_network", lines)

    def rel(kind, name, comment, **parms):
        parms[DATA_NAME] = name  # constraint_name in the network is the node's name
        return (f"box3d_{kind}conrel", name, parms, comment)

    suspension = dict(enable_suspension_spring=1, suspension_stiffness=5.0, suspension_damping_ratio=0.7,
                      enable_suspension_limit=1, lower_suspension_limit=-0.15, upper_suspension_limit=0.15)
    joints = [
        rel("wheel", "front_wheels", "Wheel: suspension + steering (held straight)",
            **suspension, enable_steering=1, steering_stiffness=10.0, steering_damping_ratio=0.7,
            target_steering_angle=0.0, max_steering_torque=500.0, enable_steering_limit=1,
            lower_steering_limit=-35.0, upper_steering_limit=35.0),
        rel("wheel", "rear_wheels", "Wheel: suspension + spin motor (rear-wheel drive)",
            **suspension, enable_spin_motor=1, spin_speed=-450.0, max_spin_torque=600.0),
        rel("wheel", "trailer_wheels", "Wheel: suspension only, free rolling", **suspension),
        rel("weld", "bolted", "Weld: tow bar bolted to the car, crate strapped to the trailer"),
        rel("spherical", "hitch_ball", "Spherical: tow ball, cone and twist limits stop jackknifing",
            enable_cone_limit=1, cone_angle=60.0, enable_twist_limit=1, lower_twist_angle=-15.0,
            upper_twist_angle=15.0),
        rel("filter", "no_collide", "Filter: car body and trailer overlap at the hitch, never collide"),
        rel("motor", "drone_follow", "Motor: drone holds its place over the trailer, capped thrust",
            linear_stiffness=1.0, linear_damping_ratio=0.5, angular_stiffness=0.0, angular_damping_ratio=0.0,
            max_spring_force=200.0, max_spring_torque=0.0, max_velocity_force=0.0, max_velocity_torque=0.0),
        rel("parallel", "drone_level", "Parallel: keeps the drone level, free to turn about the vertical",
            stiffness=2.0, damping_ratio=1.0),
        rel("revolute", "gate_hinges", "Revolute: self-closing spring hinges",
            enable_spring=1, stiffness=1.0, damping_ratio=0.3, target_angle=0.0,
            enable_limit=1, lower_angle=-100.0, upper_angle=100.0),
        rel("distance", "chains", "Distance: chains, slack up to their length",
            enable_spring=1, stiffness=0.0, enable_limit=1, min_length=0.0, limit_max_length=1,
            max_length=chain),
        rel("spherical", "bollard_base", "Spherical: sprung rubber base, bends up to 85 degrees, no spinning",
            enable_spring=1, stiffness=3.0, damping_ratio=0.3, enable_cone_limit=1, cone_angle=85.0,
            enable_twist_limit=1, lower_twist_angle=-10.0, upper_twist_angle=10.0),
        rel("prismatic", "buffer_slide", "Prismatic: buffer plate slides along the road on a spring",
            enable_spring=1, stiffness=2.0, damping_ratio=0.5, target_translation=0.0,
            enable_limit=1, lower_translation=-0.05, upper_translation=1.0),
    ]
    dop, _, names_used = build_dops("car_park_sim", path, network_path=network, joints=joints)
    wrong = {k: v for k, v in names_used.items() if v != k}
    if wrong:
        raise SetupError(f"relationship Data Names did not take: {wrong}")

    note = f"""CAR PARK: every Box3D joint type, each doing its real-world job. A car towing a
trailer drives through a car park entrance and into a buffer stop.

  Wheel      the car's and trailer's wheels: suspension spring and limits; the rear
             wheels have a spin motor (the drive), the front ones steering.
  Weld       parts fixed together: the tow bar bolted to the car, the crate strapped
             to the trailer.
  Spherical  the tow ball (cone and twist limits stop the trailer jackknifing or
             rolling over), and the rubber base of each flexible bollard: it bends
             any way when the car runs it over and its spring stands it back up.
  Filter     the trailer's coupler tucks under the car's bumper: their shapes overlap,
             so a filter joint stops them colliding (no other effect).
  Motor      the camera drone: a spring holds it at its place over the trailer, with
             Max Spring Force (200 N) as its thrust, so it lags and sways.
  Parallel   the drone's flight controller: keeps it level, free to turn about the
             vertical.
  Revolute   the entrance gates: spring hinges that swing shut behind the trailer.
  Distance   the chains of the height bar: slack (Enable Spring with Stiffness 0) up
             to Max Length ({chain:.2f} m). The car fits under; the crate knocks the bar.
  Prismatic  the buffer stop: the plate slides along the road only, on a spring,
             within its limits.

car_park_network holds one line per joint; its constraint_name is the name of the
relationship node in car_park_sim, and its orient sets the joint's axes (a hinge
turns about z, a slider moves along x, a wheel's suspension runs along x).

Try: delete no_collide (the overlap shoves the car's tail up and it barely moves);
on front_wheels set Target Steering Angle to 10; on drone_follow drop Max Spring
Force below the drone's weight (about 80 N); turn Enable Spring off on chains (rigid
rods); on rear_wheels make Spin Speed more negative to drive faster."""
    tidy(360, note, dop, note_size=(11, 9))
    return dop, 360


SCENES = {
    "shapes": scene_shapes,
    "many_bodies": scene_many_bodies,
    "building": scene_building,
    "bridge_chain": scene_bridge_chain,
    "wrecking_ball": scene_wrecking_ball,
    "paddle_mixer": scene_paddle_mixer,
    "forces": scene_forces,
    "terrain": scene_terrain,
    "car_park": scene_car_park,
}


def check(dop, frames):
    """Simulates the scene and reports timing and solver messages. dop: the DOP network, or a solver SOP."""
    if isinstance(dop, hou.SopNode):
        def advance():
            dop.geometry()
        nodes = [dop]
    else:
        def advance():
            dop.simulation().objects()
        nodes = dop.children()
    start = time.time()
    hou.setFrame(1)
    advance()
    first = time.time() - start
    start = time.time()
    for f in range(2, frames + 1):
        hou.setFrame(f)
        advance()
    per = (time.time() - start) / max(frames - 1, 1)
    msgs = []
    for node in nodes:
        for get in ("errors", "warnings"):
            try:
                msgs += getattr(node, get)()
            except hou.Error:
                pass
    hou.setFrame(1)
    return f"first frame {first:.2f} s, then {per * 1000:.1f} ms per frame" + (
        "; messages: " + " | ".join(msgs) if msgs else "")


def jobs(args, ap):
    """(label, output path, build) for each scene to make."""
    if args.samples:
        import box3d_samples  # the Box3D sample ports; imported here, as it needs hou and the plugin's nodes
        chosen = [s for s in box3d_samples.SAMPLES.values()
                  if not args.scenes or s.key in args.scenes or s.key.split("/")[0] in args.scenes]
        known = set(box3d_samples.SAMPLES) | {k.split("/")[0] for k in box3d_samples.SAMPLES}
        unknown = [n for n in args.scenes if n not in known]
        if unknown:
            ap.error(f"unknown sample(s) or categories {unknown}; choose from {sorted(known)}")

        def runner(s):
            return lambda: (box3d_samples.start_scene(), s.build(s))[1]

        return [(s.key, os.path.join(OUT_DIR, *s.key.split("/")) + ".hip", runner(s))
                for s in chosen]
    unknown = [s for s in args.scenes if s not in SCENES]
    if unknown:
        ap.error(f"unknown scene(s) {unknown}; choose from {list(SCENES)}")
    return [(name, os.path.join(OUT_DIR, f"{number:02d}_{name}.hip"), build)
            for number, (name, build) in enumerate(SCENES.items(), 1) if not args.scenes or name in args.scenes]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", nargs="*", help="scene names (default: all); with --samples, categories or "
                                              "category/sample names")
    ap.add_argument("--samples", action="store_true", help="build the Box3D sample ports instead of the scenes")
    ap.add_argument("--check", action="store_true", help="simulate each scene and print timings")
    args = ap.parse_args()
    failed = False
    for name, path, build in jobs(args, ap):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        hou.hipFile.clear(suppress_save_prompt=True)
        try:
            dop, frames = build()
        except SetupError as e:
            # Saved anyway, so the broken node can be opened and looked at.
            bad = path.replace(".hip", "_FAILED.hip")
            hou.hipFile.save(bad.replace("\\", "/"))
            print(f"  FAILED {name} (saved as {os.path.relpath(bad)}): {e}")
            failed = True
            continue
        hou.hipFile.save(path.replace("\\", "/"))
        line = f"  saved  {os.path.relpath(path)}"
        if args.check:
            line += "  (" + check(dop, frames) + ")"
        print(line)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
