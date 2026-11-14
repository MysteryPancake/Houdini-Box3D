"""Behaviour checks added after the first full pass: each joint node toggle doing what it says,
friction / bounce mixing, negative "never" values, determinism (repeat runs, thread counts,
substeps and frame rate), contact behaviour (slopes, bounce heights, CCD, sleep, rolling
resistance, shape types), bad input and rewiring mid-simulation, a large-scene timing baseline,
and constraints across objects, to the world, to a kinematic piece and keyed mid-simulation."""

import math
import time

import hou

from b3dtest import (
    Result,
    SetupError,
    box3d_data,
    box3d_solver,
    bullet_solver,
    close,
    constraint_lines,
    create,
    display,
    finish,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    point_attributes,
    set_expression,
    set_menu_by_label,
    set_parm,
    sop_solver,
    step,
    vex_value,
)
from checks_constraints import FACE, broken_count, hanging_pair
from checks_solver import stack_scene

G = 9.80665
GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}
MIXING = ("Geometric Mean", "Minimum", "Maximum", "Average", "Multiply")


def mix(rule, a, b):
    return {"Geometric Mean": math.sqrt(a * b), "Minimum": min(a, b), "Maximum": max(a, b),
            "Average": 0.5 * (a + b), "Multiply": a * b}[rule]


def box3d_messages(node):
    return sorted({m.strip() for m in node_messages(node) if "Box3D" in m})


# ---------------------------------------------------------------------------
# Joint node toggles: limit, motor, prismatic limit, spring rest length, Breakable
# ---------------------------------------------------------------------------


def swing_angle(pos):
    """Angle (degrees) of the hanging piece's centroid about the anchor FACE, from +x: 0 at the
    start, -90 hanging straight down."""
    return math.degrees(math.atan2(pos[1] - FACE[1], pos[0] - FACE[0]))


def test_joint_revolute_limit():
    """A revolute joint (hinge about z) holding a piece out sideways: with Enable Limit and the
    node's default limits (+-45) it stops swinging at 45 degrees; without, it swings down past 80."""
    rows, ok = [], True
    for limit in (0, 1):
        dop, names = hanging_pair(f"revolute_limit{limit}", None,
                                  conrels=[("box3d_revoluteconrel", {"enable_limit": limit})])
        joint = dop.node("joint0")
        lo, hi = joint.parm("lower_angle").eval(), joint.parm("upper_angle").eval()
        track = step(dop, 72, lambda sim, f: swing_angle(packed_positions(sim, "pieces")[names[1]]))
        lowest = min(track.values())
        if limit:
            good = close(lowest, -45.0, 5.0)
            rows.append(f"limit on (node limits {lo:g} to {hi:g}): lowest angle {lowest:.1f} (expected -45)")
        else:
            good = lowest < -80.0
            rows.append(f"limit off: lowest angle {lowest:.1f} (expected below -80)")
        ok = ok and good
    return [Result("joint_revolute_limit", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_joint_revolute_motor():
    """A revolute motor at 90 deg/s (the node's default Max Motor Torque) spins a 1 kg piece at
    pi/2 rad/s after 2 s, no gravity; with the motor off it stays still."""
    rows, ok = [], True
    for motor in (0, 1):
        dop, names = hanging_pair(f"revolute_motor{motor}", None, gravity=(0, 0, 0),
                                  conrels=[("box3d_revoluteconrel", {"enable_motor": motor, "motor_speed": 90.0})],
                                  packed_parms={"computemass": 0, "mass": 1.0})
        torque = dop.node("joint0").parm("max_motor_torque").eval()
        track = step(dop, 48, lambda sim, f: point_attributes(sim, "pieces", names[1], ["w"])["w"])
        wz = abs(track[48][2])
        if motor:
            good = close(wz, math.pi / 2, 0.05 * math.pi / 2)
            rows.append(f"motor on (Max Motor Torque {torque:g}): |w.z| {wz:.3f} rad/s (expected 1.571)")
        else:
            good = wz < 0.01
            rows.append(f"motor off: |w.z| {wz:.3f} rad/s (expected 0)")
        ok = ok and good
    return [Result("joint_revolute_motor", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_joint_prismatic_limit():
    """A prismatic joint (slide along x) with gravity along +x: with Enable Limit (-0.5 to 0.5) the
    piece slides 0.5 and stops; without, it slides about 10 in 2 s. It stays at y = 2 either way."""
    rows, ok = [], True
    for limit in (0, 1):
        dop, names = hanging_pair(f"prismatic_limit{limit}", None, gravity=(5, 0, 0),
                                  conrels=[("box3d_prismaticconrel", {"enable_limit": limit,
                                                                      "lower_translation": -0.5,
                                                                      "upper_translation": 0.5})])
        track = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
        dx, y = track[48][0] - 1.0, track[48][1]
        good = close(y, 2.0, 0.02) and (close(dx, 0.5, 0.05) if limit else dx > 5.0)
        rows.append(f"limit {'on' if limit else 'off'}: slid {dx:.3f} (expected {'0.5' if limit else 'about 10'}), "
                    f"y {y:.3f}")
        ok = ok and good
    return [Result("joint_prismatic_limit", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_joint_spring_rest_length():
    """A distance joint between two centroids 1 apart, with Enable Spring (node defaults) and Set
    Rest Length 2, no gravity: the free piece settles 2 from the fixed one. With Set Rest Length off
    it keeps the starting 1."""
    rows, ok = [], True
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1}}]
    for set_rest in (0, 1):
        dop, names = hanging_pair(f"spring_rest{set_rest}", None, gravity=(0, 0, 0), pieces=pieces,
                                  anchors=((0, 2, 0), (1, 2, 0)),
                                  conrels=[("box3d_distanceconrel", {"enable_spring": 1, "set_rest_length": set_rest,
                                                                     "restlength": 2.0})],
                                  packed_parms={"computemass": 0, "mass": 1.0})
        joint = dop.node("joint0")
        hz, ratio = joint.parm("stiffness").eval(), joint.parm("damping_ratio").eval()
        track = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))
        d = (track[72][names[1]] - track[72][names[0]]).length()
        want = 2.0 if set_rest else 1.0
        good = close(d, want, 0.03)
        rows.append(f"Set Rest Length {'on' if set_rest else 'off'}: distance {d:.3f} after 3 s (expected {want:g}; "
                    f"stiffness {hz:g} Hz, damping ratio {ratio:g})")
        ok = ok and good
    return [Result("joint_spring_rest_length", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_joint_breakable():
    """A weld with Breakable on and the node's default thresholds holds a 1 kg piece and breaks
    under a 1000 kg one; with Breakable off the 1000 kg piece holds."""
    rows, ok = [], True
    for mass, breakable, want in ((1.0, 1, "held"), (1000.0, 1, "broke"), (1000.0, 0, "held")):
        dop, names = hanging_pair(f"breakable_{int(mass)}_{breakable}", "weld",
                                  conrels=[("box3d_weldconrel", {"breakable": breakable})],
                                  packed_parms={"computemass": 0, "mass": mass})
        joint = dop.node("joint0")
        force, torque = joint.parm("force_threshold").eval(), joint.parm("torque_threshold").eval()
        y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
        got = "held" if y > 1.9 else "broke"
        ok = ok and got == want
        rows.append(f"{mass:g} kg, Breakable {'on' if breakable else 'off'}: {got} (expected {want})")
    return [Result("joint_breakable", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + f". Default thresholds: force {force:g}, torque {torque:g}")]


# ---------------------------------------------------------------------------
# Negative "never" values, as older scenes and primitive attributes use them
# ---------------------------------------------------------------------------


def test_negative_never_values():
    """Break Force / Torque of -1 with Breakable on (the old way to say never) keep a weld holding a
    1000 kg piece; the parameters must also accept -1 rather than clamping it to 0 (which breaks at
    once). Also reports what an old scene with a real Break Force does now (Breakable loads off)."""
    results = []
    dop, names = hanging_pair("never_values", "weld",
                              conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": -1.0,
                                                             "torque_threshold": -1.0})],
                              packed_parms={"computemass": 0, "mass": 1000.0})
    joint = dop.node("joint0")
    read_back = (joint.parm("force_threshold").eval(), joint.parm("torque_threshold").eval())
    y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
    ok = y > 1.9 and read_back == (-1.0, -1.0)
    results.append(Result("negative_never_values", Result.PASS if ok else Result.FAIL,
                          f"Breakable on, thresholds set to -1: read back as {read_back}; the 1000 kg piece "
                          f"{'held' if y > 1.9 else 'fell'} (y={y:.2f})"
                          + ("" if read_back == (-1.0, -1.0) else
                             ". The parameter clamps -1: an old scene saved with -1 would load as 0")))

    dop, names = hanging_pair("never_values_old", "weld",
                              conrels=[("box3d_weldconrel", {"breakable": 0, "force_threshold": 100.0,
                                                             "torque_threshold": 100.0})],
                              packed_parms={"computemass": 0, "mass": 1000.0})
    y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
    results.append(Result("negative_never_values", Result.INFO,
                          f"an old scene with Break Force 100 (Breakable loads off): the 1000 kg piece "
                          f"{'held' if y > 1.9 else 'fell'} (before the toggle it would have broken)"))
    return results


# ---------------------------------------------------------------------------
# Friction and bounce mixing (menu labels -> rules)
# ---------------------------------------------------------------------------


def slide_distance(name, mu_box, mu_ground, mixing=None, v0=3.0, solver_kind="box3d"):
    """A box sliding at v0 on the ground: how far it goes before stopping (Box3D, or Bullet)."""
    pieces = [dict(GROUND, attrs={"active": 0, "friction": mu_ground, "bounce": 0.0}),
              {"size": (1, 1, 1), "center": (0, 0.5, 0),
               "attrs": {"active": 1, "friction": mu_box, "bounce": 0.0, "v": (v0, 0.0, 0.0)}}]
    path, names = packed_boxes(name + "_pieces", pieces)
    dop = new_dopnet(name)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    if solver_kind == "bullet":
        solver = bullet_solver(dop, obj)
    else:
        solver = box3d_solver(dop, obj)
        if mixing:
            set_menu_by_label(solver, ["frictionmixing"], mixing)
    finish(dop, solver)
    track = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
    return track[72][0], track[72][1]


def test_friction_mixing():
    """A box (friction 0.2) sliding at 3 m/s on ground with friction 0.8, under each Friction Mixing
    menu item: the stopping distance v^2 / (2 mu g) gives the mixed friction."""
    rows, ok = [], True
    for rule in MIXING:
        mu = mix(rule, 0.2, 0.8)
        d, y = slide_distance("friction_" + rule.split()[0].lower(), 0.2, 0.8, rule)
        want = 9.0 / (2 * mu * G)
        mu_got = 9.0 / (2 * G * d) if d > 1e-6 else float("inf")
        good = close(d, want, 0.15 * want + 0.03)
        ok = ok and good
        rows.append(f"{rule}: slid {d:.3f} (expected {want:.3f}, friction {mu:.2f}; measured {mu_got:.2f})")
    return [Result("friction_mixing", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def bounce_drop(name, e_box, e_ground, mixing=None, height=2.0, solver_kind="box3d"):
    """A box dropped with its bottom `height` above the ground (Box3D, or Bullet). Returns the
    measured restitution sqrt(rebound peak / height) and the track."""
    pieces = [dict(GROUND, attrs={"active": 0, "bounce": e_ground, "friction": 0.5}),
              {"size": (1, 1, 1), "center": (0, 0.5 + height, 0), "attrs": {"active": 1, "bounce": e_box,
                                                                             "friction": 0.5}}]
    path, names = packed_boxes(name + "_pieces", pieces)
    dop = new_dopnet(name)
    obj = packed_object(dop, "pieces", path)
    if solver_kind == "bullet":
        solver = bullet_solver(dop, obj)
    else:
        solver = box3d_solver(dop, obj)
        set_parm(solver, ["bouncethreshold"], 0.1)
        if mixing:
            set_menu_by_label(solver, ["restitutionmixing"], mixing)
    finish(dop, solver)
    track = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1] - 0.5)
    frames = sorted(track)
    impact = next((f for f in frames[1:] if track[f] > track[f - 1] + 1e-5), None)
    if impact is None:
        return 0.0, track
    peak = max(track[f] for f in frames if impact <= f <= impact + 40)
    return math.sqrt(max(peak, 0.0) / height), track


def test_bounce_mixing():
    """A box (bounce 0.2) dropped 2 m onto ground with bounce 0.8, under each Bounce Mixing menu
    item (Bounce Threshold 0.1): the rebound height gives the mixed bounce."""
    rows, ok = [], True
    for rule in MIXING:
        e = mix(rule, 0.2, 0.8)
        got, _ = bounce_drop("bounce_" + rule.split()[0].lower(), 0.2, 0.8, rule)
        good = close(got, e, 0.05)
        ok = ok and good
        rows.append(f"{rule}: measured bounce {got:.3f} (expected {e:.3f})")
    return [Result("bounce_mixing", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_bullet_mixing():
    """How Houdini's Bullet combines two bodies' friction and bounce, to choose Box3D's mixing
    defaults: a box (friction 0.2, bounce 0.2) sliding on and dropped onto ground with 0.8, in
    Bullet; the measured values against each Box3D mixing rule. INFO."""
    rules = ("Geometric Mean", "Minimum", "Maximum", "Average", "Multiply")
    v0 = 3.0
    x, _ = slide_distance("bullet_mix_slide", 0.2, 0.8, v0=v0, solver_kind="bullet")
    mu = v0 * v0 / (2 * G * x) if x > 1e-6 else float("inf")
    e, _ = bounce_drop("bullet_mix_bounce", 0.2, 0.8, solver_kind="bullet")

    def nearest(measured):
        return min(rules, key=lambda r: abs(mix(r, 0.2, 0.8) - measured))

    table = ", ".join(f"{r} {mix(r, 0.2, 0.8):.2f}" for r in rules)
    return [Result("bullet_mixing", Result.INFO,
                   f"Bullet friction: slid {x:.3f} from 3 m/s, so friction {mu:.3f}, nearest rule {nearest(mu)}; "
                   f"bounce {e:.3f}, nearest rule {nearest(e)} (0.2 with 0.8 under each rule: {table})")]


def test_bounce_heights():
    """Boxes with bounce 0.3, 0.6 and 0.9 dropped 2 m onto ground with bounce 1 (default mixing,
    Minimum, so the box's own bounce is used): each rebounds to bounce^2 x 2 m."""
    rows, ok = [], True
    for e in (0.3, 0.6, 0.9):
        got, _ = bounce_drop(f"bounce_height_{int(e * 10)}", e, 1.0)
        good = close(got, e, 0.05)
        ok = ok and good
        rows.append(f"bounce {e:g}: rebounded {got * got * 2:.3f} m, measured bounce {got:.3f}")
    return [Result("bounce_heights", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------


def pile_pieces(count=40):
    """Boxes in a loose, tilted column over the ground, the same every call."""
    pieces = [GROUND]
    for i in range(count):
        pieces.append({"size": (0.6, 0.4, 0.8),
                       "center": ((i % 4) * 0.7 - 1.0, 1.0 + (i // 4) * 0.6, ((i * 7) % 5) * 0.15 - 0.3),
                       "rotate": ((i * 37) % 90, (i * 53) % 90, (i * 19) % 90)})
    return pieces


def pile_state(sim):
    obj = sim.findObject("pieces")
    geo = obj.geometry()
    out = {}
    for pr in geo.prims():
        pts = pr.points()
        if not pts:
            continue
        m = pr.intrinsicValue("packedfulltransform")
        out[pts[0].attribValue("name")] = (pts[0].position(), tuple(m))
    return out


def state_difference(a, b):
    worst = 0.0
    for k in a:
        if k not in b:
            return float("inf")
        worst = max(worst, (a[k][0] - b[k][0]).length(), max(abs(x - y) for x, y in zip(a[k][1], b[k][1])))
    return worst


def pile_run(name, path, solver_parms=None, frames=48):
    dop = new_dopnet(name)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path))
    for parm, value in (solver_parms or {}).items():
        set_parm(solver, [parm], value)
    finish(dop, solver)
    return dop, step(dop, frames, lambda sim, f: pile_state(sim) if f == frames else None)[frames]


def test_determinism_repeat():
    """The same 40-box pile run twice in one session (two DOP networks), and the first again after
    Reset Simulation: all three end bit for bit the same."""
    path, _ = packed_boxes("repeat_pieces", pile_pieces())
    dop_a, a = pile_run("repeat_a", path)
    _, b = pile_run("repeat_b", path)
    try:
        dop_a.parm("resimulate").pressButton()
    except (AttributeError, hou.Error):
        raise SetupError("the DOP network has no Reset Simulation (resimulate) button")
    hou.setFrame(1)
    c = step(dop_a, 48, lambda sim, f: pile_state(sim) if f == 48 else None)[48]
    d_ab, d_ac = state_difference(a, b), state_difference(a, c)
    ok = d_ab == 0.0 and d_ac == 0.0
    return [Result("determinism_repeat", Result.PASS if ok else Result.FAIL,
                   f"frame 48: second network differs by {d_ab:.2e}, rerun after Reset Simulation by {d_ac:.2e} "
                   "(expected 0)")]


def test_determinism_threads():
    """The 40-box pile with 1 worker thread, 4 threads and Use All Threads: identical results (if
    not, multithreaded simulations won't cache or resimulate reliably)."""
    path, _ = packed_boxes("threads_det_pieces", pile_pieces())
    _, one = pile_run("threads_1", path, {"autothreads": 0, "threads": 1})
    _, four = pile_run("threads_4", path, {"autothreads": 0, "threads": 4})
    _, every = pile_run("threads_all", path, {"autothreads": 1})
    d4, dall = state_difference(one, four), state_difference(one, every)
    ok = d4 == 0.0 and dall == 0.0
    return [Result("determinism_threads", Result.PASS if ok else Result.FAIL,
                   f"frame 48 compared with 1 thread: 4 threads differ by {d4:.2e}, all threads by {dall:.2e} "
                   "(expected 0)")]


def test_substeps_fps():
    """A three-box stack rests 2.5 tall with Substeps 1 and 4 at 24 fps, and with Substeps 4 at
    48 fps, without jitter."""
    rows, ok = [], True
    for fps, substeps in ((24, 1), (24, 4), (48, 4)):
        hou.setFps(fps)
        dop, names = stack_scene(f"substeps_{fps}_{substeps}")
        solver = dop.node("box3d_solver")
        set_parm(solver, ["substeps"], substeps)
        frames = 4 * fps
        track = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[3]])
        rest = track[frames][1]
        jitter = max((track[f] - track[f - 1]).length() for f in range(frames - fps // 2, frames + 1)) * fps
        # Box3D caps contact stiffness at 1/8 of the substep rate (3 Hz at 24 fps x 1), so the stack
        # sinks into soft contacts there; it only has to stay stable and not sink far.
        soft = fps * substeps < 48
        good = close(rest, 2.5, 0.25 if soft else 0.03) and jitter < 0.05
        ok = ok and good
        rows.append(f"{fps} fps, Substeps {substeps}: top box at {rest:.3f} (2.5 = stacked"
                    f"{', soft contacts expected' if soft else ''}), jitter {jitter:.4f}/s")
    hou.setFps(24)
    return [Result("substeps_fps", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Contact behaviour
# ---------------------------------------------------------------------------


def test_friction_slope():
    """A box (friction 0.5) on a ramp (friction 1): under the default Multiply mixing the contact
    has friction 0.5, so it should start sliding above atan 0.5 = 26.6 degrees: it stays put at
    22 degrees and slides at 31."""
    rows, ok = [], True
    for angle, should_slide in ((22.0, False), (31.0, True)):
        t = math.radians(angle)
        n = (-math.sin(t), math.cos(t), 0.0)  # the ramp's surface normal
        pieces = [{"size": (20, 1, 20), "center": tuple(-0.5 * c for c in n), "rotate": (0, 0, angle),
                   "attrs": {"active": 0, "friction": 1.0, "bounce": 0.0, "bullet_georep": "convexhull"}},
                  {"size": (1, 1, 1), "center": tuple(0.501 * c for c in n), "rotate": (0, 0, angle),
                   "attrs": {"active": 1, "friction": 0.5, "bounce": 0.0, "bullet_georep": "convexhull"}}]
        path, names = packed_boxes(f"slope_{int(angle)}_pieces", pieces)
        dop = new_dopnet(f"slope_{int(angle)}")
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
        track = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
        moved = (track[48] - track[2]).length()
        expected = 0.5 * G * (math.sin(t) - 0.5 * math.cos(t)) * (46 / hou.fps()) ** 2
        good = moved > 0.3 if should_slide else moved < 0.05
        ok = ok and good
        rows.append(f"{angle:g} degrees: moved {moved:.3f} in 2 s ("
                    + (f"slides; about {expected:.2f} expected)" if should_slide else "expected to stay put)"))
    return [Result("friction_slope", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def ccd_run(name, wall_active, fast, continuous=True):
    """A 0.2 box fired at 200 m/s at a 0.05 thick wall, no gravity. Returns (projectile x, wall x)
    after 6 frames."""
    pieces = [{"size": (0.05, 4, 4), "center": (0, 0, 0),
               "attrs": {"active": wall_active, "fastobject": 0, "v": (0.0, 0.0, 0.0)}},
              {"size": (0.2, 0.2, 0.2), "center": (-3, 0, 0),
               "attrs": {"active": 1, "fastobject": fast, "v": (200.0, 0.0, 0.0)}}]
    path, names = packed_boxes(name + "_pieces", pieces)
    dop = new_dopnet(name)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True), gravity=(0, 0, 0))
    set_parm(solver, ["enablecontinuous"], 1 if continuous else 0)
    finish(dop, solver)
    pos = step(dop, 6, lambda sim, f: packed_positions(sim, "pieces"))[6]
    return pos[names[1]][0], pos[names[0]][0]


def test_ccd():
    """A small box at 200 m/s against a thin wall: a static wall stops it with Continuous Collision
    and is tunnelled without; a moving wall stops it only when the box is a Fast Object."""
    rows, ok = [], True
    cases = (("static wall, Continuous on", 0, 0, True, True),
             ("static wall, Continuous off", 0, 0, False, False),
             ("moving wall, Fast Object on", 1, 1, True, True),
             ("moving wall, Fast Object off", 1, 0, True, None))
    for i, (label, wall_active, fast, continuous, want_stop) in enumerate(cases):
        px, wx = ccd_run(f"ccd_{i}", wall_active, fast, continuous)
        stopped = px < wx
        if want_stop is not None:
            ok = ok and stopped == want_stop
        rows.append(f"{label}: {'stopped' if stopped else 'went through'} (box x {px:.2f}, wall x {wx:.2f})"
                    + ("" if want_stop is None else f", expected {'stopped' if want_stop else 'through'}"))
    return [Result("ccd", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_sleep():
    """A box resting on the ground falls asleep (its awake attribute is written 0); with Box3D Data's
    Allow Sleeping off it stays awake. A box with Start Asleep hangs in the air until another box
    lands on it, then both fall."""
    results = []
    awake = {}
    for allow in (1, 0):
        pieces = [dict(GROUND, attrs={"active": 0, "awake": 1}),
                  {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"active": 1, "awake": 1}}]
        path, names = packed_boxes(f"sleep_{allow}_pieces", pieces)
        dop = new_dopnet(f"sleep_{allow}")
        data = box3d_data(dop, packed_object(dop, "pieces", path), "data", enablesleep=allow)
        finish(dop, box3d_solver(dop, data))
        awake[allow] = step(dop, 96, lambda sim, f: point_attributes(sim, "pieces", names[1], ["awake"])["awake"])[96]
    ok = awake[1] == 0 and awake[0] == 1
    results.append(Result("sleep", Result.PASS if ok else Result.FAIL,
                          f"awake after 4 s at rest: {awake[1]} with Allow Sleeping, {awake[0]} without "
                          "(expected 0 and 1)"))

    heights = {}
    for hit in (0, 1):
        path_a, names_a = packed_boxes(f"asleep_{hit}_a", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        dop = new_dopnet(f"asleep_{hit}")
        sleeper = box3d_data(dop, packed_object(dop, "sleeper", path_a), "data", startasleep=1)
        objs = [sleeper]
        if hit:
            path_b, _ = packed_boxes("asleep_hammer", [{"size": (1, 1, 1), "center": (0, 6, 0)}])
            objs.append(packed_object(dop, "hammer", path_b))
        finish(dop, box3d_solver(dop, merge(dop, objs)))
        heights[hit] = step(dop, 48, lambda sim, f: packed_positions(sim, "sleeper")[names_a[0]][1])[48]
    ok = close(heights[0], 3.0, 0.01) and heights[1] < 2.0
    results.append(Result("sleep", Result.PASS if ok else Result.FAIL,
                          f"Start Asleep box in the air after 2 s: y={heights[0]:.2f} alone (expected 3), "
                          f"y={heights[1]:.2f} after another box lands on it (expected below 2)"))
    return results


def test_rolling_resistance():
    """A sphere rolling at 3 m/s (spinning to match) keeps rolling with rolling resistance 0 and
    slows down with 0.3."""
    rows, speeds = [], {}
    for rr in (0.0, 0.3):
        pieces = [dict(GROUND, attrs={"active": 0, "rollingresistance": 0.0, "bullet_georep": "box",
                                      "bullet_autofit": 1, "bounce": 0.0}),
                  {"size": (1, 1, 1), "center": (0, 0.5, 0),
                   "attrs": {"active": 1, "rollingresistance": rr, "bullet_georep": "sphere", "bullet_autofit": 1,
                             "bounce": 0.0, "v": (3.0, 0.0, 0.0), "w": (0.0, 0.0, -6.0)}}]
        path, names = packed_boxes(f"rolling_{int(rr * 10)}_pieces", pieces)
        dop = new_dopnet(f"rolling_{int(rr * 10)}")
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True)))
        v = step(dop, 48, lambda sim, f: point_attributes(sim, "pieces", names[1], ["v"])["v"])[48]
        speeds[rr] = math.hypot(v[0], v[2])
        rows.append(f"rolling resistance {rr:g}: speed {speeds[rr]:.2f} after 2 s")
    ok = speeds[0.0] > 2.8 and speeds[0.3] < 2.0
    return [Result("rolling_resistance", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected about 3, and below 2)")]


SHAPES = (
    # label, size, georep, autofit, primS (half extents), radius, length, primR (Euler degrees), rest height
    ("box, autofit", (1, 1, 1), "box", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), 0.5),
    # AutoFit as Bullet's (autofit_bullet): a sphere through the cube's corners; a capsule around the
    # 1 x 1 cross-section, as tall as the piece.
    ("sphere, autofit", (1, 1, 1), "sphere", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), math.sqrt(3) / 2),
    ("capsule, autofit (standing, 1 x 2)", (1, 2, 1), "capsule", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), 1.0),
    ("box, bullet_primS 0.5 0.25 0.5", (1, 1, 1), "box", 0, (0.5, 0.25, 0.5), 1.0, 1.0, (0, 0, 0), 0.25),
    ("sphere, bullet_radius 0.3", (1, 1, 1), "sphere", 0, (1.0, 1.0, 1.0), 0.3, 1.0, (0, 0, 0), 0.3),
    ("capsule, radius 0.25 length 1 (standing)", (1, 1, 1), "capsule", 0, (1.0, 1.0, 1.0), 0.25, 1.0,
     (0, 0, 0), 0.75),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 90 (lying)", (1, 1, 1), "capsule", 0, (1.0, 1.0, 1.0),
     0.25, 1.0, (0, 0, 90), 0.25),
)


def test_shape_types():
    """Box, sphere and capsule bullet_georep, fitted to the piece and sized by bullet_primS /
    bullet_radius / bullet_length with bullet_autofit 0: each rests at its own half height. A
    capsule's axis is the body's Y; bullet_primR turns it to lie down."""
    pieces = [dict(GROUND, attrs={"active": 0, "bullet_georep": "box", "bullet_autofit": 1,
                                  "bullet_primS": (1.0, 1.0, 1.0), "bullet_radius": 1.0, "bullet_length": 1.0,
                                  "bullet_primR": (0.0, 0.0, 0.0), "bounce": 0.0})]
    for i, (_, size, rep, fit, prim_s, radius, length, prim_r, _) in enumerate(SHAPES):
        pieces.append({"size": size, "center": (0, 1.5, (i - 3) * 3),
                       "attrs": {"active": 1, "bullet_georep": rep, "bullet_autofit": fit, "bullet_primS": prim_s,
                                 "bullet_radius": radius, "bullet_length": length,
                                 "bullet_primR": tuple(float(a) for a in prim_r), "bounce": 0.0}})
    path, names = packed_boxes("shapes_pieces", pieces)
    dop = new_dopnet("shapes")
    finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
    final = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))[72]
    rows, ok = [], True
    for i, shape in enumerate(SHAPES):
        y = final[names[i + 1]][1]
        good = close(y, shape[-1], 0.02)
        ok = ok and good
        rows.append(f"{shape[0]}: rests at {y:.3f} (expected {shape[-1]:.3f})")
    return [Result("shape_types", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Bad input and rewiring mid-simulation
# ---------------------------------------------------------------------------


def run_with_solver(name, path, frames=24, keep_velocity=False):
    dop = new_dopnet(name)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=keep_velocity))
    finish(dop, solver)
    track = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces"))
    return track, box3d_messages(solver)


def add_wrangle(sop_path, snippet):
    prev = hou.node(sop_path)
    wr = create(prev.parent(), "attribwrangle", "edit")
    wr.setInput(0, prev)
    set_parm(wr, ["snippet"], snippet)
    display(wr)
    return wr.path()


AUTOFIT_SIZE = (0.6, 0.4, 0.8)
AUTOFIT_TILT = (0.0, 0.0, 30.0)  # degrees, about z
AUTOFIT_CENTER = (0.0, 0.6, 0.0)


def autofit_scene(name, solver_kind, georep, rotate_before_pack, size=AUTOFIT_SIZE, tilt=AUTOFIT_TILT):
    """A 0.6 x 0.4 x 0.8 box tilted 30 degrees about z, dropped on a static ground, both packed
    pieces of one RBD Packed Object. rotate_before_pack: the tilt is baked into the packed
    geometry (as Assemble or Pack after a Transform SOP leave it: the packed transform is
    identity), else it is on the packed transform. georep: the piece's bullet_georep (with
    bullet_autofit 1), or None to leave both attributes off (each solver's own default)."""
    geo = new_geo(name + "_geo")

    def branch(tag, size, center, rotate, before, attrs):
        box = create(geo, "box", f"{tag}_box")
        set_parm(box, ["size"], tuple(size))
        last = box
        if before:
            xf = create(geo, "xform", f"{tag}_place")
            xf.setInput(0, last)
            set_parm(xf, ["r"], tuple(rotate))
            set_parm(xf, ["t"], tuple(center))
            last = xf
        asm = create(geo, "assemble", f"{tag}_pack")
        asm.setInput(0, last)
        set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
        last = asm
        if not before:
            # The box is centred on the origin, so rotating there turns the packed piece in place.
            xf = create(geo, "xform", f"{tag}_place")
            xf.setInput(0, last)
            set_parm(xf, ["r"], tuple(rotate))
            set_parm(xf, ["t"], tuple(center))
            last = xf
        sets = [f's@name = "{tag}";'] + [f"{vex_value(v)[0]}@{k} = {vex_value(v)[1]};" for k, v in attrs.items()]
        wr = create(geo, "attribwrangle", f"{tag}_attrs")
        wr.setInput(0, last)
        set_parm(wr, ["snippet"], " ".join(sets))
        return wr

    # Both pieces get the same attributes: Merge would give the ground an empty bullet_georep and
    # bullet_autofit 0 otherwise (with RBD Packed Object's bullet_primS 1 1 1, a 2 x 2 x 2 box).
    piece_attrs = {"active": 1, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0), "bounce": 0.0}
    ground_attrs = {"active": 0, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0), "bounce": 0.0}
    if georep is not None:
        piece_attrs.update({"bullet_georep": georep, "bullet_autofit": 1})
        ground_attrs.update({"bullet_georep": "box", "bullet_autofit": 1})
    ground = branch("ground", GROUND["size"], GROUND["center"], (0, 0, 0), True, ground_attrs)
    # Another size starts higher: half the sum of its sides clears any shape fitted around it.
    lift = AUTOFIT_CENTER[1] if tuple(size) == AUTOFIT_SIZE else 0.5 * sum(size) + 0.1
    center = (AUTOFIT_CENTER[0], lift, AUTOFIT_CENTER[2])
    piece = branch("piece", size, center, tilt, rotate_before_pack, piece_attrs)
    m = create(geo, "merge", "merge")
    m.setInput(0, ground)
    m.setInput(1, piece)
    display(m)

    dop = new_dopnet(name)
    obj = packed_object(dop, "pieces", m.path(), keep_velocity=True)
    finish(dop, box3d_solver(dop, obj) if solver_kind == "box3d" else bullet_solver(dop, obj))
    return dop


def autofit_rest(dop, rotate_before_pack, frames=72, tilt=AUTOFIT_TILT):
    """Drops the piece and returns (rest height of its point, tilt in degrees, bullet_georep and
    bullet_autofit as the solver saw them on frame 1). Tilt: the smallest angle between one of
    the box's own axes and vertical (0 = lying on a face)."""
    def sample(sim, f):
        obj = sim.findObject("pieces")
        geo = obj.geometry()
        prim = next(pr for pr in geo.prims() if pr.points() and pr.points()[0].attribValue("name") == "piece")
        m = hou.Matrix4(prim.intrinsicValue("packedfulltransform"))
        seen = point_attributes(sim, "pieces", "piece", ["bullet_georep", "bullet_autofit"]) if f == 1 else None
        return prim.points()[0].position(), m, seen

    track = step(dop, frames, sample)
    pos, m, _ = track[frames]
    seen = track[1][2]
    # The box's axes in the packed geometry's own space: turned by the tilt when it was baked in.
    local = hou.hmath.buildRotate(hou.Vector3(tilt)) if rotate_before_pack else hou.hmath.identityTransform()
    origin = hou.Vector3(0, 0, 0) * local * m
    tilt = 90.0
    for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
        d = (hou.Vector3(axis) * local * m - origin).normalized()
        tilt = min(tilt, math.degrees(math.acos(max(-1.0, min(1.0, abs(d[1]))))))
    return pos[1], tilt, seen


def test_autofit_bullet():
    """A tilted box dropped in Box3D and in Bullet with the same shape settings ends up the same
    way: lying flat (tilt 0) when the shape fits the box, standing on an edge at its starting
    30 degrees when a box shape is fitted to the axis-aligned bounds of the tilted geometry.
    Covers convex hull, box with AutoFit with the tilt baked into the packed geometry and on the
    packed transform, sphere and capsule with AutoFit (rest height only: they roll), and no
    bullet_georep at all, where each solver uses its default (both a convex hull)."""
    cases = (
        # label, bullet_georep (None: unset), tilt baked into the packed geometry
        ("convex hull, tilt baked in", "convexhull", True),
        ("box + autofit, tilt baked in", "box", True),
        ("box + autofit, tilt on the packed transform", "box", False),
        ("sphere + autofit, tilt baked in", "sphere", True),
        ("capsule + autofit, tilt baked in", "capsule", True),
        ("no bullet_georep, tilt baked in", None, True),
    )
    results = []
    for i, case in enumerate(cases):
        label, georep, baked = case[:3]
        size, tilt = case[3:] if len(case) > 3 else (AUTOFIT_SIZE, AUTOFIT_TILT)
        # A sphere or capsule can roll, so only its rest height says how it was fitted (sphere:
        # its radius; capsule lying down: its radius too).
        rolls = georep in ("sphere", "capsule")
        got = {}
        for solver_kind in ("box3d", "bullet"):
            dop = autofit_scene(f"autofit_{i}_{solver_kind}", solver_kind, georep, baked, size, tilt)
            got[solver_kind] = autofit_rest(dop, baked, tilt=tilt)
        (yb, tb, sb), (yt, tt, st) = got["box3d"], got["bullet"]
        msg = (f"{label}: Box3D rests at y {yb:.3f}, tilt {tb:.1f} deg; Bullet y {yt:.3f}, tilt {tt:.1f} deg "
               f"(flat: tilt 0 and y 0.2 or 0.3; fitted to the tilted bounds: tilt 30, y about 0.32)")
        same = (rolls or abs(tb - tt) < 5.0) and abs(yb - yt) < 0.05
        results.append(Result("autofit_bullet", Result.PASS if same else Result.FAIL,
                              msg if same else msg + f". Attributes seen on frame 1: Box3D {sb}, Bullet {st}"))

    # Capsule length: a standing capsule topples one way or the other, so this one is static and a plate
    # dropped on it finds its top.
    top_b3, top_bt = capsule_top("box3d"), capsule_top("bullet")
    same = close(top_b3, top_bt, 0.05)
    results.append(Result("autofit_bullet", Result.PASS if same else Result.FAIL,
                          f"capsule length: the top of a static AutoFit capsule on a standing 0.4 x 1.2 x 0.4 piece is "
                          f"{top_b3:.3f} above its centre in Box3D, {top_bt:.3f} in Bullet (0.883 = enclosing the "
                          f"piece, 0.6 = caps inside it)"))
    return results


def capsule_top(solver_kind, frames=48):
    """Height of a static AutoFit capsule's top above its centre: a 0.6 x 0.1 x 0.6 plate is dropped on
    the capsule of a standing 0.4 x 1.2 x 0.4 piece; the first frame its fall stops is where it touched
    (no bounce). The plate may tip off afterwards."""
    base = {"bullet_autofit": 1, "bounce": 0.0, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0)}
    pieces = [{"size": (0.4, 1.2, 0.4), "center": (0, 1.0, 0),
               "attrs": dict(base, active=0, bullet_georep="capsule")},
              {"size": (0.6, 0.1, 0.6), "center": (0, 2.5, 0),
               "attrs": dict(base, active=1, bullet_georep="box")}]
    path, names = packed_boxes(f"capsule_top_{solver_kind}_pieces", pieces)
    dop = new_dopnet(f"capsule_top_{solver_kind}")
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    finish(dop, box3d_solver(dop, obj) if solver_kind == "box3d" else bullet_solver(dop, obj))
    ys = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])
    for f in range(3, frames + 1):
        fall, before = ys[f - 1] - ys[f], ys[f - 2] - ys[f - 1]
        if before > 1e-4 and fall < 0.5 * before:
            return ys[f] - 0.05 - 1.0  # the plate's underside, from the capsule's centre
    return float("nan")


def test_bad_input():
    """No crash, and the good pieces still land, with: empty input geometry; a zero-size piece; a
    piece at an infinite position and one with a NaN velocity; two pieces with the same name."""
    results = []

    geo = new_geo("bad_empty")
    empty = create(geo, "attribwrangle", "nothing")
    set_parm(empty, ["class"], 0)
    set_parm(empty, ["snippet"], "// no geometry")
    display(empty)
    track, msgs = run_with_solver("bad_empty", empty.path(), 12)
    results.append(Result("bad_input", Result.PASS, f"empty input: 12 frames, no crash; messages {msgs or 'none'}"))

    good = {"size": (1, 1, 1), "center": (3, 2, 0)}

    def lands(track, name):
        y = track[max(track)][name][1] if track[max(track)] and name in track[max(track)] else float("nan")
        return close(y, 0.5, 0.05), y

    path, names = packed_boxes("bad_zero", [GROUND, good, {"size": (0, 0, 0), "center": (0, 2, 0)}])
    track, msgs = run_with_solver("bad_zero", path, 48)
    ok, y = lands(track, names[1])
    results.append(Result("bad_input", Result.PASS if ok else Result.FAIL,
                          f"zero-size piece: the good box rests at y={y:.3f}; messages {msgs or 'none'}"))

    path, names = packed_boxes("bad_inf", [GROUND, good, {"size": (1, 1, 1), "center": (0, 2, 0)},
                                           {"size": (1, 1, 1), "center": (-3, 2, 0)}])
    path = add_wrangle(path, f'float big = 1e30; float inf = big * big * big;\n'
                             f'if (s@name == "{names[2]}") @P.x = inf;\n'
                             f'if (s@name == "{names[3]}") v@v = set(inf - inf, 0, 0);')
    track, msgs = run_with_solver("bad_inf", path, 48, keep_velocity=True)
    ok, y = lands(track, names[1])
    results.append(Result("bad_input", Result.PASS if ok else Result.FAIL,
                          f"infinite position and NaN velocity: the good box rests at y={y:.3f}; "
                          f"messages {msgs or 'none'}"))

    path, names = packed_boxes("bad_dupe", [GROUND, {"size": (1, 1, 1), "center": (0, 2, 0)},
                                            {"size": (1, 1, 1), "center": (3, 2, 0)}])
    path = add_wrangle(path, f'if (s@name != "{names[0]}") s@name = "dupe";')
    dop = new_dopnet("bad_dupe")
    solver = box3d_solver(dop, packed_object(dop, "pieces", path))
    finish(dop, solver)
    ys = step(dop, 48, lambda sim, f: [pt.position()[1] for pt in sim.findObject("pieces").geometry().points()
                                       if pt.attribValue("name") == "dupe"])[48]
    ok = len(ys) == 2 and all(close(y, 0.5, 0.05) for y in ys)
    results.append(Result("bad_input", Result.PASS if ok else Result.FAIL,
                          f"two pieces named 'dupe': rest at {[round(y, 3) for y in ys]} (expected both 0.5); "
                          f"messages {box3d_messages(solver) or 'none'}"))
    return results


def test_rewire_midsim():
    """Mid-simulation: the solver is bypassed at frame 12 and reconnected at 18, then deleted and
    replaced by a new Box3D Solver at 30. No crash; the box still ends up resting on the ground."""
    path, names = packed_boxes("rewire_pieces", [GROUND, {"size": (1, 1, 1), "center": (0, 4, 0)}])
    dop = new_dopnet("rewire")
    obj = packed_object(dop, "pieces", path)
    solver = box3d_solver(dop, obj)
    out = finish(dop, solver)
    log = []
    for f in range(1, 61):
        if f == 12:
            out.setInput(0, obj)
        elif f == 18:
            out.setInput(0, solver)
        elif f == 30:
            solver.destroy()
            solver = box3d_solver(dop, obj, name="box3d_solver_new")
            out.setInput(0, solver)
        hou.setFrame(f)
        pos = packed_positions(dop.simulation(), "pieces")
        if f in (11, 17, 29, 60):
            log.append(f"frame {f}: y={pos[names[1]][1]:.2f}")
    y = packed_positions(dop.simulation(), "pieces")[names[1]][1]
    return [Result("rewire_midsim", Result.PASS if close(y, 0.5, 0.05) else Result.FAIL,
                   "; ".join(log) + " (expected resting at 0.5 at the end)")]


# ---------------------------------------------------------------------------
# A large scene: a timing baseline
# ---------------------------------------------------------------------------


def test_large_scene():
    """About 5000 boxes dropped in a lattice: time to the first frame and per frame over 48
    frames (INFO, a baseline to catch slowdowns), plus a sanity check that the pile came down and
    stayed finite. The solver's default contacts are bouncy (damping ratio 3), so a box may rebound
    above where it started; only one flung far (over twice the lattice's height) fails."""
    nx, ny, nz = 20, 12, 20
    geo = new_geo("large_points")
    pts = create(geo, "attribwrangle", "lattice")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        f"int n = 0;\nfor (int k = 0; k < {ny}; k++) for (int j = 0; j < {nz}; j++) for (int i = 0; i < {nx}; i++) {{\n"
        "  vector p = set(i - 9.5, 0.6 + k * 1.05, j - 9.5) + 0.05 * (rand(set(i, j, k)) - 0.5);\n"
        '  int pt = addpoint(0, p); setpointattrib(0, "name", pt, sprintf("p%d", n)); n++;\n}'))
    box = create(geo, "box")
    set_parm(box, ["size"], (0.8, 0.8, 0.8))
    copy = create(geo, "copytopoints")
    copy.setInput(0, box)
    copy.setInput(1, pts)
    set_parm(copy, ["pack", "packgeo"], 1)
    display(copy)
    ground_path, _ = packed_boxes("large_ground", [dict(GROUND, size=(40, 1, 40))])
    dop = new_dopnet("large")
    objs = [packed_object(dop, "pieces", copy.path()), packed_object(dop, "ground", ground_path)]
    finish(dop, box3d_solver(dop, merge(dop, objs)))
    count = len(copy.geometry().prims())
    start = time.time()
    hou.setFrame(1)
    dop.simulation().objects()
    first = time.time() - start
    start = time.time()
    for f in range(2, 49):
        hou.setFrame(f)
        dop.simulation().objects()
    per_frame = (time.time() - start) / 47
    ys = [pt.position()[1] for pt in dop.simulation().findObject("pieces").geometry().points()]
    finite = all(math.isfinite(y) for y in ys)
    top = max(ys) if ys else float("nan")
    mean = sum(ys) / len(ys) if ys else float("nan")
    start_top = 0.6 + (ny - 1) * 1.05
    start_mean = 0.6 + (ny - 1) * 1.05 / 2
    ok = finite and count == nx * ny * nz and mean < start_mean and top < 2 * start_top
    return [Result("large_scene", Result.INFO if ok else Result.FAIL,
                   f"{count} boxes: first frame {first:.2f} s, then {per_frame * 1000:.0f} ms per frame over 47 frames; "
                   f"on frame 48 mean height {mean:.2f} (started {start_mean:.2f}: should be lower), highest box "
                   f"y={top:.2f} (started up to {start_top:.2f}; fails above {2 * start_top:.2f})"
                   + ("" if finite else "; NON-FINITE POSITIONS"))]


# ---------------------------------------------------------------------------
# Constraints: across objects, to the world, to a kinematic piece, keyed mid-simulation
# ---------------------------------------------------------------------------


def network_dop(dop, objects_node, network_path, conrel_type, parms=None):
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, objects_node)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network_path)
    joint = create(dop, conrel_type, "joint0")
    for parm, value in (parms or {}).items():
        set_parm(joint, [parm], value)
    net.setInput(1, joint)
    return net, joint


def test_cross_object_constraint():
    """A weld between a fixed piece in one RBD Packed Object and a hanging piece in another (both
    pieces named piece0, so the line names them object/piece): the hanging piece is held."""
    path_a, names_a = packed_boxes("cross_a_geo", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}}])
    path_b, names_b = packed_boxes("cross_b_geo", [{"size": (1, 1, 1), "center": (1, 2, 0)}])
    dop = new_dopnet("cross_object")
    objs = merge(dop, [packed_object(dop, "obj_a", path_a), packed_object(dop, "obj_b", path_b)])
    network = constraint_lines("cross_network", [(f"obj_a/{names_a[0]}", FACE, f"obj_b/{names_b[0]}", FACE, "weld")])
    net, _ = network_dop(dop, objs, network, "box3d_weldconrel")
    solver = box3d_solver(dop, net)
    finish(dop, solver)
    y = step(dop, 48, lambda sim, f: packed_positions(sim, "obj_b")[names_b[0]])[48][1]
    return [Result("cross_object_constraint", Result.PASS if y > 1.9 else Result.FAIL,
                   f"hanging piece (other object) at y={y:.3f} after 2 s (2.0 = held); messages "
                   f"{box3d_messages(solver) or 'none'}")]


def test_world_constraint():
    """A piece welded to the world (the line's other end has an empty name) stays where it is."""
    path, names = packed_boxes("world_geo", [{"size": (1, 1, 1), "center": (0, 2, 0)}])
    dop = new_dopnet("world_constraint")
    network = constraint_lines("world_network", [("", (0, 2.5, 0), names[0], (0, 2.5, 0), "weld")])
    net, _ = network_dop(dop, packed_object(dop, "pieces", path), network, "box3d_weldconrel")
    solver = box3d_solver(dop, net)
    finish(dop, solver)
    y = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces")[names[0]])[48][1]
    return [Result("world_constraint", Result.PASS if close(y, 2.0, 0.02) else Result.FAIL,
                   f"piece at y={y:.3f} after 2 s (2.0 = held); messages {box3d_messages(solver) or 'none'}")]


def test_kinematic_anchor():
    """A piece welded to a kinematic piece (active 0, animated 1) that a SOP Solver moves along x at
    1 unit/s: the welded piece follows, staying 1 to its right at y = 2."""
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0, "animated": 1}},
              {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1, "animated": 0}}]
    path, names = packed_boxes("kin_anchor_geo", pieces)
    dop = new_dopnet("kinematic_anchor")
    network = constraint_lines("kin_anchor_network", [(names[0], FACE, names[1], FACE, "weld")])
    net, _ = network_dop(dop, packed_object(dop, "pieces", path), network, "box3d_weldconrel")
    mover = sop_solver(dop, f'if (s@name == "{names[0]}") @P.x = @Time;')
    finish(dop, multisolver(dop, net, [mover, box3d_solver(dop, None)]))
    pos = step(dop, 49, lambda sim, f: packed_positions(sim, "pieces"))[49]
    kx, bx, by = pos[names[0]][0], pos[names[1]][0], pos[names[1]][1]
    ok = close(kx, 2.0, 0.05) and close(bx - kx, 1.0, 0.05) and close(by, 2.0, 0.05)
    return [Result("kinematic_anchor", Result.PASS if ok else Result.FAIL,
                   f"kinematic piece at x={kx:.3f} (expected 2.0), welded piece at x={bx:.3f}, y={by:.3f} "
                   "(expected 1 to its right, at y = 2)")]


def test_joint_keyed_midsim():
    """A weld whose Breakable is keyed on at frame 24 (its operation menu at Set Always, Break Force
    1e-3): it holds until 24, then breaks; frames before 24 match a run that never breaks, and
    scrubbing back to 12 shows it held."""
    def build(tag, keyed):
        dop, names = hanging_pair(f"keyed_{tag}", "weld",
                                  conrels=[("box3d_weldconrel", {"breakable": 0, "force_threshold": 1e-3})])
        if keyed:
            joint = dop.node("joint0")
            if joint.parm("parmop_breakable") is None:
                ops = ", ".join(p.name() for p in joint.parms() if p.name().startswith("parmop"))
                raise SetupError(f"no parmop_breakable on the weld node; operation menus: {ops}")
            set_expression(joint, ["breakable"], "$F >= 24")
            set_menu_by_label(joint, ["parmop_breakable"], "Always")
        return dop, names

    dop_k, names = build("on", True)
    keyed = step(dop_k, 36, lambda sim, f: (packed_positions(sim, "pieces")[names[1]], broken_count(sim)))
    hou.setFrame(12)
    back = packed_positions(dop_k.simulation(), "pieces")[names[1]][1]
    dop_p, _ = build("off", False)
    plain = step(dop_p, 20, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
    d20 = (keyed[20][0] - plain[20]).length()
    y23, y36 = keyed[23][0][1], keyed[36][0][1]
    ok = y23 > 1.9 and y36 < 1.5 and d20 == 0.0 and back > 1.9
    return [Result("joint_keyed_midsim", Result.PASS if ok else Result.FAIL,
                   f"y={y23:.2f} on frame 23, {y36:.2f} on frame 36 (held, then fell); broken lines on 36: "
                   f"{keyed[36][1]}; frame 20 differs from a never-breaking run by {d20:.2e} (expected 0); "
                   f"after scrubbing back to 12, y={back:.2f} (expected held)")]


TESTS = {
    "joint_revolute_limit": test_joint_revolute_limit,
    "joint_revolute_motor": test_joint_revolute_motor,
    "joint_prismatic_limit": test_joint_prismatic_limit,
    "joint_spring_rest_length": test_joint_spring_rest_length,
    "joint_breakable": test_joint_breakable,
    "negative_never_values": test_negative_never_values,
    "friction_mixing": test_friction_mixing,
    "bounce_mixing": test_bounce_mixing,
    "bullet_mixing": test_bullet_mixing,
    "determinism_repeat": test_determinism_repeat,
    "determinism_threads": test_determinism_threads,
    "substeps_fps": test_substeps_fps,
    "friction_slope": test_friction_slope,
    "bounce_heights": test_bounce_heights,
    "ccd": test_ccd,
    "sleep": test_sleep,
    "rolling_resistance": test_rolling_resistance,
    "shape_types": test_shape_types,
    "autofit_bullet": test_autofit_bullet,
    "bad_input": test_bad_input,
    "rewire_midsim": test_rewire_midsim,
    "large_scene": test_large_scene,
    "cross_object_constraint": test_cross_object_constraint,
    "world_constraint": test_world_constraint,
    "kinematic_anchor": test_kinematic_anchor,
    "joint_keyed_midsim": test_joint_keyed_midsim,
}
