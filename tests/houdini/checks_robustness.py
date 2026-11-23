"""Bad input, rewiring mid-simulation, a large-scene timing baseline, and constraints across
objects, to the world, to a kinematic piece and keyed mid-simulation."""

import math
import time

import hou

from b3dtest import (
    box3d_solver,
    close,
    constraint_lines,
    create,
    display,
    finish,
    GROUND,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_expression,
    set_parm,
    SetupError,
    sop_solver,
    step,
    set_menu_token,
)
from checks_constraints import FACE, broken_count, hanging_pair
from checks_common import box3d_messages
from checks_contact import add_wrangle, run_with_solver


# ---------------------------------------------------------------------------
# Bad input and rewiring mid-simulation
# ---------------------------------------------------------------------------


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
    obj = packed_object(dop, "pieces", path, bounce=0.0)  # the box lands and stays (not bouncing at frame 60)
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
            set_menu_token(joint, ["parmop_breakable"], "always")
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


def test_parallel_matches_serial():
    """The plugin's parallel passes give the same simulation as one thread. The 3000-box pile of the benchmark is
    played for 40 frames in a fresh hython with HOUDINI_MAXTHREADS=1 (every parallel pass runs on one thread) and
    in another with all threads: the sum of the boxes' positions and their bounds agree within 1e-3."""
    from bench import run_isolated
    from bench_scenes import SCENARIOS

    params = dict(SCENARIOS["pile"].defaults, n=3000, frames=40)
    got = {}
    for threads in (1, None):
        record = run_isolated({"scenario": "pile", "kind": "box3d", "params": params, "threads": threads,
                               "budget": 600, "no_perfmon": True, "no_rewind": True})
        if "error" in record:
            raise SetupError(f"benchmark run with threads={threads} failed: {record['error'][-300:]}")
        got[threads] = record["end"]["pieces"]
    one, many = got[1], got[None]
    diffs = [abs(a - b) for key in ("sum", "min", "max") for a, b in zip(one[key], many[key])]
    ok = one["points"] == many["points"] == 3072 and max(diffs) < 1e-3
    return [Result("parallel_matches_serial", Result.PASS if ok else Result.FAIL,
                   f"{one['points']} / {many['points']} boxes (expected 3072), largest difference in the position sums "
                   f"and bounds between 1 thread and all threads {max(diffs):.2e} (expected under 1e-3)")]


TESTS = {
    "parallel_matches_serial": test_parallel_matches_serial,
    "bad_input": test_bad_input,
    "rewire_midsim": test_rewire_midsim,
    "large_scene": test_large_scene,
    "cross_object_constraint": test_cross_object_constraint,
    "world_constraint": test_world_constraint,
    "kinematic_anchor": test_kinematic_anchor,
    "joint_keyed_midsim": test_joint_keyed_midsim,
}
