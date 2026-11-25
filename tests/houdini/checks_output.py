"""What the solver writes back: velocity, Impacts, mass, aging, point force / torque, Gravity and Force DOPs."""

import math
import os
import tempfile

import hou
from b3dtest import (
    Result,
    SetupError,
    box3d_data,
    box3d_solver,
    close,
    create,
    finish,
    ground_object,
    merge,
    multisolver,
    new_dopnet,
    packed_boxes,
    packed_object,
    packed_positions,
    packed_transform,
    point_attributes,
    set_parm,
    sop_solver,
    step,
)


G = 9.80665


def test_velocity_edit():
    """A SOP Solver that sets only `v` on a floating box (no gravity) on frame 10 changes its velocity and leaves it
    where it is: x stays 0 until the edit, then moves at 2 m/s (about 0.92 by frame 20: 11 steps at 24 fps), y stays
    5 throughout. An edit of v alone does not move the body (it is not set to a new transform)."""
    path, names = packed_boxes("velocity_edit_pieces", [
        {"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"active": 1, "v": (0.0, 0.0, 0.0)}}])
    dop = new_dopnet("velocity_edit")
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    edit = sop_solver(dop, "if (@Frame == 10) v@v = {2, 0, 0};")
    solver = box3d_solver(dop, None, gravity=(0, 0, 0))
    finish(dop, multisolver(dop, obj, [edit, solver]))
    track = step(dop, 24, lambda sim, f: tuple(packed_positions(sim, "pieces")[names[0]]))
    x9, x20, y_all = track[9][0], track[20][0], [t[1] for t in track.values()]
    ok = abs(x9) < 1e-3 and close(x20, 2.0 * 11 / hou.fps(), 0.1) and all(close(y, 5.0, 1e-3) for y in y_all)
    return [Result("velocity_edit", Result.PASS if ok else Result.FAIL,
                   f"x on frame 9 {x9:.4f} (expected 0), on frame 20 {x20:.3f} (expected {2.0 * 11 / hou.fps():.3f}); "
                   f"y from {min(y_all):.4f} to {max(y_all):.4f} (expected 5)")]


def test_written_velocity():
    """The solver writes v and w on the points: a falling box has v.y = -g t after one second, and
    a spinning box (no gravity) keeps w = 1 rad/s and turns 1 radian in a second."""
    results = []
    path, names = packed_boxes("vel_fall", [{"size": (1, 1, 1), "center": (0, 50, 0)}])
    dop = new_dopnet("vel_fall")
    finish(dop, box3d_solver(dop, packed_object(dop, "piece", path)))
    attrs = step(dop, 25, lambda sim, f: point_attributes(sim, "piece", names[0], ["v", "w"]))
    vy = attrs[25]["v"][1]
    results.append(Result("written_velocity", Result.PASS if close(vy, -G, 0.02 * G) else Result.FAIL,
                          f"falling box after 1 s: v.y = {vy:.3f} (expected {-G:.3f})"))

    path, names = packed_boxes("vel_spin", [{"size": (1, 1, 1), "center": (0, 0, 0), "attrs": {"w": (0.0, 1.0, 0.0)}}])
    dop = new_dopnet("vel_spin")
    finish(dop, box3d_solver(dop, packed_object(dop, "piece", path, keep_velocity=True), gravity=(0, 0, 0)))

    def sample(sim, f):
        return point_attributes(sim, "piece", names[0], ["w"]), packed_transform(sim, "piece")

    track = step(dop, 25, sample)
    w = track[25][0]["w"]
    r = track[25][1]
    angle = math.atan2(-r[0][2], r[0][0])  # rotation of the x axis about y
    ok = close(w[1], 1.0, 0.02) and close(angle, 1.0, 0.03)
    results.append(Result("written_velocity", Result.PASS if ok else Result.FAIL,
                          f"spinning box after 1 s: w = ({w[0]:.3f}, {w[1]:.3f}, {w[2]:.3f}) rad/s, turned {angle:.3f} rad "
                          "(expected w.y = 1 and 1 rad)"))
    return results


# ---------------------------------------------------------------------------
# Impacts (status: addPointImpact arguments; impulse in N*s)
# ---------------------------------------------------------------------------


def impact_records(sim, obj_name):
    """This frame's impact records as plain {field: value} dicts. (DopRecord objects are live: read
    later, they show whatever frame is current then.)"""
    obj = sim.findObject(obj_name)
    data = obj.findSubData("Impacts") if obj is not None else None
    if data is None:
        return []
    for rtype in data.recordTypes():
        if "mpact" in rtype:
            return [{n: r.field(n) for n in r.fieldNames()} for r in data.records(rtype)]
    return []


def test_impacts():
    """With Add Impact Data, a 1 kg box dropped 1 m records impacts whose impulses on the landing
    frame add up to about m*v (4.4 N*s), with its point number and a time."""
    path, names = packed_boxes("impact_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
    dop = new_dopnet("impacts")
    obj = packed_object(dop, "piece", path, computemass=0, mass=1.0, bounce=0.0)
    data = box3d_data(dop, obj, "data", addimpact=1)
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "impacts_ground", bounce=0.0), data])))
    track = step(dop, 30, lambda sim, f: impact_records(sim, "piece"))
    first = next((f for f in sorted(track) if track[f]), None)
    if first is None:
        return [Result("impacts", Result.FAIL, "no Impacts records on any frame")]
    recs = track[first]
    fields = list(recs[0])
    imp_field = next((n for n in fields if "impulse" in n.lower()), None)
    if imp_field is None:
        raise SetupError(f"Impacts records have no impulse field; fields: {fields}")
    total = sum(abs(r[imp_field]) if not isinstance(r[imp_field], (tuple, list))
                else math.sqrt(sum(c * c for c in r[imp_field])) for r in recs)
    expected = math.sqrt(2 * G * 1.0)
    extras = {n: recs[0][n] for n in fields if n.lower() in ("ptnum", "time", "otherobjid", "primnum")}
    results = [Result("impacts", Result.PASS if 0.7 * expected <= total <= 1.6 * expected else Result.FAIL,
                      f"frame {first}: {len(recs)} record(s), impulses sum to {total:.2f} N*s (expected about "
                      f"{expected:.2f})")]
    # The box is point 0 of its object; the time is the end of the step that hit (frame `first`).
    ptnum = next((recs[0][n] for n in fields if n.lower() == "ptnum"), None)
    t = next((recs[0][n] for n in fields if n.lower() == "time"), None)
    t_expected = (first - 1) / hou.fps()
    ok = ptnum == 0 and t is not None and abs(t - t_expected) <= 1.0 / hou.fps() + 1e-6
    results.append(Result("impacts", Result.PASS if ok else Result.FAIL,
                          f"first record: point number {ptnum} (expected 0), time {t} (expected about "
                          f"{t_expected:.4f}); all fields {extras}"))
    return results


def test_impacts_substeps():
    """With Substeps 1, 2 and 4 a box dropped on the ground records one impact per contact point of the frame, not
    one per step: at most 6 records on any frame (a landing face has 4 points) for each, and the impulses of the
    landing frames add up to about the same (within 40% of Substeps 1, which a frame's steps share out differently)."""
    results, sums = [], {}
    for substeps in (1, 2, 4):
        tag = f"impacts_sub{substeps}"
        path, _ = packed_boxes(tag + "_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
        dop = new_dopnet(tag)
        obj = packed_object(dop, "piece", path, computemass=0, mass=1.0, bounce=0.0)
        data = box3d_data(dop, obj, "data", addimpact=1)
        solver = box3d_solver(dop, merge(dop, [ground_object(dop, tag + "_ground", bounce=0.0), data]))
        set_parm(solver, ["substeps"], substeps)
        finish(dop, solver)
        track = step(dop, 30, lambda sim, f: impact_records(sim, "piece"))
        fields = next((list(r[0]) for r in track.values() if r), [])
        imp = next((n for n in fields if "impulse" in n.lower()), None)
        if imp is None:
            raise SetupError(f"Impacts records have no impulse field; fields: {fields}")
        most = max(len(r) for r in track.values())
        sums[substeps] = sum(abs(rec[imp]) for r in track.values() for rec in r)
        results.append((substeps, most))
    ok = all(most <= 6 for _, most in results) and all(
        0.6 * sums[1] <= sums[s] <= 1.4 * sums[1] for s in (2, 4))
    return [Result("impacts_substeps", Result.PASS if ok else Result.FAIL,
                   "; ".join(f"Substeps {s}: at most {m} records a frame, impulses sum to {sums[s]:.2f}"
                             for s, m in results) + " (expected at most 6 records, sums within 40%)")]


# ---------------------------------------------------------------------------
# mass written only with computemass (status: mass is written only with computemass)
# ---------------------------------------------------------------------------


def test_mass_written():
    """With Compute Mass on (density 500, set on RBD Packed Object, which writes them onto the
    points), the solver writes mass = 500 for a 1 m cube; with it off the mass attribute (3) is
    left alone."""
    rows, ok = [], True
    for compute in (1, 0):
        path, names = packed_boxes(f"mass_{compute}", [{"size": (1, 1, 1), "center": (0, 0.5, 0)}])
        dop = new_dopnet(f"mass_{compute}")
        obj = packed_object(dop, "piece", path, computemass=compute, density=500.0, mass=3.0)
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, f"mass_ground_{compute}"), obj])))
        attrs = step(dop, 3, lambda sim, f: point_attributes(sim, "piece", names[0], ["mass"]))
        before, after = attrs[1]["mass"], attrs[3]["mass"]
        rows.append(f"Compute Mass {'on' if compute else 'off'}: mass {before} on frame 1, {after} on frame 3")
        if compute and not (after is not None and close(after, 500.0, 1.0)):
            ok = False
        if not compute and not (after is not None and close(after, 3.0, 1e-3)):
            ok = False
    return [Result("mass_written", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Aging (status: age counts DOP seconds, dead once age > life, life 0 never dies)
# ---------------------------------------------------------------------------


def test_aging():
    """With Enable Aging, age grows by one per second (frames 2..25: the object is not solved on its
    creation frame). As RBD Bullet Solver, a piece dies once its age passes its life, a life of 0 on the first step,
    and with Reap Dead Points on (the default) it is deleted; a piece with a long life lives on."""
    pieces = [
        {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"life": 0.5, "age": 0.0}},
        {"size": (1, 1, 1), "center": (3, 0.5, 0), "attrs": {"life": 0.0, "age": 0.0}},
        {"size": (1, 1, 1), "center": (6, 0.5, 0), "attrs": {"life": 100.0, "age": 0.0}},
    ]
    path, names = packed_boxes("aging_pieces", pieces)
    dop = new_dopnet("aging")
    obj = packed_object(dop, "pieces", path)
    solver = box3d_solver(dop, merge(dop, [ground_object(dop, "aging_ground"), obj]))
    set_parm(solver, ["enableaging"], 1)
    finish(dop, solver)

    def sample(sim, f):
        return {n: point_attributes(sim, "pieces", n, ["age", "dead", "name"]) for n in names}

    track = step(dop, 25, sample)

    def gone(i):
        return next((f for f in sorted(track) if track[f][names[i]] is None or track[f][names[i]].get("name") != names[i]),
                    None)

    long_lived = track[25][names[2]]
    age = long_lived["age"]
    rows = [f"life 100 piece: age {age:.4f} on frame 25 (1.0 expected), dead={long_lived['dead']}",
            f"life 0.5 piece gone on frame {gone(0)} (expected 14: age 0.5 passes on step 13), "
            f"life 0 piece gone on frame {gone(1)} (expected 2)"]
    ok = close(age, 1.0, 1e-3) and not long_lived["dead"] and gone(0) in (13, 14) and gone(1) == 2
    return [Result("aging", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Point force / torque (status: world N and N*m, one step, then zeroed)
# ---------------------------------------------------------------------------


def test_point_force():
    """A point `force` of 24 N on a 1 kg box (no gravity) adds 1 m/s in its first 1/24 s step and
    is then zeroed; a `torque` of 1 N*m on a 1 kg unit box (I = 1/6) adds 0.25 rad/s."""
    pieces = [
        {"size": (1, 1, 1), "center": (0, 0, 0), "attrs": {"force": (24.0, 0.0, 0.0), "torque": (0.0, 0.0, 0.0)}},
        {"size": (1, 1, 1), "center": (5, 0, 0), "attrs": {"force": (0.0, 0.0, 0.0), "torque": (0.0, 0.0, 1.0)}},
    ]
    path, names = packed_boxes("force_pieces", pieces)
    dop = new_dopnet("force")
    obj = packed_object(dop, "pieces", path, computemass=0, mass=1.0)
    finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))

    def sample(sim, f):
        return {n: point_attributes(sim, "pieces", n, ["v", "w", "force"]) for n in names}

    track = step(dop, 6, sample)
    v2, v6 = track[2][names[0]]["v"][0], track[6][names[0]]["v"][0]
    f3 = track[3][names[0]]["force"]
    w2 = track[2][names[1]]["w"][2]
    ok = close(v2, 1.0, 0.05) and close(v6, v2, 1e-3) and f3 is not None and max(abs(c) for c in f3) < 1e-6 \
        and close(w2, 0.25, 0.02)
    return [Result("point_force", Result.PASS if ok else Result.FAIL,
                   f"force: v.x {v2:.3f} after the first step, {v6:.3f} on frame 6 (expected 1 and unchanged), force "
                   f"on frame 3 = {f3}; torque: w.z {w2:.3f} (expected 0.25)")]


# ---------------------------------------------------------------------------
# Gravity Force DOP adds to the solver's Gravity (status: Force DOPs evaluated once per step)
# ---------------------------------------------------------------------------


def test_gravity_dop():
    """A Gravity Force DOP pulls at g with the solver's Gravity at 0, and adds to it (2g) with the
    solver's own Gravity on."""
    rows, ok = [], True
    for solver_g, expected in (((0, 0, 0), -G), (None, -2 * G)):
        tag = "own_off" if solver_g else "own_on"
        path, names = packed_boxes("gravity_" + tag, [{"size": (1, 1, 1), "center": (0, 50, 0)}])
        dop = new_dopnet("gravity_" + tag)
        solver = box3d_solver(dop, packed_object(dop, "piece", path), gravity=solver_g)
        grav = create(dop, "gravity", "gravity_force")
        grav.setInput(0, solver)
        finish(dop, grav)
        vy = step(dop, 25, lambda sim, f: point_attributes(sim, "piece", names[0], ["v"]))[25]["v"][1]
        rows.append(f"solver Gravity {'0' if solver_g else 'on'}: v.y after 1 s = {vy:.2f} (expected {expected:.2f})")
        ok = ok and close(vy, expected, 0.03 * abs(expected))
    return [Result("gravity_dop", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_force_dops_many_and_sampled():
    """Force DOPs on many bodies and over a body's size. 300 boxes under a Gravity Force DOP (the Force
    DOPs of an object are evaluated in parallel from a few hundred bodies) all take the same pull:
    v.y = -g after 1 s. A Drag Force DOP slows a 2 x 2 x 2 box by a different amount when its Sample Mode
    is Sphere or Circle (force over the body's size) than at a Point (INFO: the ratios)."""
    results = []
    count = 300
    pieces = [{"size": (1, 1, 1), "center": (3.0 * i, 50, 0)} for i in range(count)]
    path, names = packed_boxes("force_many", pieces)
    dop = new_dopnet("force_many")
    solver = box3d_solver(dop, packed_object(dop, "piece", path), gravity=(0, 0, 0))
    grav = create(dop, "gravity", "gravity_force")
    grav.setInput(0, solver)
    finish(dop, grav)
    attrs = step(dop, 25, lambda sim, f: [point_attributes(sim, "piece", n, ["v"])["v"][1] for n in names]
                 if f == 25 else None)[25]
    lo, hi = min(attrs), max(attrs)
    ok = close(lo, -G, 0.03 * G) and close(hi, -G, 0.03 * G)
    results.append(Result("force_dops_many_and_sampled", Result.PASS if ok else Result.FAIL,
                          f"{count} boxes under a Gravity Force DOP: v.y after 1 s from {lo:.3f} to {hi:.3f} "
                          f"(expected {-G:.3f} for each)"))

    slowed = {}
    for mode, label in ((1, "Point"), (2, "Circle"), (3, "Sphere")):
        path, names = packed_boxes(f"force_drag_{label.lower()}",
                                   [{"size": (2, 2, 2), "center": (0, 50, 0), "attrs": {"v": (10.0, 0.0, 0.0)}}])
        dop = new_dopnet(f"force_drag_{label.lower()}")
        solver = box3d_solver(dop, packed_object(dop, "piece", path, keep_velocity=True), gravity=(0, 0, 0))
        drag = create(dop, "drag", "drag_force")
        drag.setInput(0, solver)
        set_parm(drag, ["samplemode"], mode)
        finish(dop, drag)
        vx = step(dop, 7, lambda sim, f: point_attributes(sim, "piece", names[0], ["v"]))[7]["v"][0]
        slowed[label] = 10.0 - vx
    point = slowed["Point"]
    results.append(Result("force_dops_many_and_sampled",
                          Result.PASS if point > 1e-4 and all(
                              abs(slowed[k] - point) > 0.1 * point for k in ("Circle", "Sphere")) else Result.FAIL,
                          "Drag Force DOP on a 2 x 2 x 2 box going 10 m/s, speed lost in 0.25 s: "
                          + ", ".join(f"{k} {v:.4f}" for k, v in slowed.items())
                          + " (expected Circle and Sphere to differ from Point, which they do when the force is "
                            "evaluated over the body's size)"))
    if point > 1e-4:
        results.append(Result("force_dops_many_and_sampled", Result.INFO,
                              f"ratios to Point: Circle {slowed['Circle'] / point:.2f}, Sphere {slowed['Sphere'] / point:.2f}"))
    return results


def _stderr_lines(run):
    """Calls run() with the process's stderr (file descriptor 2) going to a file, so the plugin's own [b3dop]
    lines can be read; returns the lines."""
    import sys

    sys.stderr.flush()
    saved = os.dup(2)
    with tempfile.TemporaryFile(mode="w+b") as tmp:
        os.dup2(tmp.fileno(), 2)
        try:
            run()
        finally:
            sys.stderr.flush()
            os.dup2(saved, 2)
            os.close(saved)
        tmp.seek(0)
        return tmp.read().decode("utf-8", "replace").splitlines()


def test_force_dops_threadsafe():
    """Which Force DOPs the solver evaluates on Houdini's thread pool. Force DOPs of an object are evaluated in
    parallel from `kMinParallelForces` bodies when they are sampled at a point and their resolver says it is
    thread-safe (`threadsafe()`, which Python cannot ask). A debug build (BOX3DDOP_DEBUG) logs each force's verdict
    when it splits them (the test lowers the body count to 1 with BOX3DDOP_MIN_PARALLEL_FORCES); without one this
    is INFO only. Gravity and Drag Force DOPs go to the pool (PASS); the others are listed (INFO)."""
    kinds = ["gravity", "drag", "windforce", "fan", "uniformforce", "buoyancyforce", "vortexforce", "magnetforce",
             "pointforce", "fieldforce", "vopforce", "referenceframeforce", "popforce", "popdrag", "popwind",
             "popfan", "popattract", "impulseforce"]
    path, _names = packed_boxes("force_threads", [{"size": (1, 1, 1), "center": (0, 50, 0)}])
    dop = new_dopnet("force_threads")
    last = box3d_solver(dop, packed_object(dop, "piece", path), gravity=(0, 0, 0))
    made = []
    for kind in kinds:
        try:
            node = dop.createNode(kind, "f_" + kind)
        except hou.OperationFailed:
            continue
        node.setInput(0, last)
        last = node
        made.append(kind)
    finish(dop, last)
    lines = [l for l in _stderr_lines(lambda: step(dop, 2, lambda sim, f: None)) if l.startswith("[b3dop] force ")]
    if not lines:
        return [Result("force_dops_threadsafe", Result.INFO,
                       "no [b3dop] force lines: needs a BOX3DDOP_DEBUG build (cmake -DBOX3DDOP_DEBUG=ON)")]
    verdicts = {}
    for l in lines:
        name, _, rest = l[len("[b3dop] force "):].partition(": ")
        verdicts[name] = rest
    results = [Result("force_dops_threadsafe", Result.INFO,
                      f"{len(made)} Force DOPs wired ({', '.join(made)}); "
                      + "; ".join(f"{n}: {v.rsplit(': ', 1)[-1]}" for n, v in sorted(verdicts.items())))]
    for n, v in sorted(verdicts.items()):
        results.append(Result("force_dops_threadsafe", Result.INFO, f"{n}: {v}"))
    gravity = [v for n, v in verdicts.items() if "ravity" in n]
    drag = [v for n, v in verdicts.items() if "rag" in n and "pop" not in n.lower()]
    ok = bool(gravity) and all(v.endswith("parallel") for v in gravity)
    results.append(Result("force_dops_threadsafe", Result.PASS if ok else Result.FAIL,
                          f"Gravity Force: {gravity or 'not logged'}; Drag Force: {drag or 'not logged'}"))
    return results


test_force_dops_threadsafe.env = {"BOX3DDOP_MIN_PARALLEL_FORCES": "1"}


TESTS = {
    "written_velocity": test_written_velocity,
    "velocity_edit": test_velocity_edit,
    "impacts": test_impacts,
    "impacts_substeps": test_impacts_substeps,
    "mass_written": test_mass_written,
    "aging": test_aging,
    "point_force": test_point_force,
    "gravity_dop": test_gravity_dop,
    "force_dops_many_and_sampled": test_force_dops_many_and_sampled,
    "force_dops_threadsafe": test_force_dops_threadsafe,
}
