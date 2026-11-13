"""Regression checks: every Box3D joint type and every kind of body doing what defines it, and a
baseline scene whose exact result is kept in baseline.json, so a change of Box3D, Houdini or the
plugin that moves anything shows up as a FAIL (run with --update-baseline after an intended
change).

The joint checks use the scene of checks_constraints.hanging_pair: a fixed piece at (0, 2, 0) and
a 1 x 1 x 1 piece at (1, 2, 0), both anchors on their shared face (0.5, 2, 0) with the anchors'
default orientation, so the joint frames are the world axes (revolute: about z; prismatic and
the wheel's suspension: along x)."""

import json
import math
import os

import hou

from b3dtest import (
    Result,
    box3d_solver,
    close,
    constraint_lines,
    create,
    find_network_geometry,
    display,
    finish,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    packed_transform,
    parm_value,
    point_attributes,
    set_parm,
    sop_solver,
    step,
)
from checks_constraints import FACE, GROUND, hanging_pair

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "baseline.json")
UPDATE_ENV = "BOX3D_TEST_UPDATE_BASELINE"  # set by run_tests.py --update-baseline
START = (1.0, 2.0, 0.0)  # the moving piece's centroid at the start


# ---------------------------------------------------------------------------
# Small measures
# ---------------------------------------------------------------------------


def turned_deg(m):
    """How far a 3x3 (rows) has turned from the identity, in degrees."""
    trace = m[0][0] + m[1][1] + m[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def z_tilt_deg(m):
    """Angle between the piece's own z axis (third row) and world z, in degrees."""
    z = m[2]
    n = math.sqrt(sum(c * c for c in z)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(z[2]) / n))))


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def moving_state(sim, name):
    """(position, 3x3 rows, w) of the moving piece."""
    return (packed_positions(sim, "pieces")[name], packed_transform(sim, "pieces", name),
            point_attributes(sim, "pieces", name, ["w"])["w"])


def pair(tag, conrel, parms=None, v=None, w=None, gravity=None, anchors=None, unit_mass=False, joints=None):
    """hanging_pair with an optional initial v / w on the moving piece. Returns (dop, names)."""
    moving = {"active": 1}
    fixed = {"active": 0}
    if v is not None or w is not None:
        moving.update({"v": tuple(v or (0, 0, 0)), "w": tuple(w or (0, 0, 0))})
        fixed.update({"v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0)})
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": fixed},
              {"size": (1, 1, 1), "center": START, "attrs": moving}]
    return hanging_pair(f"jt_{tag}", joints, conrels=[(conrel, parms or {})] if conrel else None,
                        gravity=gravity, pieces=pieces, anchors=anchors, keep_velocity=v is not None or w is not None,
                        packed_parms={"computemass": 0, "mass": 1.0} if unit_mass else None)


# ---------------------------------------------------------------------------
# Every Box3D joint type
# ---------------------------------------------------------------------------


def joint_weld():
    dop, n = pair("weld", "box3d_weldconrel")
    p, m, _ = moving_state_at(dop, n[1], 48)
    ok = dist(p, START) < 0.02 and turned_deg(m) < 2.0
    return ok, f"weld: moved {dist(p, START):.3f}, turned {turned_deg(m):.1f} deg after 2 s (expected held: < 0.02, < 2)"


def joint_spherical():
    dop, n = pair("spherical", "box3d_sphericalconrel", v=(0, 0, 2))
    p, low = swing_track(dop, n[1])
    r = dist(p, FACE)
    ok = close(r, 0.5, 0.03) and abs(p[2]) > 0.1 and low < 1.7
    return ok, (f"spherical (pushed along z): centroid {r:.3f} from the anchor (expected 0.5), z {p[2]:.2f} "
                f"(expected off the plane), lowest y {low:.2f} (expected swung below 1.7)")


def joint_revolute():
    dop, n = pair("revolute", "box3d_revoluteconrel", v=(0, 0, 2))
    p, low = swing_track(dop, n[1])
    r = dist(p, FACE)
    ok = close(r, 0.5, 0.03) and abs(p[2]) < 0.02 and low < 1.7
    return ok, (f"revolute (hinge about z, pushed along z): centroid {r:.3f} from the anchor (expected 0.5), "
                f"z {p[2]:.3f} (expected 0: stays in its plane), lowest y {low:.2f} (expected swung below 1.7)")


def joint_prismatic():
    dop, n = pair("prismatic", "box3d_prismaticconrel", v=(1, 0, 0.5))
    p, m, _ = moving_state_at(dop, n[1], 48)
    ok = p[0] > 2.5 and close(p[1], 2.0, 0.02) and abs(p[2]) < 0.02 and turned_deg(m) < 2.0
    return ok, (f"prismatic (slides along x, pushed along x and z): x {p[0]:.2f} (expected about 3), y {p[1]:.3f} "
                f"(expected 2: no fall), z {p[2]:.3f} (expected 0), turned {turned_deg(m):.1f} deg (expected 0)")


def joint_distance():
    # Anchors at the centroids: a 1-unit rod with both ends free to turn.
    dop, n = pair("distance", "box3d_distanceconrel", anchors=((0, 2, 0), START))
    p, low = swing_track(dop, n[1])
    r = dist(p, (0, 2, 0))
    ok = close(r, 1.0, 0.03) and low < 1.2
    return ok, (f"distance (anchors at the centroids): {r:.3f} apart (expected 1.0), lowest y {low:.2f} "
                "(expected swung below 1.2)")


def joint_motor():
    dop, n = pair("motor", "box3d_motorconrel", gravity=(0, 0, 0), unit_mass=True,
                  parms={"linear_velocity": (1.0, 0.0, 0.0), "max_velocity_force": 1000.0,
                         "max_velocity_torque": 1000.0})
    p, m, _ = moving_state_at(dop, n[1], 48)
    dx = p[0] - START[0]
    ok = close(dx, 2.0, 0.15) and close(p[1], 2.0, 0.02) and turned_deg(m) < 2.0
    return ok, (f"motor (Linear Velocity 1 0 0, no gravity): moved {dx:.2f} along x in 2 s (expected 2), y {p[1]:.3f}, "
                f"turned {turned_deg(m):.1f} deg (expected 0)")


def joint_parallel():
    # No position constraint, only the z axes kept parallel: it drifts down freely, its tilt is undone.
    dop, n = pair("parallel", "box3d_parallelconrel", gravity=(0, 0, 0), unit_mass=True, v=(0, -1, 0), w=(2, 0, 0),
                  parms={"stiffness": 5.0, "damping_ratio": 1.0})
    p, m, _ = moving_state_at(dop, n[1], 48)
    ok = close(p[1], 0.0, 0.1) and z_tilt_deg(m) < 10.0
    return ok, (f"parallel (drifting down at 1 m/s, spun about x at 2 rad/s, no gravity): y {p[1]:.2f} after 2 s "
                f"(expected 0: position free), z axis {z_tilt_deg(m):.1f} deg off (expected under 10; free it "
                "would have turned 229)")


def joint_wheel():
    rows, ok = [], True
    # Both anchors at the wheel's centre: a wheel spins about its anchor (on the face it would
    # crank about an edge point). Free: the spin about z (frame B) is kept, x / y turning stopped.
    centre = (START, START)
    dop, n = pair("wheel", "box3d_wheelconrel", gravity=(0, 0, 0), unit_mass=True, w=(2, 0, 3), anchors=centre)
    p, _, w = moving_state_at(dop, n[1], 24)
    good = close(w[2], 3.0, 0.3) and abs(w[0]) < 0.1 and abs(w[1]) < 0.1 and close(p[1], 2.0, 0.02)
    rows.append(f"wheel (spun 2 rad/s about x and 3 about z, no gravity): w ({w[0]:.2f}, {w[1]:.2f}, {w[2]:.2f}) "
                f"after 1 s (expected about (0, 0, 3)), y {p[1]:.3f}")
    ok = ok and good
    # Spin motor at the node's 360 deg/s.
    dop, n = pair("wheel_motor", "box3d_wheelconrel", gravity=(0, 0, 0), unit_mass=True, anchors=centre,
                  parms={"enable_spin_motor": 1, "spin_speed": 360.0})
    _, _, w = moving_state_at(dop, n[1], 48)
    good = close(abs(w[2]), 2 * math.pi, 0.3)
    rows.append(f"spin motor 360 deg/s: |w.z| {abs(w[2]):.2f} rad/s (expected 6.28)")
    return ok and good, "; ".join(rows)


def joint_filter():
    # The moving piece starts overlapping the fixed one: with the filter it falls through freely.
    overlap = ((0.4, 2, 0), (0.4, 2, 0))
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0.4, 2, 0), "attrs": {"active": 1}}]
    dop, n = hanging_pair("jt_filter", None, conrels=[("box3d_filterconrel", {})], pieces=pieces, anchors=overlap)
    p = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces")[n[1]])[24]
    dop_c, n_c = hanging_pair("jt_filter_control", "no_such_constraint", pieces=pieces, anchors=overlap)
    pc = step(dop_c, 24, lambda sim, f: packed_positions(sim, "pieces")[n_c[1]])[24]
    # A piece made on frame 1 moves from the next step: 23 frames of fall.
    t = 23 / hou.fps()
    fall_y = 2.0 - 0.5 * 9.80665 * t * t
    free = close(p[1], fall_y, 0.15) and close(p[0], 0.4, 0.02)
    pushed = dist(pc, p) > 0.05
    return free and pushed, (f"filter (pieces overlapping): with the filter at ({p[0]:.2f}, {p[1]:.2f}) after 1 s "
                             f"(expected free fall to about (0.40, {fall_y:.2f})); without it at ({pc[0]:.2f}, {pc[1]:.2f}) "
                             "(expected pushed apart, somewhere else)")


def moving_state_at(dop, name, frame):
    return step(dop, frame, lambda sim, f: moving_state(sim, name) if f == frame else None)[frame]


def swing_track(dop, name, frame=48):
    """(final position, lowest y over the run) of the moving piece: a pendulum can be back near
    the top on any one frame, so "it swung" is judged by the lowest point."""
    track = step(dop, frame, lambda sim, f: packed_positions(sim, "pieces")[name])
    return track[frame], min(p[1] for p in track.values())


JOINTS = (joint_weld, joint_spherical, joint_revolute, joint_prismatic, joint_distance, joint_motor,
          joint_parallel, joint_wheel, joint_filter)


def test_joint_types():
    """Each of the nine Box3D joint types, at its node defaults (plus the settings named in its
    line), between a fixed piece and a hanging one: weld holds position and rotation; spherical
    keeps the anchor point but lets the piece swing out of plane; revolute swings only about z;
    prismatic slides only along x without turning; distance keeps the length; motor drives the
    piece at its linear velocity; parallel keeps the z axes together but not the position; wheel
    spins freely about z only, and its spin motor drives it; filter lets overlapping pieces pass
    through each other."""
    results = []
    for check in JOINTS:
        ok, msg = check()
        results.append(Result("joint_types", Result.PASS if ok else Result.FAIL, msg))
    return results


def test_anchor_positions():
    """The constraint network's anchors after a prismatic piece slides 0.5 along x (to its limit):
    P stays the authored anchor (Houdini reads P as the anchor's initial position and moves its
    constraint guide with the piece itself; a live P was drawn twice as far), and current_P holds
    where the anchor is now, 0.5 further along."""
    dop, n = pair("anchors", "box3d_prismaticconrel", gravity=(5, 0, 0),
                  parms={"enable_limit": 1, "lower_translation": -0.5, "upper_translation": 0.5})

    def sample(sim, f):
        if f != 48:
            return None
        geo = find_network_geometry(sim)
        if geo is None:
            return None
        pt = next((q for q in geo.points() if q.attribValue("name") == n[1]), None)
        if pt is None:
            return None
        cur = pt.attribValue("current_P") if geo.findPointAttrib("current_P") is not None else None
        return tuple(pt.position()), cur

    got = step(dop, 48, sample)[48]
    if got is None:
        return [Result("anchor_positions", Result.FAIL, "no constraint network geometry with the moving piece's anchor")]
    p, cur = got
    slid = (FACE[0] + 0.5, FACE[1], FACE[2])
    ok_p = dist(p, FACE) < 1e-4
    ok_cur = cur is not None and dist(cur, slid) < 0.02
    cur_txt = f"({cur[0]:.3f}, {cur[1]:.3f}, {cur[2]:.3f})" if cur is not None else "missing"
    return [Result("anchor_positions", Result.PASS if ok_p and ok_cur else Result.FAIL,
                   f"moving piece's anchor: P ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) (expected the authored "
                   f"{FACE}), current_P {cur_txt} (expected about {slid})")]


def test_sidefx_soft():
    """SideFX's Bullet Soft Constraint Relationship (10 Hz, damping ratio 1; Rest Length set to 0,
    since its default 1 pushes the anchors a unit apart) holding a hanging piece, in Box3D and in
    Bullet: both keep it within a few centimetres of where it started."""
    out = {}
    for kind in ("box3d", "bullet"):
        dop, n = hanging_pair(f"soft_{kind}", None, conrels=[("bulletsoftconrel", {"restlength": 0.0})],
                            solver_kind=kind)
        out[kind] = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces")[n[1]] if f == 48 else None)[48]
    db, dt = dist(out["box3d"], START), dist(out["bullet"], START)
    ok = db < 0.1 and abs(db - dt) < 0.1
    return [Result("sidefx_soft", Result.PASS if ok else Result.FAIL,
                   f"moved {db:.3f} in Box3D, {dt:.3f} in Bullet after 2 s (expected both under 0.1)")]


# ---------------------------------------------------------------------------
# Kinds of body
# ---------------------------------------------------------------------------


def test_body_types():
    """Every kind of body in one scene over a ground, after 2 s: a dynamic box lands (y 0.5); a
    static one (active 0) stays at 3 and a box dropped on it rests on top (4.0); a disabled one
    (enabled 0) stays at 3 and a box dropped on it falls through to the ground; a shapeless one
    (bullet_georep none) stays and a box falls through it too; a kinematic one (animated 1, moved
    up 1 unit/s by a SOP Solver) follows its animation to y 5."""
    def box(x, y, **attrs):
        base = {"active": 1, "animated": 0, "enabled": 1, "bullet_georep": "box"}
        base.update(attrs)
        return {"size": (1, 1, 1), "center": (x, y, 0), "attrs": base}

    labels = ["ground", "dynamic", "static", "on_static", "disabled", "through_disabled", "none", "through_none",
              "kinematic"]
    pieces = [dict(GROUND, attrs={"active": 0, "animated": 0, "enabled": 1, "bullet_georep": "box"}),
              box(-3, 3), box(0, 3, active=0), box(0, 5), box(3, 3, enabled=0), box(3, 5),
              box(6, 3, active=0, bullet_georep="none"), box(6, 5), box(9, 3, active=0, animated=1)]
    path, names = packed_boxes("bodies_pieces", pieces)
    piece = dict(zip(labels, names))
    dop = new_dopnet("bodies")
    obj = packed_object(dop, "pieces", path)
    mover = sop_solver(dop, f'if (s@name == "{piece["kinematic"]}") @P.y = 3 + @Time;')
    finish(dop, multisolver(dop, obj, [mover, box3d_solver(dop, None)]))
    final = step(dop, 49, lambda sim, f: packed_positions(sim, "pieces") if f == 49 else None)[49]
    want = {"dynamic": (0.5, 0.05), "static": (3.0, 0.001), "on_static": (4.0, 0.05), "disabled": (3.0, 0.001),
            "through_disabled": (0.5, 0.05), "none": (3.0, 0.001), "through_none": (0.5, 0.05),
            "kinematic": (3.0 + 48 / hou.fps(), 0.05)}
    results = []
    for label, (y, tol) in want.items():
        got = final[piece[label]][1]
        results.append(Result("body_types", Result.PASS if close(got, y, tol) else Result.FAIL,
                              f"{label}: y {got:.3f} (expected {y:g})"))
    return results


def cup_scene(tag, georep):
    """A static cup (a 2 x 2 floor and two walls packed into ONE primitive, each a named piece)
    with bullet_georep `georep`, and a ball dropped into it. Returns (dop, ball name)."""
    geo = new_geo(f"cup_{tag}")
    parts = []
    for i, (label, size, center) in enumerate((("floor", (2, 0.2, 2), (0, 0.1, 0)),
                                               ("left", (0.2, 1, 2), (-0.9, 0.5, 0)),
                                               ("right", (0.2, 1, 2), (0.9, 0.5, 0)))):
        b = create(geo, "box", label)
        set_parm(b, ["size"], size)
        set_parm(b, ["t"], center)
        named = create(geo, "attribwrangle", f"name_{label}")
        named.setInput(0, b)
        set_parm(named, ["class"], 1)  # primitives
        set_parm(named, ["snippet"], f's@name = "{label}";')
        parts.append(named)
    m = create(geo, "merge", "parts")
    for i, p in enumerate(parts):
        m.setInput(i, p)
    pack = create(geo, "pack", "pack")
    pack.setInput(0, m)
    attrs = create(geo, "attribwrangle", "attrs")
    attrs.setInput(0, pack)
    set_parm(attrs, ["snippet"], f's@name = "cup"; i@active = 0; i@animated = 0; s@bullet_georep = "{georep}"; '
                                 'v@v = {0, 0, 0}; v@w = {0, 0, 0};')
    display(attrs)
    ball_path, ball = packed_boxes(f"cup_{tag}_ball", [{"size": (0.5, 0.5, 0.5), "center": (0, 2.5, 0),
                                                        "attrs": {"active": 1, "bullet_georep": "sphere"}}])
    dop = new_dopnet(f"cup_{tag}")
    cup = packed_object(dop, "cup", attrs.path())
    ball_obj = packed_object(dop, "ball", ball_path)
    finish(dop, box3d_solver(dop, merge(dop, [cup, ball_obj])))
    return dop, ball[0]


def test_shape_kinds():
    """A ball (AutoFit sphere, radius 0.43) dropped into a static cup made of three named pieces
    packed into one primitive: with concave or compound (one hull per piece) it lands inside, on
    the floor (y about 0.63); with convexhull (one hull round everything) it rests on the hull's
    top, level with the walls (about 1.43)."""
    results = []
    for georep, inside in (("concave", True), ("compound", True), ("convexhull", False)):
        dop, ball = cup_scene(georep, georep)
        y = step(dop, 48, lambda sim, f: packed_positions(sim, "ball")[ball][1] if f == 48 else None)[48]
        ok = (y < 1.0) if inside else (y > 1.2)
        results.append(Result("shape_kinds", Result.PASS if ok else Result.FAIL,
                              f"{georep}: ball at y {y:.3f} (expected {'inside, about 0.63' if inside else 'on top, about 1.43'})"))
    return results


# ---------------------------------------------------------------------------
# Baseline: one fixed scene whose exact result is recorded
# ---------------------------------------------------------------------------

SHAPES = ("box", "sphere", "capsule", "convexhull")


def baseline_scene():
    """A mixed pile of 24 pieces (box / sphere / capsule / convex hull, tilted in their SOPs) and a
    4-link revolute chain off a fixed piece, over a ground. Changing anything here changes the
    baseline: run with --update-baseline afterwards. Returns (dop, {label: piece name})."""
    pieces, labels = [GROUND], ["ground"]
    for i in range(24):
        pieces.append({"size": (0.6, 0.4, 0.8),
                       "center": ((i % 4) * 0.8 - 1.2, 1.0 + (i // 4) * 0.7, ((i * 7) % 5) * 0.2 - 0.4),
                       "rotate": ((i * 37) % 90, (i * 53) % 90, (i * 19) % 90),
                       "attrs": {"active": 1, "bullet_georep": SHAPES[i % 4]}})
        labels.append(f"pile{i:02d}")
    for k in range(5):  # chain: link 0 fixed
        pieces.append({"size": (1, 0.3, 0.3), "center": (4.0 + k, 4.0, 0),
                       "attrs": {"active": 0 if k == 0 else 1, "bullet_georep": "box"}})
        labels.append(f"chain{k}")
    pieces[0] = dict(GROUND, attrs={"active": 0, "bullet_georep": "box"})
    path, names = packed_boxes("baseline_pieces", pieces)
    label_of = dict(zip(labels, names))
    dop = new_dopnet("baseline")
    obj = packed_object(dop, "pieces", path)
    hinge = create(dop, "box3d_revoluteconrel", "hinge")
    data_name = parm_value(hinge, ["dataname", "data_name", "DataName"], "revolute")
    lines = [(label_of[f"chain{k}"], (4.5 + k, 4.0, 0), label_of[f"chain{k + 1}"], (4.5 + k, 4.0, 0), data_name)
             for k in range(4)]
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines("baseline_network", lines))
    net.setInput(1, hinge)
    finish(dop, box3d_solver(dop, net))
    return dop, label_of


def snapshot(sim, label_of):
    """{label: [x, y, z, then the 3x3 rows]} of every piece, rounded."""
    geo = sim.findObject("pieces").geometry()
    by_name = {}
    for pr in geo.prims():
        pts = pr.points()
        if pts:
            m = pr.intrinsicValue("packedfulltransform")
            by_name[pts[0].attribValue("name")] = list(pts[0].position()) + list(m[0:3]) + list(m[4:7]) + list(m[8:11])
    return {label: [round(v, 6) for v in by_name[name]] for label, name in label_of.items() if name in by_name}


def test_baseline():
    """A fixed mixed scene (24 tilted boxes, spheres, capsules and hulls in a pile, a 4-link
    revolute chain) simulated for 2 s and compared with the result recorded in baseline.json:
    every piece within 1e-3 of where it was, and turned the same. Recorded on the first run (INFO);
    after an intended change of behaviour, run_tests.py --update-baseline records it again. A FAIL
    means something changed the simulation: the plugin, Box3D or Houdini."""
    dop, label_of = baseline_scene()
    got = step(dop, 48, lambda sim, f: snapshot(sim, label_of) if f == 48 else None)[48]
    record = {"houdini": hou.applicationVersionString(), "frame": 48, "pieces": got}
    if os.environ.get(UPDATE_ENV) or not os.path.exists(BASELINE):
        first = not os.path.exists(BASELINE)
        with open(BASELINE, "w") as f:
            json.dump(record, f, indent=1, sort_keys=True)
        return [Result("baseline", Result.INFO, f"baseline {'recorded' if first else 'updated'}: {len(got)} pieces "
                                                f"(Houdini {record['houdini']}), in {os.path.basename(BASELINE)}")]
    with open(BASELINE) as f:
        old = json.load(f)
    diffs = []
    for label, now in got.items():
        before = old["pieces"].get(label)
        if before is None:
            diffs.append((math.inf, label, "new piece"))
            continue
        dp = max(abs(now[i] - before[i]) for i in range(3))
        dr = max(abs(now[i] - before[i]) for i in range(3, 12))
        diffs.append((max(dp, dr), label, f"moved {dp:.2e}, rotation {dr:.2e}"))
    missing = sorted(set(old["pieces"]) - set(got))
    diffs.sort(reverse=True)
    worst = diffs[0][0] if diffs else 0.0
    ok = worst <= 1e-3 and not missing
    top = "; ".join(f"{label} {what}" for _, label, what in diffs[:3])
    msg = (f"largest difference from the baseline {worst:.2e} (allowed 1e-3) over {len(got)} pieces; "
           f"worst: {top}" + (f"; missing: {', '.join(missing)}" if missing else "")
           + f". Baseline from Houdini {old.get('houdini', '?')}, this run {record['houdini']}")
    return [Result("baseline", Result.PASS if ok else Result.FAIL, msg)]


TESTS = {
    "joint_types": test_joint_types,
    "anchor_positions": test_anchor_positions,
    "sidefx_soft": test_sidefx_soft,
    "body_types": test_body_types,
    "shape_kinds": test_shape_kinds,
    "baseline": test_baseline,
}
