"""Joint node toggles (limit, motor, prismatic limit, spring rest length, Breakable) and
negative "never" values."""

import math

from b3dtest import (
    close,
    packed_positions,
    point_attributes,
    Result,
    step,
)
from checks_constraints import FACE, hanging_pair


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


def test_joint_attributes_replace():
    """A `lower_angle` primitive attribute of 0.5 on a revolute joint with Enable Limit (node limits +-45):
    by Houdini's rule it multiplies, so the piece stops at -22.5 degrees. With Attributes Replace Values on
    it is the limit itself, 0.5 degrees, so the piece hardly swings down."""
    rows, ok = [], True
    for replace in (0, 1):
        dop, names = hanging_pair(f"attributes_replace{replace}", None, line_attrs={"lower_angle": 0.5},
                                  conrels=[("box3d_revoluteconrel", {"enable_limit": 1,
                                                                     "attributes_replace": replace})])
        track = step(dop, 72, lambda sim, f: swing_angle(packed_positions(sim, "pieces")[names[1]]))
        lowest = min(track.values())
        expected = 0.5 if replace else -22.5
        good = close(lowest, expected, 3.0)
        rows.append(f"replace {'on' if replace else 'off'}: lowest angle {lowest:.1f} (expected {expected:g})")
        ok = ok and good
    return [Result("joint_attributes_replace", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


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


def test_weld_hold_separation():
    """A Weld whose anchors are the two pieces' own centroids, 1 apart, no gravity. With Hold Separation on
    the pieces stay 1 apart (the anchors' offset is kept) and do not turn; the unheld Weld pulls the
    anchors together instead, so that distance is only reported."""
    rows, ok = [], True
    for hold in (1, 0):
        dop, names = hanging_pair(f"weld_hold{hold}", "weld", conrels=[("box3d_weldconrel", {"hold_separation": hold})],
                                  gravity=(0, 0, 0), anchors=((0.0, 2.0, 0.0), (1.0, 2.0, 0.0)))
        track = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces") if f == 48 else None)[48]
        a, b = track[names[0]], track[names[1]]
        gap = math.dist(a, b)
        if hold:
            good = close(gap, 1.0, 0.03)
            ok = ok and good
        rows.append(f"Hold Separation {'on' if hold else 'off'}: pieces {gap:.3f} apart after 2 s"
                    f"{' (expected 1)' if hold else ''}")
    return [Result("weld_hold_separation", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS = {
    "joint_revolute_limit": test_joint_revolute_limit,
    "joint_revolute_motor": test_joint_revolute_motor,
    "joint_attributes_replace": test_joint_attributes_replace,
    "weld_hold_separation": test_weld_hold_separation,
    "joint_prismatic_limit": test_joint_prismatic_limit,
    "joint_spring_rest_length": test_joint_spring_rest_length,
    "joint_breakable": test_joint_breakable,
    "negative_never_values": test_negative_never_values,
}
