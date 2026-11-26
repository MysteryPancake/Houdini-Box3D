"""Friction and bounce mixing: the solver's mixing rules against measured slides and bounces."""

import math

from b3dtest import (
    box3d_solver,
    bullet_solver,
    close,
    finish,
    GROUND,
    new_dopnet,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_menu_token,
    set_parm,
    step,
)
from checks_common import G, MIXING, mix


# ---------------------------------------------------------------------------
# Friction and bounce mixing
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
            set_menu_token(solver, ["frictionmixing"], str(MIXING.index(mixing)))
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
            set_menu_token(solver, ["restitutionmixing"], str(MIXING.index(mixing)))
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
    Bullet; the measured values against each Box3D mixing rule. INFO. Bullet multiplies bounce, as its
    SIM_BulletState::simResolveCollision does, but 0.2 with 0.8 (0.16 multiplied, 0.20 as a minimum)
    is too close to tell by the rebound (which reads a little over the rule), so the second result drops
    a box of 0.5 and 0.8 on ground of 0.5, where the rules are far apart, in Bullet and in Box3D with its
    default Bounce Mixing: PASS when the two rebounds agree within 0.08 (multiply 0.25 and 0.40, minimum 0.5)."""
    rules = ("Geometric Mean", "Minimum", "Maximum", "Average", "Multiply")
    v0 = 3.0
    x, _ = slide_distance("bullet_mix_slide", 0.2, 0.8, v0=v0, solver_kind="bullet")
    mu = v0 * v0 / (2 * G * x) if x > 1e-6 else float("inf")
    e, _ = bounce_drop("bullet_mix_bounce", 0.2, 0.8, solver_kind="bullet")

    def nearest(measured):
        return min(rules, key=lambda r: abs(mix(r, 0.2, 0.8) - measured))

    table = ", ".join(f"{r} {mix(r, 0.2, 0.8):.2f}" for r in rules)
    results = [Result("bullet_mixing", Result.INFO,
                      f"Bullet friction: slid {x:.3f} from 3 m/s, so friction {mu:.3f}, nearest rule {nearest(mu)}; "
                      f"bounce {e:.3f}, nearest rule {nearest(e)} (0.2 with 0.8 under each rule: {table})")]
    rows, ok = [], True
    for e_box, e_ground in ((0.5, 0.5), (0.8, 0.5)):
        tag = f"{int(e_box * 10)}_{int(e_ground * 10)}"
        bullet, _ = bounce_drop(f"bullet_mix_bounce_{tag}", e_box, e_ground, solver_kind="bullet")
        ours, _ = bounce_drop(f"box3d_mix_bounce_{tag}", e_box, e_ground)
        ok = ok and abs(bullet - ours) < 0.08
        rows.append(f"box {e_box} on ground {e_ground}: Bullet {bullet:.3f}, Box3D {ours:.3f} (multiply "
                    f"{e_box * e_ground:.2f}, minimum {min(e_box, e_ground):.2f})")
    results.append(Result("bullet_mixing", Result.PASS if ok else Result.FAIL, "default Bounce Mixing: " + "; ".join(rows)))
    return results


def test_bounce_heights():
    """Boxes with bounce 0.3, 0.6 and 0.9 dropped 2 m onto ground with bounce 1 (default mixing,
    Multiply, so the box's own bounce is used): each rebounds to bounce^2 x 2 m."""
    rows, ok = [], True
    for e in (0.3, 0.6, 0.9):
        got, _ = bounce_drop(f"bounce_height_{int(e * 10)}", e, 1.0)
        good = close(got, e, 0.05)
        ok = ok and good
        rows.append(f"bounce {e:g}: rebounded {got * got * 2:.3f} m, measured bounce {got:.3f}")
    return [Result("bounce_heights", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS = {
    "friction_mixing": test_friction_mixing,
    "bounce_mixing": test_bounce_mixing,
    "bullet_mixing": test_bullet_mixing,
    "bounce_heights": test_bounce_heights,
}
