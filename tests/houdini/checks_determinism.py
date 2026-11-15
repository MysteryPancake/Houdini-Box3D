"""Determinism: repeat runs, thread counts, substeps and frame rate."""

import hou

from b3dtest import (
    box3d_solver,
    close,
    finish,
    GROUND,
    new_dopnet,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_parm,
    SetupError,
    step,
)
from checks_solver import stack_scene


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
        set_parm(solver, ["solverpasses"], substeps)
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


TESTS = {
    "determinism_repeat": test_determinism_repeat,
    "determinism_threads": test_determinism_threads,
    "substeps_fps": test_substeps_fps,
}
