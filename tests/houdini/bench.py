"""Timing runs of Box3D against RBD Bullet Solver on the scenes in bench_scenes.py. Not part of run_tests.py: nothing
here passes or fails, and a run can take many minutes.

Run from the repository root, with the plugin built:

    hython tests/houdini/bench.py --list
    hython tests/houdini/bench.py pile                       # a scenario's default sweep, both solvers
    hython tests/houdini/bench.py pile:n=20000,frames=100    # one variant (any scenario parameter)
    hython tests/houdini/bench.py joints:type=hard_all,layout=grid --kinds box3d --repeat 3
    hython tests/houdini/bench.py pile --threads 1           # HOUDINI_MAXTHREADS=1, Box3D on all of them
    hython tests/houdini/bench.py pile --compare tests/houdini/out/bench/before.json

Each run is its own hython process (a fresh hip file, a cold start). It plays frames 1..N, timing every frame
(setFrame and the DOP cook, as a user waits for it), then resets the simulation and times frame 2 again (the world
rebuild). Frames are grouped into the scene's phases and reported as median, 95th percentile and maximum per frame.
Houdini's Performance Monitor records the solvers' stages (Box3D's are Pre-Solve / Stepping Simulation /
Post-Solve, so what is not Stepping World is the plugin's own code) and each DOP node's time.

Results are saved to tests/houdini/out/bench/<time>.json and latest.json (compare against a saved one with
--compare). Timing is noisy: use --repeat, keep other work off the machine, and treat under about 10% as noise.
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
import tempfile
import time
import traceback

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import hou  # noqa: E402

import bench_scenes  # noqa: E402

OUT_DIR = os.path.join(HERE, "out", "bench")


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------


def parse_value(text):
    try:
        return json.loads(text)
    except ValueError:
        return text


def parse_spec(token):
    """`name` or `name:key=value,key=value` -> (name, overrides or None)."""
    name, _, rest = token.partition(":")
    if name not in bench_scenes.SCENARIOS:
        raise SystemExit(f"unknown scenario {name}. Use --list.")
    if not rest:
        return name, None
    overrides = {}
    for item in rest.split(","):
        key, _, value = item.partition("=")
        overrides[key.strip()] = parse_value(value.strip())
    return name, overrides


def label_of(name, params, shown):
    return name + (":" + ",".join(f"{k}={params[k]}" for k in shown if k in params) if shown else "")


# ---------------------------------------------------------------------------
# The child: one scene, one solver
# ---------------------------------------------------------------------------


def stage_times(stats, dop_path):
    """{stage or node name: {frame: ms}} from a Performance Monitor stats tree: the solver's stages (nodes named
    {Category}/{Stage}, nested as the Bullet solver's are, keyed here as Category/Stage; Category alone is left out)
    and the time of each node directly in the DOP network (as node:<name>, children included)."""
    out = {}

    def add(name, frames):
        per = out.setdefault(name, {})
        for frame, values in frames.items():
            per[int(frame)] = per.get(int(frame), 0.0) + float(values.get("Time", 0.0))

    def walk(node, chain):
        name = node.get("name", "")
        frames = node.get("stats", {}).get("0", {})
        path = node.get("path", "")
        braced = name.startswith("{") and name.endswith("}")
        if braced and name[1:].startswith("SIM_"):
            chain = []  # a solver's own node: its stages are below
        elif braced:
            chain = chain + [name[1:-1]]
            if "/" in name:  # a flat name {Category/Stage} (others' solvers)
                add(name.strip("{}"), frames)
            elif len(chain) >= 2:
                add("/".join(chain), frames)
        elif path.startswith(dop_path + "/") and "/" not in path[len(dop_path) + 1:]:
            add("node:" + name, frames)
        for child in node.get("children", []):
            walk(child, chain)

    walk(stats, [])
    return out


def run_child(spec, result_file):
    """Builds and plays one scene; writes a record (JSON) to result_file."""
    name, kind, params = spec["scenario"], spec["kind"], spec["params"]
    record = {"scenario": name, "kind": kind, "params": params, "threads": spec.get("threads")}
    try:
        hou.hipFile.clear(suppress_save_prompt=True)
        start = time.perf_counter()
        scene = bench_scenes.SCENARIOS[name].build(kind, dict(params))
        record["build_s"] = time.perf_counter() - start
        dop = scene.dop

        def cook():
            if scene.sop is not None:
                scene.sop.cook(force=False)
            else:
                dop.simulation().objects()

        if spec.get("threads"):
            for node in dop.children():
                if node.type().name() == "box3d_solver":
                    node.parm("autothreads").set(1)  # all of Houdini's threads, which HOUDINI_MAXTHREADS caps
        profile = None if spec.get("no_perfmon") else hou.perfMon.startProfile("bench")
        frames_ms, budget, begin = {}, spec["budget"], time.perf_counter()
        for frame in range(1, scene.frames + 1):
            t0 = time.perf_counter()
            hou.setFrame(frame)
            cook()
            frames_ms[frame] = (time.perf_counter() - t0) * 1000.0
            if time.perf_counter() - begin > budget:
                record["truncated"] = frame
                break
        record["frames_ms"] = frames_ms
        # Where each object ended up: a sanity check that the scene did what its docstring says.
        record["end"] = {}
        for obj in dop.simulation().objects():
            geo = obj.geometry()
            if geo is not None and geo.points():
                box = geo.boundingBox()
                positions = [pt.position() for pt in geo.points()]
                record["end"][obj.name()] = {"points": len(geo.points()), "min": list(box.minvec()),
                                             "max": list(box.maxvec()),
                                             "sum": [sum(p[k] for p in positions) for k in range(3)]}
        if profile is not None:
            profile.stop()
            record["stages"] = stage_times(profile.stats(), dop.path())
        if not spec.get("no_rewind") and not record.get("truncated") and scene.frames >= 2:
            # Reset Simulation, then frame 1 and frame 2 again: new objects are only solved from frame 2, so that is
            # where the world is rebuilt (the SOPs stay cached). rewind_ms is that frame 2.
            hou.setFrame(1)
            cook()
            dop.parm("resimulate").pressButton()
            dop.cook(force=True)
            hou.setFrame(1)
            cook()
            t0 = time.perf_counter()
            hou.setFrame(2)
            cook()
            record["rewind_ms"] = (time.perf_counter() - t0) * 1000.0
        record["phases"] = [list(p) for p in scene.phases]
    except Exception:
        record["error"] = traceback.format_exc()
    with open(result_file, "w") as f:
        json.dump(record, f)
    return 0


# ---------------------------------------------------------------------------
# The parent: spawn the runs, summarise
# ---------------------------------------------------------------------------


def hython_path():
    hfs = os.environ.get("HFS")
    if hfs:
        for exe in ("hython.exe", "hython"):
            path = os.path.join(hfs, "bin", exe)
            if os.path.exists(path):
                return path
    return sys.executable


def run_isolated(spec):
    fd, result_file = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.remove(result_file)
    cmd = [hython_path(), os.path.abspath(__file__), "--child", json.dumps(spec), "--result-file", result_file]
    env = dict(os.environ)
    if spec.get("threads"):
        env["HOUDINI_MAXTHREADS"] = str(spec["threads"])
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace", env=env,
                              timeout=spec["budget"] * 3 + 600)
        tail = "\n".join(((proc.stdout or "") + (proc.stderr or "")).strip().splitlines()[-8:])
    except subprocess.TimeoutExpired:
        return {**spec, "error": "no result (hang?)"}
    if not os.path.exists(result_file):
        return {**spec, "error": f"hython exited with code {proc.returncode} and no result:\n{tail}"}
    with open(result_file) as f:
        record = json.load(f)
    os.remove(result_file)
    return record


def percentile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))]


def summarise(record):
    """Adds `phase_stats` ({label: median / p95 / max / mean / frames}) and `stage_means` ({stage: {label: ms}})."""
    if "error" in record or "frames_ms" not in record:
        return record
    frames = {int(f): ms for f, ms in record["frames_ms"].items()}
    record["first_ms"] = frames.get(1)
    stats, labels = {}, []
    # Frame 1 on its own: the scene's SOPs cook and the world is first built there.
    for label, lo, hi in [["first", 1, 1]] + record["phases"]:
        values = [ms for f, ms in frames.items() if lo <= f <= hi]
        if values:
            labels.append((label, lo, hi))
            stats[label] = {"median": statistics.median(values), "p95": percentile(values, 0.95),
                            "max": max(values), "mean": statistics.fmean(values), "frames": len(values)}
    record["phase_stats"] = stats
    means = {}
    for stage, per_frame in (record.get("stages") or {}).items():
        per_frame = {int(f): v for f, v in per_frame.items()}  # JSON keys are strings
        per_phase = {}
        for label, lo, hi in labels:
            values = [per_frame.get(f, 0.0) for f in range(lo, hi + 1) if f in frames]
            per_phase[label] = statistics.fmean(values) if values else 0.0
        means[stage] = per_phase
    record["stage_means"] = means
    return summarise_total(record)


def summarise_total(record):
    record["total_s"] = sum(float(v) for v in record["frames_ms"].values()) / 1000.0
    return record


def median_run(records):
    """The run with the median total time (a run that failed is never the one kept if another succeeded)."""
    good = [r for r in records if "error" not in r]
    if not good:
        return records[0]
    good.sort(key=lambda r: r["total_s"])
    return good[len(good) // 2]


def ms(value):
    return "     -" if value is None else f"{value:6.1f}" if value < 1000 else f"{value:6.0f}"


def print_record(record, show_stages):
    head = f"  {record['kind']:6}"
    if "error" in record:
        print(f"{head} ERROR {record['error'].strip().splitlines()[-1] if record['error'].strip() else ''}")
        print("        " + record["error"].strip().replace("\n", "\n        "))
        return
    parts = [f"first {ms(record.get('first_ms'))}"]
    for label, s in record["phase_stats"].items():
        if label != "first":
            parts.append(f"{label} {ms(s['median'])} (p95 {ms(s['p95']).strip()}, max {ms(s['max']).strip()})")
    if record.get("rewind_ms") is not None:
        parts.append(f"rewind {ms(record['rewind_ms']).strip()}")
    if record.get("truncated"):
        parts.append(f"STOPPED at frame {record['truncated']} (budget)")
    print(head + "  " + " | ".join(parts) + "   [ms per frame]")
    if show_stages and record.get("stage_means"):
        labels = list(record["phase_stats"])
        rows = sorted(record["stage_means"].items(), key=lambda kv: -max(kv[1].values() or [0]))
        print("         stage (mean ms per frame)".ljust(60) + "".join(f"{lab:>10}" for lab in labels))
        for stage, per in rows:
            if max(per.values() or [0]) < 0.05:
                continue
            print("         " + stage.ljust(51) + "".join(f"{per.get(lab, 0):10.2f}" for lab in labels))


def key_of(record):
    return json.dumps([record["scenario"], record["params"], record["kind"], record.get("threads")], sort_keys=True)


def print_comparison(records, path):
    with open(path) as f:
        old = {key_of(r): r for r in json.load(f)}
    print(f"\nAgainst {path} (new / old, per phase median; below 1 is faster)")
    for r in records:
        before = old.get(key_of(r))
        if not before or "phase_stats" not in r or "phase_stats" not in before:
            continue
        cells = []
        for label, s in r["phase_stats"].items():
            if label in before["phase_stats"] and before["phase_stats"][label]["median"] > 0:
                cells.append(f"{label} {s['median'] / before['phase_stats'][label]['median']:.2f}")
        if before.get("first_ms") and r.get("first_ms"):
            cells.insert(0, f"first {r['first_ms'] / before['first_ms']:.2f}")
        shown = ",".join(f"{k}={v}" for k, v in r["params"].items())
        print(f"  {r['scenario']}:{shown} {r['kind']}: " + ", ".join(cells))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenarios", nargs="*", help="scenario names, or name:key=value,... for one variant")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--kinds", default="box3d,bullet", help="solvers to run (default both)")
    parser.add_argument("--repeat", type=int, default=1, help="runs per variant; the median-total run is kept")
    parser.add_argument("--budget", type=float, default=240.0, help="seconds of stepping before a run stops early")
    parser.add_argument("--threads", type=int, help="HOUDINI_MAXTHREADS for the runs, Box3D on all of them")
    parser.add_argument("--no-stages", action="store_true", help="do not print the Performance Monitor stages")
    parser.add_argument("--no-perfmon", action="store_true", help="do not record the Performance Monitor")
    parser.add_argument("--no-rewind", action="store_true", help="skip the reset and second frame 1")
    parser.add_argument("--compare", help="a saved results file to compare with")
    parser.add_argument("--child", help=argparse.SUPPRESS)
    parser.add_argument("--result-file", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.child:
        return run_child(json.loads(args.child), args.result_file)
    if args.list:
        for name, sc in bench_scenes.SCENARIOS.items():
            print(f"{name:10} {sc.doc}\n{'':10} defaults {sc.defaults}; sweep {sc.sweep}\n")
        return 0
    wanted = args.scenarios or list(bench_scenes.SCENARIOS)
    kinds = [k for k in args.kinds.split(",") if k]
    records = []
    for token in wanted:
        name, overrides = parse_spec(token)
        sc = bench_scenes.SCENARIOS[name]
        variants = [overrides] if overrides is not None else sc.sweep
        for variant in variants:
            params = {**sc.defaults, **variant}
            shown = sorted(set(variant) | ({"n"} & set(params)))
            print(f"\n{label_of(name, params, shown)}", flush=True)
            available = sc.kinds_for(params) if sc.kinds_for else sc.kinds
            for kind in kinds:
                if kind not in available:
                    continue
                runs = []
                for _ in range(max(args.repeat, 1)):
                    spec = {"scenario": name, "kind": kind, "params": params, "threads": args.threads,
                            "budget": args.budget, "no_perfmon": args.no_perfmon, "no_rewind": args.no_rewind}
                    runs.append(summarise(run_isolated(spec)))
                record = median_run(runs)
                records.append(record)
                print_record(record, not args.no_stages)
                sys.stdout.flush()
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for path in (os.path.join(OUT_DIR, f"{stamp}.json"), os.path.join(OUT_DIR, "latest.json")):
        with open(path, "w") as f:
            json.dump(records, f)
    print(f"\nSaved to {os.path.join(OUT_DIR, stamp + '.json')}")
    if args.compare:
        print_comparison(records, args.compare)
    return 1 if any("error" in r for r in records) else 0


if __name__ == "__main__":
    sys.exit(main())
