"""Checks of the Box3D plugin in Houdini, for items in "Untested and unverified"
(docs/agents/status.md). Each test builds its own scene from scratch, simulates it and prints
PASS / FAIL (or INFO for comparisons that need a human verdict). The tests live in the
checks_*.py modules next to this file; this is the runner.

Run from the repository root, with the plugin built (build.bat copies it to the dso folder):

    hython tests/houdini/run_tests.py                 # every test
    hython tests/houdini/run_tests.py two_rbd network # some tests (names below)
    hython tests/houdini/run_tests.py --list
    hython tests/houdini/run_tests.py network --repeat 20   # for intermittent crashes

On Windows, test_houdini.bat in the repository root finds Houdini and runs this.

Each test runs in its own hython process, so a crash (CRASH) doesn't stop the others. Each
test's scene is saved to tests/houdini/out/<test>.hip to open and inspect afterwards.
Exit code 1 if anything failed, errored or crashed.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import traceback

# No __pycache__ next to the scripts (b3dtest is imported from this folder). The child
# processes run this file too, so they inherit it.
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hou  # noqa: E402

import checks_constraints  # noqa: E402
import checks_core  # noqa: E402
import checks_more  # noqa: E402
import checks_output  # noqa: E402
import checks_solver  # noqa: E402
from b3dtest import Result, SetupError  # noqa: E402

# Every test, in run order: module by module.
MODULES = [checks_core, checks_solver, checks_output, checks_constraints, checks_more]
TESTS = {}
for module in MODULES:
    for name, fn in module.TESTS.items():
        assert name not in TESTS, f"duplicate test name {name}"
        TESTS[name] = fn


def run_child(name, out_dir, result_file):
    """Runs one test in this process and writes its results to result_file (JSON)."""
    hou.hipFile.clear(suppress_save_prompt=True)
    try:
        got = TESTS[name]()
    except SetupError as e:
        got = [Result(name, Result.SETUP, str(e))]
    except Exception:
        got = [Result(name, Result.ERROR, traceback.format_exc())]
    try:
        hou.hipFile.save(os.path.join(out_dir, f"{name}.hip"))
    except hou.OperationFailed:
        pass
    with open(result_file, "w") as f:
        json.dump([[r.status, r.message] for r in got], f)
    return 0


def hython_path():
    hfs = os.environ.get("HFS")
    if hfs:
        for exe in ("hython.exe", "hython"):
            path = os.path.join(hfs, "bin", exe)
            if os.path.exists(path):
                return path
    return sys.executable


def run_isolated(name, out_dir):
    """Runs one test in a separate hython process. A crash (no results written) is reported as
    CRASH with the end of the process's output."""
    fd, result_file = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.remove(result_file)
    cmd = [hython_path(), os.path.abspath(__file__), "--child", name, "--out", out_dir,
           "--result-file", result_file]
    env = dict(os.environ, **getattr(TESTS[name], "env", {}))  # e.g. HOUDINI_MAXTHREADS=1
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=1800, env=env)
        output = (proc.stdout or "") + (proc.stderr or "")
        code = proc.returncode
    except subprocess.TimeoutExpired:
        return [Result(name, Result.CRASH, "no result after 30 minutes (hang?)")]
    if not os.path.exists(result_file):
        tail = "\n".join(output.strip().splitlines()[:12])
        return [Result(name, Result.CRASH, f"hython exited with code {code} and no result. Start of its output:\n{tail}")]
    with open(result_file) as f:
        data = json.load(f)
    os.remove(result_file)
    return [Result(name, status, message) for status, message in data]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("tests", nargs="*", help="tests to run (default: all)")
    parser.add_argument("--list", action="store_true", help="list the tests and exit")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"),
                        help="folder for each test's saved .hip")
    parser.add_argument("--repeat", type=int, default=1,
                        help="run the selected tests this many times (for intermittent crashes)")
    parser.add_argument("--child", help=argparse.SUPPRESS)
    parser.add_argument("--result-file", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.child:
        return run_child(args.child, args.out, args.result_file)
    if args.list:
        for module in MODULES:
            print(f"[{module.__name__}]")
            for name, fn in module.TESTS.items():
                print(f"  {name:22} {' '.join(fn.__doc__.split())}")
        return 0
    unknown = [t for t in args.tests if t not in TESTS]
    if unknown:
        print(f"unknown test(s): {', '.join(unknown)}. Use --list.")
        return 2
    os.makedirs(args.out, exist_ok=True)

    results = []
    for run in range(1, max(args.repeat, 1) + 1):
        if args.repeat > 1:
            print(f"=== run {run} of {args.repeat}", flush=True)
        for name in args.tests or TESTS:
            print(f"--- {name}", flush=True)
            got = run_isolated(name, args.out)
            for r in got:
                print(f"  {r.status:5}  {r.message}", flush=True)
            results.extend(got)

    print("\nSummary")
    for status in (Result.PASS, Result.FAIL, Result.INFO, Result.SETUP, Result.ERROR, Result.CRASH):
        n = sum(r.status == status for r in results)
        if n:
            print(f"  {status:5} {n}")
    if any(r.status == Result.SETUP for r in results):
        print("\nSETUP means this script guessed a node or parameter name wrong for your Houdini version;"
              " the message lists the real names. It is not a plugin failure.")
    bad = any(r.status in (Result.FAIL, Result.ERROR, Result.SETUP, Result.CRASH) for r in results)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
