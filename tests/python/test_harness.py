"""Checks on the Python side that need no Houdini: run with

    python -I tests/python/test_harness.py

The Houdini checks (tests/houdini) import `hou`, so this loads them against a stand-in and looks at
what can be known without a Houdini: that every module is registered, every test is named and
documented the same way, the scripts have no undefined or unused names (pyflakes, if installed),
the pure helpers give the right numbers, the docs name tests that exist, baseline.json is
well formed and no file mixes line endings. A test that talks to Houdini is only run by
tests/houdini/run_tests.py."""

import ast
import glob
import importlib
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HOUDINI_TESTS = os.path.join(ROOT, "tests", "houdini")
sys.dont_write_bytecode = True  # no __pycache__ next to the scripts

# The stand-in: any attribute is another stand-in, so module-level code runs and nothing is drawn.
sys.modules["hou"] = mock.MagicMock()
sys.path.insert(0, HOUDINI_TESTS)


def check_modules():
    """The names of the checks_*.py modules in tests/houdini, from the files."""
    paths = glob.glob(os.path.join(HOUDINI_TESTS, "checks_*.py"))
    # *_common.py files hold helpers shared by the others, not tests
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in paths if not p.endswith("_common.py"))


def registered_modules():
    """The modules run_tests.py lists in MODULES, read from its source (importing it would start
    nothing, but reading it keeps the stand-in out of the way)."""
    with open(os.path.join(HOUDINI_TESTS, "run_tests.py"), encoding="utf-8") as f:
        text = f.read()
    block = re.search(r"MODULES\s*=\s*\[(.*?)\]", text, re.S)
    return sorted(re.findall(r"checks_\w+", block.group(1))) if block else []


def load(name):
    return importlib.import_module(name)


def core_test_names():
    """The names the C++ tests register, as {"name", function}."""
    names = set()
    for path in glob.glob(os.path.join(ROOT, "tests", "core_*.cpp")):
        with open(path, encoding="utf-8") as f:
            names.update(re.findall(r'\{\s*"([a-z_0-9]+)",\s*test\w+\s*\}', f.read()))
    return names


class Registration(unittest.TestCase):
    def test_every_module_is_run(self):
        self.assertEqual(check_modules(), registered_modules(),
                         "a checks_*.py file is missing from MODULES in run_tests.py, or the other way round")

    def test_names_are_unique_and_documented(self):
        seen = {}
        for module in check_modules():
            for name, fn in load(module).TESTS.items():
                self.assertNotIn(name, seen, f"{name} is in {module} and {seen.get(name)}")
                seen[name] = module
                self.assertTrue(callable(fn), f"{module}.{name} is not callable")
                self.assertTrue((fn.__doc__ or "").strip(), f"{module}.{name} has no docstring")
        self.assertGreater(len(seen), 100)

    def test_results_carry_their_tests_name(self):
        """Result("name", ...) inside a test says which test it belongs to, so the summary groups it
        right. A test may make results for others (a probe), so only literal first arguments that are
        not a registered test name at all are reported."""
        registered = {n for m in check_modules() for n in load(m).TESTS}
        bad = []
        for module in check_modules():
            with open(os.path.join(HOUDINI_TESTS, module + ".py"), encoding="utf-8") as f:
                tree = ast.parse(f.read())
            for call in ast.walk(tree):
                if (isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Result" and call.args
                        and isinstance(call.args[0], ast.Constant) and isinstance(call.args[0].value, str)
                        and call.args[0].value not in registered):
                    bad.append(f"{module}:{call.lineno} {call.args[0].value}")
        self.assertEqual(bad, [], "Result names that no test is registered under")

    def test_bullet_tests_use_both_solvers(self):
        """A test in checks_bullet.py compares two solvers, so it has to run each."""
        with open(os.path.join(HOUDINI_TESTS, "checks_bullet.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn('KINDS = ("box3d", "bullet")', src)
        for name in load("checks_bullet").TESTS:
            self.assertTrue(name.startswith("bullet_"), name)


class Lint(unittest.TestCase):
    def test_pyflakes(self):
        if importlib.util.find_spec("pyflakes") is None:
            self.skipTest("pyflakes is not installed (pip install pyflakes)")
        files = sorted(glob.glob(os.path.join(HOUDINI_TESTS, "*.py")) + glob.glob(os.path.join(ROOT, "tools", "**", "*.py"), recursive=True)
                       + glob.glob(os.path.join(ROOT, "tests", "python", "*.py")))
        # The viewer states run inside Houdini, which provides `hou` and `kwargs`.
        files = [f for f in files if not f.endswith("_state.py")]
        run = subprocess.run([sys.executable, "-m", "pyflakes"] + files, capture_output=True, text=True)
        self.assertEqual(run.stdout + run.stderr, "")

    def test_every_script_compiles(self):
        for path in glob.glob(os.path.join(ROOT, "**", "*.py"), recursive=True):
            if os.sep + "extern" + os.sep in path:
                continue
            with open(path, encoding="utf-8") as f:
                compile(f.read(), path, "exec")


class Helpers(unittest.TestCase):
    """The pure helpers the Houdini checks lean on: a wrong one would make every comparison wrong."""

    def test_close_and_det3(self):
        b3dtest = load("b3dtest")
        self.assertTrue(b3dtest.close(1.0, 1.04, 0.05))
        self.assertFalse(b3dtest.close(1.0, 1.06, 0.05))
        self.assertAlmostEqual(b3dtest.det3([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), 1.0)
        self.assertAlmostEqual(b3dtest.det3([[1, 0, 0], [0, 1, 0], [0, 0, -1]]), -1.0)
        self.assertAlmostEqual(b3dtest.det3([[2, 0, 0], [0, 3, 0], [0, 0, 4]]), 24.0)

    def test_rotation_angle(self):
        bullet = load("checks_bullet")
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        c, s = math.cos(math.radians(40)), math.sin(math.radians(40))
        about_z = [[c, s, 0], [-s, c, 0], [0, 0, 1]]
        self.assertAlmostEqual(bullet.rotation_angle(identity, identity), 0.0, places=6)
        self.assertAlmostEqual(bullet.rotation_angle(identity, about_z), 40.0, places=6)
        self.assertAlmostEqual(bullet.rotation_angle(about_z, identity), 40.0, places=6)

    def test_quat_angle(self):
        bullet = load("checks_bullet")
        h = math.sin(math.radians(20))
        turned = (0.0, 0.0, h, math.cos(math.radians(20)))  # 40 degrees about z
        self.assertAlmostEqual(bullet.quat_angle((0, 0, 0, 1), turned), 40.0, places=6)
        # q and -q are the same rotation
        self.assertAlmostEqual(bullet.quat_angle((0, 0, 0, 1), tuple(-v for v in turned)), 40.0, places=6)
        self.assertAlmostEqual(bullet.quat_angle(turned, turned), 0.0, places=4)

    def test_vex_value(self):
        vex = load("b3dtest").vex_value
        self.assertEqual(vex(3)[0], "i")
        self.assertEqual(vex(0.5)[0], "f")
        self.assertEqual(vex((1.0, 2.0, 3.0))[0], "v")
        self.assertEqual(vex("a")[0], "s")

    def test_mixing_rules(self):
        common = load("checks_common")
        self.assertAlmostEqual(common.mix("Minimum", 0.2, 0.8), 0.2)
        self.assertAlmostEqual(common.mix("Maximum", 0.2, 0.8), 0.8)
        self.assertAlmostEqual(common.mix("Average", 0.2, 0.8), 0.5)
        self.assertAlmostEqual(common.mix("Multiply", 0.2, 0.8), 0.16)
        self.assertAlmostEqual(common.mix("Geometric Mean", 0.2, 0.8), 0.4)


class Docs(unittest.TestCase):
    def test_docs_name_tests_that_exist(self):
        """`name` after the word test or tests, in the docs, is a registered test (Houdini or core)."""
        known = {n for m in check_modules() for n in load(m).TESTS} | core_test_names()
        missing = []
        paths = glob.glob(os.path.join(ROOT, "docs", "**", "*.md"), recursive=True) + [os.path.join(ROOT, "CLAUDE.md"),
                                                                                     os.path.join(ROOT, "README.md")]
        for path in paths:
            with open(path, encoding="utf-8") as f:
                text = f.read()
            for match in re.finditer(r"\b[Tt]ests? `([a-z][a-z_0-9]+)`", text):
                if match.group(1) not in known:
                    missing.append(f"{os.path.relpath(path, ROOT)}: {match.group(1)}")
        self.assertEqual(missing, [])

    def test_baseline_is_well_formed(self):
        with open(os.path.join(HOUDINI_TESTS, "baseline.json"), encoding="utf-8") as f:
            data = json.load(f)
        self.assertEqual(data["frame"], 48)
        self.assertTrue(data["houdini"])
        self.assertGreater(len(data["pieces"]), 10)
        for label, values in data["pieces"].items():
            self.assertEqual(len(values), 12, label)  # position (3) and rotation (9)
            self.assertTrue(all(math.isfinite(v) for v in values), label)


class Files(unittest.TestCase):
    def test_no_file_mixes_line_endings(self):
        """A file is all LF or all CRLF: an editor or script that rewrites part of one makes git
        show every line as changed."""
        mixed = []
        skip = (os.sep + "extern" + os.sep, os.sep + ".git" + os.sep, os.sep + "__pycache__" + os.sep)
        for dirpath, _, names in os.walk(ROOT):
            if any(s in dirpath + os.sep for s in skip):
                continue
            for name in names:
                if not name.endswith((".py", ".md", ".cpp", ".h", ".txt", ".json", ".bat", ".sh", ".cmake")):
                    continue
                with open(os.path.join(dirpath, name), "rb") as f:
                    data = f.read()
                crlf = data.count(b"\r\n")
                lf = data.count(b"\n") - crlf
                if crlf and lf:
                    mixed.append(f"{os.path.relpath(os.path.join(dirpath, name), ROOT)} ({crlf} CRLF, {lf} LF)")
        self.assertEqual(mixed, [])

    def test_text_files_end_with_a_newline(self):
        bad = []
        for pattern in ("tests/**/*.py", "tools/**/*.py", "docs/**/*.md"):
            for path in glob.glob(os.path.join(ROOT, pattern), recursive=True):
                with open(path, "rb") as f:
                    data = f.read()
                if data and not data.endswith(b"\n"):
                    bad.append(os.path.relpath(path, ROOT))
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
