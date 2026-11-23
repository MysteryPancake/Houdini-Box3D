#!/usr/bin/env python3
"""Cross-platform launcher: finds Houdini, then runs the Houdini checks, the example scenes or the core tests.

    python tools/b3d.py hfs                    print the Houdini install used
    python tools/b3d.py test [args]            hython tests/houdini/run_tests.py [args]   (test.bat on Windows)
    python tools/b3d.py examples [args]        hython examples/make_examples.py [args]    (make_examples.bat)
    python tools/b3d.py bench [args]           hython tests/houdini/bench.py [args]       (timings, Box3D vs Bullet)
    python tools/b3d.py ctest [args]           ctest --test-dir build [args] (-C Release is added on Visual Studio)

Houdini is found as build.sh / build.bat do: $HFS if it has the HDK (toolkit/cmake), else the registry's
ActiveVersion (Windows), else the newest install in the usual places (Windows: Program Files\\Side Effects
Software, Linux: /opt/hfs*, macOS: /Applications/Houdini). Build the plugin first (build.sh / build.bat):
the checks and scenes load it from the dso folder.
"""

import glob
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def is_hfs(path):
    return bool(path) and os.path.isdir(os.path.join(path, "toolkit", "cmake"))


def version_key(path):
    """The version numbers in the last part of `path` that has any (macOS: .../Versions/20.5/Resources)."""
    for part in reversed(re.split(r"[/\\]+", path.rstrip("/\\"))):
        numbers = [int(n) for n in re.findall(r"\d+", part)]
        if numbers:
            return numbers
    return [0]


def newest(candidates):
    found = [c for c in candidates if is_hfs(c)]
    return max(found, key=version_key) if found else None


def registry_hfs():
    """The install the Windows installer records as active, or None."""
    try:
        import winreg
    except ImportError:
        return None
    base = r"SOFTWARE\Side Effects Software"
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as k:
            version = winreg.QueryValueEx(k, "ActiveVersion")[0]
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base + "\\Houdini " + version) as k:
            path = winreg.QueryValueEx(k, "InstallPath")[0]
    except OSError:
        return None
    return path if is_hfs(path) else None


def find_hfs():
    env = os.environ.get("HFS")
    if is_hfs(env):
        return env
    if sys.platform == "win32":
        found = registry_hfs()
        if found:
            return found
        pf = os.environ.get("ProgramFiles", r"C:\Program Files")
        found = newest(glob.glob(os.path.join(pf, "Side Effects Software", "Houdini *")))
    elif sys.platform == "darwin":
        found = newest(glob.glob("/Applications/Houdini/Houdini*/Frameworks/Houdini.framework/Versions/*/Resources"))
    else:
        found = newest(glob.glob("/opt/hfs*"))
    return found


def hython(hfs):
    exe = os.path.join(hfs, "bin", "hython.exe" if sys.platform == "win32" else "hython")
    return exe if os.path.exists(exe) else None


def run(cmd):
    print(" ".join(cmd), flush=True)
    env = dict(os.environ)
    # Houdini uses %HOME% as its preferences folder when it is set (Git Bash sets it), which would look for
    # the plugin in C:\Users\<you>\houdiniX.Y instead of Documents\houdiniX.Y. The .bat launchers clear it too.
    if sys.platform == "win32" and "HOUDINI_USER_PREF_DIR" not in env:
        env.pop("HOME", None)
    return subprocess.call(cmd, cwd=ROOT, env=env)


def main(argv):
    if not argv or argv[0] not in ("hfs", "test", "examples", "bench", "ctest"):
        print(__doc__)
        return 0 if not argv else 2
    cmd, rest = argv[0], argv[1:]
    if cmd == "ctest":
        extra = ["-C", "Release"] if (ROOT / "build" / "Box3DDOP.sln").exists() else []
        return run(["ctest", "--test-dir", "build", "--output-on-failure", *extra, *rest])
    hfs = find_hfs()
    if not hfs:
        print("ERROR: could not find a Houdini install with the HDK (toolkit/cmake).\n"
              "Run from Houdini's command line tools (or `source houdini_setup`), or set HFS.", file=sys.stderr)
        return 1
    print("Using Houdini:", hfs)
    if cmd == "hfs":
        return 0
    exe = hython(hfs)
    if not exe:
        print(f"ERROR: hython not found in {hfs}/bin", file=sys.stderr)
        return 1
    script = {"test": "tests/houdini/run_tests.py", "bench": "tests/houdini/bench.py"}.get(
        cmd, "examples/make_examples.py")
    return run([exe, script, *rest])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
