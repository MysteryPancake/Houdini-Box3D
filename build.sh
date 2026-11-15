#!/usr/bin/env bash
set -euo pipefail

# --- Houdini install: $HFS if set (run `source houdini_setup` first), else the newest install:
#     Linux /opt/hfs*, macOS /Applications/Houdini/Houdini*/Frameworks/Houdini.framework/Versions/*/Resources ---
if [ -z "${HFS:-}" ]; then
    HFS="$(for d in /opt/hfs* /Applications/Houdini/Houdini*/Frameworks/Houdini.framework/Versions/*/Resources; do
               [ -d "$d/toolkit/cmake" ] && printf '%s\n' "$d"
           done | sort -V | tail -n 1)"
fi
if [ ! -d "$HFS/toolkit/cmake" ]; then
    echo "ERROR: Houdini toolkit not found at \"$HFS/toolkit/cmake\""
    echo "Run 'source houdini_setup' from your Houdini install, or set HFS."
    exit 1
fi
export HFS

cd "$(dirname "${BASH_SOURCE[0]}")"

# --- Make sure the Box3D submodule is present ---
if [ ! -f extern/box3d/CMakeLists.txt ]; then
    echo "Fetching Box3D submodule..."
    git submodule update --init --recursive
fi

# --- Wipe build/ if this folder was moved or renamed ---
MARKER="build/.configured_from"
if [ -f "$MARKER" ] && [ "$(cat "$MARKER")" != "$PWD" ]; then
    echo "Folder changed from \"$(cat "$MARKER")\", resetting build folder..."
    rm -rf build
fi

# --- Debug logging (prints to Houdini's console): off unless you run "./build.sh debug" ---
DBG=OFF
if [ "${1:-}" = "debug" ]; then DBG=ON; fi
echo "Debug logging: $DBG"

# --- Configure (every build, so BOX3DDOP_DEBUG never keeps a stale cached value) ---
cmake -S . -B build -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}" -DBOX3DDOP_DEBUG="$DBG"
echo "$PWD" > "$MARKER"

# --- Build ---
JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
cmake --build build -j "$JOBS"

echo
echo "BUILD OK. Close Houdini before rebuilding; the plugin is copied to ~/houdiniX.Y/dso."
