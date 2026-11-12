#!/usr/bin/env bash
set -euo pipefail

# --- Houdini install: $HFS if set (run `source houdini_setup` first), else the newest /opt/hfs* ---
if [ -z "${HFS:-}" ]; then
    HFS="$(printf '%s\n' /opt/hfs* | sort -V | tail -n 1)"
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
cmake --build build -j "$(nproc)"

echo
echo "BUILD OK. Close Houdini before rebuilding; the plugin is copied to ~/houdiniX.Y/dso."
