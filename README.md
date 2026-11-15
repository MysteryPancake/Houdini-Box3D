# Houdini Box3D

Houdini DOP wrapper for [Box3D](https://github.com/erincatto/box3d), currently one of the fastest rigid body solvers.

My goal is keeping Box3D's great performance preserved in Houdini as much as possible.

I made design decisions based on the RBD Bullet Solver where possible, so it should feel familiar to use.

> [!CAUTION]
> This is heavily AI generated, as it was made for fun!
>
> However, it's not complete slop. I've tested all the features and cleaned up as much as possible.
>
> This isn't normally how I develop software, I just wanted to give it a try.

## Installing

A prebuilt Windows build is available in the [Releases](../../releases) section. To install it, put it in your Houdini packages folder:

```
Documents\houdini22.0\packages
```

It only loads in the Houdini version it was built for. For other versions or Linux, see [Building](#building).

## Building

You need CMake, a C++17 compiler, git and a Houdini install (for the HDK). Clone with submodules so Box3D is included:

```
git clone --recurse-submodules https://github.com/MysteryPancake/Houdini-Box3D
```

**Windows:** open **Houdini X.Y.Z > Command Line Tools** from the Start menu, then run `build.bat` (or `cmake -S . -B build -G "Visual Studio 17 2022" -A x64`, then `cmake --build build --config Release`). The DLL is copied to `Documents\houdiniX.Y\dso`.

**Linux:** install `g++ cmake git mesa-common-dev libglu1-mesa-dev libxi-dev` (Red Hat: `gcc-c++ cmake git mesa-libGL-devel mesa-libGLU-devel libXi-devel`), ideally the gcc version named in your Houdini download (`...-gccX.Y.tar.gz`), then:

```
source /opt/hfsX.Y.ZZZ/houdini_setup
./build.sh
```

The plugin is copied to `~/houdiniX.Y/dso`.

Always build **Release**, never Debug, and close Houdini before rebuilding.

### Packaging

To share the plugin, build it, then make a zip in the build folder:

```
cd build
cpack -G ZIP -C Release
```

(`-C Release` is needed on Windows. On Linux, plain `cpack -G ZIP` works.) This makes `Box3D-Houdini-<version>-<system>.zip`. Extract it into your `houdiniX.Y/packages` folder so `Box3D.json` sits directly inside it. It only loads in the Houdini version it was built for.

The zip includes the Box3D Packed Object, Box3D Configure and Box3D Solver SOP assets only if hython was found during the build. If you test the package on the machine you built on, first remove the copies the build put in your `houdiniX.Y/dso` and `otls` folders.

## Quick start

In a DOP network, add a **Box3D Solver** and wire your objects into it like any DOP solver. For packed pieces, use a **Box3D Packed Object**. Its settings (shape, mass, friction, bounce, ...) apply to every piece that doesn't set its own attribute.

Like Houdini's RBD solvers, every point is one rigid body and the point is its **centre of mass** (for packed pieces, the pivot, which the Pack SOP puts at the centroid). Objects from **RBD Object** and similar nodes are one body each instead, with their results in the object's Position data.

Example scenes: run `make_examples.bat` (Linux: `hython examples/make_examples.py`) and open `examples/scenes`.

## Nodes

| Node | Use |
|---|---|
| Box3D Solver | The solver |
| Box3D Packed Object | Packed geometry as a Box3D object, with Box3D Data's settings on the node |
| Box3D Solver (SOP) | Like RBD Bullet Solver: geometry (packed or not), constraints, proxy and collision in, simulated out |
| Box3D Configure (SOP) | Like RBD Configure: packs geometry and proxy, then sets Box3D Data's settings as point attributes on a group of pieces (a group name, not a pattern) |
| Box3D Data | Per-object settings, for objects made by other nodes |
| Box3D Weld / Spherical / Revolute / Prismatic / Distance / Motor / Parallel / Wheel / Filter Constraint Relationship | Joint types, used with a Constraint Network |

## Reference

- [Packed objects and constraints](docs/nodes.md): RBD Packed Object, unpacked geometry, other constraint types
- [Point attributes](docs/attributes.md), including collision filtering
- [Shapes](docs/shapes.md)
- [Static Objects](docs/static-objects.md)
- [Joints](docs/joints.md)
- [Things to know](docs/notes.md): where this differs from RBD Bullet Solver

## Troubleshooting

**"world limit reached".** Simulating on from an earlier frame keeps extra copies of the simulation alive for the cached frames. Reset the simulation or clear the DOP cache, or build with `-DBOX3DDOP_MAX_WORLDS=<n>`.

**Debug logging:** configure with `-DBOX3DDOP_DEBUG=ON` to print to Houdini's console.

## For developers

See [CLAUDE.md](CLAUDE.md). Before committing, format the code with `cmake --build build --target format`. CI checks it with `--target format-check`.

## License

MIT, see [LICENSE](LICENSE).
