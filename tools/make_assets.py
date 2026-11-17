"""Builds the Box3D digital assets in one hython session.

    hython tools/make_assets.py <otls folder>

Writes box3d_packedobject.hda, box3d_configure.hda, box3d_constraint_properties.hda and
box3d_solver_sop.hda into the folder, in that order (the Solver SOP contains the Packed Object, so it
comes last). One process, so Houdini starts
and checks out its license once. The build runs this (CMakeLists.txt, BOX3DDOP_MAKE_HDA) whenever the
plugin or any script in tools/ changes.

Never fails the build: each asset's problems are printed as warnings and the script exits 0, as
the three make_*.py scripts do on their own.
"""

import os
import sys
import traceback

# No __pycache__ next to the scripts (they are imported from this folder).
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PACKED = "box3d_packedobject.hda"
CONFIGURE = "box3d_configure.hda"
CONSTRAINT_PROPERTIES = "box3d_constraint_properties.hda"
SOLVER_SOP = "box3d_solver_sop.hda"


def run(label, fn, *args):
    try:
        fn(*args)
    except Exception:  # noqa: BLE001 - never fail the plugin build over an asset
        print(f"make_assets: WARNING: the {label} asset was not built:\n" + traceback.format_exc())


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 0
    import make_configure_sop
    import make_constraint_properties_sop
    import make_packed_object
    import make_solver_sop

    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    packed = os.path.join(out, PACKED)
    run("Box3D Packed Object", make_packed_object.build, packed)
    run("Box3D Configure", make_configure_sop.build, os.path.join(out, CONFIGURE))
    run("Box3D Constraint Properties", make_constraint_properties_sop.build,
        os.path.join(out, CONSTRAINT_PROPERTIES))
    run("Box3D Solver SOP", make_solver_sop.build, os.path.join(out, SOLVER_SOP), packed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
