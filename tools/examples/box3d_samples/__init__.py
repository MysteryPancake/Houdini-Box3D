"""Box3D's own samples (extern/box3d/samples) as Houdini scenes, one module per sample source file.

make_examples.py --samples builds them into examples/<category>/<sample>.hip.
Which samples port and how: docs/agents/sample-port-plan.md.
"""

from . import continuous, issues, mesh, restitution, robustness, shapes, stacking  # each registers its samples
from .common import SAMPLES, start_scene

__all__ = ["SAMPLES", "start_scene", "continuous", "issues", "mesh", "restitution", "robustness", "shapes", "stacking"]
