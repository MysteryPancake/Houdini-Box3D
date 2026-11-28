"""Box3D's own samples (extern/box3d/samples) as Houdini scenes, one module per sample source file.

make_examples.py --samples builds them into examples/<category>/<sample>.hip.
Which samples port and how: docs/agents/sample-port-plan.md.
"""

from . import (continuous, events, issues, joints, mesh, restitution, robustness, shapes,  # register their samples
               stacking)
from .common import SAMPLES, start_scene

__all__ = ["SAMPLES", "start_scene", "continuous", "events", "issues", "joints", "mesh", "restitution", "robustness",
           "shapes", "stacking"]
