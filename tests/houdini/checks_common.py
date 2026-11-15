"""Constants and small helpers shared by the checks_* modules."""

import math

from b3dtest import node_messages

G = 9.80665
MIXING = ("Geometric Mean", "Minimum", "Maximum", "Average", "Multiply")


def mix(rule, a, b):
    return {"Geometric Mean": math.sqrt(a * b), "Minimum": min(a, b), "Maximum": max(a, b),
            "Average": 0.5 * (a + b), "Multiply": a * b}[rule]


def box3d_messages(node):
    """The messages on a Box3D Solver node (all of them are the solver's), sorted, once each."""
    return sorted({m.strip() for m in node_messages(node)})
