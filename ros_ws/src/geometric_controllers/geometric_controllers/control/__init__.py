"""Core control logic (ROS-agnostic).

This package mirrors the MATLAB split between:
- potentials: pose error -> 6D error vector
- adaptation: parameter estimation and payload handling

ROS nodes should only call into these modules and publish results.
"""

from .potentials import Potential, PotentialFactory
from .adaptation import (
    Adaptation,
    EuclideanAdaptation,
    GeoAwareAdaptation,
    NoAdaptation,
)

__all__ = [
    "Potential",
    "PotentialFactory",
    "Adaptation",
    "EuclideanAdaptation",
    "GeoAwareAdaptation",
    "NoAdaptation",
]
