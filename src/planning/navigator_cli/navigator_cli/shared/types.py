#!/usr/bin/env python3
"""
Type definitions for the ROS workspace domain.
This module contains all the custom types, data classes, and interfaces used across the domain.
"""


from .constants import FLOAT_TOLERANCE

import math
from typing import Optional

from .model import FrozenModel, Model

# --------------- Data Classes ----------------

class Point(Model):
    """3D point representation."""
    x: float
    y: float
    z: float

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        super().__init__(x=x, y=y, z=z)

    def distance_to(self, other: "Point") -> float:
        return math.sqrt((self.x - other.x) ** 2 +
                         (self.y - other.y) ** 2 +
                         (self.z - other.z) ** 2)

    def __eq__(self, other):
        if not isinstance(other, Point):
            return NotImplemented
        return (abs(self.x - other.x) < FLOAT_TOLERANCE and
                abs(self.y - other.y) < FLOAT_TOLERANCE and
                abs(self.z - other.z) < FLOAT_TOLERANCE)

    def __hash__(self):
        # Hash based on rounded values to be consistent with __eq__
        return hash((round(self.x / FLOAT_TOLERANCE),
                     round(self.y / FLOAT_TOLERANCE),
                     round(self.z / FLOAT_TOLERANCE)))

    def __add__(self, other: "Point") -> "Point":
        if not isinstance(other, Point):
            return NotImplemented
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

class Quaternion(Model):
    """Quaternion representation for orientations."""
    x: float
    y: float
    z: float
    w: float

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        super().__init__(x=x, y=y, z=z, w=w)

    def __eq__(self, other):
        if not isinstance(other, Quaternion):
            return NotImplemented
        return (abs(self.x - other.x) < FLOAT_TOLERANCE and
                abs(self.y - other.y) < FLOAT_TOLERANCE and
                abs(self.z - other.z) < FLOAT_TOLERANCE and
                abs(self.w - other.w) < FLOAT_TOLERANCE)

    def __hash__(self):
        return hash((round(self.x / FLOAT_TOLERANCE),
                     round(self.y / FLOAT_TOLERANCE),
                     round(self.z / FLOAT_TOLERANCE),
                     round(self.w / FLOAT_TOLERANCE)))


class TrajectoryMovementOptions(FrozenModel):
    """Per-edge / per-call planning and execution settings."""
    speed: Optional[float] = None  # (0, 1], cuMotion time_dilation_factor
    max_attempts: Optional[int] = None  # planning attempts before giving up
