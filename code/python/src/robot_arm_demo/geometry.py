"""Small geometry helpers for the robot arm demo."""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees, radians


@dataclass(frozen=True)
class Point2D:
    """A point in a 2D Cartesian coordinate system."""

    x: float
    y: float

    def as_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)


def deg_to_rad(value: float) -> float:
    return radians(value)


def rad_to_deg(value: float) -> float:
    return degrees(value)
