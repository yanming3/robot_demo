"""Kinematics for a 2D two-link robot arm."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, sin, sqrt

from .geometry import Point2D


@dataclass(frozen=True)
class JointAngles:
    """Joint angles in radians."""

    shoulder: float
    elbow: float

    def as_tuple(self) -> tuple[float, float]:
        return (self.shoulder, self.elbow)


@dataclass(frozen=True)
class PlanarArm2D:
    """A planar robot arm with two revolute joints."""

    link_1: float = 1.0
    link_2: float = 1.0

    def __post_init__(self) -> None:
        if self.link_1 <= 0 or self.link_2 <= 0:
            raise ValueError("Link lengths must be positive.")

    @property
    def max_reach(self) -> float:
        return self.link_1 + self.link_2

    @property
    def min_reach(self) -> float:
        return abs(self.link_1 - self.link_2)

    def forward_kinematics(self, angles: JointAngles) -> Point2D:
        """Return end-effector position for the given joint angles."""

        shoulder = angles.shoulder
        elbow = angles.elbow
        x = self.link_1 * cos(shoulder) + self.link_2 * cos(shoulder + elbow)
        y = self.link_1 * sin(shoulder) + self.link_2 * sin(shoulder + elbow)
        return Point2D(x, y)

    def inverse_kinematics(self, target: Point2D, elbow_up: bool = False) -> JointAngles:
        """Solve joint angles for a reachable target point.

        Raises:
            ValueError: if the target is outside the arm workspace.
        """

        distance = sqrt(target.x * target.x + target.y * target.y)
        if distance > self.max_reach:
            raise ValueError(
                f"Target ({target.x:.3f}, {target.y:.3f}) is outside max reach "
                f"[{self.min_reach:.3f}, {self.max_reach:.3f}]."
            )
        elif distance < self.min_reach:
            raise ValueError(
                f"Target ({target.x:.3f}, {target.y:.3f}) is inside min reach "
                f"[{self.min_reach:.3f}, {self.max_reach:.3f}]."
            )

        cos_elbow = (
            target.x * target.x
            + target.y * target.y
            - self.link_1 * self.link_1
            - self.link_2 * self.link_2
        ) / (2 * self.link_1 * self.link_2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow = acos(cos_elbow)
        if elbow_up:
            elbow = -elbow

        k1 = self.link_1 + self.link_2 * cos(elbow)
        k2 = self.link_2 * sin(elbow)
        shoulder = atan2(target.y, target.x) - atan2(k2, k1)
        return JointAngles(shoulder=shoulder, elbow=elbow)
