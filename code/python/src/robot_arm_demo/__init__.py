"""Learning-oriented robot arm demo package."""

from .geometry import Point2D
from .planar_arm import JointAngles, PlanarArm2D
from .trajectory import joint_space_trajectory

__all__ = [
    "JointAngles",
    "PlanarArm2D",
    "Point2D",
    "joint_space_trajectory",
]
