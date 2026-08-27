"""Trajectory helpers."""

from __future__ import annotations

from .planar_arm import JointAngles


def joint_space_trajectory(
    start: JointAngles,
    end: JointAngles,
    steps: int,
) -> list[JointAngles]:
    """Linearly interpolate joint angles from start to end.

    The returned list includes both start and end. For example, ``steps=4`` returns
    five samples.
    """

    if steps < 1:
        raise ValueError("steps must be at least 1.")

    samples: list[JointAngles] = []
    for index in range(steps + 1):
        ratio = index / steps
        samples.append(
            JointAngles(
                shoulder=start.shoulder + (end.shoulder - start.shoulder) * ratio,
                elbow=start.elbow + (end.elbow - start.elbow) * ratio,
            )
        )
    return samples
