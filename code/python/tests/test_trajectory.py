import pytest

from robot_arm_demo.planar_arm import JointAngles
from robot_arm_demo.trajectory import joint_space_trajectory


def test_includes_start_and_end() -> None:
    start = JointAngles(shoulder=0.0, elbow=0.0)
    end = JointAngles(shoulder=1.0, elbow=2.0)

    samples = joint_space_trajectory(start, end, steps=4)

    assert len(samples) == 5
    assert samples[0] == start
    assert samples[-1] == end


def test_rejects_invalid_step_count() -> None:
    with pytest.raises(ValueError):
        joint_space_trajectory(
            JointAngles(shoulder=0.0, elbow=0.0),
            JointAngles(shoulder=1.0, elbow=1.0),
            steps=0,
        )
