from math import pi

import pytest

from robot_arm_demo.geometry import Point2D
from robot_arm_demo.planar_arm import JointAngles, PlanarArm2D


def test_forward_kinematics_straight_arm() -> None:
    arm = PlanarArm2D(link_1=1.0, link_2=1.0)

    point = arm.forward_kinematics(JointAngles(shoulder=0.0, elbow=0.0))

    assert point.x == pytest.approx(2.0, abs=1e-9)
    assert point.y == pytest.approx(0.0, abs=1e-9)


def test_forward_kinematics_right_angle() -> None:
    arm = PlanarArm2D(link_1=1.0, link_2=1.0)

    point = arm.forward_kinematics(JointAngles(shoulder=pi / 2, elbow=0.0))

    assert point.x == pytest.approx(0.0, abs=1e-9)
    assert point.y == pytest.approx(2.0, abs=1e-9)


def test_inverse_kinematics_reaches_target() -> None:
    arm = PlanarArm2D(link_1=1.0, link_2=1.0)
    target = Point2D(1.0, 1.0)

    angles = arm.inverse_kinematics(target)
    reached = arm.forward_kinematics(angles)

    assert reached.x == pytest.approx(target.x, abs=1e-9)
    assert reached.y == pytest.approx(target.y, abs=1e-9)


def test_inverse_kinematics_rejects_unreachable_target() -> None:
    arm = PlanarArm2D(link_1=1.0, link_2=1.0)

    with pytest.raises(ValueError):
        arm.inverse_kinematics(Point2D(3.0, 0.0))
