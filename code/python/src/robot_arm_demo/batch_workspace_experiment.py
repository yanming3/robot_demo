"""Command-line demo for the 2D robot arm."""

from __future__ import annotations

from .geometry import Point2D, rad_to_deg
from .planar_arm import JointAngles, PlanarArm2D
from .trajectory import joint_space_trajectory

def main() -> int:
    arm = PlanarArm2D(link_1=1.0, link_2=0.4)
    cases = [
        (Point2D(1.0, 0.2), 8),
        (Point2D(1.5, 0.0), 8),
        (Point2D(0.2, 0.0), 8),
        (Point2D(1.0, 0.2), 0),
    ]
    print(f"target_x,target_y,steps,status,stage,reason")
    start = JointAngles(shoulder=0.0, elbow=0.0)
    for target, steps in cases:
        goal = None
        try:
            goal = arm.inverse_kinematics(target, elbow_up=False)
            trajectory = joint_space_trajectory(start, goal, steps)
            print(f"{target.x:.3f},{target.y:.3f},{steps},ok,done,reachable")
        except ValueError as error:
            if goal is None:
                print(f"{target.x:.3f},{target.y:.3f},{steps},ng,ik,{error}")
            else:
                print(f"{target.x:.3f},{target.y:.3f},{steps},ng,trajectory,{error}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
