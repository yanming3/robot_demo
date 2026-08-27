"""Command-line demo for the 2D robot arm."""

from __future__ import annotations

import argparse
import csv
import json
from .geometry import Point2D, rad_to_deg
from .planar_arm import JointAngles, PlanarArm2D
from .trajectory import joint_space_trajectory


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a 2D two-link robot arm demo.")
    parser.add_argument("--link-1", type=float, default=1.0, help="Length of link 1.")
    parser.add_argument("--link-2", type=float, default=1.0, help="Length of link 2.")
    parser.add_argument(
        "--target",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        default=(1.2, 0.8),
        help="Target end-effector position.",
    )
    parser.add_argument("--steps", type=int, default=8, help="Interpolation steps.")
    parser.add_argument(
        "--elbow-up", action="store_true", help="Use elbow-up IK solution."
    )
    parser.add_argument(
        "--output", type=str, help="Write trajectory rows to a CSV file."
    )
    return parser


def run_demo(args: argparse.Namespace) -> int:
    arm = PlanarArm2D(link_1=args.link_1, link_2=args.link_2)
    target = Point2D(args.target[0], args.target[1])
    start = JointAngles(shoulder=0.0, elbow=0.0)
    goal = arm.inverse_kinematics(target, elbow_up=args.elbow_up)
    trajectory = joint_space_trajectory(start, goal, args.steps)

    rows = []
    for index, angles in enumerate(trajectory):
        end_effector = arm.forward_kinematics(angles)
        rows.append(
            {
                "step": index,
                "shoulder_deg": rad_to_deg(angles.shoulder),
                "elbow_deg": rad_to_deg(angles.elbow),
                "end_x": end_effector.x,
                "end_y": end_effector.y,
            }
        )

    print("step,shoulder_deg,elbow_deg,end_x,end_y")
    for row in rows:
        print(
            f"{row['step']},"
            f"{row['shoulder_deg']:.3f},"
            f"{row['elbow_deg']:.3f},"
            f"{row['end_x']:.3f},"
            f"{row['end_y']:.3f}"
        )

    if args.output:
        fieldnames = ["step", "shoulder_deg", "elbow_deg", "end_x", "end_y"]
        with open(args.output, "w", newline="", encoding="utf-8") as output_file:
            if args.output.endswith(".json"):
                json.dump(rows, output_file, indent=2)
            else:
                writer = csv.DictWriter(output_file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run_demo(args)
    except ValueError as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
