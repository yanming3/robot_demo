import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("panda_mujoco_demo")

    # Build robot description from the MuJoCo xacro.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "panda.mujoco.urdf.xacro"]),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )
    robot_description_str = robot_description_content.perform(context)
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    parameters_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    mujoco_plugins_file = PathJoinSubstitution(
        [pkg_share, "config", "mujoco_ros2_control_plugins.yaml"]
    )

    nodes = []

    # Robot state publisher.
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ros2_control node with MuJoCo (single process: simulation + controller_manager).
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(parameters_file),
                ParameterFile(mujoco_plugins_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        )
    )

    # Controller spawners (activate as a group after the sim is up).
    controllers_to_spawn = [
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_hand_controller",
    ]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", parameters_file],
                output="both",
            )
        )

    # MoveIt2 move_group, using the standard Panda SRDF but the MuJoCo URDF.
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": "mock_components",
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=[
                "ompl",
            ]
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters.update(robot_description)

    # Relax the trajectory start-point tolerance. The default 0.01 rad (from
    # moveit_resources_panda_moveit_config) is too strict for back-to-back
    # moves: after MOVE_ABOVE executes, the PlanningSceneMonitor has not yet
    # absorbed the new joint_state when DESCEND is planned, so the validated
    # trajectory start (stale scene state) deviates from the live ros2_control
    # state and MoveIt rejects it with CONTROL_FAILED. 0.05 rad (~2.9 deg) is
    # still tight enough to catch a genuinely wrong start state while tolerating
    # this transient desync. See pick_place_state_machine.py for the matching
    # post-move settle delay.
    moveit_parameters.setdefault("trajectory_execution", {})[
        "allowed_start_tolerance"
    ] = 0.05

    nodes.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_parameters,
                {"use_sim_time": True},
            ],
            arguments=["--ros-args", "--log-level", "info"],
        )
    )

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )
    return LaunchDescription([headless, OpaqueFunction(function=launch_setup)])
