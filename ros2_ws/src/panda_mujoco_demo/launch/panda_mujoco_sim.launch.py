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
    # NOTE: this is the "sim core" launch used during Phase 1 when MoveIt2 / move_group
    # is not yet built. The full demo launch is panda_mujoco.launch.py.
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

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run simulation without visualization window",
    )
    return LaunchDescription([headless, OpaqueFunction(function=launch_setup)])
