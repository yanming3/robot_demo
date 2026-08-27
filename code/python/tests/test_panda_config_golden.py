"""1:1 数值锁：panda_mujoco 配置的每个字段必须与原硬编码逐位一致。

本文件是重构期最强的防漂移护栏——任何字段值变化都会在这里立刻爆红。
"""

from robot_arm_demo.demos.panda_mujoco.config import build_panda_mujoco_config


def test_arm_config_golden():
    arm = build_panda_mujoco_config().arm
    assert arm.move_group_name == "panda_arm"
    assert arm.base_frame == "panda_link0"
    assert arm.eef_link == "panda_link8"

    assert arm.gripper_command_topic == "/panda_hand_controller/gripper_cmd"
    assert arm.gripper_joint_names == ("panda_finger_joint1", "panda_finger_joint2")
    assert arm.gripper_max_width == 0.08
    assert arm.gripper_open_pos == 0.04
    assert arm.gripper_grasp_pos == 0.024

    assert arm.finger_tip_offset == 0.1034
    assert arm.min_eef_z == 0.20

    assert arm.home_joint_names == (
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    )
    assert arm.home_joint_values == (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
    assert arm.home_joint_tolerance == 0.01

    assert arm.tip_orientation_wxyz == (0.0, 0.9238795, -0.3826834, 0.0)
    assert arm.default_velocity_scaling == 0.3
    assert arm.num_planning_attempts == 5
    assert arm.allowed_planning_time == 5.0
    assert arm.replan_attempts == 2
    assert arm.position_tolerance == 0.005
    assert arm.orientation_tolerance == 0.05

    assert arm.reachable_x_min == 0.25
    assert arm.sanity_box == (0.20, 0.60, -0.35, 0.35, 0.0, 0.30)


def test_object_config_golden():
    obj = build_panda_mujoco_config().object
    assert obj.object_id == "coke"
    assert obj.radius == 0.033
    assert obj.height == 0.122
    assert obj.place_position == (0.0, 0.28, 0.061)


def test_grasp_config_golden():
    g = build_panda_mujoco_config().grasp
    # 派生的接触窗口 = [radius-0.003, radius] = [0.030, 0.033]
    assert g.contact_qpos_min == 0.030
    assert g.contact_qpos_max == 0.033

    assert g.coarse_step == 0.003
    assert g.fine_step == 0.001
    assert g.min_qpos == 0.028
    assert g.displacement_tolerance == 0.003

    assert g.micro_lift_height == 0.015
    assert g.micro_lift_z_min == 0.010
    assert g.micro_lift_lateral_max == 0.005
    assert g.micro_lift_velocity_scaling == 0.1

    assert g.max_grasp_attempts == 3
    assert g.stable_samples == 2
    assert g.stable_interval == 0.1
    assert g.step_settle == 0.2

    assert g.hover_offset == 0.12
    assert g.lift_z_min == 0.38
    assert g.post_move_settle == 0.3
    assert g.release_settle == 0.5
    assert g.recover_settle == 0.3
    assert g.close_max_effort == 10.0
    assert g.open_max_effort == 0.0


def test_camera_config_golden():
    cam = build_panda_mujoco_config().camera
    assert cam.frame_id == "camera_link"
    assert cam.fx == 554.0
    assert cam.fy == 554.0
    assert cam.cx == 320.0
    assert cam.cy == 240.0
    assert cam.assumed_depth == 0.76
