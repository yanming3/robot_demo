"""panda_mujoco demo 的全部配置数值（从原 pick_place_state_machine /
perception_node 硬编码常量 1:1 迁入，经 test_panda_config_golden 锁定）。"""

from __future__ import annotations

from ...core.data import (
    ArmConfig,
    CameraConfig,
    GraspConfig,
    ObjectConfig,
    PickPlaceConfig,
)
from ...core.grasp import build_grasp_config


def build_panda_mujoco_config() -> PickPlaceConfig:
    """构造 panda + MuJoCo 场景的完整配置。数值与原硬编码逐位一致。"""
    arm = ArmConfig(
        move_group_name="panda_arm",
        base_frame="panda_link0",
        eef_link="panda_link8",

        gripper_command_topic="/panda_hand_controller/gripper_cmd",
        gripper_joint_names=("panda_finger_joint1", "panda_finger_joint2"),
        gripper_max_width=0.08,
        gripper_open_pos=0.04,
        # 历史：一次性命令到过盈位置(0.024)曾穿透碰撞体/夹不紧致 LIFT 滑落，
        # 已由两段式收紧取代；仅存档，FSM 不再使用。
        gripper_grasp_pos=0.024,

        finger_tip_offset=0.1034,
        # eef_link 最低下降高度：hand_c 掌心底部在 link8 下 ~0.0403，若降到
        # 0.16 会压到可乐顶面(≈0.122)把可乐撞倒。0.20 → 掌心底≈0.1597，
        # 距顶面约 3.8cm 安全距离。
        min_eef_z=0.20,

        home_joint_names=(
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7",
        ),
        home_joint_values=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
        home_joint_tolerance=0.01,

        tip_orientation_wxyz=(0.0, 0.9238795, -0.3826834, 0.0),
        default_velocity_scaling=0.3,
        num_planning_attempts=5,
        allowed_planning_time=5.0,
        replan_attempts=2,
        position_tolerance=0.005,
        orientation_tolerance=0.05,

        reachable_x_min=0.25,
        sanity_box=(0.20, 0.60, -0.35, 0.35, 0.0, 0.30),
    )

    obj = ObjectConfig(
        object_id="coke",
        radius=0.033,
        height=0.122,
        # 放置点在 +Y 方向、z=0.061 底部贴桌面；LIFT 后 joint1 绕 base
        # 旋转 ~90°，演示"抓起→转身→放下"。桌内 y∈[-0.4,0.4]、距 base
        # 0.28m 可达。
        place_position=(0.0, 0.28, 0.061),
    )

    grasp = _build_panda_grasp()

    camera = CameraConfig(
        frame_id="camera_link",
        fx=554.0,
        fy=554.0,
        cx=320.0,
        cy=240.0,
        # MuJoCo 可乐中心 (0.3,0,0.061)、相机 (0.4,0.5,0.625) 的直线距离。
        assumed_depth=0.76,
    )

    return PickPlaceConfig(arm=arm, object=obj, grasp=grasp, camera=camera)


def _build_panda_grasp() -> GraspConfig:
    """Panda 夹爪的抓取调参（build_grasp_config 默认值即 panda 实测值）。"""
    cfg = build_grasp_config(radius=0.033)
    assert cfg.min_qpos == 0.028, "min_qpos 必须小于 contact_qpos_min"
    return cfg
