"""抓取判定数学的纯逻辑单测（与 panda demo 数值一致性联动 golden 锁）。"""

import pytest

from robot_arm_demo.core.data import ArmConfig, ObjectConfig
from robot_arm_demo.core.grasp import (
    build_grasp_config,
    compute_grasp_z,
    compute_hover_z,
    compute_lift_z,
    compute_recover_grasp_z,
    compute_place_grasp_z,
    displacement_ok,
    finger_contact_ok,
    micro_lift_ok,
)


@pytest.fixture
def arm():
    return ArmConfig(
        move_group_name="panda_arm",
        base_frame="panda_link0",
        eef_link="panda_link8",
        gripper_command_topic="/t",
        gripper_joint_names=("f1", "f2"),
        gripper_max_width=0.08,
        gripper_open_pos=0.04,
        gripper_grasp_pos=0.024,
        finger_tip_offset=0.1034,
        min_eef_z=0.20,
        home_joint_names=("j1",),
        home_joint_values=(0.0,),
        home_joint_tolerance=0.01,
        tip_orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
        default_velocity_scaling=0.3,
        num_planning_attempts=5,
        allowed_planning_time=5.0,
        replan_attempts=2,
        position_tolerance=0.005,
        orientation_tolerance=0.05,
        reachable_x_min=0.25,
        sanity_box=(0.20, 0.60, -0.35, 0.35, 0.0, 0.30),
    )


def test_build_grasp_config_derives_contact_band():
    g = build_grasp_config(radius=0.033)
    assert g.contact_qpos_max == 0.033
    assert g.contact_qpos_min == 0.030
    assert g.min_qpos == 0.028


def test_build_grasp_config_radius_generic():
    """非可乐半径也必须派生出 [r-0.003, r] 窗口。"""
    g = build_grasp_config(radius=0.05)
    assert g.contact_qpos_max == 0.05
    assert g.contact_qpos_min == pytest.approx(0.047)


def test_build_grasp_config_rejects_contact_override():
    with pytest.raises(TypeError):
        build_grasp_config(radius=0.033, contact_qpos_min=0.01)


def test_compute_grasp_z_formula_and_clamp(arm):
    import dataclasses

    low_arm = dataclasses.replace(arm, min_eef_z=0.10)
    # 无钳制路径：可乐中心 z≈0.059 → grasp_z = 0.059+0.1034 = 0.1624
    assert compute_grasp_z(0.059, low_arm) == pytest.approx(0.1624)


def test_compute_grasp_z_clamped_to_min(arm):
    # demo 现实形态：桌面上的可乐需求 z=0.1624 < min_eef_z=0.20 → 钳到 0.20
    # （这也是 micro-lift 目标 z=0.215=0.20+0.015 的由来）
    assert compute_grasp_z(0.059, arm) == 0.20
    assert compute_grasp_z(0.02, arm) == 0.20


def test_compute_hover_and_lift():
    g = build_grasp_config(radius=0.033)
    assert compute_hover_z(0.20, g) == pytest.approx(0.32)
    # lift 锚点是悬停位：max(cur_hover, 0.38)
    assert compute_lift_z(0.32, g) == 0.38
    assert compute_lift_z(0.45, g) == 0.45


def test_compute_recover_grasp_z_matches_main_formula(arm):
    import dataclasses

    low_arm = dataclasses.replace(arm, min_eef_z=0.10)
    assert compute_recover_grasp_z(0.059, low_arm) == pytest.approx(0.1624)
    # demo 现实形态：桌面可乐 → max(0.1624, 0.20) = 0.20
    assert compute_recover_grasp_z(0.059, arm) == 0.20


def test_compute_place_grasp_z(arm):
    obj = ObjectConfig(object_id="coke", radius=0.033, height=0.122,
                       place_position=(0.0, 0.28, 0.061))
    # 0.061 + 0.1034 = 0.1644 —— 与 demo 日志中的放置下降高度一致
    assert compute_place_grasp_z(obj, arm) == pytest.approx(0.1644)


def test_finger_contact_window():
    g = build_grasp_config(radius=0.033)
    assert finger_contact_ok(0.0309, g)          # 实测样本
    assert finger_contact_ok(0.030, g)           # 下界闭区间
    assert finger_contact_ok(0.033, g)           # 上界闭区间
    assert not finger_contact_ok(0.0299, g)
    assert not finger_contact_ok(0.034, g)
    assert not finger_contact_ok(None, g)        # 信号缺失 → 重试


def test_displacement_tolerance():
    g = build_grasp_config(radius=0.033)
    assert displacement_ok(0.003, g)
    assert displacement_ok(0.0, g)
    assert not displacement_ok(0.0031, g)
    assert not displacement_ok(None, g)


def test_micro_lift_thresholds():
    g = build_grasp_config(radius=0.033)
    # dz 恰达下界、侧漂恰低于上限 → 通过（demo 第二次尝试即此形态）
    assert micro_lift_ok(0.0615, 0.0721, 0.0049, g)
    # dz 不足
    assert not micro_lift_ok(0.0615, 0.0700, 0.001, g)
    # 侧漂过大（第一次尝试失败即此形态）
    assert not micro_lift_ok(0.0615, 0.0734, 0.006, g)
    # 边界: dz == 0.010 通过; lateral == 0.005 不通过（严格小于）
    assert micro_lift_ok(0.0, 0.010, 0.0, g)
    assert not micro_lift_ok(0.0, 0.010, 0.005, g)
