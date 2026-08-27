"""抓取相关的纯函数：配置派生 + 判定数学（零 ROS 依赖）。

这些函数把原 pick_place_state_machine.py 里散落的算术集中成可单测的
纯逻辑；FSM 只调用它们，阈值比较符逐字保留。
"""

from __future__ import annotations

from .data import ArmConfig, GraspConfig, ObjectConfig

# 「碰住未穿透」窗口的半带宽：contact 下界 = radius - CONTACT_BAND
CONTACT_BAND = 0.003


def _contact_window(radius: float, band: float) -> tuple[float, float]:
    """[radius-band, radius]，对下界做 12 位舍入去掉浮点尘埃。

    必须逐位复现原硬编码语义：0.033-0.003 的二进制结果是
    0.030000000000000002（> 字面量 0.030），会让 "x>=下界" 在 x==0.030
    处由 True 翻成 False。round(...,12) 使派生值与原字面量逐位一致。
    """
    return round(radius - band, 12), radius


def build_grasp_config(radius: float, **overrides) -> GraspConfig:
    """由物体半径派生接触窗口，构造 GraspConfig。

    contact_qpos_max = radius，contact_qpos_min = radius - CONTACT_BAND，
    其余字段取通用默认值（与 panda demo 的实测调参一致），可覆盖。
    """
    contact_min, contact_max = _contact_window(radius, CONTACT_BAND)
    defaults = dict(
        coarse_step=0.003,
        fine_step=0.001,
        min_qpos=0.028,
        displacement_tolerance=0.003,
        micro_lift_height=0.015,
        micro_lift_z_min=0.010,
        micro_lift_lateral_max=0.005,
        micro_lift_velocity_scaling=0.1,
        max_grasp_attempts=3,
        stable_samples=2,
        stable_interval=0.1,
        step_settle=0.2,
        hover_offset=0.12,
        lift_z_min=0.38,
        post_move_settle=0.3,
        release_settle=0.5,
        recover_settle=0.3,
        close_max_effort=10.0,
        open_max_effort=0.0,
    )
    overlap = set(overrides) & {
        "contact_qpos_min", "contact_qpos_max"
    }
    if overlap:
        raise TypeError(f"contact_qpos_* 由 radius 派生，不可覆盖: {overlap}")
    return GraspConfig(
        contact_qpos_min=contact_min,
        contact_qpos_max=contact_max,
        **{**defaults, **overrides},
    )


def compute_grasp_z(object_z: float, arm: ArmConfig) -> float:
    """指尖对准物体中心所需 eef_link z = object_z + finger_tip_offset，
    并钳制到安全下限 min_eef_z（防掌心压倒目标物）。"""
    grasp_z = object_z + arm.finger_tip_offset
    if grasp_z < arm.min_eef_z:
        grasp_z = arm.min_eef_z
    return grasp_z


def compute_hover_z(grasp_z: float, grasp: GraspConfig) -> float:
    """悬停高度：抓取点上方 hover_offset。"""
    return grasp_z + grasp.hover_offset


def compute_recover_grasp_z(object_z: float, arm: ArmConfig) -> float:
    """recover_grasp 用：max(cz + offset, min_eef_z)，与主路径同公式。"""
    return max(object_z + arm.finger_tip_offset, arm.min_eef_z)


def compute_place_grasp_z(object_cfg: ObjectConfig, arm: ArmConfig) -> float:
    """放置点对应的 eef_link z（让物体底部贴桌面）。"""
    return object_cfg.place_position[2] + arm.finger_tip_offset


def compute_lift_z(cur_hover_z: float, grasp: GraspConfig) -> float:
    """完整抬起目标 z = max(当前悬停z, lift_z_min)。注意锚点是悬停位。"""
    return max(cur_hover_z, grasp.lift_z_min)


def finger_contact_ok(finger_avg: float | None, grasp: GraspConfig) -> bool:
    """双信号之一：手指平均位置落在「碰住未穿透」窗口内。"""
    if finger_avg is None:
        return False
    return grasp.contact_qpos_min <= finger_avg <= grasp.contact_qpos_max


def displacement_ok(displacement: float | None, grasp: GraspConfig) -> bool:
    """双信号之二：收紧过程中物体位移在容差内。"""
    if displacement is None:
        return False
    return displacement <= grasp.displacement_tolerance


def micro_lift_ok(before_z: float, after_z: float,
                  lateral: float, grasp: GraspConfig) -> bool:
    """微抬验证：物体 z 跟随抬升 ≥ micro_lift_z_min 且侧向漂移 < 上限。"""
    dz = after_z - before_z
    return dz >= grasp.micro_lift_z_min and lateral < grasp.micro_lift_lateral_max
