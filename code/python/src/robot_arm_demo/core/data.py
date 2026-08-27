"""机器人无关的数据模型：配置 dataclass 与任务指令。

字段取值全部由 demos/<name>/config.py 提供，core 不含任何具体
机器人/物体/场景的数值。历史调参注释随字段保留。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmConfig:
    """机械臂本体 + MoveIt 规划参数（每臂一份）。"""

    # MoveIt / TF 帧与规划组
    move_group_name: str          # 如 "panda_arm"
    base_frame: str               # 如 "panda_link0"（感知结果、约束的参考系）
    eef_link: str                 # 末端 link，position constraint 作用点

    # 夹爪（gripper command 走 GripperCommand action）
    gripper_command_topic: str
    gripper_joint_names: tuple[str, ...]   # 手指关节（按顺序），/joint_states 读回用
    gripper_max_width: float      # 最大开口（文档性；夹爪几何上确有此限制）
    gripper_open_pos: float       # 每侧手指"全开"位置
    # 历史：一次性命令到过盈位置曾致穿透/夹不紧，已被两段式收紧取代，
    # 此值仅存档不再被 FSM 使用。
    gripper_grasp_pos: float

    # 几何偏移与安全下限
    finger_tip_offset: float      # eef_link → 指尖闭合点 (TCP) 的 z 距离
    min_eef_z: float              # eef_link 最低下降高度（防掌心压倒目标物）

    # home 位形（RETURN_HOME 目标；需与 initial_positions.yaml / scene keyframe 一致）
    home_joint_names: tuple[str, ...]
    home_joint_values: tuple[float, ...]
    home_joint_tolerance: float   # 每关节 ±rad 容差，给 IK 留裕量

    # MoveIt 笛卡尔目标的固定姿态与规划器参数
    tip_orientation_wxyz: tuple[float, float, float, float]
    default_velocity_scaling: float
    num_planning_attempts: int
    allowed_planning_time: float
    replan_attempts: int
    position_tolerance: float     # PositionConstraint 球半径 (m)
    orientation_tolerance: float  # OrientationConstraint 三轴绝对容差

    # 感知可达性保护（perception 用）
    reachable_x_min: float        # 反投影 x 下限（base frame, m）
    sanity_box: tuple[float, float, float, float, float, float]  # x,y,z 各 [min,max]


@dataclass(frozen=True)
class ObjectConfig:
    """抓取目标物体（每场景一份）。"""

    object_id: str          # MuJoCo free-joint 名 / planning-scene collision id
    radius: float           # 圆柱半径 (m)
    height: float           # 圆柱高 (m)
    place_position: tuple[float, float, float]  # 放置点中心（base frame, m）


@dataclass(frozen=True)
class GraspConfig:
    """物理抓取算法的通用调参（跨机器人通常可复用）。

    contact_qpos_min/max 由 build_grasp_config(radius) 从物体半径派生：
    「碰住未穿透」窗口 = [radius-0.003, radius]。
    """

    coarse_step: float             # 两段式第一阶段步长 (m)
    fine_step: float               # 接触后精调步长 (m)
    min_qpos: float                # 收紧安全下限（防指尖穿透碰撞体）(m)
    contact_qpos_min: float        # 「碰住未穿透」判据下界
    contact_qpos_max: float        # 判据上界（= 物体半径）
    displacement_tolerance: float  # 收紧过程中物体位移上限 (m)

    micro_lift_height: float       # 微抬高度 (m)
    micro_lift_z_min: float        # 物体 z 需跟着抬升的最小量 → 物理抓牢
    micro_lift_lateral_max: float  # 微抬侧向漂移上限 (m)
    micro_lift_velocity_scaling: float

    max_grasp_attempts: int        # 1 次初始 + N-1 次重试
    stable_samples: int            # 双信号连续达标采样次数（防抖动）
    stable_interval: float         # 采样间隔 (s)
    step_settle: float             # 每步收紧后稳定等待 (s)

    hover_offset: float            # 悬停高度 = grasp_z + offset (m)
    lift_z_min: float              # 完整抬起的目标 z 下限 (m)
    post_move_settle: float        # 每次 MoveIt 成功后的落定等待 (s)
    release_settle: float          # 松爪后让物体落定的等待 (s)
    recover_settle: float          # 抓取恢复后落定等待 (s)

    close_max_effort: float        # 收紧力矩上限
    open_max_effort: float         # 张开力矩上限（0 = 不受限速求合拢）


@dataclass(frozen=True)
class CameraConfig:
    """感知相机内参（Pinhole；CameraPlugin 输出须与此一致）。"""

    frame_id: str
    fx: float
    fy: float
    cx: float
    cy: float
    assumed_depth: float           # 无深度图时的假设距离 (m)


@dataclass(frozen=True)
class DetectorConfig:
    """颜色分割主检测器（特征色阈值随目标物体材质/场景光照而定）。"""

    name: str          # 检测结果回填的名称（感知层对外语义）
    mask_r_min: int    # R 通道下限（覆盖暗红主体 + 高光）
    mask_r_max: int    # R 通道上限（排除过曝像素）
    mask_g_max: int    # G 严格压死 → 排除棕色桌面 / 橙色机械臂
    mask_b_max: int
    min_pixels: int    # 少于该像素数视为未检测到


@dataclass(frozen=True)
class VlmConfig:
    """VLM 兜底检测器（OpenAI 兼容接口）。"""

    base_url: str
    model: str
    prompt_template: str   # {target} 占位符
    max_retries: int


@dataclass(frozen=True)
class PickPlaceConfig:
    """一个 demo 的完整配置（demo 入口构造并注入）。"""

    arm: ArmConfig
    object: ObjectConfig
    grasp: GraspConfig
    camera: CameraConfig
    detector: DetectorConfig | None = None   # 感知节点用；纯规划 demo 可省
    vlm: VlmConfig | None = None             # 颜色分割失败后的兜底


@dataclass(frozen=True)
class TaskCommand:
    """/robot_command 的结构化形式（parse_task_command 的产物）。

    destination/constraints 为 LLM schema 字段，解析保留但下游暂不消费；
    action 目前仅支持 "pick"。
    """

    target_object: str
    action: str
    position: tuple[float, float, float] | None   # base_frame 下的目标中心 (m)
    destination: str | None = None
    constraints: tuple[str, ...] = ()

    @property
    def supported(self) -> bool:
        """与旧行为一致：非 pick 或缺 position → warn 留 IDLE（不抛异常）。"""
        return self.action == "pick" and self.position is not None
