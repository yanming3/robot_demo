"""core 与具体后端之间的边界 —— 全部用 typing.Protocol 结构化约束。

每个 Protocol 方法一一对应原 pick_place_state_machine/perception_node 里
的一个 action-client/service/subscription 调用；语义契约写在 docstring，
由 adapters/ 内的具体类逐字段复刻实现。core 与 pytest 不 import 本模块
之外的任何 ROS 代码。
"""

from __future__ import annotations

from typing import Mapping, Protocol, runtime_checkable


@runtime_checkable
class Logger(Protocol):
    """最小日志面；rclpy logger 由适配层包装。"""

    def info(self, msg: str) -> None: ...
    def warn(self, msg: str) -> None: ...
    def error(self, msg: str) -> None: ...


@runtime_checkable
class ArmController(Protocol):
    """机械臂运动 + Planning Scene 编辑（MoveGroup 实现见 adapters.moveit_arm）。"""

    def move_cartesian(
        self,
        position,
        orientation_wxyz=None,
        velocity_scaling=None,
    ) -> bool:
        """笛卡尔位姿目标：eef_link 到 position（球形容差）+ 姿态约束。

        orientation_wxyz 为 None 时用配置的 tip_orientation_wxyz；
        velocity_scaling 为 None 时用配置默认值。成功后做 post_move_settle。
        返回 MoveIt error_code==1。
        """
        ...

    def move_to_joint(self, joint_values, velocity_scaling=None) -> bool:
        """按关节空间约束移动到 joint_values（顺序与 home_joint_names 一致）。

        关节容差取配置 home_joint_tolerance。成功后做 post_move_settle。
        """
        ...

    def remove_object_from_scene(self, object_id: str) -> bool:
        """从 Planning Scene 移除 collision object（不存在时无害返回）。"""
        ...


@runtime_checkable
class Gripper(Protocol):
    """夹爪命令 + 真实手指位置读取（GripperCommand 实现，见 adapters.gripper_action）。"""

    def move_to(self, position: float, max_effort: float | None = None) -> bool:
        """发送 GripperCommand goal，返回 result.reached_goal。"""
        ...

    def read_finger_positions(self) -> "dict[str, float] | None":
        """从缓存 /joint_states 读各手指真实位置；键为 gripper_joint_names。

        尚未收到消息返回 None；消息中缺位置回退 NaN（原行为保留）。
        """
        ...


@runtime_checkable
class ObjectPoseSource(Protocol):
    """物体 world 位姿来源（MuJoCo free joint 实现，见 adapters.mujoco_free_joint）。"""

    def get_object_pose(self, object_id: str) -> "tuple[float, float, float] | None":
        """返回物体中心 world 位姿；尚未收到返回 None。"""
        ...


@runtime_checkable
class ObjectDetector(Protocol):
    """2D 检测器。返回与旧节点相同的 dict 契约：
    {"name": str, "bbox": [x_min,y_min,x_max,y_max], "center": (u,v)|None}
    检测失败返回 None。
    """

    def detect(self, target_name: str, img) -> dict | None: ...


@runtime_checkable
class TfTransform(Protocol):
    """坐标变换（tf2 实现，感知节点专用）。"""

    def transform_point(
        self, source_frame: str, point_xyz, target_frame: str
    ) -> "tuple[float, float, float] | None":
        """点变换失败或 tf 不可用返回 None。"""
        ...
