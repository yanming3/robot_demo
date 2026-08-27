"""Gripper 接口的 GripperCommand 实现。

复刻原 send_gripper_goal（返回 reached_goal）与 /joint_states 手指读取
（缺位置回退 NaN，见 M0 教训：reached_goal ≠ 物理夹持成功的证据）。
"""

from __future__ import annotations

import threading

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..core.data import ArmConfig, GraspConfig
from .moveit_arm import _wait_future


class GripperAction:
    """在宿主 Node 上创建 gripper action 客户端并订阅 /joint_states。"""

    def __init__(self, node: Node, arm_cfg: ArmConfig, grasp_cfg: GraspConfig):
        self.node = node
        self.arm = arm_cfg
        self.grasp = grasp_cfg
        self._log = node.get_logger()

        self._client = ActionClient(node, GripperCommand, arm_cfg.gripper_command_topic)
        self._client.wait_for_server()

        # 缓存最新 /joint_states 供闭合后读取真实手指位置（M0）
        self._latest_joint_state = None
        self._joint_state_lock = threading.Lock()
        self._joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg):
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def move_to(self, position: float, max_effort: float | None = None) -> bool:
        if max_effort is None:
            max_effort = self.grasp.close_max_effort
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self._log.info(f"Sending gripper goal: position={position}")
        future = self._client.send_goal_async(goal)
        if not _wait_future(self.node, future, timeout=10.0):
            self._log.error("Gripper goal send timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._log.error("Gripper goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not _wait_future(self.node, result_future, timeout=10.0):
            self._log.error("Gripper goal timed out")
            return False
        result = result_future.result()
        if result is None:
            self._log.error("Gripper result None")
            return False
        self._log.info(
            f"Gripper result: position={result.result.position:.4f}, "
            f"reached={result.result.reached_goal}"
        )
        return result.result.reached_goal

    def read_finger_positions(self):
        """读 /joint_states 里配置的手指关节真实位置；未收到消息返回 None。"""
        with self._joint_state_lock:
            if self._latest_joint_state is None:
                return None
            names = list(self._latest_joint_state.name)
            positions = list(self._latest_joint_state.position)
        fingers = {}
        target_names = self.arm.gripper_joint_names
        for i, name in enumerate(names):
            if name in target_names:
                fingers[name] = positions[i] if i < len(positions) else float("nan")
        return fingers
