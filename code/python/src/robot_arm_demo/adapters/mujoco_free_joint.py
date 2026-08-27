"""ObjectPoseSource 的 MuJoCo FreeJointStateArray 实现。

复刻原 coke_pose_callback：按 body 名缓存 free joint 的 world 位姿。
MuJoCo 场景中 panda_link0 即世界原点（world == base frame），
坐标可直接用于抓取微调。
"""

from __future__ import annotations

import threading

from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rclpy.node import Node


class MujocoFreeJointPoseSource:
    """订阅 FreeJointStateArray，缓存第一个匹配 object_id 的 body 位姿。"""

    def __init__(self, node: Node, object_id: str,
                 topic: str = "/free_joint_state_publisher/free_joint_states"):
        self._object_id = object_id
        self._lock = threading.Lock()
        self._latest_pose = None
        self._sub = node.create_subscription(
            FreeJointStateArray, topic, self._callback, 10
        )

    def _callback(self, msg):
        """缓存目标物体 free joint 的 world 位姿（MuJoCo ground truth）。"""
        for fj in msg.free_joints:
            if fj.name == self._object_id:
                with self._lock:
                    self._latest_pose = (
                        fj.pose.pose.position.x,
                        fj.pose.pose.position.y,
                        fj.pose.pose.position.z,
                    )
                return

    def get_object_pose(self, object_id: str):
        """返回物体中心 world 位姿；尚未收到消息返回 None。"""
        # 幂等校验：控制器传入的 id 应与构造时一致（当前架构单物体，防御即可）
        assert object_id == self._object_id, "object_id 与构造时不一致"
        with self._lock:
            return self._latest_pose
