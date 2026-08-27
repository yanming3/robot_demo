"""Logger 协议的 rclpy 实现 + tf2 变换实现（感知节点用）。"""

from __future__ import annotations

import rclpy.duration
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class RclLogger:
    """把 node.get_logger() 适配成 core.interfaces.Logger。"""

    def __init__(self, ros_logger):
        self._log = ros_logger

    def info(self, msg: str) -> None:
        self._log.info(msg)

    def warn(self, msg: str) -> None:
        self._log.warn(msg)

    def error(self, msg: str) -> None:
        self._log.error(msg)


class Tf2PointTransform:
    """tf2 Buffer 封装：PointStamped → target_frame 的 (x, y, z)。

    与原实现一致：stamp 填宿主节点当前时钟（在该时刻做 tf 查找，
    timeout 2s 等待变换可用）。
    """

    def __init__(self, node: Node, buffer, logger):
        self.node = node
        self.buffer = buffer
        self.log = logger

    def transform_point(self, source_frame: str, point_xyz, target_frame: str):
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = self.node.get_clock().now().to_msg()
        point.point.x = float(point_xyz[0])
        point.point.y = float(point_xyz[1])
        point.point.z = float(point_xyz[2])
        try:
            out = self.buffer.transform(
                point, target_frame,
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            return out.point.x, out.point.y, out.point.z
        except Exception as e:
            self.log.error(f"tf2 transform failed: {e}")
            return None
