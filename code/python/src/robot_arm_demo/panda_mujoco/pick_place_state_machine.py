#!/usr/bin/env python3
"""Panda pick-place 状态机节点（thin 入口）。

FSM 全部逻辑在 core.pick_place.PickPlaceController（机器人无关）；
本文件只做装配：创建 rclpy Node → 注入 adapters + config → 订阅
/robot_command 并把 JSON 交给 controller.handle_command。
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_arm_demo.core.command import parse_task_command
from robot_arm_demo.core.pick_place import PickPlaceController
from robot_arm_demo.adapters.logger import RclLogger
from robot_arm_demo.demos.panda_mujoco.config import (
    build_adapters,
    build_panda_mujoco_config,
)


class PickPlaceStateMachineNode(Node):
    def __init__(self, config=None):
        super().__init__("pick_place_state_machine")
        self.cfg = config or build_panda_mujoco_config()

        # 装配协议实现（构造即 wait_for_server，与原 __init__ 时序一致）
        bundle = build_adapters(self, self.cfg)
        self.controller = PickPlaceController(
            arm=bundle.arm,
            gripper=bundle.gripper,
            pose_source=bundle.pose_source,
            config=self.cfg,
            logger=RclLogger(self.get_logger()),
        )
        self.get_logger().info("Action servers connected.")

        self.command_sub = self.create_subscription(
            String, "/robot_command", self.command_callback, 10
        )
        self.get_logger().info("Waiting for commands on /robot_command ...")

    def command_callback(self, msg):
        """收到感知节点发布的 JSON 指令，解析后交给控制器执行。"""
        task = parse_task_command(msg.data)
        if task is None:
            self.get_logger().error(f"Invalid JSON: {msg.data}")
            return
        self.get_logger().info(
            f"Received command: target={task.target_object}, action={task.action}"
        )
        self.controller.handle_command(task)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = PickPlaceStateMachineNode()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info("Pick-place state machine ready. Waiting for commands...")
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
