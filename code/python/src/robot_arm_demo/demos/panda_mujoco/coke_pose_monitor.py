#!/usr/bin/env python3
"""可乐位姿观测器（事件驱动，调试工具，不进主流程）。

订阅 /free_joint_state_publisher/free_joint_states（可乐 world 位姿，MuJoCo ground truth）
+ /joint_states（手指 qpos），只在关键事件时打印一行：

  可乐位移 > 1cm → COKE_MOVED 事件
  手指 f1 变化 > 3mm → FINGER 事件

用于精确定位「可乐被推走」发生在 DESCEND 还是 CLOSE_GRIPPER：
对比 COKE_MOVED 与 FINGER（闭合起点）的先后顺序。

输出格式（空格分隔）：
  wallclock_ts  coke_x  coke_y  coke_z  f1  event...

建议运行时重定向到文件：`... > /tmp/coke_monitor.log 2>&1`。
"""

import time
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

COKE_MOVE_THRESHOLD = 0.01     # 可乐位移告警阈值（米）
FINGER_DELTA_THRESHOLD = 0.003  # 手指 qpos 变化阈值（米）


class CokePoseMonitor(Node):
    def __init__(self):
        super().__init__("coke_pose_monitor")
        self.latest_coke = None
        self.latest_joint = None
        self.lock = threading.Lock()

        self.create_subscription(
            FreeJointStateArray,
            "/free_joint_state_publisher/free_joint_states",
            self.coke_cb,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        self.prev_coke = None
        self.prev_f1 = None
        self.timer = self.create_timer(0.05, self.tick)  # 20Hz 采样
        print("wallclock_ts coke_x coke_y coke_z f1 event", flush=True)

    def coke_cb(self, msg):
        for fj in msg.free_joints:
            if fj.name == "coke":
                with self.lock:
                    self.latest_coke = fj
                return

    def joint_cb(self, msg):
        with self.lock:
            self.latest_joint = msg

    def _read_f1(self, joint_msg):
        for name, pos in zip(joint_msg.name, joint_msg.position):
            if name == "panda_finger_joint1":
                return float(pos)
        return None

    def tick(self):
        with self.lock:
            coke = self.latest_coke
            joint = self.latest_joint
        if coke is None:
            return

        x = coke.pose.pose.position.x
        y = coke.pose.pose.position.y
        z = coke.pose.pose.position.z
        f1 = self._read_f1(joint) if joint else None

        events = []
        if self.prev_coke is not None:
            dx = x - self.prev_coke[0]
            dy = y - self.prev_coke[1]
            dz = z - self.prev_coke[2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist > COKE_MOVE_THRESHOLD:
                events.append(
                    f"COKE_MOVED {dist * 100:.1f}cm "
                    f"dX={dx * 100:.1f} dY={dy * 100:.1f} dZ={dz * 100:.1f}"
                )
        if self.prev_f1 is not None and f1 is not None:
            if abs(f1 - self.prev_f1) > FINGER_DELTA_THRESHOLD:
                events.append(f"FINGER {self.prev_f1:.4f}->{f1:.4f}")

        if events:
            ts = time.time()
            f1_str = f"{f1:.4f}" if f1 is not None else "-1.0000"
            print(f"{ts:.3f} {x:.4f} {y:.4f} {z:.4f} {f1_str} {' '.join(events)}", flush=True)

        self.prev_coke = (x, y, z)
        if f1 is not None:
            self.prev_f1 = f1


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = CokePoseMonitor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor.shutdown()


if __name__ == "__main__":
    main()
