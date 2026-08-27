#!/usr/bin/env python3
"""感知节点（thin 入口）。

订阅 /camera (RGB) → 收到 LLM 指令后 颜色分割(主)/VLM(兜底) 检测目标物体
→ bbox 中心 (+假设深度) 内参反投影 → tf2 转换 camera frame → base frame
→ 发布 /robot_command (JSON, 含目标 3D 位置)。

算法与 ROS 管线在 core/adapters；本文件只做装配与编排。
环境变量:
    DASHSCOPE_API_KEY  Qwen API 密钥（必需）
"""

import json
import math
import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from openai import OpenAI

from robot_arm_demo.core.camera import backproject_pinhole
from robot_arm_demo.core.command import parse_task_command
from robot_arm_demo.adapters.detectors import ColorDetector, QwenVlDetector
from robot_arm_demo.adapters.logger import RclLogger, Tf2PointTransform
from robot_arm_demo.adapters.mujoco_free_joint import MujocoFreeJointPoseSource
from robot_arm_demo.demos.panda_mujoco.config import build_panda_mujoco_config


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.cfg = build_panda_mujoco_config()
        self.log = RclLogger(self.get_logger())

        api_key = os.environ.get("DASHSCOPE_API_KEY")
        if not api_key:
            self.get_logger().error("DASHSCOPE_API_KEY not set, exiting.")
            raise SystemExit(1)
        client = OpenAI(api_key=api_key, base_url=self.cfg.vlm.base_url)

        # 检测器：颜色分割优先（快、准、确定性），VLM 兜底
        self.color_detector = ColorDetector(self.cfg.detector, self.log)
        self.vlm_detector = QwenVlDetector(self.cfg.vlm, client, self.log)

        # 目标物体 ground truth 位姿源（world == base frame），用于偏差对比
        self.pose_source = MujocoFreeJointPoseSource(
            self, self.cfg.object.object_id
        )

        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_sub = self.create_subscription(
            Image, "/camera", self.image_callback, 10
        )
        # NOTE 2026-08-15: 深度图与 tf/RGB 帧存在时序错位，反投影坐标漂移
        # （Y 0.001→0.032、Z 0.061→0.098），导致夹爪 DESCEND 撞倒可乐。
        # 已停用深度订阅，回退 camera.assumed_depth（固定场景标定准确）。

        self.command_sub = self.create_subscription(
            String, "/llm_command", self.command_callback, 10
        )
        self.robot_command_pub = self.create_publisher(String, "/robot_command", 10)

        # tf2
        self.tf_buffer = None
        self.transformer = None
        try:
            import tf2_ros
            import tf2_geometry_msgs  # noqa: F401 注册 PointStamped
            self.tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf_buffer, self)
            self.transformer = Tf2PointTransform(self, self.tf_buffer, self.log)
        except ImportError:
            self.get_logger().warn("tf2_ros not available, coordinate transform disabled.")

        self.get_logger().info("Perception node ready. Waiting for commands on /llm_command ...")

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image = msg

    def _get_image(self):
        """读最新 RGB 帧转 PIL，保存调试图到 /tmp，返回 img 或 None。"""
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().error("No image available.")
                return None
            image_msg = self.latest_image
        from PIL import Image as PILImage
        img = PILImage.frombytes(
            "RGB", (image_msg.width, image_msg.height), bytes(image_msg.data)
        )
        debug_path = "/tmp/perception_latest.jpg"
        img.save(debug_path, format="JPEG")
        self.get_logger().info(f"Saved debug image to {debug_path}")
        return img

    def _read_assumed_depth(self):
        """真实深度路径已停用（时序错位致坐标漂移），恒回退假设深度。"""
        return self.cfg.camera.assumed_depth

    def _log_gt_comparison(self, position):
        """打印检测结果 vs ground truth 的对比与偏差。"""
        gt = self.pose_source.get_object_pose(self.cfg.object.object_id)
        if gt is None:
            self.log.warn("[GT-VLM] 未收到可乐 ground truth（free_joint_states），跳过对比")
            return
        dx = position[0] - gt[0]
        dy = position[1] - gt[1]
        dz = position[2] - gt[2]
        dist = math.hypot(math.hypot(dx, dy), dz)
        self.log.info(
            f"[GT-VLM] VLM=({position[0]:.4f}, {position[1]:.4f}, "
            f"{position[2]:.4f})  GT=({gt[0]:.4f}, {gt[1]:.4f}, {gt[2]:.4f})  "
            f"err=({dx:.4f}, {dy:.4f}, {dz:.4f})  dist={dist:.4f}m"
        )

    def _backproject(self, bbox, center=None):
        """2D bbox(+质心) → 相机坐标系 3D 点 (X,Y,Z)。"""
        cam = self.cfg.camera
        if center is not None:
            u, v = center
        else:
            x_min, y_min, x_max, y_max = bbox
            u = (x_min + x_max) / 2.0
            v = (y_min + y_max) / 2.0
        self.log.info(f"bbox center: u={u:.1f}, v={v:.1f}")

        Z = self._read_assumed_depth()
        if Z is None or Z <= 0.0:
            self.log.warn(
                f"No valid depth at ({u:.1f},{v:.1f}), fallback to assumed_depth={cam.assumed_depth}"
            )
            Z = cam.assumed_depth
        else:
            self.log.info(f"Measured depth at ({u:.1f},{v:.1f}): Z={Z:.4f}")

        X, Y, Z = backproject_pinhole(u, v, Z, cam)
        self.log.info(f"3D point in camera_link: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        return X, Y, Z

    def command_callback(self, msg):
        """LLM 指令 → 检测 → 反投影 → tf2 → 发布 /robot_command。"""
        task = parse_task_command(msg.data)
        if task is None:
            self.log.error(f"Invalid JSON: {msg.data}")
            return
        target = task.target_object
        self.log.info(f"Received LLM command: target={target}, action={task.action}")
        if task.action != "pick":
            self.log.warn(f"Unsupported action: {task.action}")
            return

        # 1. 读帧
        img = self._get_image()
        if img is None:
            return

        # 2. 检测：颜色分割优先，失败再 VLM 兜底
        detection = self.color_detector.detect(target, img)
        if detection is None:
            self.log.warn("Color detection failed, falling back to VLM.")
            detection = self.vlm_detector.detect(target, img)
        if detection is None:
            self.log.error(f"Failed to detect {target} (color + VLM).")
            return
        bbox = detection.get("bbox")
        if not bbox or len(bbox) != 4:
            self.log.error(f"Invalid bbox: {bbox}")
            return

        # 3. 2D → 3D 反投影（相机坐标系）
        point_camera = self._backproject(bbox, center=detection.get("center"))

        # 4. tf2 坐标转换到 base frame
        if self.transformer is None:
            self.log.error("tf2 not available.")
            return
        xyz = self.transformer.transform_point(
            self.cfg.camera.frame_id, point_camera, self.cfg.arm.base_frame
        )
        if xyz is None:
            self.log.error("Coordinate transform failed.")
            return
        x, y, z = xyz

        # 可达性保护：base 下 X 过近不可达（Z 不 clamp——状态机负责指尖偏移）
        if x < self.cfg.arm.reachable_x_min:
            self.log.warn(
                f"X={x:.3f} too close, clamping to {self.cfg.arm.reachable_x_min}"
            )
            x = self.cfg.arm.reachable_x_min
        self.log.info(f"3D point in {self.cfg.arm.base_frame}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        # 5. 合理性校验：目标应在桌面上方合理范围（挡住 VLM 幻觉）
        x_min, x_max, y_min, y_max, z_min, z_max = self.cfg.arm.sanity_box
        if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
            self.log.error(
                f"Detected position out of table range: ({x:.3f},{y:.3f},{z:.3f}), rejected."
            )
            return

        # 6. 检测 vs ground truth 对比（发现识别偏差）
        self._log_gt_comparison([x, y, z])

        # 7. 发布带 3D 位置的指令
        robot_cmd = {
            "target_object": target,
            "action": task.action,
            "position": [x, y, z],
        }
        out_msg = String()
        out_msg.data = json.dumps(robot_cmd, ensure_ascii=False)
        self.robot_command_pub.publish(out_msg)
        self.get_logger().info(f"Published /robot_command: {out_msg.data}")


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
