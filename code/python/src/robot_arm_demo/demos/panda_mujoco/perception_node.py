#!/usr/bin/env python3
"""感知节点（thin 入口）—— 云端位姿链路线。

订阅 /camera(RGB) + /camera/depth + /camera/camera_info(内参)，
收到 /llm_command 后：
  ① 采 RGBD + 真机内参（优先 camera_info，回退 config 默认值）
  ② POST 云端 /v1/estimate_pose（Grounding-DINO+SAM2 → 位姿；Stage 3 接 FoundationPose）
     云端不可用时回退本地几何（颜色 mask + 深度点云），不再使用 assumed_depth
  ③ tf2 变换「完整位姿」camera 系 → base 系（PoseStamped，位置+四元数）
  ④ 发布 /robot_command { position, orientation(xyzw) }（base 系）

算法与 wire 契约在 perception_service/，ROS 装配在本文件。
环境变量:
    POSE_SERVICE_URL   云端服务地址（config 已读，默认 http://127.0.0.1:8000）
"""

import json
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from robot_arm_demo.adapters.logger import RclLogger, Tf2PoseTransform
from robot_arm_demo.adapters.mujoco_free_joint import MujocoFreeJointPoseSource
from robot_arm_demo.core.command import parse_task_command
from robot_arm_demo.demos.panda_mujoco.config import build_panda_mujoco_config
from robot_arm_demo.perception_service import contract, geometry_pose
from robot_arm_demo.perception_service.cloud_client import PoseServiceClient


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.cfg = build_panda_mujoco_config()
        self.log = RclLogger(self.get_logger())

        # 云端位姿服务（Stage 1 mock / Stage 3 FoundationPose，同一契约）
        if self.cfg.service is None:
            self.get_logger().error("No pose service configured, exiting.")
            raise SystemExit(1)
        self.service = PoseServiceClient(self.cfg.service, self.log)

        # 目标物体 ground truth 位姿源（world == base frame），仅用于偏差对比
        self.pose_source = MujocoFreeJointPoseSource(
            self, self.cfg.object.object_id
        )

        self.lock = threading.Lock()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.create_subscription(
            Image, self.cfg.camera.rgb_topic, self._rgb_cb, 10
        )
        self.create_subscription(
            Image, self.cfg.camera.depth_topic, self._depth_cb, 10
        )
        self.create_subscription(
            CameraInfo, self.cfg.camera.camera_info_topic, self._camera_info_cb, 10
        )

        self.command_sub = self.create_subscription(
            String, "/llm_command", self.command_callback, 10
        )
        self.robot_command_pub = self.create_publisher(String, "/robot_command", 10)

        # tf2（PoseStamped 完整位姿变换）
        self.tf_pose = None
        try:
            import tf2_ros
            import tf2_geometry_msgs  # noqa: F401 注册 PoseStamped
            self.tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf_buffer, self)
            self.tf_pose = Tf2PoseTransform(self, self.tf_buffer, self.log)
        except ImportError:
            self.get_logger().warn("tf2_ros not available, coordinate transform disabled.")

        self.get_logger().info(
            f"Perception node ready (service={self.cfg.service.base_url}). "
            "Waiting for commands on /llm_command ..."
        )

    # ── 订阅回调 ──

    def _rgb_cb(self, msg):
        with self.lock:
            self.latest_rgb = msg

    def _depth_cb(self, msg):
        with self.lock:
            self.latest_depth = msg

    def _camera_info_cb(self, msg):
        with self.lock:
            self.latest_camera_info = msg

    # ── 帧/内参读取 ──

    def _read_frame(self):
        """返回 (rgb(u8 HxWx3), depth(f32 HxW|None), intrinsics{...}, frame_id)。"""
        with self.lock:
            rgb_msg = self.latest_rgb
            depth_msg = self.latest_depth
            info = self.latest_camera_info
        if rgb_msg is None:
            self.log.error("No RGB image available.")
            return None
        self._save_debug(rgb_msg)
        rgb = self._msg_to_rgb(rgb_msg)
        if rgb is None:
            self.log.error("Unsupported RGB encoding.")
            return None
        depth = self._msg_to_depth(depth_msg) if depth_msg is not None else None
        intrinsics, frame_id = self._intrinsics(info, rgb_msg.width, rgb_msg.height)
        return rgb, depth, intrinsics, frame_id

    def _save_debug(self, rgb_msg):
        from PIL import Image as PILImage
        try:
            arr = self._msg_to_rgb(rgb_msg)
            PILImage.fromarray(arr).save("/tmp/perception_latest.jpg", format="JPEG")
            # self.get_logger().info("Saved debug image to /tmp/perception_latest.jpg")
        except Exception as e:  # noqa: BLE001
            self.log.warn(f"Failed to save debug image: {e}")

    def _msg_to_rgb(self, msg):
        h, w = msg.height, msg.width
        import numpy as np
        if msg.encoding == "rgb8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3).copy()
        if msg.encoding == "bgr8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3).copy()
            return arr[:, :, ::-1]
        return None

    def _msg_to_depth(self, msg):
        import numpy as np
        h, w = msg.height, msg.width
        if msg.encoding == "32FC1":
            arr = np.frombuffer(msg.data, dtype="<f4").reshape(h, w).copy()
            arr[~np.isfinite(arr)] = np.nan
            return arr
        if msg.encoding == "16UC1":  # mm
            arr = np.frombuffer(msg.data, dtype="<u2").reshape(h, w).astype(np.float32)
            arr = arr / 1000.0
            arr[~np.isfinite(arr)] = np.nan
            return arr
        self.log.warn(f"Unsupported depth encoding: {msg.encoding}")
        return None

    def _intrinsics(self, info, width, height):
        """优先真机 camera_info，回退 config 默认值（sim）。"""
        cam = self.cfg.camera
        if info is not None and info.k and len(info.k) >= 9:
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            dist = list(info.d or [])
            self.log.info(
                f"Using camera_info intrinsics: fx={fx:.2f} fy={fy:.2f} "
                f"cx={cx:.2f} cy={cy:.2f}"
            )
            frame_id = info.header.frame_id or cam.frame_id
            return (
                contract.build_intrinsics(fx, fy, cx, cy, width, height, dist),
                frame_id,
            )
        self.log.warn(
            "camera_info unavailable, falling back to config default intrinsics "
            f"(fx={cam.fx}, fy={cam.fy}, cx={cam.cx}, cy={cam.cy})."
        )
        return (
            contract.build_intrinsics(
                cam.fx, cam.fy, cam.cx, cam.cy, width, height, None
            ),
            cam.frame_id,
        )

    def _log_gt_comparison(self, position):
        """打印检测结果 vs ground truth（MuJoCo）的对比与偏差。"""
        gt = self.pose_source.get_object_pose(self.cfg.object.object_id)
        if gt is None:
            self.log.warn("[GT] 未收到可乐 ground truth（free_joint_states），跳过对比")
            return
        import math
        dx = position[0] - gt[0]
        dy = position[1] - gt[1]
        dz = position[2] - gt[2]
        dist = math.hypot(math.hypot(dx, dy), dz)
        self.log.info(
            f"[GT] est=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})  "
            f"GT=({gt[0]:.4f}, {gt[1]:.4f}, {gt[2]:.4f})  "
            f"err=({dx:.4f}, {dy:.4f}, {dz:.4f})  dist={dist:.4f}m"
        )

    # ── 指令处理 ──

    def command_callback(self, msg):
        """LLM 指令 → 云/本地位姿 → tf2 → 发布 /robot_command。"""
        task = parse_task_command(msg.data)
        if task is None:
            self.log.error(f"Invalid JSON: {msg.data}")
            return
        target = task.target_object
        self.log.info(f"Received LLM command: target={target}, action={task.action}")
        if task.action != "pick":
            self.log.warn(f"Unsupported action: {task.action}")
            return

        frame = self._read_frame()
        if frame is None:
            return
        rgb, depth, intrinsics, frame_id = frame

        # 1. 云端位姿（主路径）
        prompt = self.cfg.service.prompt_template.format(object=target)
        req = contract.build_request(
            rgb, depth, intrinsics, prompt, target, frame_id,
            return_mask=self.cfg.service.return_mask,
        )
        self.log.info(f"POST /v1/estimate_pose ...")
        res = self.service.estimate(req)
        position_cam = res.get("position")
        orient_xyzw = res.get("orientation_xyzw")

        # 2. 本地几何兜底：颜色 mask + 深度点云（不再用 assumed_depth）
        if position_cam is None or orient_xyzw is None:
            self.log.warn("Cloud pose failed, falling back to local geometry.")
            if self.cfg.detector is None or depth is None:
                self.log.error("No local fallback (detector/depth) available.")
                return
            mask = geometry_pose.cola_color_mask(rgb, self.cfg.detector)
            res = geometry_pose.pose_from_mask(rgb, depth, intrinsics, mask)
            position_cam = res.get("position")
            orient_xyzw = res.get("orientation_xyzw")
        if position_cam is None or orient_xyzw is None:
            self.log.error("Failed to estimate 6D pose (cloud + local geometry).")
            return

        # 3. tf2 完整位姿 camera → base
        if self.tf_pose is None:
            self.log.error("tf2 not available.")
            return
        out = self.tf_pose.transform_pose(
            frame_id, position_cam, orient_xyzw, self.cfg.arm.base_frame
        )
        if out is None:
            self.log.error("Coordinate transform failed.")
            return
        x, y, z, qx, qy, qz, qw = out

        # 4. 可达性保护：base 下 X 过近不可达
        if x < self.cfg.arm.reachable_x_min:
            self.log.warn(
                f"X={x:.3f} too close, clamping to {self.cfg.arm.reachable_x_min}"
            )
            x = self.cfg.arm.reachable_x_min
        self.log.info(
            f"Base pose: pos=({x:.4f}, {y:.4f}, {z:.4f}) "
            f"quat(xyzw)=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})"
        )

        # 5. 合理性校验：目标应在桌面上方合理范围
        x_min, x_max, y_min, y_max, z_min, z_max = self.cfg.arm.sanity_box
        if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
            self.log.error(
                f"Detected pose out of table range: ({x:.3f},{y:.3f},{z:.3f}), rejected."
            )
            return

        # 6. 偏差对比（可选）
        self._log_gt_comparison([x, y, z])

        # 7. 发布带位置+姿态的指令
        robot_cmd = {
            "target_object": target,
            "action": task.action,
            "position": [x, y, z],
            "orientation": [qx, qy, qz, qw],
        }
        out_msg = String()
        out_msg.data = json.dumps(robot_cmd, ensure_ascii=False)
        self.robot_command_pub.publish(out_msg)
        self.get_logger().info(f"Published /robot_command: {out_msg.data}")


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = PerceptionNode()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info("Perception node spinning on a separate thread.")
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
