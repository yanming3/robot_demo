#!/usr/bin/env python3
"""V5-T005: 感知节点。

订阅 /camera (RGB) → 收到 LLM 指令后调 VLM 检测目标物体
→ bbox 中心 + 假设深度 + 内参反投影 → tf2 转换 camera_link → panda_link0
→ 发布 /robot_command (JSON, 含目标 3D 位置)

环境变量:
    DASHSCOPE_API_KEY  Qwen API 密钥（必需）
"""

import base64
import json
import math
import os
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from openai import OpenAI
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

# 相机内参（MuJoCo 相机 fovy 已按 fx=554 校准，见 scene.xml）
# fx = (width/2) / tan(fov/2), fov=1.047 rad (60°)
CAMERA_FX = 554.0
CAMERA_FY = 554.0
CAMERA_CX = 320.0
CAMERA_CY = 240.0

# demo 阶段假设深度（camera 坐标系 Z 方向距离）
# MuJoCo 里可乐中心 (0.3,0,0.061), 相机 (0.4,0.5,0.625)（panda_link0 为原点），
# 距离约 0.76m。相机 fovy 已按 fx=554 校准，反投影恢复的可乐深度实测 0.7603。
ASSUMED_DEPTH = 0.76

DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
VLM_MODEL = "qwen-vl-max"

VLM_PROMPT_TEMPLATE = """图片尺寸是 640x480 像素。这是机器人仿真相机俯视拍摄桌面的图片。
请仔细识别图中的"{target}"（红色罐装饮料），返回它的 bounding box。
格式：{{"objects": [{{"name": "{target}", "bbox": [x_min, y_min, x_max, y_max]}}]}}
坐标必须在 0-640 (x) 和 0-480 (y) 范围内。只返回 JSON。如果看不到，返回 {{"objects": []}}。"""

# VLM 重试次数
VLM_MAX_RETRIES = 3

class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        api_key = os.environ.get("DASHSCOPE_API_KEY")
        if not api_key:
            self.get_logger().error("DASHSCOPE_API_KEY not set, exiting.")
            raise SystemExit(1)
        self.client = OpenAI(api_key=api_key, base_url=DASHSCOPE_BASE_URL)

        self.latest_image = None
        self.image_lock = threading.Lock()

        self.image_sub = self.create_subscription(
            Image, "/camera", self.image_callback, 10
        )
        # 订阅深度图（32FC1, 640x480），用于可乐位姿的真实深度反投影，
        # 替代 ASSUMED_DEPTH=0.76 的假设深度。
        # NOTE 2026-08-15: 深度图与 tf/RGB 帧存在时序错位，反投影坐标漂移
        # （Y 0.001→0.032、Z 0.061→0.098），导致夹爪在 DESCEND 阶段撞倒可乐。
        # 已停用深度订阅，回退 ASSUMED_DEPTH（固定场景下标定准确）。
        self.latest_depth = None
        self.depth_lock = threading.Lock()
        # self.depth_sub = self.create_subscription(
        #     Image, "/camera/depth", self.depth_callback, 10
        # )
        # 订阅 LLM Planner 发布的指令
        self.command_sub = self.create_subscription(
            String, "/llm_command", self.command_callback, 10
        )
        # 发布带 3D 位置的指令给状态机
        self.robot_command_pub = self.create_publisher(String, "/robot_command", 10)

        # 订阅可乐 ground truth 位姿（MuJoCo free_joint_state_publisher 发布的
        # world 坐标，world == panda_link0），用于与 VLM 反投影结果对比，
        # 第一时间发现 VLM 识别偏差。
        self.latest_coke_gt = None
        self.coke_gt_lock = threading.Lock()
        self.coke_gt_sub = self.create_subscription(
            FreeJointStateArray,
            "/free_joint_state_publisher/free_joint_states",
            self.coke_gt_callback,
            10,
        )

        # tf2
        self.tf_buffer = None
        self.tf_listener = None
        try:
            import tf2_ros
            import tf2_geometry_msgs  # 注册 PointStamped
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        except ImportError:
            self.get_logger().warn("tf2_ros not available, coordinate transform disabled.")

        self.get_logger().info("Perception node ready. Waiting for commands on /llm_command ...")

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image = msg

    def coke_gt_callback(self, msg):
        """缓存可乐 ground truth world 位姿（MuJoCo 真实物理位置）。"""
        for fj in msg.free_joints:
            if fj.name == "coke":
                with self.coke_gt_lock:
                    self.latest_coke_gt = (
                        fj.pose.pose.position.x,
                        fj.pose.pose.position.y,
                        fj.pose.pose.position.z,
                    )
                return

    def _read_coke_gt(self):
        """读最新可乐 ground truth 位姿 (x, y, z)，未收到返回 None。"""
        with self.coke_gt_lock:
            return self.latest_coke_gt

    def _log_gt_comparison(self, vlm_position):
        """打印 VLM 反投影结果 vs ground truth 的对比与偏差。"""
        gt = self._read_coke_gt()
        if gt is None:
            self.get_logger().warn(
                "[GT-VLM] 未收到可乐 ground truth（free_joint_states），跳过对比"
            )
            return
        dx = vlm_position[0] - gt[0]
        dy = vlm_position[1] - gt[1]
        dz = vlm_position[2] - gt[2]
        dist = math.hypot(math.hypot(dx, dy), dz)
        self.get_logger().info(
            f"[GT-VLM] VLM=({vlm_position[0]:.4f}, {vlm_position[1]:.4f}, "
            f"{vlm_position[2]:.4f})  GT=({gt[0]:.4f}, {gt[1]:.4f}, {gt[2]:.4f})  "
            f"err=({dx:.4f}, {dy:.4f}, {dz:.4f})  dist={dist:.4f}m"
        )

    def depth_callback(self, msg):
        """缓存最新深度图（32FC1, 640x480）。"""
        with self.depth_lock:
            self.latest_depth = msg

    def _read_depth_at(self, u: float, v: float):
        """读深度图 (u,v) 像素的真实深度（米）。失败或深度无效返回 None。

        已停用（2026-08-15）：深度图帧与 tf/RGB 错位导致坐标漂移，恒返回 None
        强制回退 ASSUMED_DEPTH。见 __init__ 中 depth_sub 的注释。
        """
        return None
        import numpy as np
        with self.depth_lock:
            if self.latest_depth is None:
                return None
            depth_msg = self.latest_depth
        if depth_msg.encoding != "32FC1":
            self.get_logger().warn(f"Unexpected depth encoding: {depth_msg.encoding}")
            return None
        arr = np.frombuffer(bytes(depth_msg.data), dtype=np.float32)
        arr = arr.reshape(depth_msg.height, depth_msg.width)
        ui = int(round(u))
        vi = int(round(v))
        ui = max(0, min(depth_msg.width - 1, ui))
        vi = max(0, min(depth_msg.height - 1, vi))
        return float(arr[vi, ui])

    def _get_image(self):
        """读最新 RGB 帧转 PIL，保存调试图到 /tmp，返回 img 或 None。"""
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().error("No image available.")
                return None
            image_msg = self.latest_image
        from PIL import Image as PILImage
        img = PILImage.frombytes("RGB", (image_msg.width, image_msg.height), bytes(image_msg.data))
        debug_path = "/tmp/perception_latest.jpg"
        img.save(debug_path, format="JPEG")
        self.get_logger().info(f"Saved debug image to {debug_path}")
        return img

    def _color_detect(self, img) -> dict:
        """颜色分割检测可乐：暗红特征色 + 最大连通区域质心。

        固定场景（相机固定、光照稳定、可乐暗红）下亚像素级准且确定性，
        作为主检测器；VLM 仅在其失败时兜底。

        返回 {"name", "bbox", "center"}，center 为最大红色连通区域的质心 (u,v)。
        """
        import numpy as np
        arr = np.array(img).astype(int)
        r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        # 可乐材质 rgba=1,0.08,0.08 渲染后 ≈ RGB(95,8,6)；顶部有亮红高光(>120)。
        # R∈[60,160] 覆盖暗红主体 + 高光，G/B 严格压死以排除棕色桌面 / 橙色机械臂。
        mask = (r >= 60) & (r <= 160) & (g < 40) & (b < 40)
        if int(mask.sum()) < 50:
            self.get_logger().warn(
                f"Color detect: only {mask.sum()} red pixels found."
            )
            return None

        ys, xs = np.where(mask)
        # 取最大连通区域，避免零星红色噪声拉偏 bbox；scipy 不可用时退化为
        # 全 mask 质心（质心本身对离群像素鲁棒）。
        try:
            from scipy import ndimage
            lbl, n = ndimage.label(mask)
            sizes = ndimage.sum(mask, lbl, range(1, n + 1))
            k = int(np.argmax(sizes)) + 1
            ys, xs = np.where(lbl == k)
        except ImportError:
            pass

        cx = float(xs.mean())
        cy = float(ys.mean())
        x_min, x_max = int(xs.min()), int(xs.max())
        y_min, y_max = int(ys.min()), int(ys.max())
        self.get_logger().info(
            f"Color detect: bbox=[{x_min},{y_min},{x_max},{y_max}], "
            f"center=({cx:.1f},{cy:.1f}), pixels={len(xs)}"
        )
        return {"name": "cola", "bbox": [x_min, y_min, x_max, y_max],
                "center": (cx, cy)}

    def detect_object(self, target_name: str, img) -> dict:
        """调 VLM 检测目标物体，返回 bbox。失败重试最多 VLM_MAX_RETRIES 次。

        img 为 _get_image 已读出的 PIL RGB 图，避免重复读帧。
        """
        import io
        buf = io.BytesIO()
        img.save(buf, format="JPEG")
        img_b64 = base64.b64encode(buf.getvalue()).decode()

        prompt = VLM_PROMPT_TEMPLATE.format(target=target_name)
        for attempt in range(1, VLM_MAX_RETRIES + 1):
            try:
                response = self.client.chat.completions.create(
                    model=VLM_MODEL,
                    messages=[{
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                            {"type": "text", "text": prompt},
                        ]
                    }],
                    response_format={"type": "json_object"},
                )
                raw = response.choices[0].message.content
                self.get_logger().info(f"VLM response (attempt {attempt}/{VLM_MAX_RETRIES}): {raw}")
                result = json.loads(raw)
                objects = result.get("objects", [])
                if objects:
                    return objects[0]
                self.get_logger().warn(f"VLM attempt {attempt} returned empty, retrying...")
                time.sleep(1.0)
            except json.JSONDecodeError:
                self.get_logger().error(f"VLM returned invalid JSON on attempt {attempt}.")
            except Exception as e:
                self.get_logger().error(f"VLM call failed on attempt {attempt}: {e}")

        # VLM 全部失败（颜色分割已在主路径试过，这里直接放弃）
        self.get_logger().warn("All VLM attempts failed.")
        return None

    def backproject_to_3d(self, bbox: list, center=None) -> PointStamped:
        """2D 中心 + 真实深度（/camera/depth）→ 相机坐标系 3D 点。

        center 为优先使用的 (u,v)（颜色分割给的质心）；为 None 时用 bbox 中心。
        """
        if center is not None:
            u, v = center
        else:
            x_min, y_min, x_max, y_max = bbox
            u = (x_min + x_max) / 2.0
            v = (y_min + y_max) / 2.0
        self.get_logger().info(f"bbox center: u={u:.1f}, v={v:.1f}")

        # 优先读深度图真实深度；读不到（无深度消息/深度<=0）回退到假设深度
        Z = self._read_depth_at(u, v)
        if Z is None or Z <= 0.0:
            self.get_logger().warn(
                f"No valid depth at ({u:.1f},{v:.1f}), fallback to ASSUMED_DEPTH={ASSUMED_DEPTH}"
            )
            Z = ASSUMED_DEPTH
        else:
            self.get_logger().info(f"Measured depth at ({u:.1f},{v:.1f}): Z={Z:.4f}")

        # 针孔相机模型反投影
        X = (u - CAMERA_CX) * Z / CAMERA_FX
        Y = (v - CAMERA_CY) * Z / CAMERA_FY
        self.get_logger().info(f"3D point in camera_link: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")

        # demo 阶段 Z 下限保护：panda_link0 坐标系下 Z < 0.2 不可达
        # 这里先不做限制，在 transform_to_base 之后做
        point = PointStamped()
        point.header.frame_id = "camera_link"
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = float(X)
        point.point.y = float(Y)
        point.point.z = float(Z)
        return point

    def transform_to_base(self, point_camera: PointStamped) -> list:
        """tf2 转换 camera_link → panda_link0。"""
        if self.tf_buffer is None:
            self.get_logger().error("tf2 not available.")
            return None
        try:
            point_base = self.tf_buffer.transform(
                point_camera, "panda_link0", timeout=rclpy.duration.Duration(seconds=2.0)
            )
            x = point_base.point.x
            y = point_base.point.y
            z = point_base.point.z

            # 可达性保护：X < 0.25 时 Panda 不可达
            # Z 不做 clamp —— 输出的 z 是可乐中心高度，状态机会加上 link8 到指尖偏移
            if x < 0.25:
                self.get_logger().warn(f"X={x:.3f} too close, clamping to 0.25")
                x = 0.25

            self.get_logger().info(f"3D point in panda_link0: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
            return [x, y, z]
        except Exception as e:
            self.get_logger().error(f"tf2 transform failed: {e}")
            return None

    def command_callback(self, msg):
        """收到 LLM 指令 → 颜色分割(主)/VLM(兜底) → 反投影 → tf2 → 发布 /robot_command。"""
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON: {msg.data}")
            return

        target = cmd.get("target_object")
        action = cmd.get("action")
        self.get_logger().info(f"Received LLM command: target={target}, action={action}")

        if action != "pick":
            self.get_logger().warn(f"Unsupported action: {action}")
            return

        # 1. 读帧
        img = self._get_image()
        if img is None:
            return

        # 2. 检测：颜色分割优先（快、准、确定性），失败再 VLM 兜底
        detection = self._color_detect(img)
        if detection is None:
            self.get_logger().warn("Color detection failed, falling back to VLM.")
            detection = self.detect_object(target, img)
        if detection is None:
            self.get_logger().error(f"Failed to detect {target} (color + VLM).")
            return
        bbox = detection.get("bbox")
        if not bbox or len(bbox) != 4:
            self.get_logger().error(f"Invalid bbox: {bbox}")
            return

        # 3. 2D → 3D 反投影
        point_camera = self.backproject_to_3d(bbox, center=detection.get("center"))

        # 4. tf2 坐标转换
        position = self.transform_to_base(point_camera)
        if position is None:
            self.get_logger().error("Coordinate transform failed.")
            return

        # 4.5 合理性校验：可乐应在桌面上方合理范围，挡住 VLM 幻觉（如 Z<0 或越界）
        x, y, z = position
        if not (0.20 <= x <= 0.60 and -0.35 <= y <= 0.35 and 0.0 <= z <= 0.30):
            self.get_logger().error(
                f"Detected position out of table range: ({x:.3f},{y:.3f},{z:.3f}), rejected."
            )
            return

        # 5. 打印检测 vs ground truth 对比（便于发现识别偏差）
        self._log_gt_comparison(position)

        # 6. 发布带 3D 位置的指令
        robot_cmd = {
            "target_object": target,
            "action": action,
            "position": position,
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
