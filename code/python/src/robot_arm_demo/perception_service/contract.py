"""感知位姿云服务：本地与云端共用的 wire 契约（不依赖 ROS）。

本模块是「request/response 的字段、单位、坐标系」的唯一来源。云端 mock /
真实 FoundationPose 服务 与 本地 perception_node 都以它为基准，避免两边
各自实现导致字段、单位、坐标系漂移。

约定（详见 docs/perception_pose_service_contract.md §3）：
- 位置单位：米 (m)
- 旋转：单位四元数 [x, y, z, w]（xyzw 顺序，右手系）
- 位姿参考系：相机光心光学坐标系（与内参 fx/fy/cx/cy 同系），用请求里的
  camera_frame_id 声明
- 深度单位：米，float32，NaN/Inf 视为无效
"""

from __future__ import annotations

import base64
import io
from typing import Any, Sequence

import numpy as np
from PIL import Image as PILImage

VERSION = "v1"


# ── 图像/深度编码 ────────────────────────────────────────────────

def encode_rgb(arr: np.ndarray) -> dict:
    """(h, w, 3) uint8 → {"encoding":"rgb8","width","height","data_b64"}。"""
    arr = np.ascontiguousarray(arr, dtype=np.uint8)
    h, w = arr.shape[:2]
    return {
        "encoding": "rgb8",
        "width": int(w),
        "height": int(h),
        "data_b64": base64.b64encode(arr.tobytes()).decode("ascii"),
    }


def encode_depth(arr: np.ndarray) -> dict:
    """(h, w) float32(米) → {"encoding":"32FC1","width","height","data_b64"}。"""
    arr = np.ascontiguousarray(arr, dtype="<f4")
    h, w = arr.shape[:2]
    return {
        "encoding": "32FC1",
        "width": int(w),
        "height": int(h),
        "data_b64": base64.b64encode(arr.tobytes()).decode("ascii"),
    }


def decode_rgb(payload: dict) -> np.ndarray:
    """请求的 image payload → (h, w, 3) uint8。"""
    w, h = int(payload["width"]), int(payload["height"])
    raw = base64.b64decode(payload["data_b64"])
    expect = w * h * 3
    if len(raw) != expect:
        raise ValueError(
            f"rgb bytes length {len(raw)} != width*height*3={expect}"
        )
    return np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)


def decode_depth(payload: dict) -> np.ndarray:
    """请求的 depth payload → (h, w) float32(米)。"""
    w, h = int(payload["width"]), int(payload["height"])
    enc = payload.get("encoding", "32FC1")
    raw = base64.b64decode(payload["data_b64"])
    if enc == "32FC1":
        expect = w * h * 4
        if len(raw) != expect:
            raise ValueError(
                f"depth bytes length {len(raw)} != width*height*4={expect}"
            )
        arr = np.frombuffer(raw, dtype="<f4").reshape(h, w).astype(np.float32)
        arr[~np.isfinite(arr)] = np.nan
        return arr
    if enc == "16UC1":
        expect = w * h * 2
        if len(raw) != expect:
            raise ValueError(
                f"depth bytes length {len(raw)} != width*height*2={expect}"
            )
        # 16UC1 通常以毫米表示（RealSense 惯例），转成米
        arr = np.frombuffer(raw, dtype="<u2").reshape(h, w).astype(np.float32) / 1000.0
        arr[~np.isfinite(arr)] = np.nan
        return arr
    raise ValueError(f"unsupported depth encoding: {enc}")


def build_intrinsics(
    fx: float, fy: float, cx: float, cy: float, width: int, height: int,
    distortion_k: Sequence[float] | None = None,
) -> dict:
    """组装 camera_intrinsics 子字典（内参/尺寸/畸变）。"""
    k = list(distortion_k or [0.0] * 5)
    if len(k) < 5:
        k = k + [0.0] * (5 - len(k))
    k = k[:5]
    return {
        "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        "width": int(width), "height": int(height),
        "distortion_k": [float(v) for v in k],
    }


def build_request(
    rgb: np.ndarray,
    depth: np.ndarray | None,
    intrinsics: dict,
    prompt: str,
    object_id: str,
    camera_frame_id: str,
    *,
    return_mask: bool = True,
    max_detections: int = 1,
    reference_object_id: str | None = None,
) -> dict:
    """组装 §4 的 estimate_pose 请求体。"""
    img_payload = encode_rgb(rgb)
    req: dict[str, Any] = {
        "version": VERSION,
        "image": img_payload,
        "camera_intrinsics": intrinsics,
        "camera_frame_id": camera_frame_id,
        "prompt": prompt,
        "object_id": object_id,
        "return_mask": bool(return_mask),
        "max_detections": int(max_detections),
        "reference_object_id": reference_object_id,
    }
    if depth is not None and depth.size:
        req["depth"] = encode_depth(depth)
    return req


# ── 响应解析 ─────────────────────────────────────────────────────

def normalize_pose(
    position,
    orientation_xyzw,
    axis=None,
    *,
    confidence: float | None = None,
    object_id: str | None = None,
    success: bool = True,
) -> dict:
    """统一为本地消费的规范化结果字典。"""
    res = {
        "success": bool(success),
        "object_id": object_id,
        "position": tuple(float(v) for v in position) if position is not None else None,
        "orientation_xyzw": (
            tuple(float(v) for v in orientation_xyzw) if orientation_xyzw is not None else None
        ),
        "axis": tuple(float(v) for v in axis) if axis is not None else None,
        "confidence": float(confidence) if confidence is not None else None,
        "error_code": None,
        "error_message": None,
    }
    return res


def parse_response(body: dict) -> dict:
    """云端响应体 → 规范化结果字典；业务失败时 success=False + error_*。

    任何字段缺失/类型不符都按失败处理（宁可失败，不产出错误位姿）。
    """
    if not isinstance(body, dict):
        return normalize_pose(None, None, success=False)
    if not body.get("success", False):
        err = body.get("error") or {}
        return {
            "success": False,
            "object_id": body.get("object_id"),
            "position": None,
            "orientation_xyzw": None,
            "axis": None,
            "confidence": None,
            "error_code": err.get("code"),
            "error_message": err.get("message"),
        }
    pose = body.get("pose")
    if not isinstance(pose, dict):
        return normalize_pose(None, None, success=False)
    pos = pose.get("position")
    ori = pose.get("orientation_quaternion_xyzw")
    if not (isinstance(pos, (list, tuple)) and len(pos) == 3):
        return normalize_pose(None, None, success=False)
    if not (isinstance(ori, (list, tuple)) and len(ori) == 4):
        return normalize_pose(None, None, success=False)
    axis = body.get("axis")
    # 四元数归一化（防服务端返回未归一化）
    q = np.array(ori, dtype=np.float64)
    n = float(np.linalg.norm(q))
    if n <= 1e-9:
        q = np.array([0.0, 0.0, 0.0, 1.0])
    else:
        q = q / n
    return normalize_pose(
        tuple(float(v) for v in pos),
        tuple(float(v) for v in q),
        tuple(float(v) for v in axis) if axis is not None else None,
        confidence=pose.get("confidence"),
        object_id=body.get("object_id"),
        success=True,
    )


def mask_to_png_b64(mask: np.ndarray) -> str | None:
    """bool mask → base64 PNG(单通道灰度 0/255)，用于响应 return_mask。"""
    if mask is None:
        return None
    arr = (np.asarray(mask).astype(np.uint8) * 255)
    img = PILImage.fromarray(arr)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def decode_mask_png(data_b64: str) -> np.ndarray:
    """base64 PNG(灰度) → bool mask。"""
    raw = base64.b64decode(data_b64)
    img = PILImage.open(io.BytesIO(raw))
    arr = np.array(img.convert("L"))
    return arr > 127
