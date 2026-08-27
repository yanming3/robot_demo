"""针孔相机反投影（纯函数，零 ROS 依赖）。"""

from __future__ import annotations

from .data import CameraConfig


def backproject_pinhole(
    u: float, v: float, depth: float, cam: CameraConfig
) -> tuple[float, float, float]:
    """像素 (u,v) + 深度 Z → 相机坐标系 3D 点。"""
    x = (u - cam.cx) * depth / cam.fx
    y = (v - cam.cy) * depth / cam.fy
    return x, y, depth
