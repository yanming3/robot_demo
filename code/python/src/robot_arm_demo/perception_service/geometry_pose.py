"""Stage 1 几何位姿：由实例 mask + 深度点云推物体中心与瓶轴（不依赖 ROS）。

Grounding-DINO + SAM2 / FoundationPose 尚未接入前，云端 mock 与本地 fallback
共用本模块；Stage 3 由 FoundationPose 取替。它对 mask 里的有限深度点做
反投影，取质心为中心、3D PCA 主轴为瓶轴方向，再据此给出一个把物体局部 +Z
轴对齐到瓶轴的四元数。

注意：单视角下圆柱表面点云的 PCA 主轴是近似值（只看到一侧曲面），仅用于
打通链路；精度上限由 Stage 3 的 FoundationPose 决定。
"""

from __future__ import annotations

import numpy as np

# 局部 +Z（物体本体轴）作为参考，旋转到估计出的瓶轴方向
_REF_AXIS = np.array([0.0, 0.0, 1.0])


def cola_color_mask(rgb: np.ndarray, det) -> np.ndarray:
    """红灯可乐的颜色掩码（阈值来自 DetectorConfig，与旧颜色分割一致）。"""
    arr = np.asarray(rgb).astype(int)
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    return (
        (r >= det.mask_r_min) & (r <= det.mask_r_max)
        & (g < det.mask_g_max) & (b < det.mask_b_max)
    )


def quaternion_align_z_to(axis) -> tuple[float, float, float, float]:
    """把 +Z 旋转到单位向量 axis 的最短弧四元数（返回 xyzw）。"""
    a = np.asarray(axis, dtype=np.float64)
    n = float(np.linalg.norm(a))
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    a = a / n
    dot = float(np.clip(np.dot(_REF_AXIS, a), -1.0, 1.0))
    if dot > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    v = np.cross(_REF_AXIS, a)
    q = np.concatenate([v, [1.0 + dot]])
    q = q / np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _backproject_mask(
    depth: np.ndarray, mask: np.ndarray, intrinsics: dict
) -> np.ndarray:
    """mask 内有限深度像素 → (N, 3) 相机系点云。"""
    w = int(intrinsics["width"])
    h = int(intrinsics["height"])
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    if np.asarray(depth).shape[:2] != (h, w):
        raise ValueError(f"depth shape {depth.shape[:2]} != intrinsics {h}x{w}")
    ys, xs = np.where(mask)
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    z = depth[ys, xs].astype(np.float64)
    ok = np.isfinite(z) & (z > 0)
    xs, ys, z = xs[ok], ys[ok], z[ok]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def pose_from_mask(
    rgb: np.ndarray,
    depth: np.ndarray,
    intrinsics: dict,
    mask: np.ndarray,
    *,
    min_points: int = 20,
) -> dict:
    """由 mask + 深度反投影推中心与瓶轴。

    返回规范化结果字典（contract.normalize_pose 形态）：
      {"success":bool, "position":(x,y,z)|None, "orientation_xyzw":(4)|None,
       "axis":(3)|None, "confidence":float|None, "object_id":None, ...}
    有效 3D 点数 < min_points 时 success=False。
    """
    if mask is None:
        return {"success": False, "position": None, "orientation_xyzw": None,
                "axis": None, "confidence": None, "object_id": None,
                "error_code": "1501", "error_message": "empty mask"}
    pts = _backproject_mask(depth, mask, intrinsics)
    if pts.shape[0] < min_points:
        return {"success": False, "position": None, "orientation_xyzw": None,
                "axis": None, "confidence": None, "object_id": None,
                "error_code": "1501",
                "error_message": f"only {pts.shape[0]} valid 3D points"}
    centroid = pts.mean(axis=0)

    # 3D PCA：主轴 = 瓶轴方向；特征值退化时退化为相机 +Z
    centered = pts - centroid
    cov = (centered.T @ centered) / centered.shape[0]
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis = eigvecs[:, int(np.argmax(eigvals))]
    # 符号整形：让 z 分量 >= 0（避免相邻帧轴方向跳变），并归一化
    if axis[2] < 0:
        axis = -axis
    axis = axis / (np.linalg.norm(axis) + 1e-12)

    # 主轴方差占比（衡量 PCA 是否退化）
    total = float(eigvals.sum()) + 1e-12
    var_ratio = float(eigvals[-1] / total) if eigvals.size else 1.0
    confidence = float(min(1.0, max(0.0, var_ratio)))

    return {
        "success": True,
        "position": (float(centroid[0]), float(centroid[1]), float(centroid[2])),
        "orientation_xyzw": quaternion_align_z_to(axis),
        "axis": (float(axis[0]), float(axis[1]), float(axis[2])),
        "confidence": confidence,
        "object_id": None,
        "error_code": None,
        "error_message": None,
    }
