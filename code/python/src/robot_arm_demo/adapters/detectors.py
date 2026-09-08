"""感知检测器：颜色分割（旧实现，现已被 geometry_pose 掩码取代，仅存档）。

阈值从 DetectorConfig 读取。新感知链路用 perception_service.geometry_pose
的 cola_color_mask + 深度点云，不再走颜色反投影/assumed_depth。
"""

from __future__ import annotations

from ..core.data import DetectorConfig


class ColorDetector:
    """颜色分割检测目标物体：特征色掩码 + 最大连通区域质心。

    固定场景（相机固定、光照稳定、目标特征色明显）下亚像素级准且确定性。
    （新感知链路已用 perception_service.geometry_pose 的掩码+深度取代本类。）
    """

    def __init__(self, detector_cfg: DetectorConfig, logger):
        self.cfg = detector_cfg
        self.log = logger

    def detect(self, target_name: str, img) -> dict | None:
        """返回 {"name","bbox","center"}，center 为最大连通区域质心 (u,v)。"""
        import numpy as np
        det = self.cfg
        arr = np.array(img).astype(int)
        r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        mask = (
            (r >= det.mask_r_min) & (r <= det.mask_r_max)
            & (g < det.mask_g_max) & (b < det.mask_b_max)
        )
        if int(mask.sum()) < det.min_pixels:
            self.log.warn(f"Color detect: only {mask.sum()} red pixels found.")
            return None

        ys, xs = np.where(mask)
        # 取最大连通区域，避免零星噪声拉偏 bbox；scipy 不可用时退化为全 mask 质心
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
        self.log.info(
            f"Color detect: bbox=[{x_min},{y_min},{x_max},{y_max}], "
            f"center=({cx:.1f},{cy:.1f}), pixels={len(xs)}"
        )
        return {"name": det.name, "bbox": [x_min, y_min, x_max, y_max],
                "center": (cx, cy)}
