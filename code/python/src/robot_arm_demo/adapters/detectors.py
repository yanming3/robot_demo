"""感知检测器：颜色分割（主）+ Qwen-VL（兜底）。

阈值与提示词从 DetectorConfig/VlmConfig 读取。
"""

from __future__ import annotations

import base64
import json
import time

from ..core.data import DetectorConfig, VlmConfig


class ColorDetector:
    """颜色分割检测目标物体：特征色掩码 + 最大连通区域质心。

    固定场景（相机固定、光照稳定、目标特征色明显）下亚像素级准且确定性，
    作为主检测器；VLM 仅在其失败时兜底。
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


class QwenVlDetector:
    """Qwen-VL 兜底检测（OpenAI 兼容接口）。"""

    def __init__(self, vlm_cfg: VlmConfig, openai_client, logger):
        self.cfg = vlm_cfg
        self.client = openai_client
        self.log = logger

    def detect(self, target_name: str, img) -> dict | None:
        """调 VLM 检测 bbox；失败重试至多 max_retries 次。"""
        import io
        buf = io.BytesIO()
        img.save(buf, format="JPEG")
        img_b64 = base64.b64encode(buf.getvalue()).decode()

        prompt = self.cfg.prompt_template.format(target=target_name)
        retries = self.cfg.max_retries
        for attempt in range(1, retries + 1):
            try:
                response = self.client.chat.completions.create(
                    model=self.cfg.model,
                    messages=[{
                        "role": "user",
                        "content": [
                            {"type": "image_url",
                             "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                            {"type": "text", "text": prompt},
                        ]
                    }],
                    response_format={"type": "json_object"},
                )
                raw = response.choices[0].message.content
                self.log.info(
                    f"VLM response (attempt {attempt}/{retries}): {raw}"
                )
                result = json.loads(raw)
                objects = result.get("objects", [])
                if objects:
                    return objects[0]
                self.log.warn(f"VLM attempt {attempt} returned empty, retrying...")
                time.sleep(1.0)
            except json.JSONDecodeError:
                self.log.error(f"VLM returned invalid JSON on attempt {attempt}.")
            except Exception as e:
                self.log.error(f"VLM call failed on attempt {attempt}: {e}")

        self.log.warn("All VLM attempts failed.")
        return None
