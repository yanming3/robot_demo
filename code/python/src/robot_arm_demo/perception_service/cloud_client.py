"""本地感知节点 → 云端位姿服务的 HTTP 客户端（stdlib urllib，零新依赖）。

只负责传输与响应解析，不关心 ROS 消息；请求体由 contract.build_request 构造，
结果用 contract.parse_response 规范化。
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

from ..core.data import PoseServiceConfig
from . import contract


class PoseServiceClient:
    """POST /v1/estimate_pose 的客户端，自带重试与超时。"""

    def __init__(self, config: PoseServiceConfig, logger):
        self.cfg = config
        self.log = logger
        self.url = config.base_url.rstrip("/") + "/v1/estimate_pose"

    def estimate(self, request: dict) -> dict:
        """发送请求，返回规范化结果字典；网络/HTTP/业务失败都 success=False。"""
        body = json.dumps(request, ensure_ascii=False).encode("utf-8")
        last_err: Exception | None = None
        for attempt in range(1, self.cfg.retries + 2):
            try:
                req = urllib.request.Request(
                    self.url, data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=self.cfg.timeout_s) as resp:
                    raw = resp.read()
                parsed = json.loads(raw)
                res = contract.parse_response(parsed)
                if not res["success"]:
                    self.log.warn(
                        f"[PoseService] attempt {attempt} failed: "
                        f"{res['error_code']} {res['error_message']}"
                    )
                return res
            except (urllib.error.HTTPError, urllib.error.URLError) as e:
                last_err = e
                self.log.warn(
                    f"[PoseService] request attempt {attempt}/{self.cfg.retries + 1} "
                    f"failed: {e}"
                )
            except (json.JSONDecodeError, ValueError) as e:
                last_err = e
                self.log.warn(f"[PoseService] bad response on attempt {attempt}: {e}")
                return contract.normalize_pose(
                    None, None, success=False
                )
            if attempt <= self.cfg.retries:
                time.sleep(0.3 * attempt)
        self.log.error(f"[PoseService] all attempts failed: {last_err}")
        return contract.normalize_pose(None, None, success=False)
