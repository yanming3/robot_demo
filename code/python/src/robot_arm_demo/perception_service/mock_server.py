"""云端位姿服务的 Stage 1 mock（stdlib http.server，零新依赖）。

实现 §5 的 /v1/estimate_pose（mask+几何位姿）与 /v1/health。它不是最终
FoundationPose 服务，而是「本地↔云」链路打通用的替身：用红灯可乐的颜色掩码
算出中心与瓶轴，走 geometry_pose.pose_from_mask。真实云端只要遵守同一契约，
返回同构响应即可无缝替换。

运行：
    PYTHONPATH=src python -m robot_arm_demo.perception_service.mock_server [port]
默认端口 8000。
"""

from __future__ import annotations

import http.server
import json
import sys
import time

import numpy as np

from . import contract, geometry_pose

DEFAULT_PORT = 8000


def estimate_pose(request: dict, det=None) -> dict:
    """按契约处理一次 estimate_pose 请求（纯函数，供测试复用）。

    返回契约响应 dict（含 success/pose/error）。
    """
    try:
        rgb = contract.decode_rgb(request["image"])
        intrinsics = request["camera_intrinsics"]
        depth = (
            contract.decode_depth(request["depth"])
            if request.get("depth") else None
        )
    except (KeyError, ValueError) as e:
        return _error("1001", f"bad request: {e}")

    if det is None:
        det = _default_detector()
    mask = geometry_pose.cola_color_mask(rgb, det)
    if int(mask.sum()) < det.min_pixels:
        return _error("1500", "no target detected")

    if depth is None:
        return _error("1501", "missing depth")

    res = geometry_pose.pose_from_mask(rgb, depth, intrinsics, mask)
    if not res["success"]:
        return _error("1501", res["error_message"])

    resp = {
        "version": contract.VERSION,
        "success": True,
        "elapsed_ms": 0.0,
        "object_id": request.get("object_id"),
        "pose": {
            "position": list(res["position"]),
            "orientation_quaternion_xyzw": list(res["orientation_xyzw"]),
            "confidence": res["confidence"],
        },
        "axis": list(res["axis"]),
        "mask": (
            contract.mask_to_png_b64(mask) if request.get("return_mask") else None
        ),
        "segmentation": {
            "method": "mock_color_geometry",
            "mask_confidence": 1.0,
        },
        "warnings": ["mock Stage-1 geometry pose; FoundationPose wired in Stage 3"],
    }
    return resp


class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # 安静化默认日志
        return

    def _send(self, code: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"status": "ok", "version": contract.VERSION,
                             "elapsed_ms": 0.0,
                             "models": {"mock": "geometry"}})
        else:
            self._send(404, _error("1005", f"unknown path {self.path}"))

    def do_POST(self):
        if self.path != "/v1/estimate_pose":
            self._send(404, _error("1005", f"unknown path {self.path}"))
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            request = json.loads(raw)
        except (ValueError, json.JSONDecodeError) as e:
            self._send(400, _error("1001", f"invalid body: {e}"))
            return
        t0 = time.time()
        resp = estimate_pose(request)
        resp["elapsed_ms"] = round((time.time() - t0) * 1000.0, 2)
        code = 200 if resp.get("success") else 422
        self._send(code, resp)


def _error(code: str, message: str) -> dict:
    return {
        "version": contract.VERSION,
        "success": False,
        "elapsed_ms": 0.0,
        "error": {"code": code, "message": message},
    }


class _Detector:
    """与 config.DetectorConfig(cola) 一致的阈值（mock 用，避免依赖 ROS）。"""

    def __init__(self):
        self.name = "cola"
        self.mask_r_min = 60
        self.mask_r_max = 160
        self.mask_g_max = 40
        self.mask_b_max = 40
        self.min_pixels = 50


def _default_detector():
    return _Detector()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    port = int(argv[0]) if argv else DEFAULT_PORT
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    print(f"[mock-server] listening on http://127.0.0.1:{port}"
          f" (POST /v1/estimate_pose, GET /health)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
