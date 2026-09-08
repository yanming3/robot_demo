"""perception_service 契约 + 几何位姿 的纯逻辑单测（不依赖 ROS）。"""

import numpy as np
import pytest

from robot_arm_demo.perception_service import contract, geometry_pose
from robot_arm_demo.perception_service.mock_server import estimate_pose

W, H = 640, 480
INTRIN = contract.build_intrinsics(554.0, 554.0, 320.0, 240.0, W, H, None)


def _red_rect_frame(depth_val=0.6):
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    rgb[220:260, 300:340] = (120, 0, 0)  # 暗红主体
    rgb[225:235, 315:325] = (140, 0, 0)  # 高光
    depth = np.full((H, W), np.nan, dtype=np.float32)
    depth[220:260, 300:340] = depth_val
    return rgb, depth


class _Det:
    name = "cola"
    mask_r_min = 60
    mask_r_max = 160
    mask_g_max = 40
    mask_b_max = 40
    min_pixels = 50


def test_encode_decode_rgb_roundtrip():
    arr = np.arange(W * H * 3, dtype=np.uint8).reshape(H, W, 3)
    payload = contract.encode_rgb(arr)
    assert payload["width"] == W and payload["height"] == H
    out = contract.decode_rgb(payload)
    assert out.shape == (H, W, 3)
    assert np.array_equal(out, arr)


def test_encode_decode_depth_roundtrip():
    arr = np.linspace(0, 1.0, W * H, dtype=np.float32).reshape(H, W)
    payload = contract.encode_depth(arr)
    out = contract.decode_depth(payload)
    assert out.shape == (H, W)
    assert np.allclose(out, arr, atol=1e-6)


def test_build_request_shape():
    rgb, depth = _red_rect_frame()
    req = contract.build_request(
        rgb, depth, INTRIN, "coke bottle", "coke", "camera_link"
    )
    assert req["version"] == "v1"
    assert req["image"]["width"] == W
    assert req["depth"]["encoding"] == "32FC1"
    assert req["camera_intrinsics"]["fx"] == 554.0
    assert req["camera_frame_id"] == "camera_link"


def test_parse_response_ok_normalizes_quaternion():
    body = {
        "version": "v1", "success": True,
        "pose": {
            "position": [0.1, -0.02, 0.6],
            "orientation_quaternion_xyzw": [0.0, 0.0, 1.0, 1.0],  # 未归一化
            "confidence": 0.9,
        },
        "axis": [0, 0, 1],
    }
    res = contract.parse_response(body)
    assert res["success"]
    assert res["position"] == (0.1, -0.02, 0.6)
    # 归一化为单位四元数
    q = np.array(res["orientation_xyzw"])
    assert abs(np.linalg.norm(q) - 1.0) < 1e-6


def test_parse_response_failure():
    res = contract.parse_response(
        {"success": False, "error": {"code": "1501", "message": "no depth"}}
    )
    assert not res["success"]
    assert res["error_code"] == "1501"


def test_geometry_pose_center():
    rgb, depth = _red_rect_frame(0.6)
    mask = geometry_pose.cola_color_mask(rgb, _Det())
    res = geometry_pose.pose_from_mask(rgb, depth, INTRIN, mask)
    assert res["success"]
    # 矩形中心 (320,240) 深度 0.6 → 相机系 (0,0,0.6)
    assert abs(res["position"][0]) < 0.02
    assert abs(res["position"][1]) < 0.02
    assert abs(res["position"][2] - 0.6) < 0.02


def test_geometry_pose_degenerate_no_points():
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    depth = np.full((H, W), np.nan, dtype=np.float32)
    mask = np.zeros((H, W), dtype=bool)
    res = geometry_pose.pose_from_mask(rgb, depth, INTRIN, mask)
    assert not res["success"]


def test_mock_server_estimate_pose_ok():
    rgb, depth = _red_rect_frame(0.6)
    req = contract.build_request(rgb, depth, INTRIN, "coke bottle", "coke", "camera_link")
    resp = estimate_pose(req, _Det())
    assert resp["success"]
    assert len(resp["pose"]["position"]) == 3
    assert abs(resp["pose"]["position"][2] - 0.6) < 0.02


def test_mock_server_estimate_pose_no_target():
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    depth = np.full((H, W), 0.6, dtype=np.float32)
    req = contract.build_request(rgb, depth, INTRIN, "coke bottle", "coke", "camera_link")
    resp = estimate_pose(req, _Det())
    assert not resp["success"]
    assert resp["error"]["code"] == "1500"
