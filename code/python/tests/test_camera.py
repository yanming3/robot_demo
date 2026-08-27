"""针孔反投影纯函数测试。"""

import math

from robot_arm_demo.core.camera import backproject_pinhole
from robot_arm_demo.core.data import CameraConfig


def _cam() -> CameraConfig:
    return CameraConfig(frame_id="camera_link", fx=554.0, fy=554.0,
                        cx=320.0, cy=240.0, assumed_depth=0.76)


def test_center_ray_hits_optical_axis():
    x, y, z = backproject_pinhole(320.0, 240.0, 0.76, _cam())
    assert (x, y) == (0.0, 0.0)
    assert z == 0.76


def test_offaxis_point_math():
    cam = _cam()
    u, v, depth = 400.0, 200.0, 0.76
    x, y, z = backproject_pinhole(u, v, depth, cam)
    exp_x = (u - cam.cx) * depth / cam.fx   # 80*0.76/554 ≈ 0.10975
    exp_y = (v - cam.cy) * depth / cam.fy   # -40*0.76/554 ≈ -0.05487
    assert math.isclose(x, exp_x, abs_tol=1e-12)
    assert math.isclose(y, exp_y, abs_tol=1e-12)
    assert z == depth
