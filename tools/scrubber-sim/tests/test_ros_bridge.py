"""ros_bridge 单元测试 — pose 序列 → nav_msgs/Path dict"""
import math
import pytest

from contour_coverage.ros_bridge import yaw_to_quaternion, poses_to_path_dict


def test_yaw_none_default_quaternion():
    q = yaw_to_quaternion(None)
    assert q == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def test_yaw_zero():
    q = yaw_to_quaternion(0.0)
    assert q["z"] == pytest.approx(0.0)
    assert q["w"] == pytest.approx(1.0)


def test_yaw_pi_half():
    """yaw = pi/2 → z = sin(pi/4) ≈ 0.7071, w = cos(pi/4) ≈ 0.7071"""
    q = yaw_to_quaternion(math.pi / 2)
    assert q["z"] == pytest.approx(math.sin(math.pi / 4))
    assert q["w"] == pytest.approx(math.cos(math.pi / 4))
    assert q["x"] == 0.0 and q["y"] == 0.0


def test_yaw_pi():
    """yaw = pi → z ≈ 1, w ≈ 0"""
    q = yaw_to_quaternion(math.pi)
    assert q["z"] == pytest.approx(1.0)
    assert q["w"] == pytest.approx(0.0, abs=1e-9)


def test_yaw_neg_pi_half():
    q = yaw_to_quaternion(-math.pi / 2)
    assert q["z"] == pytest.approx(-math.sin(math.pi / 4))
    assert q["w"] == pytest.approx(math.cos(math.pi / 4))


def test_empty_poses():
    d = poses_to_path_dict([])
    assert d["header"]["frame_id"] == "map"
    assert d["poses"] == []


def test_single_pose_with_yaw():
    d = poses_to_path_dict([(1.0, 2.0, 0.0)])
    assert len(d["poses"]) == 1
    p = d["poses"][0]
    assert p["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 0.0}
    assert p["pose"]["orientation"]["w"] == pytest.approx(1.0)


def test_single_pose_yaw_none():
    d = poses_to_path_dict([(0.5, -0.5, None)])
    p = d["poses"][0]
    assert p["pose"]["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def test_multi_pose_preserves_order_and_yaw():
    poses = [(0, 0, 0.0), (1, 0, math.pi / 4), (1, 1, math.pi / 2)]
    d = poses_to_path_dict(poses)
    assert len(d["poses"]) == 3
    # 顺序保留
    assert d["poses"][0]["pose"]["position"]["x"] == 0.0
    assert d["poses"][2]["pose"]["position"]["y"] == 1.0
    # yaw=pi/2 的 quaternion 正确
    q3 = d["poses"][2]["pose"]["orientation"]
    assert q3["z"] == pytest.approx(math.sin(math.pi / 4))


def test_custom_frame_id():
    d = poses_to_path_dict([(0, 0, 0)], frame_id="odom")
    assert d["header"]["frame_id"] == "odom"
    assert d["poses"][0]["header"]["frame_id"] == "odom"


def test_custom_stamp():
    d = poses_to_path_dict([(0, 0, 0)], stamp_sec=12345, stamp_nanosec=678)
    assert d["header"]["stamp"]["sec"] == 12345
    assert d["header"]["stamp"]["nanosec"] == 678


def test_position_z_always_zero():
    """所有 pose 的 z 强制为 0 (2D 平面)"""
    d = poses_to_path_dict([(1.5, 2.5, 1.0), (3, 4, 2)])
    for p in d["poses"]:
        assert p["pose"]["position"]["z"] == 0.0


def test_integration_with_build_path():
    """端到端: rings → build_path → poses_to_path_dict 不出错"""
    from contour_coverage.geometry import generate_concentric_rings, OffsetParams
    from contour_coverage.path_builder import build_path, PathParams

    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = [[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)]]
    rings = generate_concentric_rings(outer, voids, OffsetParams(spacing=0.55, safety_margin=0.6))
    poses = build_path(rings, PathParams(densify_step=0.1))
    path = poses_to_path_dict(poses, frame_id="map")

    assert path["header"]["frame_id"] == "map"
    assert len(path["poses"]) == len(poses)
    # 每个 pose 都有完整结构
    for p in path["poses"]:
        assert "position" in p["pose"]
        assert "orientation" in p["pose"]
        q = p["pose"]["orientation"]
        # quaternion 单位长度 (允许浮点误差)
        norm = q["x"]**2 + q["y"]**2 + q["z"]**2 + q["w"]**2
        assert norm == pytest.approx(1.0, abs=1e-9)
