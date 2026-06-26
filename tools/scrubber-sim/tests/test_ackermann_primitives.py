"""K0 Ackermann 运动原语测试"""
import math

import pytest

from ackermann_primitives.primitives import (
    CurvatureViolation,
    InvalidPrimitive,
    Pose2D,
    assert_curvature_within,
    max_curvature,
    normalize_angle,
    path_length,
    sample_arc,
    sample_line,
    sample_u_turn,
)


def assert_pose_close(actual: Pose2D, expected: Pose2D, tol: float = 1e-6) -> None:
    assert actual.x == pytest.approx(expected.x, abs=tol)
    assert actual.y == pytest.approx(expected.y, abs=tol)
    assert normalize_angle(actual.yaw - expected.yaw) == pytest.approx(0.0, abs=tol)


def test_sample_line_keeps_yaw_and_spacing():
    """直线段：yaw 不变、总长度正确、点距不超过 step"""
    poses = sample_line(Pose2D(1.0, 2.0, 0.0), length=2.0, step=0.3)

    assert_pose_close(poses[0], Pose2D(1.0, 2.0, 0.0))
    assert_pose_close(poses[-1], Pose2D(3.0, 2.0, 0.0))
    assert path_length(poses) == pytest.approx(2.0, abs=1e-6)
    for i in range(len(poses) - 1):
        d = math.hypot(poses[i + 1].x - poses[i].x, poses[i + 1].y - poses[i].y)
        assert d <= 0.3 + 1e-6
    assert max_curvature(poses) == pytest.approx(0.0, abs=1e-9)


def test_sample_line_with_nonzero_yaw():
    """非零 yaw 直线也沿车头方向走"""
    poses = sample_line(Pose2D(0.0, 0.0, math.pi / 2), length=1.5, step=0.2)

    assert poses[-1].x == pytest.approx(0.0, abs=1e-6)
    assert poses[-1].y == pytest.approx(1.5, abs=1e-6)
    assert poses[-1].yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_sample_arc_left_quarter_turn_geometry():
    """左转 90°：从 +x 朝向转到 +y，终点在 (R, R)"""
    radius = 2.0
    poses = sample_arc(
        Pose2D(0.0, 0.0, 0.0),
        radius=radius,
        angle=math.pi / 2,
        step=0.1,
        direction="left",
    )

    assert_pose_close(poses[0], Pose2D(0.0, 0.0, 0.0))
    assert_pose_close(poses[-1], Pose2D(radius, radius, math.pi / 2), tol=1e-5)
    assert path_length(poses) == pytest.approx(radius * math.pi / 2, rel=0.001)
    assert max_curvature(poses) == pytest.approx(1.0 / radius, rel=1e-3)


def test_sample_arc_right_quarter_turn_geometry():
    """右转 90°：从 +x 朝向转到 -y，终点在 (R, -R)"""
    radius = 2.0
    poses = sample_arc(
        Pose2D(0.0, 0.0, 0.0),
        radius=radius,
        angle=math.pi / 2,
        step=0.1,
        direction="right",
    )

    assert_pose_close(poses[-1], Pose2D(radius, -radius, -math.pi / 2), tol=1e-5)
    assert max_curvature(poses) == pytest.approx(1.0 / radius, rel=1e-3)


def test_sample_u_turn_half_circle():
    """U 掉头是半圆：终点朝向反向，横向偏移 2R"""
    radius = 2.0
    poses = sample_u_turn(Pose2D(0.0, 0.0, 0.0), radius=radius, step=0.1, direction="left")

    assert_pose_close(poses[-1], Pose2D(0.0, 2.0 * radius, -math.pi), tol=1e-5)
    assert path_length(poses) == pytest.approx(math.pi * radius, rel=0.001)
    assert max_curvature(poses) == pytest.approx(1.0 / radius, rel=1e-3)


def test_assert_curvature_within_accepts_safe_radius():
    """R=2 的圆弧在 max_curvature=0.5 下可执行"""
    poses = sample_u_turn(Pose2D(0.0, 0.0, 0.0), radius=2.0, step=0.1)

    assert_curvature_within(poses, max_allowed=0.5, tolerance=1e-4)


def test_assert_curvature_within_rejects_too_tight_turn():
    """R=1 的圆弧超过 R_safe=2 对应的曲率上限 0.5"""
    poses = sample_u_turn(Pose2D(0.0, 0.0, 0.0), radius=1.0, step=0.1)

    with pytest.raises(CurvatureViolation, match="exceeds allowed"):
        assert_curvature_within(poses, max_allowed=0.5, tolerance=1e-4)


def test_invalid_primitive_parameters():
    """无效参数要明确报错，避免静默生成不可解释路径"""
    start = Pose2D(0.0, 0.0, 0.0)

    with pytest.raises(InvalidPrimitive, match="step"):
        sample_line(start, length=1.0, step=0.0)
    with pytest.raises(InvalidPrimitive, match="radius"):
        sample_arc(start, radius=0.0, angle=1.0)
    with pytest.raises(InvalidPrimitive, match="angle"):
        sample_arc(start, radius=1.0, angle=-1.0)
    with pytest.raises(InvalidPrimitive, match="direction"):
        sample_arc(start, radius=1.0, angle=1.0, direction="clockwise")  # type: ignore[arg-type]


def test_normalize_angle_range():
    """角度归一化始终落在 [-pi, pi)"""
    for angle in [-10.0, -math.pi, 0.0, math.pi, 10.0]:
        out = normalize_angle(angle)
        assert -math.pi <= out < math.pi

    assert normalize_angle(math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-math.pi) == pytest.approx(-math.pi)
