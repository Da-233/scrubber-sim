"""path_linter 单元测试 —— 阿卡曼路径可执行性闸门

覆盖四项检查各自的红绿、K1 真实输出过闸门、曲率/原地转的隔离、
以及 assert_path_executable 闸门语义。
"""
import math

import pytest

from ackermann_primitives.boustrophedon import generate_lawnmower
from ackermann_primitives.primitives import Pose2D, sample_arc, sample_line
from path_linter import (
    LintConfig,
    PathNotExecutable,
    assert_path_executable,
    lint_path,
)


# --- 1. K1 真实输出过闸门(这是把 linter 焊成 K1 测试闸门的关键用例) ---

def test_k1_lawnmower_passes_all_checks():
    """K1 纯弓字在足够大的房间里应四项全绿(含整车越界检查)。"""
    width, height, turn_radius, margin = 12.0, 10.0, 2.0, 2.0
    path = generate_lawnmower(
        width=width,
        height=height,
        lane_spacing=2 * turn_radius,
        turn_radius=turn_radius,
        margin=margin,
    )
    outer = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]
    cfg = LintConfig(r_min=turn_radius, outer=outer)  # 半圆掉头曲率恰为 1/turn_radius
    report = lint_path(path, cfg)
    assert report.ok, report.summary()
    assert all(not c.skipped for c in report.checks)  # 四项都真跑了


# --- 2. 曲率超限 ---

def test_curvature_violation_fails():
    """R=1.0 圆弧在 R_min=1.2 下曲率超限。"""
    path = sample_arc(Pose2D(0, 0, 0), radius=1.0, angle=math.pi)
    report = lint_path(path, LintConfig(r_min=1.2))
    curv = next(c for c in report.checks if c.name == "curvature")
    assert not curv.ok
    assert not report.ok


def test_curvature_ok_when_radius_above_rmin():
    """R=2.0 圆弧在 R_min=1.2 下曲率合规。"""
    path = sample_arc(Pose2D(0, 0, 0), radius=2.0, angle=math.pi)
    report = lint_path(path, LintConfig(r_min=1.2))
    curv = next(c for c in report.checks if c.name == "curvature")
    assert curv.ok, curv.detail


# --- 3. 原地转,且与曲率检查隔离 ---

def test_in_place_turn_fails_and_does_not_trigger_curvature():
    """v≈0 仍转向应被 in_place_turn 抓到,而不是被曲率检查误报成尖峰。"""
    path = [Pose2D(0, 0, 0.0), Pose2D(0, 0, 0.5), Pose2D(0, 0, 1.0)]
    report = lint_path(path, LintConfig(r_min=1.2))
    inplace = next(c for c in report.checks if c.name == "in_place_turn")
    curv = next(c for c in report.checks if c.name == "curvature")
    assert not inplace.ok
    assert curv.ok  # 原地段不计入曲率,避免数值尖峰假阳
    assert not report.ok


# --- 4. 越界(整车 footprint 扫出外墙) ---

def test_out_of_bounds_fails():
    """直线扫出小房间外 → 越界报警,穿障不触发。"""
    path = sample_line(Pose2D(-3, 0, 0), length=6.0)
    outer = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    report = lint_path(path, LintConfig(r_min=1.2, outer=outer))
    bounds = next(c for c in report.checks if c.name == "out_of_bounds")
    obst = next(c for c in report.checks if c.name == "obstacle")
    assert not bounds.ok
    assert obst.ok  # 无 voids


# --- 5. 穿障(整车 footprint 压到障碍) ---

def test_obstacle_collision_fails():
    """直线穿过房间中央障碍 → 穿障报警,越界不触发。"""
    path = sample_line(Pose2D(-4, 0, 0), length=8.0)
    outer = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    void = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
    report = lint_path(path, LintConfig(r_min=1.2, outer=outer, voids=[void]))
    obst = next(c for c in report.checks if c.name == "obstacle")
    bounds = next(c for c in report.checks if c.name == "out_of_bounds")
    assert not obst.ok
    assert bounds.ok


# --- 6. 不给 outer 时跳过越界/穿障,纯几何仍可判 ---

def test_bounds_checks_skipped_without_outer():
    path = sample_line(Pose2D(0, 0, 0), length=5.0)
    report = lint_path(path, LintConfig(r_min=1.2))  # 无 outer
    bounds = next(c for c in report.checks if c.name == "out_of_bounds")
    obst = next(c for c in report.checks if c.name == "obstacle")
    assert bounds.skipped and obst.skipped
    assert report.ok  # 直线几何合规,跳过的项不算失败


# --- 7. assert_path_executable 闸门语义 ---

def test_assert_raises_on_bad_path():
    path = sample_arc(Pose2D(0, 0, 0), radius=0.8, angle=math.pi)
    with pytest.raises(PathNotExecutable):
        assert_path_executable(path, LintConfig(r_min=1.2))


def test_assert_passes_on_good_path():
    path = sample_line(Pose2D(0, 0, 0), length=3.0)
    assert_path_executable(path, LintConfig(r_min=1.2))  # 不抛即通过


# --- 8. 接受 (x, y, yaw) 元组与 Pose2D 两种输入 ---

def test_accepts_tuple_input():
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    report = lint_path(path, LintConfig(r_min=1.2))
    assert report.ok, report.summary()
