"""K1 纯弓字生成器测试"""
import pytest

from ackermann_primitives.boustrophedon import BoustrophedonError, generate_lawnmower
from ackermann_primitives.primitives import max_curvature, path_length
from path_linter import LintConfig, assert_path_executable


def test_lawnmower_stays_inside_room_with_margin():
    """所有路径点都在 margin 内缩后的可用矩形内"""
    path = generate_lawnmower(
        width=12.0,
        height=10.0,
        lane_spacing=4.0,
        turn_radius=2.0,
        margin=2.0,
        step=0.2,
    )

    assert path
    for p in path:
        assert -6.0 + 2.0 - 1e-6 <= p.x <= 6.0 - 2.0 + 1e-6
        assert -5.0 + 2.0 - 1e-6 <= p.y <= 5.0 - 2.0 + 1e-6


def test_lawnmower_curvature_limited_by_turn_radius():
    """K1 只用直线 + R=2 半圆，最大曲率不超过 0.5"""
    path = generate_lawnmower(
        width=14.0,
        height=12.0,
        lane_spacing=4.0,
        turn_radius=2.0,
        margin=2.0,
        step=0.1,
    )

    assert max_curvature(path) <= 0.5 + 1e-3


def test_lawnmower_has_multiple_sweeps_and_alternating_heading():
    """生成多条扫线，并在每次 U 掉头后反向行驶"""
    path = generate_lawnmower(
        width=14.0,
        height=14.0,
        lane_spacing=4.0,
        turn_radius=2.0,
        margin=2.0,
        step=0.2,
    )

    ys = sorted({round(p.y, 1) for p in path})
    assert min(ys) == pytest.approx(-5.0)
    assert max(ys) == pytest.approx(3.0)
    assert len(ys) > 10  # 半圆掉头会产生连续 y，不只是几条离散线

    # 起点向右，终点在第三条扫线右端，仍朝 +x。
    assert path[0].yaw == pytest.approx(0.0)
    assert path[-1].yaw == pytest.approx(0.0, abs=1e-6)
    assert path[-1].x > path[0].x
    assert path[-1].y > path[0].y


def test_lawnmower_length_is_lines_plus_half_circle_turns():
    """总长度≈扫线长度×条数 + 半圆长度×掉头次数"""
    width = 12.0
    height = 10.0
    margin = 2.0
    radius = 2.0
    path = generate_lawnmower(
        width=width,
        height=height,
        lane_spacing=2.0 * radius,
        turn_radius=radius,
        margin=margin,
        step=0.05,
    )

    usable_width = width - 2.0 * margin - 2.0 * radius
    # usable_height=6, spacing=4 → 2 条扫线，1 次掉头；左右还要预留 R 给半圆外鼓
    expected = usable_width * 2 + 3.141592653589793 * radius
    assert path_length(path) == pytest.approx(expected, rel=0.001)


def test_lawnmower_rejects_lane_spacing_not_matching_turn_diameter():
    """K1 初版不做非半圆连接，避免生成不可解释曲线"""
    with pytest.raises(BoustrophedonError, match="lane_spacing == 2"):
        generate_lawnmower(
            width=12.0,
            height=10.0,
            lane_spacing=1.0,
            turn_radius=2.0,
            margin=2.0,
            step=0.2,
        )


def test_lawnmower_rejects_room_too_small_after_margin():
    """房间太小或 margin 过大时直接拒绝，不硬塞小半径路径"""
    with pytest.raises(BoustrophedonError, match="too small"):
        generate_lawnmower(
            width=5.0,
            height=5.0,
            lane_spacing=4.0,
            turn_radius=2.0,
            margin=2.2,
            step=0.2,
        )


def test_lawnmower_rejects_negative_or_zero_params():
    """参数错误要早报错"""
    with pytest.raises(BoustrophedonError, match="width"):
        generate_lawnmower(
            width=0.0,
            height=10.0,
            lane_spacing=4.0,
            turn_radius=2.0,
            margin=2.0,
        )
    with pytest.raises(BoustrophedonError, match="margin"):
        generate_lawnmower(
            width=12.0,
            height=10.0,
            lane_spacing=4.0,
            turn_radius=2.0,
            margin=-0.1,
        )


def test_lawnmower_output_passes_path_linter_gate():
    """闸门焊死:K1 生成的路径必须过 path_linter 四项(曲率/原地转/越界/穿障)。

    K1 若回归到不可执行(曲率超限、掉头外鼓越界等),这里直接挂——
    不可执行的路径不该流到远程实跑。
    """
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
    assert_path_executable(path, LintConfig(r_min=turn_radius, outer=outer))
