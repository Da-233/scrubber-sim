"""wall_follower 单元测试 — 合成雷达数据测控制律/通行宽度/闭合判定"""
import math
import pytest

from wall_follower.scan_utils import (
    ScanData, normalize_angle, wall_distance,
    passable_width_ahead, front_clearance, min_range_in_sector,
)
from wall_follower.follower import (
    FollowerParams, WallEvent, compute_control,
)
from wall_follower.closure import ClosureParams, ClosureTracker


# ---------------------------------------------------------------------------
# 合成 scan 工具
# ---------------------------------------------------------------------------

def make_scan(beams, angle_min=-math.pi, angle_increment=math.radians(1), range_max=12.0):
    """beams: dict {角度(deg): 距离(m)} 或 list[距离]。其余 beam 填 inf"""
    n = int(round(2 * math.pi / angle_increment))
    ranges = [float("inf")] * n
    if isinstance(beams, dict):
        for deg, r in beams.items():
            idx = int(round((math.radians(deg) - angle_min) / angle_increment)) % n
            ranges[idx] = r
    else:
        ranges = list(beams)
    return ScanData(ranges=ranges, angle_min=angle_min,
                    angle_increment=angle_increment, range_max=range_max)


# ---------------------------------------------------------------------------
# normalize_angle
# ---------------------------------------------------------------------------

def test_normalize_angle():
    assert normalize_angle(0) == pytest.approx(0)
    assert normalize_angle(math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)


# ---------------------------------------------------------------------------
# scan_utils
# ---------------------------------------------------------------------------

def test_wall_distance_right():
    """右侧 -90° 有墙 0.6m"""
    scan = make_scan({-90: 0.6})
    assert wall_distance(scan, "right") == pytest.approx(0.6, abs=0.05)


def test_wall_distance_left():
    scan = make_scan({90: 0.8})
    assert wall_distance(scan, "left") == pytest.approx(0.8, abs=0.05)


def test_wall_distance_none_when_open():
    """侧面无回波 → None"""
    scan = make_scan({})
    assert wall_distance(scan, "right") is None


def test_front_clearance():
    scan = make_scan({0: 1.2})
    assert front_clearance(scan) == pytest.approx(1.2, abs=0.05)


def test_min_range_sector_picks_nearest():
    scan = make_scan({-90: 1.0, -85: 0.5, -95: 0.8})
    # 扇区取最近 0.5
    assert min_range_in_sector(scan, -math.pi / 2, math.radians(20)) == pytest.approx(0.5, abs=0.05)


def test_passable_width_corridor():
    """前方 1.5m 处左墙 y=+0.8, 右墙 y=-0.8 → 宽 1.6m"""
    # 左墙: 角度 atan2(0.8, 1.5)≈28°, 距离 hypot(1.5,0.8)=1.7
    aL = math.degrees(math.atan2(0.8, 1.5))
    aR = math.degrees(math.atan2(-0.8, 1.5))
    rL = math.hypot(1.5, 0.8)
    scan = make_scan({round(aL): rL, round(aR): rL})
    w = passable_width_ahead(scan, lookahead=1.5, band=0.3)
    assert w == pytest.approx(1.6, abs=0.2)


def test_passable_width_open_returns_none():
    scan = make_scan({})
    assert passable_width_ahead(scan) is None


# ---------------------------------------------------------------------------
# follower 控制律
# ---------------------------------------------------------------------------

def test_follow_on_target_distance_low_omega():
    """右墙正好 0.6m → omega 接近 0, 直行"""
    scan = make_scan({-90: 0.6, 0: 5.0})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.event == WallEvent.FOLLOW
    assert cmd.v == pytest.approx(0.4)
    assert abs(cmd.omega) < 0.05


def test_follow_too_far_turns_toward_wall():
    """右墙 1.0m (>0.6 太远) → 应朝右转 (omega < 0)"""
    scan = make_scan({-90: 1.0, 0: 5.0})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.event == WallEvent.FOLLOW
    assert cmd.omega < 0


def test_follow_too_close_turns_away():
    """右墙 0.3m (<0.6 太近) → 应朝左转离墙 (omega > 0)"""
    scan = make_scan({-90: 0.3, 0: 5.0})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.omega > 0


def test_concave_corner_stops_and_turns():
    """正前方 0.3m (<0.5) → 凹角, 停 + 转"""
    scan = make_scan({-90: 0.6, 0: 0.3})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.event == WallEvent.CONCAVE_CORNER
    assert cmd.v == 0.0
    assert abs(cmd.omega) > 0


def test_blocked_when_narrow():
    """前方 1.5m 通道宽 1.0m (<1.4) → BLOCKED"""
    aL = math.degrees(math.atan2(0.5, 1.5))
    aR = math.degrees(math.atan2(-0.5, 1.5))
    r = math.hypot(1.5, 0.5)
    scan = make_scan({-90: 0.6, round(aL): r, round(aR): r})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.event == WallEvent.BLOCKED
    assert cmd.v == 0.0


def test_lost_wall_searches():
    """侧墙无回波 + 前方开阔 → LOST_WALL, 朝墙侧转找"""
    scan = make_scan({0: 5.0})
    cmd = compute_control(scan, FollowerParams(side="right"))
    assert cmd.event == WallEvent.LOST_WALL
    # 右贴墙丢墙 → 朝右找 (omega < 0)
    assert cmd.omega < 0


def test_left_side_mirror():
    """左贴墙太远 → 朝左转 (omega > 0)"""
    scan = make_scan({90: 1.0, 0: 5.0})
    cmd = compute_control(scan, FollowerParams(side="left"))
    assert cmd.event == WallEvent.FOLLOW
    assert cmd.omega > 0


def test_omega_clamped():
    """墙距误差极大 → omega 限幅"""
    scan = make_scan({-90: 2.9, 0: 5.0})  # 2.9 < wall_lost 3.0, 仍 FOLLOW
    p = FollowerParams(side="right", max_omega=0.8)
    cmd = compute_control(scan, p)
    assert abs(cmd.omega) <= 0.8 + 1e-9


# ---------------------------------------------------------------------------
# closure 闭合判定
# ---------------------------------------------------------------------------

def test_closure_not_closed_at_start():
    t = ClosureTracker()
    t.update(0, 0, 0)
    assert not t.is_closed(slam_loop_closed=True)


def test_closure_full_loop():
    """走一个方形回到起点, 累计航向 360°, slam 闭环 → 闭合"""
    t = ClosureTracker()
    # 方形: (0,0)→(5,0)→(5,5)→(0,5)→(0,0), yaw 每边转 90°
    path = [
        (0, 0, 0),
        (5, 0, 0),
        (5, 0, math.pi / 2),
        (5, 5, math.pi / 2),
        (5, 5, math.pi),
        (0, 5, math.pi),
        (0, 5, -math.pi / 2),
        (0, 0, -math.pi / 2),
        (0, 0, 0),
    ]
    for (x, y, yaw) in path:
        t.update(x, y, yaw)
    assert t.dist_to_start() == pytest.approx(0, abs=0.01)
    assert t.cum_heading >= math.radians(350)
    assert t.is_closed(slam_loop_closed=True)
    # slam 没报闭环 → 不算闭合
    assert not t.is_closed(slam_loop_closed=False)


def test_closure_min_distance_guard():
    """原地起步不算闭合 (min_distance 保护)"""
    t = ClosureTracker(ClosureParams(min_distance=2.0))
    t.update(0, 0, 0)
    t.update(0.1, 0, 0)
    assert not t.is_closed(slam_loop_closed=True)


def test_closure_failed_over_max_distance():
    t = ClosureTracker(ClosureParams(max_distance=10.0))
    # 走一条 > 10m 的直线不回头
    for i in range(20):
        t.update(i, 0, 0)
    assert t.is_failed()


def test_closure_distance_accumulates():
    t = ClosureTracker()
    t.update(0, 0, 0)
    t.update(3, 4, 0)   # +5
    t.update(3, 4, 0)   # +0
    assert t.cum_distance == pytest.approx(5.0)
