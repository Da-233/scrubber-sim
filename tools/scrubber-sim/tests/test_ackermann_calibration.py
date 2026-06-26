"""O0 Ackermann 标定分析测试"""
import csv
import math

import pytest

from ackermann_primitives.calibration import (
    TimedPose2D,
    align_by_time,
    compute_trajectory_error,
    endpoint_error,
    estimate_turn_radius,
    fit_arc_radius_by_distance_and_heading,
    fit_circle_radius,
    interpolate_pose,
    load_timed_xytheta_csv,
    load_xytheta_csv,
    trajectory_distance,
)


def write_csv(path, rows, header=("x", "y", "theta")):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def quarter_circle_rows(radius: float = 2.0, count: int = 40):
    rows = []
    for i in range(count):
        a = (math.pi / 2) * i / (count - 1)
        rows.append((radius * math.sin(a), radius * (1.0 - math.cos(a)), a))
    return rows


def test_load_xytheta_csv_without_time_uses_row_order(tmp_path):
    """无时间列也能读取，兼容简单离线 CSV"""
    p = tmp_path / "traj.csv"
    write_csv(p, [(0, 0, 0), (1.5, 0.2, 0.1)])

    rows = load_xytheta_csv(p)
    timed = load_timed_xytheta_csv(p)

    assert rows == [(0.0, 0.0, 0.0), (1.5, 0.2, 0.1)]
    assert [r.t for r in timed] == [0.0, 1.0]


def test_load_timed_csv_accepts_time_and_yaw_columns(tmp_path):
    """兼容远程 recorder 常见列名 time/yaw"""
    p = tmp_path / "traj.csv"
    write_csv(
        p,
        [(10.0, 0.0, 0.0, 0.0), (10.5, 1.0, 0.0, 0.2)],
        header=("time", "x", "y", "yaw"),
    )

    rows = load_timed_xytheta_csv(p)

    assert rows[0] == TimedPose2D(t=10.0, x=0.0, y=0.0, yaw=0.0)
    assert rows[1] == TimedPose2D(t=10.5, x=1.0, y=0.0, yaw=0.2)


def test_trajectory_distance_for_two_segments():
    """3-4-5 两段，总长 10"""
    rows = [(0, 0, 0), (3, 4, 0), (6, 8, 0)]

    assert trajectory_distance(rows) == pytest.approx(10.0)


def test_endpoint_error_between_two_trajectories():
    """末端误差只比较最后一个点的 XY"""
    a = [(0, 0, 0), (1, 1, 0)]
    b = [(0, 0, 0), (4, 5, 0)]

    assert endpoint_error(a, b) == pytest.approx(5.0)


def test_interpolate_pose_wraps_yaw_short_way():
    """航向跨 pi 边界时按短弧插值，不绕远路"""
    traj = [
        TimedPose2D(0.0, 0.0, 0.0, math.radians(170)),
        TimedPose2D(1.0, 1.0, 0.0, math.radians(-170)),
    ]

    mid = interpolate_pose(traj, 0.5)

    assert mid.x == pytest.approx(0.5)
    assert abs(abs(mid.yaw) - math.pi) < math.radians(1)


def test_align_by_time_interpolates_target_to_reference_times():
    """odom 采样较稀疏时，插值到 truth 时间戳"""
    truth = [
        TimedPose2D(0.0, 0.0, 0.0, 0.0),
        TimedPose2D(0.5, 0.5, 0.0, 0.0),
        TimedPose2D(1.0, 1.0, 0.0, 0.0),
    ]
    odom = [
        TimedPose2D(0.0, 0.0, 0.0, 0.0),
        TimedPose2D(1.0, 2.0, 0.0, 0.0),
    ]

    ref, aligned = align_by_time(truth, odom)

    assert [p.t for p in ref] == [0.0, 0.5, 1.0]
    assert [p.x for p in aligned] == pytest.approx([0.0, 1.0, 2.0])


def test_compute_trajectory_error_summary():
    """误差摘要包含末端、最大、平均 XY/yaw 误差"""
    truth = [
        TimedPose2D(0.0, 0.0, 0.0, 0.0),
        TimedPose2D(1.0, 1.0, 0.0, 0.0),
        TimedPose2D(2.0, 2.0, 0.0, 0.0),
    ]
    odom = [
        TimedPose2D(0.0, 0.0, 0.0, 0.0),
        TimedPose2D(1.0, 2.0, 0.0, 0.1),
        TimedPose2D(2.0, 4.0, 0.0, 0.2),
    ]

    err = compute_trajectory_error(truth, odom)

    assert err.count == 3
    assert err.endpoint_xy_error == pytest.approx(2.0)
    assert err.max_xy_error == pytest.approx(2.0)
    assert err.mean_xy_error == pytest.approx(1.0)
    assert err.max_yaw_error == pytest.approx(0.2)
    assert err.mean_yaw_error == pytest.approx(0.1)


def test_fit_circle_radius_for_quarter_circle():
    """圆拟合能从四分之一圆点列恢复半径"""
    rows = quarter_circle_rows(radius=2.0, count=50)

    assert fit_circle_radius(rows) == pytest.approx(2.0, rel=0.02)


def test_fit_arc_radius_by_distance_and_heading():
    """弧长 / 航向变化估计半径"""
    rows = quarter_circle_rows(radius=3.0, count=80)

    assert fit_arc_radius_by_distance_and_heading(rows) == pytest.approx(3.0, rel=0.02)


def test_estimate_turn_radius_reports_both_methods():
    """O0 报告同时给圆拟合半径和弧长航向半径，便于诊断不一致"""
    rows = quarter_circle_rows(radius=2.5, count=60)

    estimate = estimate_turn_radius(rows)

    assert estimate.circle_radius == pytest.approx(2.5, rel=0.02)
    assert estimate.distance_heading_radius == pytest.approx(2.5, rel=0.02)
    assert estimate.distance == pytest.approx(2.5 * math.pi / 2, rel=0.001)
    assert estimate.heading_change == pytest.approx(math.pi / 2)


def test_degenerate_circle_fit_rejected():
    """共线点无法拟合圆，要明确报错"""
    with pytest.raises(ValueError, match="degenerate"):
        fit_circle_radius([(0, 0, 0), (1, 0, 0), (2, 0, 0)])


def test_arc_radius_rejects_too_small_heading_change():
    """直线轨迹不能用弧长/航向估半径"""
    with pytest.raises(ValueError, match="heading change"):
        fit_arc_radius_by_distance_and_heading([(0, 0, 0), (1, 0, 0)])
