"""O0 底盘/odom 标定分析工具

本模块只做纯数据分析，不依赖 ROS：
- 读取 odom/truth CSV 轨迹
- 按时间戳线性插值对齐
- 计算 odom-vs-truth 误差
- 拟合转弯半径

CSV 兼容列名：
- 时间: t / time / stamp / timestamp，可缺省；缺省时用行号作时间
- 位姿: x, y, theta 或 x, y, yaw
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from .primitives import normalize_angle


Row = tuple[float, float, float]


@dataclass(frozen=True)
class TimedPose2D:
    """带时间戳的二维位姿"""

    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryError:
    """两条已对齐轨迹的误差摘要"""

    count: int
    endpoint_xy_error: float
    max_xy_error: float
    mean_xy_error: float
    max_yaw_error: float
    mean_yaw_error: float


@dataclass(frozen=True)
class TurnRadiusEstimate:
    """转弯半径估计结果"""

    circle_radius: float
    distance_heading_radius: float
    distance: float
    heading_change: float


def _first_present(row: dict[str, str], names: tuple[str, ...]) -> str | None:
    for name in names:
        if name in row and row[name] != "":
            return row[name]
    return None


def load_xytheta_csv(path: str | Path) -> list[Row]:
    """读取 x/y/theta CSV，返回无时间戳 tuple

    兼容 `theta` 和 `yaw` 两种航向列名。
    """

    return [(p.x, p.y, p.yaw) for p in load_timed_xytheta_csv(path)]


def load_timed_xytheta_csv(path: str | Path) -> list[TimedPose2D]:
    """读取带时间戳的 x/y/yaw CSV

    如果没有时间列，用行号 0,1,2... 作为时间，便于单元测试和离线 CSV。
    """

    rows: list[TimedPose2D] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            t_raw = _first_present(row, ("t", "time", "stamp", "timestamp"))
            yaw_raw = _first_present(row, ("theta", "yaw"))
            if yaw_raw is None:
                raise ValueError("CSV must contain theta or yaw column")
            rows.append(TimedPose2D(
                t=float(t_raw) if t_raw is not None else float(idx),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(yaw_raw),
            ))
    if not rows:
        raise ValueError(f"empty trajectory: {path}")
    return rows


def trajectory_distance(rows: Iterable[Row] | Iterable[TimedPose2D]) -> float:
    """计算轨迹折线长度"""

    pts = list(rows)
    total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        ax, ay = _xy(a)
        bx, by = _xy(b)
        total += math.hypot(bx - ax, by - ay)
    return total


def endpoint_error(a: list[Row] | list[TimedPose2D], b: list[Row] | list[TimedPose2D]) -> float:
    """两条轨迹末端 XY 偏差"""

    if not a or not b:
        raise ValueError("endpoint_error requires two non-empty trajectories")
    ax, ay = _xy(a[-1])
    bx, by = _xy(b[-1])
    return math.hypot(ax - bx, ay - by)


def _xy(row: Row | TimedPose2D) -> tuple[float, float]:
    if isinstance(row, TimedPose2D):
        return row.x, row.y
    return row[0], row[1]


def _yaw(row: Row | TimedPose2D) -> float:
    if isinstance(row, TimedPose2D):
        return row.yaw
    return row[2]


def _as_row(row: Row | TimedPose2D) -> Row:
    if isinstance(row, TimedPose2D):
        return row.x, row.y, row.yaw
    return row


def _unwrap_delta(a: float, b: float) -> float:
    return normalize_angle(b - a)


def _unwrap_yaws(yaws: list[float]) -> list[float]:
    if not yaws:
        return []
    out = [yaws[0]]
    for yaw in yaws[1:]:
        out.append(out[-1] + _unwrap_delta(out[-1], yaw))
    return out


def interpolate_pose(traj: list[TimedPose2D], t: float) -> TimedPose2D:
    """在轨迹上按时间线性插值一个位姿"""

    if not traj:
        raise ValueError("trajectory is empty")
    if t < traj[0].t or t > traj[-1].t:
        raise ValueError(f"time {t} outside trajectory range [{traj[0].t}, {traj[-1].t}]")
    if t == traj[0].t:
        return traj[0]
    if t == traj[-1].t:
        return traj[-1]

    for a, b in zip(traj[:-1], traj[1:]):
        if a.t <= t <= b.t:
            if b.t == a.t:
                return a
            r = (t - a.t) / (b.t - a.t)
            dyaw = _unwrap_delta(a.yaw, b.yaw)
            return TimedPose2D(
                t=t,
                x=a.x + (b.x - a.x) * r,
                y=a.y + (b.y - a.y) * r,
                yaw=normalize_angle(a.yaw + dyaw * r),
            )
    # 浮点边界兜底
    return traj[-1]


def align_by_time(
    reference: list[TimedPose2D],
    target: list[TimedPose2D],
) -> tuple[list[TimedPose2D], list[TimedPose2D]]:
    """把 target 插值到 reference 的时间戳上

    只保留两条轨迹共同时间范围内的 reference 点。
    """

    if not reference or not target:
        raise ValueError("align_by_time requires two non-empty trajectories")
    start = max(reference[0].t, target[0].t)
    end = min(reference[-1].t, target[-1].t)
    if start > end:
        raise ValueError("trajectories do not overlap in time")

    ref_aligned: list[TimedPose2D] = []
    target_aligned: list[TimedPose2D] = []
    for p in reference:
        if start <= p.t <= end:
            ref_aligned.append(p)
            target_aligned.append(interpolate_pose(target, p.t))
    if not ref_aligned:
        raise ValueError("no reference samples in overlapping time range")
    return ref_aligned, target_aligned


def compute_trajectory_error(
    truth: list[TimedPose2D],
    odom: list[TimedPose2D],
) -> TrajectoryError:
    """按 truth 时间戳对齐 odom，并计算误差摘要"""

    truth_a, odom_a = align_by_time(truth, odom)
    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    for gt, od in zip(truth_a, odom_a):
        xy_errors.append(math.hypot(od.x - gt.x, od.y - gt.y))
        yaw_errors.append(abs(_unwrap_delta(gt.yaw, od.yaw)))

    return TrajectoryError(
        count=len(xy_errors),
        endpoint_xy_error=xy_errors[-1],
        max_xy_error=max(xy_errors),
        mean_xy_error=sum(xy_errors) / len(xy_errors),
        max_yaw_error=max(yaw_errors),
        mean_yaw_error=sum(yaw_errors) / len(yaw_errors),
    )


def fit_circle_radius(rows: list[Row] | list[TimedPose2D]) -> float:
    """用代数最小二乘拟合圆半径"""

    pts = [_as_row(r) for r in rows]
    if len(pts) < 3:
        raise ValueError("need at least three points to fit a circle")

    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    n = float(len(pts))
    for x, y, _ in pts:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sxz += x * z
        syz += y * z
        sz += z

    mat = [
        [sxx, sxy, sx, sxz],
        [sxy, syy, sy, syz],
        [sx, sy, n, sz],
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(mat[r][col]))
        if abs(mat[pivot][col]) < 1e-12:
            raise ValueError("degenerate circle fit")
        mat[col], mat[pivot] = mat[pivot], mat[col]
        scale = mat[col][col]
        for j in range(col, 4):
            mat[col][j] /= scale
        for r in range(3):
            if r == col:
                continue
            factor = mat[r][col]
            for j in range(col, 4):
                mat[r][j] -= factor * mat[col][j]

    a, b, c = mat[0][3], mat[1][3], mat[2][3]
    cx = a / 2.0
    cy = b / 2.0
    return math.sqrt(max(0.0, c + cx * cx + cy * cy))


def fit_arc_radius_by_distance_and_heading(rows: list[Row] | list[TimedPose2D]) -> float:
    """用弧长 / 航向变化估计半径"""

    pts = list(rows)
    if len(pts) < 2:
        raise ValueError("need at least two points")
    distance = trajectory_distance(pts)
    yaws = _unwrap_yaws([_yaw(p) for p in pts])
    dtheta = yaws[-1] - yaws[0]
    if abs(dtheta) < 1e-9:
        raise ValueError("heading change is too small for arc radius")
    return abs(distance / dtheta)


def estimate_turn_radius(rows: list[Row] | list[TimedPose2D]) -> TurnRadiusEstimate:
    """同时给出两种半径估计，便于 O0 对照诊断"""

    pts = list(rows)
    yaws = _unwrap_yaws([_yaw(p) for p in pts])
    heading_change = yaws[-1] - yaws[0]
    distance = trajectory_distance(pts)
    return TurnRadiusEstimate(
        circle_radius=fit_circle_radius(pts),
        distance_heading_radius=fit_arc_radius_by_distance_and_heading(pts),
        distance=distance,
        heading_change=heading_change,
    )
