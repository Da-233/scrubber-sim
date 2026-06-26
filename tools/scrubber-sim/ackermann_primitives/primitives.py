"""Ackermann 运动原语与曲率检查

K0 只做纯几何路径生成，不依赖 ROS：
- 直线段
- 固定半径圆弧
- 半圆 U 掉头
- 离散路径曲率估计与约束检查

约定：
- 坐标单位 m，yaw 单位 rad
- yaw=0 表示朝 +x
- turn direction: left 为正曲率，right 为负曲率
- 返回路径包含起点和终点
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal


TurnDirection = Literal["left", "right"]


@dataclass(frozen=True)
class Pose2D:
    """二维位姿"""

    x: float
    y: float
    yaw: float


class CurvatureViolation(ValueError):
    """路径曲率超过阿卡曼底盘可执行上限"""


class InvalidPrimitive(ValueError):
    """运动原语参数无效"""


def normalize_angle(angle: float) -> float:
    """归一化到 [-pi, pi)"""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _validate_step(step: float) -> None:
    if step <= 0.0:
        raise InvalidPrimitive(f"step must be > 0, got {step}")


def _num_segments(length: float, step: float) -> int:
    """按最大步长切段，保证至少 1 段"""

    return max(1, int(math.ceil(abs(length) / step)))


def sample_line(start: Pose2D, length: float, step: float = 0.1) -> list[Pose2D]:
    """从 start 沿当前 yaw 采样直线段

    length 可为负；负值表示沿 yaw 反方向倒退采样。K0/K1 主线不用倒车，
    但这里保留几何能力，方便测试和后续分析。
    """

    _validate_step(step)
    segments = _num_segments(length, step)
    poses: list[Pose2D] = []
    for i in range(segments + 1):
        s = length * i / segments
        poses.append(Pose2D(
            x=start.x + s * math.cos(start.yaw),
            y=start.y + s * math.sin(start.yaw),
            yaw=normalize_angle(start.yaw),
        ))
    return poses


def sample_arc(
    start: Pose2D,
    radius: float,
    angle: float,
    step: float = 0.1,
    direction: TurnDirection = "left",
) -> list[Pose2D]:
    """从 start 采样固定半径圆弧

    Args:
        start: 起点位姿。
        radius: 圆弧半径，必须 > 0。
        angle: 圆弧扫过角，必须 >= 0。半圆 U 掉头为 pi。
        step: 相邻采样点最大弧长。
        direction: left 或 right。

    Returns:
        包含起点和终点的 Pose2D 列表。
    """

    _validate_step(step)
    if radius <= 0.0:
        raise InvalidPrimitive(f"radius must be > 0, got {radius}")
    if angle < 0.0:
        raise InvalidPrimitive(f"angle must be >= 0, got {angle}")
    if direction not in ("left", "right"):
        raise InvalidPrimitive(f"direction must be 'left' or 'right', got {direction!r}")

    sign = 1.0 if direction == "left" else -1.0
    arc_length = radius * angle
    segments = _num_segments(arc_length, step)

    # 圆心在车辆左/右法线方向 radius 处。
    cx = start.x - sign * radius * math.sin(start.yaw)
    cy = start.y + sign * radius * math.cos(start.yaw)

    poses: list[Pose2D] = []
    for i in range(segments + 1):
        theta = sign * angle * i / segments
        yaw = normalize_angle(start.yaw + theta)
        # 由圆心 + 当前 yaw 对应的半径向量还原车体位置。
        x = cx + sign * radius * math.sin(yaw)
        y = cy - sign * radius * math.cos(yaw)
        poses.append(Pose2D(x=x, y=y, yaw=yaw))
    return poses


def sample_u_turn(
    start: Pose2D,
    radius: float,
    step: float = 0.1,
    direction: TurnDirection = "left",
) -> list[Pose2D]:
    """采样半圆 U 掉头"""

    return sample_arc(start, radius=radius, angle=math.pi, step=step, direction=direction)


def path_length(poses: list[Pose2D]) -> float:
    """计算离散路径折线长度"""

    if len(poses) < 2:
        return 0.0
    return sum(
        math.hypot(poses[i + 1].x - poses[i].x, poses[i + 1].y - poses[i].y)
        for i in range(len(poses) - 1)
    )


def _curvature_from_three_points(a: Pose2D, b: Pose2D, c: Pose2D) -> float:
    """三点外接圆曲率估计；近似共线返回 0"""

    ab = math.hypot(b.x - a.x, b.y - a.y)
    bc = math.hypot(c.x - b.x, c.y - b.y)
    ca = math.hypot(a.x - c.x, a.y - c.y)
    if ab == 0.0 or bc == 0.0 or ca == 0.0:
        return 0.0

    cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    area2 = abs(cross)
    if area2 < 1e-12:
        return 0.0

    # 三角形面积 A = area2 / 2，外接圆半径 R = abc / (4A)
    # 曲率 1/R = 4A/(abc) = 2*area2/(abc)
    return 2.0 * area2 / (ab * bc * ca)


def curvatures(poses: list[Pose2D]) -> list[float]:
    """返回相邻 pose 的离散曲率绝对值

    Pose2D 已带 yaw，因此用 |Δyaw| / Δs 估计曲率。
    这比三点外接圆更适合 K1 这种“直线 + 圆弧”C1 连续路径：
    直线接圆弧处曲率从 0 跳到 1/R，但车辆 yaw 是连续的；三点窗口跨越
    直线/圆弧边界会把这个曲率跳变误估成极大尖峰。
    """

    if len(poses) < 2:
        return []
    values: list[float] = []
    for a, b in zip(poses[:-1], poses[1:]):
        ds = math.hypot(b.x - a.x, b.y - a.y)
        if ds == 0.0:
            continue
        values.append(abs(normalize_angle(b.yaw - a.yaw)) / ds)
    return values


def max_curvature(poses: list[Pose2D]) -> float:
    """路径最大离散曲率绝对值"""

    values = curvatures(poses)
    return max(values) if values else 0.0


def assert_curvature_within(
    poses: list[Pose2D],
    max_allowed: float,
    tolerance: float = 1e-6,
) -> None:
    """断言路径曲率不超过 max_allowed

    max_allowed 常取 1/R_min 或 1/R_safe。
    """

    if max_allowed < 0.0:
        raise InvalidPrimitive(f"max_allowed must be >= 0, got {max_allowed}")
    actual = max_curvature(poses)
    if actual > max_allowed + tolerance:
        raise CurvatureViolation(
            f"max curvature {actual:.6f} exceeds allowed {max_allowed:.6f}"
        )
