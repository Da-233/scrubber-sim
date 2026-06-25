"""LaserScan 抽象 + 扇区最近距离 / 前方通行宽度

纯算法层, 不依赖 ROS。ROS 节点把 sensor_msgs/LaserScan 转成本模块的
ScanData 即可复用全部几何逻辑, 保持本地可单测的边界 (同 contour_coverage)。

坐标约定 (机器人体坐标系, 与 ROS REP-103 一致):
    x 向前, y 向左, theta 逆时针, beam 角 0 = 正前方
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


def normalize_angle(a: float) -> float:
    """归一化到 [-pi, pi)"""
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class ScanData:
    """LaserScan 的纯数据抽象

    ranges: 每个 beam 的距离 (m), inf / nan 表示无回波
    angle_min: 第 0 个 beam 的角度 (rad)
    angle_increment: 相邻 beam 角度差 (rad)
    range_max: 最大有效距离, 超过视作无障碍
    """
    ranges: List[float]
    angle_min: float
    angle_increment: float
    range_max: float = 12.0

    def beam_angle(self, i: int) -> float:
        return self.angle_min + i * self.angle_increment

    def valid_points(self) -> List[Tuple[float, float]]:
        """转成体坐标系 (x, y) 点列表, 跳过 inf/nan/超量程"""
        pts: List[Tuple[float, float]] = []
        for i, r in enumerate(self.ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= 0.0 or r > self.range_max:
                continue
            a = self.beam_angle(i)
            pts.append((r * math.cos(a), r * math.sin(a)))
        return pts


def min_range_in_sector(scan: ScanData, center_angle: float, half_width: float) -> Optional[float]:
    """扇区 [center-half, center+half] 内的最近有效距离

    Returns None 表示该扇区全无回波 (开阔)
    """
    best: Optional[float] = None
    for i, r in enumerate(scan.ranges):
        if r is None or math.isinf(r) or math.isnan(r):
            continue
        if r <= 0.0 or r > scan.range_max:
            continue
        a = normalize_angle(scan.beam_angle(i))
        d = abs(normalize_angle(a - center_angle))
        if d <= half_width:
            if best is None or r < best:
                best = r
    return best


def wall_distance(scan: ScanData, side: str, sector_half: float = math.radians(20)) -> Optional[float]:
    """车侧到墙的距离。side='right' 取 -90° 扇区, 'left' 取 +90°

    取扇区内最近距离作为贴墙距 (墙是连续面, 最近点代表当前侧距)
    """
    center = -math.pi / 2 if side == "right" else math.pi / 2
    return min_range_in_sector(scan, center, sector_half)


def passable_width_ahead(
    scan: ScanData,
    lookahead: float = 1.5,
    band: float = 0.25,
) -> Optional[float]:
    """前方 lookahead 处垂直于航向的通行宽度

    取所有落在 x ∈ [lookahead-band, lookahead+band] 的障碍点,
    找最近的左侧点 (y>0 最小) 和右侧点 (y<0 最大), 宽度 = y_left - y_right。

    Returns:
        通行宽度 (m); None 表示该处左右都开阔 (无墙夹峙, 不构成"堵")
    """
    left_y: Optional[float] = None    # 最靠中线的左障碍 y (>0 最小)
    right_y: Optional[float] = None   # 最靠中线的右障碍 y (<0 最大)
    for (x, y) in scan.valid_points():
        if abs(x - lookahead) > band:
            continue
        if y > 0:
            if left_y is None or y < left_y:
                left_y = y
        elif y < 0:
            if right_y is None or y > right_y:
                right_y = y
    if left_y is None and right_y is None:
        return None
    # 单侧有墙: 用另一侧开阔 (视作很宽), 不构成堵
    lo = right_y if right_y is not None else -scan.range_max
    hi = left_y if left_y is not None else scan.range_max
    return hi - lo


def front_clearance(scan: ScanData, half_width: float = math.radians(15)) -> Optional[float]:
    """正前方扇区最近距离 (凹角/迎面墙判定)"""
    return min_range_in_sector(scan, 0.0, half_width)
