"""外圈闭合判定 (M1-Mode Phase 1.A 退出条件)

spec §5.1 闭合判定 (三个全要满足):
    - 物理位置回起点 ±0.5m
    - 累计航向 ≥ 350° (防小空间转圈误判)
    - slam_toolbox 报告闭环 event (本模块不含, 由节点传入)

失败兜底: 累计里程 > 200m 未闭合 → 进 M3-Mode

纯算法: 喂入连续位姿 (x, y, yaw), 累积里程/转角, 判断是否满足几何闭合。
slam 闭环 event 作为外部布尔输入与几何条件 AND。
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from .scan_utils import normalize_angle


@dataclass
class ClosureParams:
    pos_tol: float = 0.5            # 回起点容差 (m)
    heading_tol: float = math.radians(350)  # 累计航向阈值 (rad)
    max_distance: float = 200.0    # 未闭合里程上限 (m) → 失败兜底
    min_distance: float = 2.0      # 起步保护: 走够这点才允许判闭合 (防原地起步即闭合)


@dataclass
class ClosureTracker:
    params: ClosureParams = field(default_factory=ClosureParams)
    _start: Optional[Tuple[float, float]] = None
    _last: Optional[Tuple[float, float, float]] = None
    cum_distance: float = 0.0      # 累计里程 (m)
    cum_heading: float = 0.0       # 累计转角绝对值 (rad)

    def update(self, x: float, y: float, yaw: float) -> None:
        """喂入一帧位姿, 累积里程和转角"""
        if self._start is None:
            self._start = (x, y)
            self._last = (x, y, yaw)
            return
        lx, ly, lyaw = self._last
        self.cum_distance += math.hypot(x - lx, y - ly)
        self.cum_heading += abs(normalize_angle(yaw - lyaw))
        self._last = (x, y, yaw)

    def dist_to_start(self) -> Optional[float]:
        if self._start is None or self._last is None:
            return None
        sx, sy = self._start
        lx, ly, _ = self._last
        return math.hypot(lx - sx, ly - sy)

    def is_closed(self, slam_loop_closed: bool = False) -> bool:
        """三条件 AND: 回起点 + 累计航向够 + slam 闭环

        slam_loop_closed 由节点从 slam_toolbox 事件传入; 仿真早期可放宽为 True
        """
        if self.cum_distance < self.params.min_distance:
            return False
        d = self.dist_to_start()
        if d is None or d > self.params.pos_tol:
            return False
        if self.cum_heading < self.params.heading_tol:
            return False
        return slam_loop_closed

    def is_failed(self) -> bool:
        """里程超限仍未闭合 → 失败兜底 (进 M3)"""
        return self.cum_distance > self.params.max_distance
