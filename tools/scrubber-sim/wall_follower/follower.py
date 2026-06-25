"""反应式贴墙控制律 + 通行闸门 (M1-Mode Phase 1.A 核心)

纯算法层: 输入一帧 ScanData + 当前贴墙侧, 输出一个控制指令 (v, omega) +
事件标签。多步恢复动作 (后退 0.5m / 转 90°) 的时序由 ROS 节点的子状态机
驱动 (依赖里程, 非单帧可决), 本模块只做"单周期反应决策 + 事件识别"。

spec §5.1:
    d_target = 0.6m, v = 0.4m/s (建图保守速度), P + 前馈, 不上 PID
    通行宽度 < W_pass(1.4m) → 标"前方堵"
    凸角(前墙消失) → 前进+右转; 凹角(前<0.3m) → 原地转
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .scan_utils import (
    ScanData, wall_distance, passable_width_ahead, front_clearance,
)


class WallEvent(Enum):
    FOLLOW = "follow"              # 正常贴墙
    BLOCKED = "blocked"           # 前方通道 < W_pass, 需后退转向
    CONCAVE_CORNER = "concave"    # 凹角(内角): 正前方逼近 (< concave_dist)
    LOST_WALL = "lost_wall"       # 侧墙丢失/突增(含凸角外角), 朝墙侧转绕过去


@dataclass
class FollowerParams:
    d_target: float = 0.6          # 目标贴墙距 (m)
    v_nominal: float = 0.4         # 建图巡航速度 (m/s)
    kp: float = 1.2                # P 增益 (距离误差 → 角速度)
    w_pass: float = 1.4            # 最小可通行宽度 (m)
    concave_dist: float = 0.5      # 正前方逼近阈值 (m), < 此值判凹角
    lookahead: float = 1.5         # 通行宽度检查前瞻距 (m)
    max_omega: float = 0.8         # 角速度限幅 (rad/s)
    side: str = "right"            # 贴墙侧 right / left
    wall_lost_range: float = 3.0   # 侧墙超此距视为丢墙 (m)


@dataclass
class ControlCommand:
    v: float                       # 线速度 (m/s)
    omega: float                   # 角速度 (rad/s), >0 左转
    event: WallEvent
    wall_dist: Optional[float] = None   # 当前侧墙距 (诊断)
    front_dist: Optional[float] = None  # 正前方距 (诊断)
    passable: Optional[float] = None    # 前方通行宽度 (诊断)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def compute_control(scan: ScanData, params: FollowerParams = FollowerParams()) -> ControlCommand:
    """单帧反应决策

    优先级 (高→低):
      1. 凹角 (正前方逼近) → 原地转离墙
      2. 前方堵 (通行宽度 < W_pass) → 标 BLOCKED (节点接管后退转向)
      3. 丢墙/凸角 (侧墙太远/无) → 朝墙侧转绕过去
      4. 正常贴墙 P 控制
    """
    side = params.side
    sign = -1.0 if side == "right" else 1.0   # 右贴墙: 误差正(太远)需右转(omega<0)

    wd = wall_distance(scan, side)
    fc = front_clearance(scan)
    pw = passable_width_ahead(scan, lookahead=params.lookahead)

    # 1. 凹角: 正前方逼近 → 原地朝离墙方向转
    if fc is not None and fc < params.concave_dist:
        return ControlCommand(
            v=0.0, omega=_clamp(-sign * params.max_omega, -params.max_omega, params.max_omega),
            event=WallEvent.CONCAVE_CORNER, wall_dist=wd, front_dist=fc, passable=pw,
        )

    # 2. 前方堵: 通行宽度不足
    if pw is not None and pw < params.w_pass:
        return ControlCommand(
            v=0.0, omega=0.0,
            event=WallEvent.BLOCKED, wall_dist=wd, front_dist=fc, passable=pw,
        )

    # 3. 丢墙 / 凸角: 侧墙无回波或突增 (外角=墙拐走) → 朝墙侧缓转绕过去重新贴上
    #    凸角与丢墙是同一几何信号 (侧距突增), 统一处理: 前进 + 朝墙侧转, 自然绕外角
    if wd is None or wd > params.wall_lost_range:
        return ControlCommand(
            v=params.v_nominal * 0.5,
            omega=_clamp(sign * params.max_omega * 0.6, -params.max_omega, params.max_omega),
            event=WallEvent.LOST_WALL, wall_dist=wd, front_dist=fc, passable=pw,
        )

    # 4. 正常贴墙 P 控制: e>0 表示离墙太远, 需朝墙转
    e = wd - params.d_target
    omega = _clamp(sign * params.kp * e, -params.max_omega, params.max_omega)
    return ControlCommand(
        v=params.v_nominal, omega=omega,
        event=WallEvent.FOLLOW, wall_dist=wd, front_dist=fc, passable=pw,
    )
