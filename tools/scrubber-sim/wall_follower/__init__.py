"""wall_follower — 反应式贴墙控制器 (M1-Mode 初始建图)

无 SLAM 地图时, 让洗地机贴外墙走一圈, 同时 slam_toolbox 增量建图。
走完一圈闭合 → map_to_polygons 提取外墙 polygon → 交给 M2 同心环覆盖。

纯算法层 (本模块, 可本地单测):
    scan_utils  — LaserScan 抽象 + 扇区距离 / 通行宽度
    follower    — P 控制律 + 通行闸门 + 事件识别
    closure     — 外圈闭合判定 (里程/转角/回起点)

ROS 节点层 (scripts/, 远程跑):
    wall_follower_node.py — sub /scan + /odom, pub /cmd_vel, 多步恢复子状态机

详见 spec §5.1: projects/自动洗地机/03_软件架构/specs/2026-06-25-同心轮廓覆盖引擎-设计.md
"""
from .scan_utils import (
    ScanData, normalize_angle, wall_distance,
    passable_width_ahead, front_clearance, min_range_in_sector,
)
from .follower import (
    FollowerParams, ControlCommand, WallEvent, compute_control,
)
from .closure import ClosureParams, ClosureTracker
from .area_builder import (
    polygon_area, polygon_perimeter, split_outer_voids,
    perimeter_check, to_area_yaml_dict,
)

__all__ = [
    "ScanData", "normalize_angle", "wall_distance",
    "passable_width_ahead", "front_clearance", "min_range_in_sector",
    "FollowerParams", "ControlCommand", "WallEvent", "compute_control",
    "ClosureParams", "ClosureTracker",
    "polygon_area", "polygon_perimeter", "split_outer_voids",
    "perimeter_check", "to_area_yaml_dict",
]
