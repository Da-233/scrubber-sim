"""同心环排序 + 密集化 + 段间连接 → pose 序列

输出: List[Pose] 喂 Nav2 FollowPath / 写 JSON
项目惯例 Pose = (x, y, yaw_or_None) (与 coverage_meter.py 一致)
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Sequence
from .geometry import Ring, Point


Pose = Tuple[float, float, Optional[float]]


@dataclass
class PathParams:
    """路径构造参数"""
    densify_step: float = 0.1      # 环上点间距 (m), Nav2 FollowPath 喜欢密集点
    add_yaw: bool = True           # 是否计算每点 yaw (基于相邻点)
    inner_to_outer: bool = False   # True: 从内圈向外螺旋 (机器人中心 spawn 时用,
                                   #   终点落外圈, 避免终点≈起点被 goal_checker 秒判到达)
    # 段间连接策略 P1 暂不实现 (P2 接入 Nav2 ComputePathToPose), 这里只输出 pose 序列


def densify_ring(ring: Ring, step: float) -> List[Point]:
    """把环上长边切成 step 长度的小段

    Args:
        ring: 输入环 (闭合, 末点 == 首点)
        step: 目标点间距 (m)
    Returns:
        密集化后的点列表 (闭合, 末点 == 首点)
    """
    coords = ring.coords
    if len(coords) < 2:
        return list(coords)

    densified: List[Point] = [coords[0]]
    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i+1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len <= step:
            densified.append((x1, y1))
            continue
        # 切成 ceil(seg_len / step) 份
        n = int(math.ceil(seg_len / step))
        for k in range(1, n + 1):
            t = k / n
            densified.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return densified


def sort_rings_outer_to_inner(rings: Sequence[Ring]) -> List[Ring]:
    """按 (depth, component_id) 升序排序"""
    return sorted(rings, key=lambda r: (r.depth, r.component_id))


def _compute_yaw(points: List[Point]) -> List[Pose]:
    """对每点计算 yaw (基于下一点方向; 末点延用倒数第二点)"""
    if len(points) < 2:
        return [(x, y, 0.0) for (x, y) in points]
    poses: List[Pose] = []
    for i in range(len(points)):
        if i < len(points) - 1:
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
        else:
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
        yaw = math.atan2(dy, dx)
        poses.append((points[i][0], points[i][1], yaw))
    return poses


def build_path(
    rings: Sequence[Ring],
    params: PathParams = PathParams(),
) -> List[Pose]:
    """主入口: 环列表 → 完整 pose 序列

    P1 实现: 简单拼接 (相邻环直连, 跨 component 也直连)
    P2 替换: 跨 component 调 Nav2 ComputePathToPose 避障连接 (spec §5.2 路径排序与连接)

    Returns:
        Pose 序列 [(x, y, yaw), ...] 不闭合 (最后一点是路径终点)
    """
    sorted_rings = sort_rings_outer_to_inner(rings)
    if params.inner_to_outer:
        sorted_rings = list(reversed(sorted_rings))  # 内圈先, 向外螺旋
    all_points: List[Point] = []
    for r in sorted_rings:
        dense = densify_ring(r, params.densify_step)
        # 闭合环的末点跟首点重复, 跳过末点 (避免相邻环之间有重复点)
        if dense and dense[0] == dense[-1]:
            dense = dense[:-1]
        all_points.extend(dense)
    if params.add_yaw:
        return _compute_yaw(all_points)
    else:
        return [(x, y, None) for (x, y) in all_points]
