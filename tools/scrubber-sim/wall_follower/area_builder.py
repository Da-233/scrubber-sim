"""M1→M2 衔接: polygon 列表 → outer + voids 拆分 + 合理性检查

map_to_polygons 用 include_outer_boundary=True 提取后, 外墙和障碍混在一个
List[Polygon] 里不区分。coverage_meter.load_area 期望的是 outer + voids 分开
的 schema。本模块补上这一步 (纯算法, 可本地单测), 让 M1→M2 全自动:

    最大面积的 polygon = 外墙 (outer)
    其余 = 内部障碍 (voids)

并实现 spec §5.1 Phase 1.B 的合理性检查:
    外墙周长 vs M1 累计里程, 误差 < 20% → 放行 M2, 否则进 M3

类型沿用项目惯例 Polygon = List[(x, y)]。
"""
from __future__ import annotations
import math
from typing import List, Tuple, Sequence, Optional

Point = Tuple[float, float]
Polygon = List[Point]


def polygon_area(poly: Sequence[Point]) -> float:
    """shoelace 面积 (绝对值, m²)。poly 可闭合可不闭合"""
    n = len(poly)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return abs(s) / 2.0


def polygon_perimeter(poly: Sequence[Point]) -> float:
    """闭合周长 (m)"""
    n = len(poly)
    if n < 2:
        return 0.0
    p = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        p += math.hypot(x1 - x0, y1 - y0)
    return p


def split_outer_voids(polygons: Sequence[Polygon]) -> Tuple[Polygon, List[Polygon]]:
    """最大面积 polygon 当外墙, 其余当 voids

    Returns:
        (outer, voids); polygons 为空 → ([], [])
    Raises:
        ValueError: 只有不足 3 点的退化输入
    """
    valid = [p for p in polygons if len(p) >= 3]
    if not valid:
        return [], []
    areas = [polygon_area(p) for p in valid]
    outer_idx = max(range(len(valid)), key=lambda i: areas[i])
    outer = valid[outer_idx]
    voids = [valid[i] for i in range(len(valid)) if i != outer_idx]
    return outer, voids


def perimeter_check(
    outer: Sequence[Point],
    m1_distance: float,
    tol: float = 0.20,
) -> Tuple[bool, float]:
    """Phase 1.B 合理性检查: 外墙周长 vs M1 累计里程

    Args:
        outer: 外墙 polygon
        m1_distance: wall_follower 闭合时的累计里程 (m)
        tol: 相对误差容差, 默认 0.20 (20%)
    Returns:
        (通过?, 相对误差)。m1_distance<=0 → (False, inf)
    """
    if m1_distance <= 0:
        return False, float("inf")
    peri = polygon_perimeter(outer)
    rel_err = abs(peri - m1_distance) / m1_distance
    return rel_err <= tol, rel_err


def to_area_yaml_dict(outer: Polygon, voids: List[Polygon]) -> dict:
    """转成 coverage_meter.load_area 认的 outer/voids schema dict

    yaml.safe_dump 这个 dict 即得 area.yaml
    """
    return {
        "outer": [[float(x), float(y)] for (x, y) in outer],
        "voids": [[[float(x), float(y)] for (x, y) in poly] for poly in voids],
    }
