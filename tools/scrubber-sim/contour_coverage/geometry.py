"""同心轮廓偏移核心 — 带洞多边形递归 buffer(-spacing)

约定:
- 输入/输出对外接口使用项目惯例 Polygon = List[Tuple[float, float]]
  (与 coverage_meter / map_to_polygons 一致)
- 内部计算使用 shapely.geometry.Polygon 做几何运算
- 全程世界坐标 (米), 不涉及像素
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from shapely.geometry import Polygon as ShPolygon, MultiPolygon, LineString, GeometryCollection
from shapely.ops import unary_union

# 项目惯例类型别名 (与 coverage_meter.py 一致)
Point = Tuple[float, float]
Polygon = List[Point]


@dataclass
class OffsetParams:
    """同心环生成参数"""
    spacing: float = 0.55          # 环间距 (有效刷宽, 默认 0.55 = 刷盘 0.6 × 92% overlap)
    safety_margin: float = 0.6     # 安全 margin (半车宽 0.5 + 余量 = 0.6)
    join_style: str = "round"      # 拐角风格: 'mitre'/'round'/'bevel'. round = 平滑但角上微漏
                                   # 阿卡曼转不了直角, 默认 round (spec §10)
    max_rings: int = 100           # 防爆环 (递归上限)
    min_ring_length: float = 0.3   # 太短的环跳过 (米)


@dataclass
class Ring:
    """一圈贴边路径 (闭合曲线, 已是 m 单位)"""
    coords: List[Point]            # [(x0,y0), (x1,y1), ..., (x0,y0)] 闭合
    depth: int                     # 第几圈, 外圈 0, 向内递增
    component_id: int              # 区域分裂后的连通块编号 (外圈一块, 障碍切开后多块)


def _to_shapely(outer: Polygon, voids: List[Polygon]) -> ShPolygon:
    """项目 Polygon (List[Point]) → shapely Polygon (with holes)

    shapely 不要求末点重复首点 (它自己闭合), 这里去掉重复末点
    """
    if outer and outer[0] == outer[-1]:
        outer = outer[:-1]
    closed_voids = []
    for v in voids:
        if v and v[0] == v[-1]:
            v = v[:-1]
        closed_voids.append(v)
    return ShPolygon(outer, holes=closed_voids)


def _from_shapely_ring(geom) -> List[Point]:
    """shapely LinearRing/LineString → List[Point] (闭合, 末点重复首点)"""
    coords = list(geom.coords)
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    return [(float(x), float(y)) for x, y in coords]


def apply_safety_margin(
    outer: Polygon,
    voids: List[Polygon],
    margin: float,
    join_style: str = "round",
):
    """安全 margin inflate:
        可清扫区 = outer ⊖ margin  −  ⋃(void ⊕ margin)

    把"避障安全距"与"环间距"彻底解耦, 根治 F2C headland 单参数双用途 (spec §1.1)。

    Returns:
        shapely Polygon 或 MultiPolygon (障碍分裂可清扫区时多块) 或 空
    """
    JOIN_MAP = {"round": 1, "mitre": 2, "bevel": 3}
    js = JOIN_MAP[join_style]

    # 外墙内缩 (先不带 holes)
    sh_outer = _to_shapely(outer, [])
    inner = sh_outer.buffer(-margin, join_style=js)
    if inner.is_empty:
        return inner

    # 障碍外扩
    if voids:
        inflated_voids = [
            ShPolygon(v[:-1] if v[0] == v[-1] else v).buffer(margin, join_style=js)
            for v in voids
        ]
        merged_obs = unary_union(inflated_voids)
        cleanable = inner.difference(merged_obs)
    else:
        cleanable = inner

    return cleanable


# 太小的区域跳过 (单位 m²). 取 1e-6 m² ≈ 1mm² 的量级, 显著小于任何实际可清扫区
_MIN_REGION_AREA = 1e-6


def _flatten_polygons(geom) -> List[ShPolygon]:
    """把 shapely 几何对象拍平成 Polygon 列表

    shapely.difference / buffer 在边界对齐时偶尔返回 GeometryCollection
    或 LineString (含混合维度), 必须显式过滤, 否则:
    - LineString 当 Polygon 用 → AttributeError on .exterior
    - GeometryCollection 双 isinstance 都漏 → 静默丢数据

    Returns:
        Polygon 列表 (空、太小、非面状几何全部过滤掉)
    """
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, ShPolygon):
        return [geom] if geom.area >= _MIN_REGION_AREA else []
    if isinstance(geom, (MultiPolygon, GeometryCollection)):
        return [
            g for g in geom.geoms
            if isinstance(g, ShPolygon) and not g.is_empty and g.area >= _MIN_REGION_AREA
        ]
    # LineString / Point / 其他: 非面状, 丢
    return []


def _extract_boundary_rings(region: ShPolygon, depth: int, component_id: int) -> List[Ring]:
    """从一个 shapely Polygon 提取所有边界环 (外环 + 所有内洞)。
    每个环作为独立 Ring 返回, 共享同一 depth 和 component_id。
    """
    rings = []
    # 外环
    rings.append(Ring(
        coords=_from_shapely_ring(region.exterior),
        depth=depth,
        component_id=component_id,
    ))
    # 内洞 (如果有)
    for hole in region.interiors:
        rings.append(Ring(
            coords=_from_shapely_ring(hole),
            depth=depth,
            component_id=component_id,
        ))
    return rings


def generate_concentric_rings(
    outer: Polygon,
    voids: List[Polygon],
    params: OffsetParams,
) -> List[Ring]:
    """主入口: outer + voids → 同心环列表

    算法 (spec §5.2):
        1. 安全 margin inflate → 可清扫区 (带洞多边形, 可能 MultiPolygon)
        2. 递归 buffer(-spacing): 每次取当前边界(外环+内洞)作一圈
        3. 区域分裂 (MultiPolygon) 时各 component 各自递归
        4. 空或太小时停止

    Returns:
        Ring 列表, 按 depth 升序; 同 depth 内按 component_id 分组
    """
    JOIN_MAP = {"round": 1, "mitre": 2, "bevel": 3}
    js = JOIN_MAP[params.join_style]

    # Step 1: 安全 margin
    cleanable = apply_safety_margin(outer, voids, params.safety_margin, params.join_style)
    if cleanable.is_empty:
        return []

    # Step 2: 递归收缩, 栈式处理 MultiPolygon/GeometryCollection
    rings: List[Ring] = []
    next_comp_id = 0
    # 栈元素: (shapely Polygon, current depth, component_id)
    stack: List[Tuple[ShPolygon, int, int]] = []

    # 初始 push: 每个 polygon 拿独立 id
    for poly in _flatten_polygons(cleanable):
        stack.append((poly, 0, next_comp_id))
        next_comp_id += 1

    while stack:
        region, depth, comp_id = stack.pop()
        if depth >= params.max_rings:
            continue
        # 此处不再需要 region.area < _MIN_REGION_AREA 检查 (push 前已过滤)
        # 提取当前边界
        new_rings = _extract_boundary_rings(region, depth, comp_id)
        # 过滤太短的环
        for r in new_rings:
            length = sum(
                math.hypot(r.coords[i+1][0] - r.coords[i][0],
                           r.coords[i+1][1] - r.coords[i][1])
                for i in range(len(r.coords) - 1)
            )
            if length >= params.min_ring_length:
                rings.append(r)
        # 向内收一个 spacing
        shrunk = region.buffer(-params.spacing, join_style=js)
        flat = _flatten_polygons(shrunk)
        if len(flat) == 1:
            # 不分裂: 保持 component_id
            stack.append((flat[0], depth + 1, comp_id))
        else:
            # 分裂 (或全部消失): 每个子 component 拿新 id
            for poly in flat:
                stack.append((poly, depth + 1, next_comp_id))
                next_comp_id += 1

    # 按 depth 升序 + component_id 分组
    rings.sort(key=lambda r: (r.depth, r.component_id))
    return rings
