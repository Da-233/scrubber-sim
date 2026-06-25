"""contour_coverage 单元测试"""
import math
from typing import List, Tuple
import pytest
from shapely.geometry import Polygon as ShPolygon
from contour_coverage.geometry import (
    OffsetParams, Ring,
    _to_shapely, _from_shapely_ring,
    apply_safety_margin,
)


def test_apply_safety_margin_simple_rect():
    """6×6 房, 无障碍, margin=0.6 → 4.8×4.8 内缩矩形"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = []
    result = apply_safety_margin(outer, voids, margin=0.6)
    # 应是单 Polygon, 面积约 (6 - 2×0.6)^2 = 4.8^2 = 23.04
    assert isinstance(result, ShPolygon)
    assert abs(result.area - 4.8 * 4.8) < 0.1


def test_apply_safety_margin_with_void():
    """6×6 房 + 1×1 障碍, margin=0.6 → outer 内缩 + void 外扩"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = [[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)]]
    result = apply_safety_margin(outer, voids, margin=0.6)
    # outer 缩成 4.8×4.8 (面积 23.04)
    # void 外扩为 1+2×0.6=2.2 边长的圆角方 (round join)
    # 简化检查: 面积应介于 (4.8^2 - 2.2^2) 和 (4.8^2 - 1^2) 之间
    assert 23.04 - 2.2**2 - 0.5 < result.area < 23.04 - 1.0


def test_apply_safety_margin_void_eats_all():
    """障碍太大, margin 后 outer 完全被吃光 → 空多边形"""
    outer = [(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]
    voids = [[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)]]
    result = apply_safety_margin(outer, voids, margin=0.6)
    # outer 缩到 -0.2×-0.2 (负 → 空)
    assert result.is_empty


from contour_coverage.geometry import generate_concentric_rings


def test_concentric_rings_simple_rect():
    """6×6 房, margin=0.6, spacing=0.55, 无障碍
    可清扫区 4.8×4.8 → 4.8/2 ≈ 2.4m 半径
    每圈缩 0.55, 应能生成约 4~5 圈 (2.4/0.55=4.36)
    """
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    rings = generate_concentric_rings(outer, [], OffsetParams(spacing=0.55, safety_margin=0.6))
    assert 3 <= len(rings) <= 6
    # 外圈在最外面
    assert rings[0].depth == 0
    # depth 单调递增
    depths = [r.depth for r in rings]
    assert depths == sorted(depths)
    # 每圈是闭合的 (末点 == 首点)
    for r in rings:
        assert r.coords[0] == r.coords[-1]
    # 每圈至少 4 个点 (起码是个矩形)
    for r in rings:
        assert len(r.coords) >= 4


def test_concentric_rings_with_void():
    """房间中间一个障碍 → 外墙圈 + 障碍圈 (boundary 自动含 holes)
    每圈 ring 应至少有 2 个 LinearRing (外环 + 内环)
    或者环数翻倍 (生成器把外环和洞分别 yield)
    """
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = [[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)]]
    rings = generate_concentric_rings(outer, voids, OffsetParams(spacing=0.55, safety_margin=0.6))
    # 至少有外墙圈和障碍圈两类, 总环数 ≥ 2
    assert len(rings) >= 2


def test_concentric_rings_multipolygon_split():
    """长条房 + 中间一个大障碍把房间切成两半 → 环按 component 分组"""
    # 10×4 长条, 中间 4×3 的障碍把可清扫区切成左右两半
    outer = [(-5, -2), (5, -2), (5, 2), (-5, 2), (-5, -2)]
    voids = [[(-2, -1.5), (2, -1.5), (2, 1.5), (-2, 1.5), (-2, -1.5)]]
    rings = generate_concentric_rings(outer, voids, OffsetParams(spacing=0.4, safety_margin=0.3))
    # 应该有至少两个不同的 component_id
    comp_ids = {r.component_id for r in rings}
    assert len(comp_ids) >= 2


def test_concentric_rings_empty_when_too_small():
    """房间太小 margin 后空 → 0 环"""
    outer = [(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3), (-0.3, -0.3)]
    rings = generate_concentric_rings(outer, [], OffsetParams(spacing=0.55, safety_margin=0.6))
    assert len(rings) == 0


def _ring_min_distance_to_voids(ring: Ring, voids: List[List[Tuple[float, float]]]) -> float:
    """计算环上任意点到任意障碍多边形的最小距离"""
    from shapely.geometry import LineString as ShLineString, Polygon as ShPoly
    ring_line = ShLineString(ring.coords)
    min_d = float('inf')
    for v in voids:
        v_closed = v[:-1] if v[0] == v[-1] else v
        v_poly = ShPoly(v_closed)
        d = ring_line.distance(v_poly)
        if d < min_d:
            min_d = d
    return min_d


def test_rings_never_intersect_voids():
    """spec §7.1 硬判据: 任意环到任意原始障碍距离 ≥ safety_margin (容忍 1cm 浮点误差)"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = [
        [(0.55, 1.85), (1.85, 1.85), (1.85, 0.55), (0.55, 0.55), (0.55, 1.85)],
        [(-1.85, -0.35), (-0.55, -0.35), (-0.55, -1.65), (-1.85, -1.65), (-1.85, -0.35)],
    ]
    margin = 0.6
    rings = generate_concentric_rings(outer, voids, OffsetParams(spacing=0.55, safety_margin=margin))
    assert len(rings) > 0
    for r in rings:
        d = _ring_min_distance_to_voids(r, voids)
        # 容忍 1cm 浮点误差 (round join_style 在角上离障碍会精确到 margin)
        assert d >= margin - 0.01, f"Ring depth={r.depth} comp={r.component_id} 距障碍 {d:.3f} < margin {margin}"


def test_rings_stay_inside_outer():
    """spec §7.1 硬判据: 任意环都在外墙内 (距外墙 ≥ margin)"""
    from shapely.geometry import LineString as ShLineString, Polygon as ShPoly
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    margin = 0.6
    rings = generate_concentric_rings(outer, [], OffsetParams(spacing=0.55, safety_margin=margin))
    outer_poly = ShPoly(outer[:-1])
    outer_boundary = outer_poly.boundary
    for r in rings:
        ring_line = ShLineString(r.coords)
        # 环完全在 outer 内部
        assert outer_poly.contains(ring_line) or outer_poly.covers(ring_line)
        # 环到外墙距离 ≥ margin - tolerance
        d = ring_line.distance(outer_boundary)
        assert d >= margin - 0.01, f"Ring depth={r.depth} 距外墙 {d:.3f} < margin {margin}"


from contour_coverage.geometry import generate_concentric_rings, Ring, OffsetParams
from contour_coverage.path_builder import (
    PathParams, densify_ring, sort_rings_outer_to_inner, build_path,
)


def test_densify_ring_adds_intermediate_points():
    """长边应被切成 step 长度的小段"""
    ring = Ring(
        coords=[(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)],  # 2×2 方
        depth=0,
        component_id=0,
    )
    densified = densify_ring(ring, step=0.5)
    # 周长 8m, 每段 0.5m → 至少 16 段, 加首点 17 点; 闭合多 1 = 18
    assert len(densified) >= 16
    # 闭合
    assert densified[0] == densified[-1]
    # 相邻点间距 ≤ step + 容忍
    for i in range(len(densified) - 1):
        d = math.hypot(densified[i+1][0] - densified[i][0],
                       densified[i+1][1] - densified[i][1])
        assert d <= 0.5 + 0.01, f"段 {i} 长 {d:.3f} > step 0.5"


def test_sort_rings_outer_to_inner():
    """外圈 (depth 小) 在前, 同 depth 按 component_id 排"""
    rings = [
        Ring(coords=[(0,0),(0,0)], depth=2, component_id=0),
        Ring(coords=[(0,0),(0,0)], depth=0, component_id=0),
        Ring(coords=[(0,0),(0,0)], depth=1, component_id=1),
        Ring(coords=[(0,0),(0,0)], depth=1, component_id=0),
    ]
    sorted_r = sort_rings_outer_to_inner(rings)
    assert [(r.depth, r.component_id) for r in sorted_r] == [(0,0),(1,0),(1,1),(2,0)]


def test_end_to_end_simple_rect():
    """端到端: 6×6 房无障碍 → pose 序列非空, 点间距合理"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    rings = generate_concentric_rings(outer, [], OffsetParams(spacing=0.55, safety_margin=0.6))
    poses = build_path(rings, PathParams(densify_step=0.1))
    assert len(poses) > 100  # 多圈环密集化后至少几百点
    # yaw 字段存在
    for p in poses:
        assert p[2] is not None
        assert -math.pi <= p[2] <= math.pi


def test_inner_to_outer_reverses_endpoints():
    """inner_to_outer=True: 起点落最内圈(近中心), 终点落最外圈(近外墙)

    机器人中心 spawn 场景需要这个, 否则同心环终点在中心≈起点被秒判到达。
    """
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    rings = generate_concentric_rings(outer, [], OffsetParams(spacing=0.55, safety_margin=0.6))
    out_in = build_path(rings, PathParams(densify_step=0.1, inner_to_outer=False))
    in_out = build_path(rings, PathParams(densify_step=0.1, inner_to_outer=True))
    # 点集相同, 只是顺序反
    assert len(out_in) == len(in_out)
    # 默认(外→内): 终点近中心; 反转(内→外): 终点近外墙
    def dist_origin(p): return math.hypot(p[0], p[1])
    assert dist_origin(out_in[-1]) < dist_origin(in_out[-1])
    # 内→外 的起点比终点更靠近中心
    assert dist_origin(in_out[0]) < dist_origin(in_out[-1])


def test_end_to_end_with_voids():
    """端到端: 6×6 房 + 2 障碍 (M5.3 测试场景) → 安全且非空"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3), (-3, -3)]
    voids = [
        [(0.55, 1.85), (1.85, 1.85), (1.85, 0.55), (0.55, 0.55), (0.55, 1.85)],
        [(-1.85, -0.35), (-0.55, -0.35), (-0.55, -1.65), (-1.85, -1.65), (-1.85, -0.35)],
    ]
    rings = generate_concentric_rings(outer, voids, OffsetParams(spacing=0.55, safety_margin=0.6))
    poses = build_path(rings, PathParams(densify_step=0.1))
    assert len(poses) > 100
    # 关键判据: 所有 pose 点都不在原始障碍内 (距障碍 ≥ margin)
    from shapely.geometry import Point as ShPoint, Polygon as ShPoly
    void_polys = [ShPoly(v[:-1]) for v in voids]
    margin = 0.6
    for p in poses:
        for vp in void_polys:
            d = ShPoint(p[0], p[1]).distance(vp)
            assert d >= margin - 0.05, f"Pose ({p[0]:.2f}, {p[1]:.2f}) 距障碍 {d:.3f} < margin"
