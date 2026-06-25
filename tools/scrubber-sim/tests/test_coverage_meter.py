"""coverage_meter 单元测试 (footprint sweep 覆盖率量化)

测试用例:
  T1 满覆盖: 弓字形扫遍空房 → 覆盖率 ≈ 100%
  T2 半覆盖: 只扫下半间 → 覆盖率 ≈ 50%
  T3 避障扫净: 房中有障碍,轨迹绕开 → 高覆盖 + 扫障=0
  T4 穿障报警: 轨迹直穿障碍 → 扫障面积 > 0
  T5 theta 缺失: 不给 theta,由运动方向推断,横刀仍覆盖一条带
  T6 越界报警: 轨迹扫出房外 → overspray > 0
  T7 补点无缝: 两个相距很远的 waypoint,补点后中间不留缝
  T8 边界: 覆盖率落在 [0,1];空轨迹 = 0
  T9 加载: area yaml + 轨迹 csv 往返
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent / "coverage_meter")
)
from coverage_meter import (  # noqa: E402
    Footprint,
    densify_trajectory,
    load_area,
    load_trajectory,
    measure_coverage,
    parse_polygon_arg,
)


# ---------------------------------------------------------------------------
# 辅助: 造轨迹
# ---------------------------------------------------------------------------

def lawnmower(x0, x1, y0, y1, lane_spacing):
    """生成覆盖 [x0,x1]x[y0,y1] 的弓字形 waypoint 列表 (含 theta)。"""
    poses = []
    y = y0
    going_right = True
    while y <= y1 + 1e-9:
        if going_right:
            poses.append((x0, y, 0.0))
            poses.append((x1, y, 0.0))
        else:
            poses.append((x1, y, math.pi))
            poses.append((x0, y, math.pi))
        going_right = not going_right
        y += lane_spacing
    return poses


def square(cx, cy, half):
    """以 (cx,cy) 为心、边长 2*half 的正方形 (CCW 闭合)。"""
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),
    ]


# 5×5m 空房,清扫宽 0.5,lane 间距 0.5(刚好相接)
ROOM = square(2.5, 2.5, 2.5)
FP = Footprint(clean_width=0.5, clean_length=0.2)


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------

def test_t1_full_coverage():
    # 弓字形扫遍全房,lane=0.5=clean_width,横向相接;留边距让边缘也覆盖
    traj = lawnmower(0.25, 4.75, 0.25, 4.75, 0.5)
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    assert r.coverage_ratio > 0.93, f"满覆盖应 >93%, got {r.coverage_ratio:.3f}"
    assert r.swept_obstacle_m2 == 0.0
    assert r.overspray_m2 == pytest.approx(0.0, abs=0.05)


def test_t2_half_coverage():
    # 只扫下半间 (y 0.25~2.4)
    traj = lawnmower(0.25, 4.75, 0.25, 2.4, 0.5)
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    assert 0.40 < r.coverage_ratio < 0.60, \
        f"半覆盖应 ~50%, got {r.coverage_ratio:.3f}"


def test_t3_avoid_obstacle_clean():
    # 房中央 1×1 障碍;弓字形但 lane 不穿过障碍中心带
    void = square(2.5, 2.5, 0.5)  # 障碍占 x,y∈[2.0,3.0]
    # 扫下半到 y=1.5(footprint 顶到 1.75,离障碍留缝),沿左墙 x=0.25 过渡
    # 到上半 y=3.5 再扫(过渡段远离障碍 x∈[2,3],不会斜穿)
    bottom = lawnmower(0.25, 4.75, 0.25, 1.5, 0.5)
    top = lawnmower(0.25, 4.75, 3.5, 4.75, 0.5)
    transit = [(0.25, 1.5, math.pi / 2), (0.25, 3.5, math.pi / 2)]
    traj = bottom + transit + top
    r = measure_coverage(ROOM, [void], traj, FP, resolution=0.05)
    assert r.swept_obstacle_m2 == 0.0, "绕开障碍不该扫到障碍"
    # 可清扫区被挖掉了中央带,覆盖到的应是上下两块
    assert 0.50 < r.coverage_ratio < 0.85


def test_t4_hit_obstacle_alarm():
    # 障碍在路径正中,横穿
    void = square(2.5, 2.5, 0.5)
    traj = [(0.25, 2.5, 0.0), (4.75, 2.5, 0.0)]  # 一条横线直穿
    r = measure_coverage(ROOM, [void], traj, FP, resolution=0.05)
    assert r.swept_obstacle_m2 > 0.3, \
        f"穿过 1×1 障碍扫障面积应显著, got {r.swept_obstacle_m2:.3f}"


def test_t5_theta_inferred_from_motion():
    # 不给 theta,横向直线运动,footprint 应扫出一条横带
    traj = [(0.5, 2.5, None), (4.5, 2.5, None)]
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    # 一条 clean_width=0.5 宽、长约 4m 的带,面积约 0.5*4=2.0 m²
    assert 1.5 < r.covered_area_m2 < 2.6, \
        f"横带面积异常, got {r.covered_area_m2:.3f}"


def test_t6_overspray_alarm():
    # 轨迹扫到房子外(x 到 6 > 5)
    traj = [(0.5, 2.5, 0.0), (6.0, 2.5, 0.0)]
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    assert r.overspray_m2 > 0.1, \
        f"扫出房外应有 overspray, got {r.overspray_m2:.3f}"


def test_t7_densify_no_gap():
    # 两个相距 4m 的 waypoint,补点后中间应连续(无缝)
    traj = [(0.5, 2.5, 0.0), (4.5, 2.5, 0.0)]
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    # 沿带方向逐列检查:从 x≈0.6 到 4.4 每一列都应有覆盖
    x_min, y_min = r.grid_origin
    res = r.resolution
    row_center = int(round((2.5 - y_min) / res))
    band = r.covered_mask[row_center - 2: row_center + 3, :]
    col_lo = int((0.7 - x_min) / res)
    col_hi = int((4.3 - x_min) / res)
    cols_with_cover = band[:, col_lo:col_hi].any(axis=0)
    assert cols_with_cover.all(), "补点后中间不应留缝"


def test_t8_bounds():
    # 覆盖率必在 [0,1]
    traj = lawnmower(0.25, 4.75, 0.25, 4.75, 0.5)
    r = measure_coverage(ROOM, [], traj, FP, resolution=0.05)
    assert 0.0 <= r.coverage_ratio <= 1.0
    # 空轨迹 = 0
    r0 = measure_coverage(ROOM, [], [], FP, resolution=0.05)
    assert r0.coverage_ratio == 0.0


def test_t9_loaders_roundtrip(tmp_path):
    # area yaml
    area = {"outer": [list(p) for p in ROOM],
            "voids": [[list(p) for p in square(2.5, 2.5, 0.5)]]}
    area_path = tmp_path / "area.yaml"
    area_path.write_text(yaml.safe_dump(area))
    outer, voids = load_area(area_path)
    assert len(outer) == 5 and len(voids) == 1

    # map_to_polygons 风格(只有 polygons)
    poly_yaml = {"polygons": [{"points": [list(p) for p in square(2.5, 2.5, 0.5)]}]}
    pp = tmp_path / "polys.yaml"
    pp.write_text(yaml.safe_dump(poly_yaml))
    outer2, voids2 = load_area(pp)
    assert outer2 == [] and len(voids2) == 1

    # 轨迹 csv 带表头
    csv_path = tmp_path / "traj.csv"
    csv_path.write_text("x,y,theta\n0.5,2.5,0.0\n4.5,2.5,0.0\n")
    traj = load_trajectory(csv_path)
    assert len(traj) == 2
    assert traj[0] == (0.5, 2.5, 0.0)

    # csv 无 theta 列
    csv2 = tmp_path / "traj2.csv"
    csv2.write_text("1.0,1.0\n2.0,2.0\n")
    traj2 = load_trajectory(csv2)
    assert traj2[0][2] is None


def test_t10_densify_pure_rotation():
    # 原地转弯 (位置不动,theta 变),不应崩 + 不漏末点
    traj = [(2.5, 2.5, 0.0), (2.5, 2.5, math.pi)]
    xy, th = densify_trajectory(traj, step=0.05)
    assert len(xy) >= 2
    assert th[-1] == pytest.approx(math.pi)


def test_t11_parse_polygon_arg():
    poly = parse_polygon_arg("0,0; 5,0; 5,5; 0,5")
    assert poly == [(0, 0), (5, 0), (5, 5), (0, 5)]
