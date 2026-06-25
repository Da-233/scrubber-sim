"""map_to_polygons 单元测试 + 合成 map 验证

测试用例:
  T1 单个矩形障碍: 0.5×0.5m 箱子放在 10×10m 空房中央
  T2 两个箱障: 两个不相邻的矩形
  T3 圆形障碍: 检查 approxPolyDP 简化是否合理
  T4 小噪点过滤: 1cm 噪点应被丢
  T5 凹形障碍(L 形): 检查 RETR_EXTERNAL 行为 — 凹陷部分被填实
  T6 贴墙障碍: 障碍碰到地图边界
  T7 方向: 验证返回 polygon 在 ROS 世界系 CW
  T8 闭合: 首末点必须相同
  T9 ROI 过滤: 只保留 ROI polygon 内的障碍
  T10 大地图外边界: include_outer=False 时不应包含整张地图轮廓
"""

import os
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

# 让测试能直接导入兄弟目录下的工具
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "map_to_polygons"))
from map_to_polygons import (  # noqa: E402
    ExtractParams,
    MapMeta,
    _ensure_cw_in_world,
    _signed_area,
    extract_polygons,
    to_yaml,
)


# ---------------------------------------------------------------------------
# 合成 map 工具
# ---------------------------------------------------------------------------

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURES.mkdir(exist_ok=True, parents=True)


def make_map(
    name: str,
    width_m: float,
    height_m: float,
    resolution: float = 0.05,
    origin: tuple = (0.0, 0.0, 0.0),
    draw_fn=None,
) -> Path:
    """生成一张合成 map: 白底(自由), draw_fn 在上面画障碍(0=黑)。
    返回 yaml 路径。"""
    W = int(round(width_m / resolution))
    H = int(round(height_m / resolution))
    img = np.full((H, W), 255, dtype=np.uint8)  # 全白(自由)
    if draw_fn is not None:
        draw_fn(img, resolution, origin)

    pgm_path = FIXTURES / f"{name}.pgm"
    yaml_path = FIXTURES / f"{name}.yaml"
    cv2.imwrite(str(pgm_path), img)

    yaml_path.write_text(yaml.safe_dump({
        "image": pgm_path.name,
        "resolution": resolution,
        "origin": list(origin),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }))
    return yaml_path


def world_to_pixel(x, y, origin, resolution, img_h):
    """ROS world → OpenCV pixel"""
    px = int(round((x - origin[0]) / resolution))
    py = int(round(img_h - (y - origin[1]) / resolution))
    return px, py


def draw_box_world(img, resolution, origin, x_min, y_min, x_max, y_max):
    """在 ROS 世界系画黑色矩形"""
    H = img.shape[0]
    px1, py1 = world_to_pixel(x_min, y_min, origin, resolution, H)
    px2, py2 = world_to_pixel(x_max, y_max, origin, resolution, H)
    # y 轴翻转后 py2 < py1
    lo_x, hi_x = sorted([px1, px2])
    lo_y, hi_y = sorted([py1, py2])
    cv2.rectangle(img, (lo_x, lo_y), (hi_x, hi_y), 0, -1)


def draw_circle_world(img, resolution, origin, cx, cy, r):
    H = img.shape[0]
    px, py = world_to_pixel(cx, cy, origin, resolution, H)
    cv2.circle(img, (px, py), int(round(r / resolution)), 0, -1)


# ---------------------------------------------------------------------------
# 测试用例
# ---------------------------------------------------------------------------

class TestBasicShapes:

    def test_T1_single_rectangle(self):
        """0.5×0.5m 箱子在 (5, 5) 中心,10×10m 房间"""
        yaml_path = make_map(
            "t1_rect", 10.0, 10.0,
            draw_fn=lambda img, r, o: draw_box_world(
                img, r, o, 4.75, 4.75, 5.25, 5.25
            ),
        )
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 1
        poly = result.polygons_world[0]
        # 矩形 simplified 后应是 4 个角(+闭合 1)
        assert len(poly) == 5
        assert poly[0] == poly[-1]  # 闭合
        # 面积接近 0.25 m^2
        assert 0.20 < result.stats[0]["area_m2"] < 0.30
        # 质心接近 (5, 5)
        cx = sum(p[0] for p in poly[:-1]) / 4
        cy = sum(p[1] for p in poly[:-1]) / 4
        assert abs(cx - 5.0) < 0.05
        assert abs(cy - 5.0) < 0.05

    def test_T2_two_boxes(self):
        """两个 0.4×0.4 箱子,(3,3) 和 (7,7)"""
        def draw(img, r, o):
            draw_box_world(img, r, o, 2.8, 2.8, 3.2, 3.2)
            draw_box_world(img, r, o, 6.8, 6.8, 7.2, 7.2)
        yaml_path = make_map("t2_two", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 2
        # 按 X 排序后,中心点应分别接近 (3,3) 和 (7,7)
        centers = sorted([
            (sum(p[0] for p in poly[:-1]) / (len(poly) - 1),
             sum(p[1] for p in poly[:-1]) / (len(poly) - 1))
            for poly in result.polygons_world
        ])
        assert abs(centers[0][0] - 3.0) < 0.05
        assert abs(centers[0][1] - 3.0) < 0.05
        assert abs(centers[1][0] - 7.0) < 0.05
        assert abs(centers[1][1] - 7.0) < 0.05

    def test_T3_circle(self):
        """半径 0.5m 圆形,中心 (5,5)"""
        yaml_path = make_map(
            "t3_circle", 10.0, 10.0,
            draw_fn=lambda img, r, o: draw_circle_world(img, r, o, 5.0, 5.0, 0.5),
        )
        # 用默认 approx_eps_m=0.05 (5cm 容差)
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 1
        # 圆经 5cm 容差简化后大概 10~20 个顶点
        n = result.stats[0]["vertices"]
        assert 8 <= n <= 30, f"圆形顶点数 {n} 不合理"
        # 面积 ~ π * 0.25 = 0.785
        assert 0.7 < result.stats[0]["area_m2"] < 0.85


class TestNoiseAndFiltering:

    def test_T4_small_noise_filtered(self):
        """1×1cm 噪点应被 min_area=0.05 m² 默认值过滤"""
        def draw(img, r, o):
            # 真障碍 0.4×0.4
            draw_box_world(img, r, o, 5.0, 5.0, 5.4, 5.4)
            # 噪点 4cm × 4cm = 0.0016 m² < 0.05
            draw_box_world(img, r, o, 1.0, 1.0, 1.04, 1.04)
        yaml_path = make_map("t4_noise", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(yaml_path)
        # 应只剩 1 个 polygon
        assert len(result.polygons_world) == 1, \
            f"应过滤掉噪点,实际 {len(result.polygons_world)} 个"

    def test_T4b_noise_kept_when_threshold_low(self):
        """min_area=0 时噪点保留"""
        def draw(img, r, o):
            draw_box_world(img, r, o, 5.0, 5.0, 5.4, 5.4)
            # 形态学开运算会吃掉极小噪点, 关掉再测
            draw_box_world(img, r, o, 1.0, 1.0, 1.15, 1.15)  # 15×15cm
        yaml_path = make_map("t4b", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(
            yaml_path, ExtractParams(min_area_m2=0.0, morph_kernel_px=0),
        )
        assert len(result.polygons_world) == 2


class TestKnownPitfalls:
    """spec §3.1 列出的 4 个已知坑,在这里固化为测试用例"""

    def test_T5_concave_L_shape(self):
        """L 形障碍:RETR_EXTERNAL 应给一个 polygon, 凹陷部分被填实"""
        def draw(img, r, o):
            # 一个 L 形 = 两个矩形拼接
            #   x: 3~5, y: 3~5  +  x: 3~4, y: 5~7
            draw_box_world(img, r, o, 3.0, 3.0, 5.0, 5.0)
            draw_box_world(img, r, o, 3.0, 5.0, 4.0, 7.0)
        yaml_path = make_map("t5_L", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 1
        # L 形外轮廓 6 顶点, simplified 后应在 5~8 之间
        n = result.stats[0]["vertices"]
        assert 5 <= n <= 8, f"L 形顶点数 {n} 不合理"
        # 面积 = 2×2 + 1×2 = 6 m² (RETR_EXTERNAL 不填凹陷,所以接近真实)
        assert 5.5 < result.stats[0]["area_m2"] < 6.5

    def test_T6_wall_attached(self):
        """贴左墙的箱子: 验证不会误把整张图轮廓包进来"""
        def draw(img, r, o):
            # 贴 x=0 墙
            draw_box_world(img, r, o, 0.0, 4.5, 0.5, 5.5)
        yaml_path = make_map("t6_wall", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 1
        # 面积 ~0.5 m²
        assert 0.4 < result.stats[0]["area_m2"] < 0.6

    def test_T7_direction_cw_in_world(self):
        """提取出来的 polygon 在 ROS 世界系下必须 CW (F2C inner voids 要求)"""
        yaml_path = make_map(
            "t7_dir", 10.0, 10.0,
            draw_fn=lambda img, r, o: draw_box_world(img, r, o, 4.0, 4.0, 6.0, 6.0),
        )
        result = extract_polygons(yaml_path)
        poly = result.polygons_world[0]
        # 去掉闭合点, 算 signed area
        n = len(poly) - 1
        s = 0.0
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        # CW in y-up (ROS) coordinate ⇒ signed area < 0
        assert s < 0, f"polygon 方向应为 CW (s<0), 实际 s={s}"

    def test_T8_closed(self):
        """polygon 首末点必须相同 (F2C 要求闭合)"""
        yaml_path = make_map(
            "t8_close", 10.0, 10.0,
            draw_fn=lambda img, r, o: draw_box_world(img, r, o, 4.0, 4.0, 6.0, 6.0),
        )
        result = extract_polygons(yaml_path)
        for poly in result.polygons_world:
            assert poly[0] == poly[-1], f"polygon 未闭合: {poly[0]} vs {poly[-1]}"


class TestROIFiltering:

    def test_T9_roi_filter(self):
        """ROI polygon 外的障碍应被丢弃"""
        def draw(img, r, o):
            draw_box_world(img, r, o, 2.0, 2.0, 2.5, 2.5)  # 内
            draw_box_world(img, r, o, 8.0, 8.0, 8.5, 8.5)  # 外
        yaml_path = make_map("t9_roi", 10.0, 10.0, draw_fn=draw)
        # ROI 是左下半区
        roi = [(0.5, 0.5), (4.5, 0.5), (4.5, 4.5), (0.5, 4.5), (0.5, 0.5)]
        params = ExtractParams(roi_polygon_world=roi)
        result = extract_polygons(yaml_path, params)
        assert len(result.polygons_world) == 1
        # 中心应该是 (2.25, 2.25)
        poly = result.polygons_world[0]
        cx = sum(p[0] for p in poly[:-1]) / (len(poly) - 1)
        assert cx < 4.0


class TestOuterBoundary:

    def test_T10_outer_boundary_excluded_by_default(self):
        """整张地图被黑边框围住时, include_outer=False 应丢掉外边界"""
        def draw(img, r, o):
            # 沿地图边界画 1 像素黑边
            H, W = img.shape
            cv2.rectangle(img, (0, 0), (W - 1, H - 1), 0, 5)
            # 真障碍
            draw_box_world(img, r, o, 4.5, 4.5, 5.5, 5.5)
        yaml_path = make_map("t10_outer", 10.0, 10.0, draw_fn=draw)
        result = extract_polygons(yaml_path)
        # 应只有 1 个 (中心箱障), 不包含整张图轮廓
        assert len(result.polygons_world) == 1, \
            f"应排除外边界, 实际 {len(result.polygons_world)} 个"


class TestRealSlamMaps:
    """A3 真 map 验证: 真 SLAM map (nav2_bringup 自带 depot/warehouse/sandbox)
    暴露了一个坑: 合成 map 默认 morph=3 把真 SLAM map 的细墙(1~2 px)全吃掉。
    修复: 默认 morph_kernel_px=0, 仅在已知障碍是大块时开启。"""

    REAL_MAPS = Path(__file__).resolve().parent / "real_maps"

    @pytest.mark.skipif(not (REAL_MAPS / "depot.yaml").exists(),
                        reason="real maps 未拷贝到 tests/real_maps/")
    def test_depot_default_no_morph(self):
        """depot map (真 SLAM): 默认 morph=0,应识别 >10 个障碍 polygon"""
        result = extract_polygons(self.REAL_MAPS / "depot.yaml")
        assert len(result.polygons_world) >= 10, \
            f"depot 应识别 >=10 个 polygon, 实际 {len(result.polygons_world)} 个"

    @pytest.mark.skipif(not (REAL_MAPS / "depot.yaml").exists(),
                        reason="real maps 未拷贝到 tests/real_maps/")
    def test_depot_morph_kills_walls(self):
        """开 3x3 morph 后 depot 几乎全清,固化为反例,提醒真 SLAM map 不要开 morph"""
        result = extract_polygons(
            self.REAL_MAPS / "depot.yaml",
            ExtractParams(morph_kernel_px=3),
        )
        # morph 把细墙都吃掉,polygon 数应大幅下降(< 10)
        assert len(result.polygons_world) < 10, \
            f"morph 应把真 SLAM map 大部分墙吃掉,实际 {len(result.polygons_world)} 个"

    @pytest.mark.skipif(not (REAL_MAPS / "warehouse.yaml").exists(),
                        reason="real maps 未拷贝到 tests/real_maps/")
    def test_warehouse_default_no_morph(self):
        """warehouse map (1674×1006 大图): 默认应识别 >20 个 polygon"""
        result = extract_polygons(self.REAL_MAPS / "warehouse.yaml")
        assert len(result.polygons_world) >= 20, \
            f"warehouse 应识别 >=20 个 polygon, 实际 {len(result.polygons_world)} 个"


class TestNonZeroOrigin:

    def test_origin_offset(self):
        """origin 偏移到 (-5, -5),验证坐标换算正确"""
        origin = (-5.0, -5.0, 0.0)
        # 在 ROS 系 (0, 0) 画箱子(对应图像中心)
        yaml_path = make_map(
            "t_origin", 10.0, 10.0, origin=origin,
            draw_fn=lambda img, r, o: draw_box_world(
                img, r, o, -0.5, -0.5, 0.5, 0.5
            ),
        )
        result = extract_polygons(yaml_path)
        assert len(result.polygons_world) == 1
        poly = result.polygons_world[0]
        cx = sum(p[0] for p in poly[:-1]) / (len(poly) - 1)
        cy = sum(p[1] for p in poly[:-1]) / (len(poly) - 1)
        # 中心应在 (0, 0)
        assert abs(cx) < 0.1, f"cx={cx}"
        assert abs(cy) < 0.1, f"cy={cy}"


class TestSignedArea:
    """辅助函数 _ensure_cw_in_world 的单元测试"""

    def test_ccw_input_flipped_to_cw(self):
        # CCW in y-up: signed area > 0
        ccw = [(0, 0), (1, 0), (1, 1), (0, 1)]
        out = _ensure_cw_in_world(ccw)
        # 翻转后应是 CW
        s = 0.0
        n = len(out)
        for i in range(n):
            x1, y1 = out[i]
            x2, y2 = out[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        assert s < 0

    def test_cw_input_kept(self):
        cw = [(0, 0), (0, 1), (1, 1), (1, 0)]
        out = _ensure_cw_in_world(cw)
        # 应保持原顺序
        assert out == cw


class TestYamlOutput:

    def test_to_yaml_roundtrip(self, tmp_path):
        yaml_path = make_map(
            "y_out", 10.0, 10.0,
            draw_fn=lambda img, r, o: draw_box_world(img, r, o, 4.5, 4.5, 5.5, 5.5),
        )
        result = extract_polygons(yaml_path)
        out = tmp_path / "polygons.yaml"
        to_yaml(result, out)
        data = yaml.safe_load(out.read_text())
        assert "polygons" in data
        assert len(data["polygons"]) == 1
        assert data["polygons"][0]["vertices"] == 4
        # 首末点相同
        pts = data["polygons"][0]["points"]
        assert pts[0] == pts[-1]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
