"""map_to_polygons — SLAM 占据栅格图 → 障碍 polygon list 提取工具

用途
====
读 ROS2 map_server 输出的 .pgm + .yaml,把图里的黑色障碍像素转成世界坐标系
下的多边形列表,用于 NavigateCompleteCoverage goal 的 polygons[1..N]
(F2C inner voids)。

数据流
======
    map.pgm + map.yaml
        ↓ cv2.imread + 阈值化
    二值图(障碍=255)
        ↓ 形态学开运算清噪
    cv2.findContours(RETR_EXTERNAL)
        ↓
    轮廓(像素坐标,任意方向)
        ↓ cv2.approxPolyDP 简化 + 面积过滤
    简化多边形
        ↓ 像素→世界坐标 + 强制 CW 方向(F2C inner voids 用)
    polygons_world: List[List[(x, y)]]

约定
====
- ROS2 map.yaml 的 origin 是图像**左下角**在 map 坐标系的位置;OpenCV 图像
  原点是**左上角** → 像素 (px, py) → 世界 (origin_x + px*res, origin_y + (H - py)*res)
- F2C 用顶点顺序判断内外:外圈 CCW, inner voids CW (按图像坐标系顺时针,
  在 ROS map 坐标系翻 y 后变 CCW; 这里返回的是 ROS 坐标系下的 CW 顺序)
- 首末点重复(F2C 要求闭合),即 poly[-1] == poly[0]

已知坑(spec §3.1)
===================
1. 凹形障碍: RETR_EXTERNAL 给外轮廓,凹陷部分会被填实
2. 贴墙障碍: 会跟墙黏成一个 polygon
3. 简化过度: approx_eps_m 太大丢角
4. 方向判定: 提取的轮廓方向不定,这里强制 CW
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import yaml

Point = Tuple[float, float]
Polygon = List[Point]


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class MapMeta:
    """对应 ROS2 map_server 的 map.yaml 关键字段"""
    image_path: Path
    resolution: float          # m / pixel
    origin: Tuple[float, float, float]  # (x, y, yaw) — yaw 此处不用
    negate: int = 0            # 0 = 黑色是障碍, 1 = 白色是障碍
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> "MapMeta":
        yaml_path = Path(yaml_path).resolve()
        with yaml_path.open() as f:
            data = yaml.safe_load(f)
        image_rel = data["image"]
        image_path = (yaml_path.parent / image_rel).resolve()
        return cls(
            image_path=image_path,
            resolution=float(data["resolution"]),
            origin=tuple(data["origin"]),
            negate=int(data.get("negate", 0)),
            occupied_thresh=float(data.get("occupied_thresh", 0.65)),
            free_thresh=float(data.get("free_thresh", 0.196)),
        )


@dataclass
class ExtractParams:
    """polygon 提取参数,从 spec §3.1 推荐值起步"""
    min_area_m2: float = 0.05      # 5cm × 10cm 以下当噪点丢
    approx_eps_m: float = 0.05     # 5cm 多边形简化容差
    # 形态学开运算核大小(px). 默认 0 = 不做。
    # ⚠️ 真 SLAM map 的墙通常只有 1~2 px 细,开 3x3 开运算会把墙全吃掉。
    # 仅在已知障碍是大块、且噪点多时开(典型: 合成测试 map)
    morph_kernel_px: int = 0
    # 是否把整张地图的外边界也提取出来(墙轮廓)
    # 大多数情况下: False, 我们只要内部障碍 voids
    include_outer_boundary: bool = False
    # ROI 裁剪(世界坐标系下),只在 polygon 内部提取障碍
    # 用于"只看清扫区内的障碍"。None = 全图
    roi_polygon_world: Polygon | None = None


@dataclass
class ExtractResult:
    polygons_world: List[Polygon] = field(default_factory=list)
    # 调试用:像素坐标多边形(没翻 y,直接 OpenCV 顺序)
    polygons_pixel: List[List[Tuple[int, int]]] = field(default_factory=list)
    # 调试用:每个 polygon 的面积(m^2)和点数
    stats: List[dict] = field(default_factory=list)
    # 用于覆盖到原图上做可视化
    binary_image: np.ndarray | None = None
    map_meta: MapMeta | None = None


# ---------------------------------------------------------------------------
# 核心算法
# ---------------------------------------------------------------------------

def _binarize(img_gray: np.ndarray, meta: MapMeta) -> np.ndarray:
    """ROS2 map_server 约定:
        像素值 p ∈ [0, 255]
        p / 255 → 占据概率(negate=0 时:像素越黑概率越高)
        > occupied_thresh → 障碍(255)
        < free_thresh → 自由(0)
        其他 → 未知(128, 这里也归到自由侧, 0)
    我们只关心障碍 → 返回二值图(障碍=255)
    """
    if meta.negate == 0:
        # 黑色是障碍: p_occ = (255 - pixel) / 255
        p_occ = (255 - img_gray.astype(np.float32)) / 255.0
    else:
        p_occ = img_gray.astype(np.float32) / 255.0
    binary = (p_occ > meta.occupied_thresh).astype(np.uint8) * 255
    return binary


def _pixel_to_world(px: int, py: int, meta: MapMeta, img_h: int) -> Point:
    x = meta.origin[0] + px * meta.resolution
    y = meta.origin[1] + (img_h - py) * meta.resolution
    return (x, y)


def _signed_area(poly_pixel: List[Tuple[int, int]]) -> float:
    """像素坐标系下的有符号面积。正 = CCW (OpenCV 图像坐标 y 朝下,
    所以这里 CCW 是图像意义上的 CCW)。"""
    n = len(poly_pixel)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = poly_pixel[i]
        x2, y2 = poly_pixel[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def _ensure_cw_in_world(poly_world: Polygon) -> Polygon:
    """F2C inner voids 要求 CW (在 ROS 世界坐标系,y 朝上)。
    判定 signed area: y 朝上时 CCW > 0, CW < 0。"""
    n = len(poly_world)
    if n < 3:
        return poly_world
    s = 0.0
    for i in range(n):
        x1, y1 = poly_world[i]
        x2, y2 = poly_world[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    # 当前是 CCW → 反转成 CW
    if s > 0:
        poly_world = list(reversed(poly_world))
    return poly_world


def extract_polygons(
    yaml_path: Path | str,
    params: ExtractParams | None = None,
) -> ExtractResult:
    """主入口。读 map.yaml,提取障碍 polygon list。"""
    params = params or ExtractParams()
    meta = MapMeta.from_yaml(Path(yaml_path))

    img = cv2.imread(str(meta.image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"读不到图像: {meta.image_path}")
    img_h, img_w = img.shape

    binary = _binarize(img, meta)

    # 形态学开运算清噪(可选)
    if params.morph_kernel_px > 0:
        k = params.morph_kernel_px
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )

    result = ExtractResult(binary_image=binary, map_meta=meta)

    eps_px = params.approx_eps_m / meta.resolution
    min_area_px = params.min_area_m2 / (meta.resolution ** 2)

    # 先一遍扫:识别"覆盖几乎整张图"的外边界轮廓索引
    outer_indices = set()
    for idx, cnt in enumerate(contours):
        x, y, w, h = cv2.boundingRect(cnt)
        if w >= img_w * 0.95 and h >= img_h * 0.95:
            outer_indices.add(idx)

    for idx, cnt in enumerate(contours):
        # 跳过外边界(除非用户显式要)
        if idx in outer_indices and not params.include_outer_boundary:
            continue

        # hierarchy[0][idx] = [next, prev, first_child, parent]
        # 如果当前 contour 的 parent 也是外边界,它实际上是"墙内"的障碍 → 保留
        # 如果当前 contour 的 parent 是另一个真障碍 → 它是"洞",跳过(F2C 不处理)
        if hierarchy is not None:
            parent = hierarchy[0][idx][3]
            if parent != -1 and parent not in outer_indices:
                # 它是某个真障碍的"洞" — F2C 不处理这种情况(暂不支持)
                continue

        area_px = cv2.contourArea(cnt)
        if area_px < min_area_px:
            continue

        # 简化多边形
        approx = cv2.approxPolyDP(cnt, eps_px, closed=True)
        poly_pixel = [(int(p[0][0]), int(p[0][1])) for p in approx]
        if len(poly_pixel) < 3:
            continue
        poly_world = [
            _pixel_to_world(px, py, meta, img_h) for (px, py) in poly_pixel
        ]
        # ROI 过滤(可选): polygon 质心必须在 ROI 内
        if params.roi_polygon_world is not None:
            cx = sum(p[0] for p in poly_world) / len(poly_world)
            cy = sum(p[1] for p in poly_world) / len(poly_world)
            if not _point_in_polygon(cx, cy, params.roi_polygon_world):
                continue

        # 强制 CW (F2C inner voids)
        poly_world = _ensure_cw_in_world(poly_world)

        # 闭合 (首点重复)
        if poly_world[0] != poly_world[-1]:
            poly_world.append(poly_world[0])

        result.polygons_world.append(poly_world)
        result.polygons_pixel.append(poly_pixel)
        result.stats.append({
            "area_m2": float(area_px * meta.resolution ** 2),
            "vertices": len(poly_world) - 1,  # 去掉闭合点
        })

    return result


def _point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    """ray casting"""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


# ---------------------------------------------------------------------------
# 输出: yaml + 可视化
# ---------------------------------------------------------------------------

def to_yaml(result: ExtractResult, output_path: Path | str) -> None:
    """导出为 yaml,供 action client 读取。
    顶层 schema:
        polygons:
          - points: [[x, y], ...]   # 闭合,首末点相同
            area_m2: float
            vertices: int
    """
    data = {
        "polygons": [
            {
                "points": [list(p) for p in poly],
                "area_m2": st["area_m2"],
                "vertices": st["vertices"],
            }
            for poly, st in zip(result.polygons_world, result.stats)
        ],
        "source_map": str(result.map_meta.image_path) if result.map_meta else None,
        "resolution": result.map_meta.resolution if result.map_meta else None,
    }
    Path(output_path).write_text(yaml.safe_dump(data, sort_keys=False))


def visualize(
    result: ExtractResult,
    output_png: Path | str,
    show_world_axis: bool = True,
) -> None:
    """生成对比图: 原 binary + 叠加 polygon"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly

    if result.map_meta is None or result.binary_image is None:
        raise ValueError("result 不完整")

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    # 左: 原二值图
    axes[0].imshow(result.binary_image, cmap="gray_r")
    axes[0].set_title("Binary (obstacle = black)")

    # 右: 二值图 + polygon 叠加
    axes[1].imshow(result.binary_image, cmap="gray_r", alpha=0.4)
    for poly_px, st in zip(result.polygons_pixel, result.stats):
        mp = MplPoly(
            poly_px, closed=True, fill=False,
            edgecolor="red", linewidth=2,
        )
        axes[1].add_patch(mp)
        cx = sum(p[0] for p in poly_px) / len(poly_px)
        cy = sum(p[1] for p in poly_px) / len(poly_px)
        axes[1].text(cx, cy, f"{st['area_m2']:.2f}m²\nv={st['vertices']}",
                     color="blue", fontsize=8, ha="center")
    axes[1].set_title(f"Polygons (n={len(result.polygons_world)})")

    for ax in axes:
        ax.set_xlabel("px")
        ax.set_ylabel("px")

    fig.tight_layout()
    fig.savefig(output_png, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="SLAM map.pgm → 障碍 polygon list (F2C inner voids)"
    )
    parser.add_argument("--map", required=True, help="map.yaml 路径")
    parser.add_argument("--output", required=True,
                        help="输出 polygons.yaml 路径")
    parser.add_argument("--viz", default=None,
                        help="(可选) 输出可视化 png 路径")
    parser.add_argument("--min-area", type=float, default=0.05,
                        help="过滤小于此面积(m²)的轮廓 (默认 0.05)")
    parser.add_argument("--approx-eps", type=float, default=0.05,
                        help="多边形简化容差(m) (默认 0.05)")
    parser.add_argument("--morph", type=int, default=0,
                        help="形态学开运算核大小(px), 0=不做 (默认 0). "
                             "⚠️ 真 SLAM map 不要开,会把细墙吃掉")
    parser.add_argument("--include-outer", action="store_true",
                        help="包含整张地图的外边界(墙轮廓)")
    args = parser.parse_args()

    params = ExtractParams(
        min_area_m2=args.min_area,
        approx_eps_m=args.approx_eps,
        morph_kernel_px=args.morph,
        include_outer_boundary=args.include_outer,
    )

    result = extract_polygons(args.map, params)
    to_yaml(result, args.output)

    print(f"提取 {len(result.polygons_world)} 个 polygon → {args.output}")
    for i, st in enumerate(result.stats):
        print(f"  #{i}: area={st['area_m2']:.3f} m², vertices={st['vertices']}")

    if args.viz:
        visualize(result, args.viz)
        print(f"可视化 → {args.viz}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
