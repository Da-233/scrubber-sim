"""coverage_meter — 洗地机实际覆盖率量化工具 (footprint sweep)

用途
====
M5.3 A8 端到端验证要回答"覆盖率 ≥ 85% 没有"。**不能用路径长度估**
(路径长 ≠ 真扫到的面积:重复扫、转弯空扫、漏扫都会让长度估失真)。
这里做真正的 footprint sweep:把清扫刷/吸水扒的矩形 footprint 沿机器人
轨迹逐点旋转盖章栅格化,再与"可清扫区"求交,算实际扫到的面积占比。

数据流
======
    轨迹 traj (x, y, theta)        清扫区 area (outer + voids)
        │                              │
        │ 补点(间距≤分辨率)            │ 栅格化
        ▼                              ▼
    密轨迹                        cleanable_mask = outer ∧ ¬voids
        │ 每点盖 footprint 矩形        │
        ▼                              │
    covered_mask ───────────────── ∧ ─┘
        │
        ▼
    coverage = (covered ∧ cleanable) / cleanable
    + 扫障面积 (covered ∧ voids)   ← 碰撞/剐蹭报警
    + 越界面积 (covered ∧ ¬outer)  ← 喷溅报警

约定
====
- 世界坐标系 = ROS map 系 (x 右, y 上, theta 逆时针)
- footprint 在机器人体坐标系: 纵向(沿航向)= length, 横向 = width,
  中心相对 base_link 偏移 offset_x (清扫盘常在车体后方/下方)
- 默认 clean_width=0.6 对齐 F2C op_width;clean_length=0.2 ≈ 吸水扒深度
- 内部统一一套世界→栅格映射 (x_min,y_min) + resolution,所有 mask 同格

为什么不是路径长度估
====================
F2C 报"路程 7.83m"只说明走了多远,答不了"扫干净没"。弓字形相邻刀
overlap、转弯重复碾压、绕障留白,都要靠面积栅格化才能如实算出来。
"半通过陷阱"的教训:数字要量化判据,别拿代理指标当结论。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

Point = Tuple[float, float]
Polygon = List[Point]
# 轨迹点: (x, y, theta)。theta 为 None 时由运动方向推断。
Pose = Tuple[float, float, Optional[float]]


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class Footprint:
    """清扫工具在机器人体坐标系下的矩形覆盖范围。

    真车上这是吸水扒 / 滚刷的有效清扫矩形,不是整车外廓
    (整车外廓比清扫范围大,用整车算覆盖率会虚高)。
    """
    clean_width: float = 0.6     # 横向(垂直航向),对齐 F2C op_width
    clean_length: float = 0.2    # 纵向(沿航向)≈ 吸水扒深度
    offset_x: float = 0.0        # 清扫盘中心相对 base_link 的纵向偏移


@dataclass
class CoverageResult:
    coverage_ratio: float = 0.0          # 扫到的可清扫格 / 总可清扫格
    cleanable_area_m2: float = 0.0       # 可清扫区总面积
    covered_area_m2: float = 0.0         # 实际扫到的可清扫面积
    swept_obstacle_m2: float = 0.0       # footprint 压到障碍上的面积(报警)
    overspray_m2: float = 0.0            # 扫到 outer 外的面积(报警)
    resolution: float = 0.05
    # 调试 / 可视化用
    cleanable_mask: Optional[np.ndarray] = None
    covered_mask: Optional[np.ndarray] = None
    void_mask: Optional[np.ndarray] = None
    grid_origin: Tuple[float, float] = (0.0, 0.0)  # (x_min, y_min)
    densified_xy: Optional[np.ndarray] = None      # 补点后的轨迹 (N,2)

    def summary(self) -> str:
        return (
            f"覆盖率 = {self.coverage_ratio * 100:.1f}%  "
            f"({self.covered_area_m2:.2f} / {self.cleanable_area_m2:.2f} m²)\n"
            f"扫障面积 = {self.swept_obstacle_m2:.3f} m²"
            f"{'  ⚠️ 碰到障碍!' if self.swept_obstacle_m2 > 1e-6 else ''}\n"
            f"越界面积 = {self.overspray_m2:.3f} m²"
            f"{'  ⚠️ 扫出区外!' if self.overspray_m2 > 1e-6 else ''}"
        )


# ---------------------------------------------------------------------------
# 世界 <-> 栅格 映射
# ---------------------------------------------------------------------------

def _world_to_grid(
    xs: np.ndarray, ys: np.ndarray, x_min: float, y_min: float, res: float
) -> np.ndarray:
    """世界坐标 -> 栅格列行 (col=x_idx, row=y_idx), 返回 int32 (N,2)。
    行 row 随 y 增大而增大(内部自洽;可视化时再翻 y 朝上)。"""
    cols = np.round((np.asarray(xs) - x_min) / res).astype(np.int32)
    rows = np.round((np.asarray(ys) - y_min) / res).astype(np.int32)
    return np.stack([cols, rows], axis=-1)


def _fill_polys(
    grid_shape: Tuple[int, int],
    polys_world: Sequence[Polygon],
    x_min: float,
    y_min: float,
    res: float,
) -> np.ndarray:
    """把若干世界坐标 polygon 填到一张 uint8 mask 上(并集)。"""
    mask = np.zeros(grid_shape, dtype=np.uint8)
    for poly in polys_world:
        if len(poly) < 3:
            continue
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        pts = _world_to_grid(np.array(xs), np.array(ys), x_min, y_min, res)
        cv2.fillPoly(mask, [pts.reshape(-1, 1, 2)], 1)
    return mask


# ---------------------------------------------------------------------------
# 轨迹补点
# ---------------------------------------------------------------------------

def _normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def densify_trajectory(
    traj: Sequence[Pose], step: float
) -> Tuple[np.ndarray, np.ndarray]:
    """把稀疏轨迹补点到相邻间距 ≤ step。
    返回 (xy[N,2], theta[N])。
    - 位置线性插值
    - theta:若两端都给了用最短角插值;否则取该段运动方向 atan2
    """
    if len(traj) == 0:
        return np.zeros((0, 2)), np.zeros((0,))
    if len(traj) == 1:
        x, y, th = traj[0]
        return np.array([[x, y]]), np.array([th if th is not None else 0.0])

    out_xy: List[Tuple[float, float]] = []
    out_th: List[float] = []
    for i in range(len(traj) - 1):
        x0, y0, th0 = traj[i]
        x1, y1, th1 = traj[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        # 段运动方向(位置不动时退化,用已给 theta)
        seg_heading = math.atan2(y1 - y0, x1 - x0) if dist > 1e-9 else None
        n = max(1, int(math.ceil(dist / step)))
        for k in range(n):  # 不含末点,避免重复(末点在下一段或收尾补)
            t = k / n
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            if th0 is not None and th1 is not None:
                dth = _normalize_angle(th1 - th0)
                pth = _normalize_angle(th0 + dth * t)
            elif th0 is not None:
                pth = th0
            elif seg_heading is not None:
                pth = seg_heading
            else:
                pth = 0.0
            out_xy.append((px, py))
            out_th.append(pth)
    # 收尾末点
    xl, yl, thl = traj[-1]
    if thl is None:
        thl = out_th[-1] if out_th else 0.0
    out_xy.append((xl, yl))
    out_th.append(thl)
    return np.array(out_xy), np.array(out_th)


# ---------------------------------------------------------------------------
# 核心: 覆盖率测量
# ---------------------------------------------------------------------------

def measure_coverage(
    outer: Polygon,
    voids: Sequence[Polygon],
    trajectory: Sequence[Pose],
    footprint: Footprint | None = None,
    resolution: float = 0.05,
) -> CoverageResult:
    """主入口。栅格化 footprint sweep,算可清扫区实际覆盖率。

    outer: 清扫外圈 polygon(世界坐标)
    voids: 障碍 inner voids 列表(从可清扫区里挖掉,且被扫到要报警)
    trajectory: [(x, y, theta), ...],theta 可为 None
    footprint: 清扫工具矩形;None 用默认(0.6×0.2)
    resolution: 栅格分辨率 m/格,越小越准越慢
    """
    footprint = footprint or Footprint()
    if len(outer) < 3:
        raise ValueError("outer polygon 至少要 3 个点")

    # --- 网格范围:outer bbox + footprint 余量(容纳越界喷溅) ---
    margin = max(footprint.clean_width, footprint.clean_length) + 2 * resolution
    oxs = [p[0] for p in outer]
    oys = [p[1] for p in outer]
    x_min = min(oxs) - margin
    y_min = min(oys) - margin
    x_max = max(oxs) + margin
    y_max = max(oys) + margin
    W = int(math.ceil((x_max - x_min) / resolution)) + 1
    H = int(math.ceil((y_max - y_min) / resolution)) + 1
    grid_shape = (H, W)

    cell_area = resolution * resolution

    # --- 区域 mask ---
    outer_mask = _fill_polys(grid_shape, [outer], x_min, y_min, resolution)
    void_mask = _fill_polys(grid_shape, voids, x_min, y_min, resolution)
    cleanable_mask = (outer_mask.astype(bool)) & (~void_mask.astype(bool))

    # --- footprint sweep ---
    # 补点步长取分辨率,确保相邻 footprint 在纵向上连续无缝
    xy, th = densify_trajectory(trajectory, step=resolution)
    covered = np.zeros(grid_shape, dtype=np.uint8)

    half_l = footprint.clean_length / 2.0
    half_w = footprint.clean_width / 2.0
    # 体坐标系四角 (纵向 lx, 横向 ly)
    local = np.array([
        [footprint.offset_x - half_l, -half_w],
        [footprint.offset_x + half_l, -half_w],
        [footprint.offset_x + half_l, +half_w],
        [footprint.offset_x - half_l, +half_w],
    ])
    for (px, py), pth in zip(xy, th):
        c, s = math.cos(pth), math.sin(pth)
        wx = px + local[:, 0] * c - local[:, 1] * s
        wy = py + local[:, 0] * s + local[:, 1] * c
        pts = _world_to_grid(wx, wy, x_min, y_min, resolution)
        cv2.fillConvexPoly(covered, pts.reshape(-1, 1, 2), 1)

    covered_b = covered.astype(bool)

    total_cleanable = int(cleanable_mask.sum())
    covered_cleanable = int((covered_b & cleanable_mask).sum())
    swept_obstacle = int((covered_b & void_mask.astype(bool)).sum())
    overspray = int((covered_b & (~outer_mask.astype(bool))).sum())

    ratio = covered_cleanable / total_cleanable if total_cleanable else 0.0

    return CoverageResult(
        coverage_ratio=ratio,
        cleanable_area_m2=total_cleanable * cell_area,
        covered_area_m2=covered_cleanable * cell_area,
        swept_obstacle_m2=swept_obstacle * cell_area,
        overspray_m2=overspray * cell_area,
        resolution=resolution,
        cleanable_mask=cleanable_mask,
        covered_mask=covered_b,
        void_mask=void_mask.astype(bool),
        grid_origin=(x_min, y_min),
        densified_xy=xy,
    )


# ---------------------------------------------------------------------------
# 加载: 区域 yaml + 轨迹 csv
# ---------------------------------------------------------------------------

def load_area(path: Path | str) -> Tuple[Polygon, List[Polygon]]:
    """读区域 yaml,返回 (outer, voids)。

    支持两种 schema:
    1. 自带 outer + voids:
         outer: [[x,y], ...]
         voids: [[[x,y],...], ...]
    2. map_to_polygons 输出(只有 voids,需另给 outer):
         polygons:
           - points: [[x,y], ...]
       此时 voids = 各 polygon 的 points,outer 需由 --outer 指定
    """
    data = yaml.safe_load(Path(path).read_text())
    if "outer" in data:
        outer = [tuple(p) for p in data["outer"]]
        voids = [[tuple(p) for p in poly] for poly in data.get("voids", [])]
        return outer, voids
    if "polygons" in data:
        voids = [[tuple(p) for p in poly["points"]] for poly in data["polygons"]]
        return [], voids
    raise ValueError(f"无法识别的区域 yaml schema: {path}")


def load_trajectory(path: Path | str) -> List[Pose]:
    """读轨迹 csv,列 x,y[,theta]。带表头自动识别(首行非数字)。
    远程 A8 用 ros2 topic echo /odom 转存,或自带小 recorder 落 csv。"""
    rows: List[Pose] = []
    with Path(path).open() as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            row = [c.strip() for c in row if c.strip() != ""]
            if not row:
                continue
            try:
                vals = [float(c) for c in row]
            except ValueError:
                if i == 0:
                    continue  # 表头
                raise
            x, y = vals[0], vals[1]
            th = vals[2] if len(vals) >= 3 else None
            rows.append((x, y, th))
    return rows


def parse_polygon_arg(s: str) -> Polygon:
    """命令行 polygon: 'x0,y0;x1,y1;...'"""
    pts: Polygon = []
    for pair in s.split(";"):
        pair = pair.strip()
        if not pair:
            continue
        x, y = pair.split(",")
        pts.append((float(x), float(y)))
    return pts


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------

def visualize(result: CoverageResult, output_png: Path | str) -> None:
    """画 4 色图:可清扫区 / 已扫 / 障碍 / 轨迹。y 轴翻成朝上。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if result.cleanable_mask is None:
        raise ValueError("result 不完整,无 mask")

    H, W = result.cleanable_mask.shape
    rgb = np.ones((H, W, 3), dtype=np.float32)  # 白底
    # 可清扫但没扫到 = 浅灰
    rgb[result.cleanable_mask] = (0.85, 0.85, 0.85)
    # 已扫到的可清扫 = 绿
    rgb[result.covered_mask & result.cleanable_mask] = (0.2, 0.7, 0.2)
    # 障碍 = 黑
    rgb[result.void_mask] = (0.1, 0.1, 0.1)
    # 扫到障碍 = 红(报警)
    rgb[result.covered_mask & result.void_mask] = (0.9, 0.1, 0.1)

    fig, ax = plt.subplots(figsize=(8, 8))
    # origin='lower' 让 row 增大朝上,对齐世界 y 朝上
    ax.imshow(rgb, origin="lower")
    if result.densified_xy is not None and len(result.densified_xy):
        x_min, y_min = result.grid_origin
        res = result.resolution
        cols = (result.densified_xy[:, 0] - x_min) / res
        rows = (result.densified_xy[:, 1] - y_min) / res
        ax.plot(cols, rows, color="blue", linewidth=0.8, alpha=0.7)
    ax.set_title(
        f"Coverage {result.coverage_ratio * 100:.1f}%  "
        f"(green=swept, grey=missed, red=hit obstacle)"
    )
    ax.set_xlabel("x [grid]")
    ax.set_ylabel("y [grid]")
    fig.tight_layout()
    fig.savefig(output_png, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description="洗地机实际覆盖率量化 (footprint sweep)"
    )
    p.add_argument("--traj", required=True, help="轨迹 csv (x,y[,theta])")
    p.add_argument("--area", default=None,
                   help="区域 yaml (outer + voids,或 map_to_polygons 输出)")
    p.add_argument("--outer", default=None,
                   help="外圈 polygon 'x0,y0;x1,y1;...' "
                        "(覆盖 / 补充 area 里的 outer)")
    p.add_argument("--clean-width", type=float, default=0.6,
                   help="清扫横向宽度 m (默认 0.6, 对齐 F2C op_width)")
    p.add_argument("--clean-length", type=float, default=0.2,
                   help="清扫纵向长度 m (默认 0.2)")
    p.add_argument("--offset-x", type=float, default=0.0,
                   help="清扫盘相对 base_link 纵向偏移 m (默认 0)")
    p.add_argument("--res", type=float, default=0.05,
                   help="栅格分辨率 m/格 (默认 0.05)")
    p.add_argument("--viz", default=None, help="(可选) 输出可视化 png")
    args = p.parse_args()

    outer: Polygon = []
    voids: List[Polygon] = []
    if args.area:
        outer, voids = load_area(args.area)
    if args.outer:
        outer = parse_polygon_arg(args.outer)
    if not outer:
        print("错误: 必须通过 --area(含 outer) 或 --outer 给出外圈 polygon",
              file=sys.stderr)
        return 2

    traj = load_trajectory(args.traj)
    fp = Footprint(
        clean_width=args.clean_width,
        clean_length=args.clean_length,
        offset_x=args.offset_x,
    )
    result = measure_coverage(outer, voids, traj, fp, resolution=args.res)
    print(result.summary())

    if args.viz:
        visualize(result, args.viz)
        print(f"可视化 → {args.viz}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
