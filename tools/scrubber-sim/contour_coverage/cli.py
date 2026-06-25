"""命令行入口: area.yaml → 同心环路径 + JSON + 可视化预览

用法:
    python3 -m contour_coverage.cli --area path/to/area_7x7.yaml \\
        --spacing 0.55 --margin 0.6 --out path.json --preview preview.png
"""
import argparse
import json
import sys
from pathlib import Path

# 引入项目惯用的 area 加载器 (与 coverage_meter 兼容)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coverage_meter.coverage_meter import load_area

from .geometry import generate_concentric_rings, OffsetParams
from .path_builder import build_path, PathParams


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--area", required=True, type=Path, help="area.yaml 路径")
    parser.add_argument("--spacing", type=float, default=0.55, help="环间距 (m), 默认 0.55")
    parser.add_argument("--margin", type=float, default=0.6, help="安全 margin (m), 默认 0.6")
    parser.add_argument("--densify-step", type=float, default=0.1, help="路径点间距 (m), 默认 0.1")
    parser.add_argument("--join", choices=["round", "mitre", "bevel"], default="round",
                        help="拐角风格, 默认 round")
    parser.add_argument("--out", type=Path, help="输出 JSON 路径 (省略则只打印统计)")
    parser.add_argument("--preview", type=Path, help="可视化 PNG 路径 (省略则不画)")
    args = parser.parse_args()

    # 1. 加载 area
    outer, voids = load_area(args.area)
    print(f"[contour_coverage] area: outer={len(outer)}pts, voids={len(voids)}")

    # 2. 生成同心环
    op = OffsetParams(
        spacing=args.spacing,
        safety_margin=args.margin,
        join_style=args.join,
    )
    rings = generate_concentric_rings(outer, voids, op)
    print(f"[contour_coverage] rings: {len(rings)} (depths: {sorted(set(r.depth for r in rings))})")

    # 3. 构造 pose 序列
    pp = PathParams(densify_step=args.densify_step)
    poses = build_path(rings, pp)
    print(f"[contour_coverage] poses: {len(poses)}")

    # 4. 输出 JSON
    if args.out:
        data = {
            "poses": [{"x": x, "y": y, "yaw": yaw} for (x, y, yaw) in poses],
            "params": {
                "spacing": args.spacing, "margin": args.margin,
                "densify_step": args.densify_step, "join": args.join,
            },
            "rings_count": len(rings),
        }
        args.out.write_text(json.dumps(data, indent=2))
        print(f"[contour_coverage] wrote {args.out}")

    # 5. 可视化
    if args.preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MPoly

        fig, ax = plt.subplots(figsize=(10, 10))
        # 外墙
        ox = [p[0] for p in outer]; oy = [p[1] for p in outer]
        ax.plot(ox, oy, "-", color="#444", lw=2.5, label="outer")
        # 障碍
        for v in voids:
            ax.add_patch(MPoly(v[:-1], closed=True, facecolor="#888", edgecolor="k", alpha=0.7))
        # 环 (按 depth 渐变色)
        max_depth = max((r.depth for r in rings), default=0)
        cmap = plt.get_cmap("viridis")
        for r in rings:
            color = cmap(r.depth / max(max_depth, 1))
            rx = [p[0] for p in r.coords]; ry = [p[1] for p in r.coords]
            ax.plot(rx, ry, "-", color=color, lw=1.2, alpha=0.8)
        # 路径 (pose 序列起点)
        if poses:
            px = [p[0] for p in poses]; py = [p[1] for p in poses]
            ax.plot(px, py, "-", color="#7B2D8E", lw=0.6, alpha=0.5, label="path")
            ax.plot(px[0], py[0], "o", color="green", ms=10, label="start")
            ax.plot(px[-1], py[-1], "s", color="red", ms=10, label="end")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=9)
        ax.set_title(f"contour_coverage: {len(rings)} rings, {len(poses)} poses")
        plt.savefig(args.preview, dpi=130, bbox_inches="tight")
        print(f"[contour_coverage] wrote preview {args.preview}")


if __name__ == "__main__":
    main()
