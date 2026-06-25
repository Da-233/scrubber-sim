#!/usr/bin/env python3
"""build_area_from_map — M1→M2 衔接 CLI: SLAM map.pgm → area.yaml

闭合后把 wall_follower 建出的占据栅格图自动转成 M2 同心环覆盖要的
area.yaml (outer + voids 分开), 消掉手工拆分外墙/障碍的步骤。

数据流
======
    m1_built_map.{yaml,pgm}  (slam_toolbox 存的图)
        │
        │ map_to_polygons.extract_polygons(include_outer=True, morph=0)
        ▼
    List[Polygon]  (外墙 + 障碍 混在一起)
        │
        │ area_builder.split_outer_voids  (最大面积=外墙)
        ▼
    outer + voids
        │
        │ (可选) perimeter_check vs M1 累计里程  [spec §5.1 Phase 1.B]
        ▼
    area.yaml  →  喂 P2 run_contour_coverage.py 跑 M2

⚠️ 真 SLAM map 必须 morph=0 (细墙 1~2px 会被形态学吃光, 见 map_to_polygons 坑)

用法
====
    python3 build_area_from_map.py \\
        --map /home/wmhn/disk2/scrubber_sim/maps/m1_built_map.yaml \\
        --out /home/wmhn/disk2/scrubber_sim/area_m1.yaml \\
        --m1-distance 28.5    # wall_follower 闭合时的累计里程, 触发周长检查

退出码: 0=成功(且周长检查通过 if 提供) / 1=提取失败 / 2=周长检查不过(进 M3)
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import yaml

from map_to_polygons.map_to_polygons import extract_polygons, ExtractParams
from wall_follower.area_builder import (
    split_outer_voids, perimeter_check, polygon_perimeter, polygon_area,
    to_area_yaml_dict,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", required=True, type=Path, help="SLAM map.yaml")
    parser.add_argument("--out", required=True, type=Path, help="输出 area.yaml")
    parser.add_argument("--morph", type=int, default=0,
                        help="形态学核 px, 真 SLAM map 必须 0 (默认)")
    parser.add_argument("--min-area", type=float, default=0.05, help="最小障碍面积 m²")
    parser.add_argument("--approx-eps", type=float, default=0.05, help="多边形简化 eps m")
    parser.add_argument("--m1-distance", type=float, default=None,
                        help="wall_follower 闭合累计里程 m; 给了就做周长合理性检查")
    parser.add_argument("--perimeter-tol", type=float, default=0.20,
                        help="周长 vs 里程 相对误差容差, 默认 0.20")
    args = parser.parse_args()

    # 1. 提取所有 polygon (含外墙)
    params = ExtractParams(
        include_outer_boundary=True,
        morph_kernel_px=args.morph,
        min_area_m2=args.min_area,
        approx_eps_m=args.approx_eps,
    )
    result = extract_polygons(args.map, params)
    polys = result.polygons_world
    print(f"[build_area] extracted {len(polys)} polygons (含外墙)")
    if not polys:
        print("[build_area] ERROR: 0 polygon, SLAM 图可能没建好或全 unknown")
        return 1

    # 2. 拆 outer / voids
    outer, voids = split_outer_voids(polys)
    if not outer:
        print("[build_area] ERROR: 无有效外墙 polygon")
        return 1
    print(f"[build_area] outer: {len(outer)}pts, 面积 {polygon_area(outer):.2f}m², "
          f"周长 {polygon_perimeter(outer):.2f}m")
    print(f"[build_area] voids: {len(voids)} 个")

    # 3. (可选) Phase 1.B 周长合理性检查
    if args.m1_distance is not None:
        ok, rel_err = perimeter_check(outer, args.m1_distance, args.perimeter_tol)
        print(f"[build_area] 周长检查: 外墙 {polygon_perimeter(outer):.2f}m vs "
              f"M1 里程 {args.m1_distance:.2f}m, 相对误差 {rel_err*100:.1f}% "
              f"(容差 {args.perimeter_tol*100:.0f}%) {'✅' if ok else '❌'}")
        if not ok:
            print("[build_area] 周长检查不过 → 该进 M3-Mode (外墙提取可疑)")
            # 仍写出 area.yaml 供人工检查, 但退出码标失败
            _write(args.out, outer, voids)
            return 2

    # 4. 写 area.yaml
    _write(args.out, outer, voids)
    print(f"[build_area] wrote {args.out}")
    print(f"[build_area] 下一步: run_contour_coverage.py --area {args.out}")
    return 0


def _write(path: Path, outer, voids):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(to_area_yaml_dict(outer, voids), allow_unicode=True))


if __name__ == "__main__":
    sys.exit(main())
