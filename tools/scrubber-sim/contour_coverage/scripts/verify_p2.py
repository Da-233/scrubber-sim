#!/usr/bin/env python3
"""verify_p2 — P2 仿真验证: rosbag/CSV → coverage_meter → 三条判据

输入: 远程 rosbag 导出的轨迹 CSV (x,y,theta) + area.yaml
输出:
    - stdout: 覆盖率 / 扫障面积 / 越界面积 三条判据 ✅/❌
    - PNG: coverage_meter 的染色图 (可清扫区 / 已扫 / 障碍 / 轨迹)
    - 退出码: 0=全过, 1=任意一条不达标

判据 (spec §7.3):
    覆盖率 ≥ 90%
    扫障面积 = 0
    越界面积 = 0

用法 (本地, 拿到远程 traj.csv 后):
    python3 verify_p2.py \\
        --traj /tmp/traj.csv \\
        --area projects/自动洗地机/.../area_7x7.yaml \\
        --out figures/2026-06-25-P2同心轮廓覆盖_仿真验证.png

把 /tf map→base_link 转 CSV 的远程一行命令 (RUNBOOK 详):
    ros2 bag export ...  # 或 python3 + rosbag2_py 提帧

或更简单: P2 跑节点时让 run_contour_coverage.py 自己 sub /odom 落 csv (TODO option)
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from coverage_meter.coverage_meter import (
    load_area, load_trajectory, measure_coverage, visualize, Footprint,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--traj", required=True, type=Path, help="轨迹 CSV (x,y,theta)")
    parser.add_argument("--area", required=True, type=Path, help="area.yaml (outer + voids)")
    parser.add_argument("--clean-width", type=float, default=0.6, help="清扫刷横向宽 m")
    parser.add_argument("--clean-length", type=float, default=0.2, help="清扫刷纵向长 m")
    parser.add_argument("--offset-x", type=float, default=0.0, help="footprint 在 base_link 的 x 偏移")
    parser.add_argument("--resolution", type=float, default=0.05, help="栅格分辨率 m/cell")
    parser.add_argument("--out", type=Path, default=Path("p2_coverage.png"), help="输出 PNG")
    parser.add_argument("--coverage-threshold", type=float, default=0.90,
                        help="覆盖率判据阈值, 默认 0.90 (spec §7.3)")
    args = parser.parse_args()

    print(f"[verify_p2] traj: {args.traj}")
    print(f"[verify_p2] area: {args.area}")

    outer, voids = load_area(args.area)
    print(f"[verify_p2] area: outer={len(outer)}pts, voids={len(voids)}")

    traj = load_trajectory(args.traj)
    print(f"[verify_p2] traj: {len(traj)} poses")
    if len(traj) < 2:
        print("[verify_p2] ERROR: trajectory too short")
        return 1

    fp = Footprint(
        clean_width=args.clean_width,
        clean_length=args.clean_length,
        offset_x=args.offset_x,
    )
    result = measure_coverage(outer, voids, traj, footprint=fp, resolution=args.resolution)

    # 计算三条判据
    cleanable_area = result.cleanable_area_m2
    covered_area = result.covered_area_m2
    coverage = result.coverage_ratio
    swept_obstacle = result.swept_obstacle_m2
    overspray = result.overspray_m2

    pass_cov = coverage >= args.coverage_threshold
    pass_obs = swept_obstacle <= 1e-9  # 严格=0 (允许浮点残留)
    pass_out = overspray <= 1e-9

    # 报告
    print()
    print("=== P2 判据 (spec §7.3) ===")
    print(f"  可清扫区面积: {cleanable_area:.3f} m²")
    print(f"  已扫面积:    {covered_area:.3f} m²")
    print(f"  [1] 覆盖率: {coverage*100:.2f}% (阈值 {args.coverage_threshold*100:.0f}%)  "
          f"{'✅' if pass_cov else '❌'}")
    print(f"  [2] 扫障面积: {swept_obstacle:.4f} m²  "
          f"{'✅' if pass_obs else '❌'}")
    print(f"  [3] 越界面积: {overspray:.4f} m²  "
          f"{'✅' if pass_out else '❌'}")
    print()

    # 出图
    args.out.parent.mkdir(parents=True, exist_ok=True)
    visualize(result, args.out)
    print(f"[verify_p2] wrote {args.out}")

    all_pass = pass_cov and pass_obs and pass_out
    if all_pass:
        print("=== P2 PASS (3/3) ===")
        return 0
    else:
        failed = []
        if not pass_cov: failed.append("覆盖率")
        if not pass_obs: failed.append("扫障")
        if not pass_out: failed.append("越界")
        print(f"=== P2 FAIL: 不达标项 = {', '.join(failed)} ===")
        return 1


if __name__ == "__main__":
    sys.exit(main())
