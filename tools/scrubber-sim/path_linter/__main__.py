"""path_linter CLI —— 上传 path 到远程前的本地闸门。

用法:
    python -m path_linter PATH.csv [--area AREA.yaml] [--r-min 1.2] \
        [--robot-length 1.4] [--robot-width 1.0]

退出码:0 = 四项全过(或越界/穿障因无 outer 跳过);1 = 有失败。
部署脚本里 `python -m path_linter k1.csv --area area.yaml || exit 1` 即可红灯拒绝 scp。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ackermann_primitives.ros_path import read_path_csv

from .path_linter import LintConfig, lint_path


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="阿卡曼路径可执行性闸门")
    p.add_argument("path_csv", help="路径 CSV (x,y,yaw/theta)")
    p.add_argument(
        "--area",
        default=None,
        help="区域 yaml (outer + voids);给了才查越界/穿障",
    )
    p.add_argument("--r-min", type=float, default=LintConfig.r_min,
                   help=f"最小转弯半径 m (默认 {LintConfig.r_min})")
    p.add_argument("--robot-length", type=float, default=LintConfig.robot_length,
                   help="整车纵向长 m (默认 1.4)")
    p.add_argument("--robot-width", type=float, default=LintConfig.robot_width,
                   help="整车横向宽 m (默认 1.0)")
    p.add_argument("--res", type=float, default=LintConfig.resolution,
                   help="越界/穿障栅格分辨率 m/格 (默认 0.05)")
    args = p.parse_args(argv)

    poses = read_path_csv(args.path_csv)

    outer = None
    voids: list = []
    if args.area:
        # 复用 coverage_meter 的区域读取(扁平模块,挂目录后导入)
        cm_dir = Path(__file__).resolve().parent.parent / "coverage_meter"
        if str(cm_dir) not in sys.path:
            sys.path.insert(0, str(cm_dir))
        from coverage_meter import load_area  # type: ignore

        outer, voids = load_area(args.area)
        if not outer:
            print("⚠️  area yaml 无 outer，越界/穿障将跳过", file=sys.stderr)
            outer = None

    cfg = LintConfig(
        r_min=args.r_min,
        outer=outer,
        voids=voids,
        robot_length=args.robot_length,
        robot_width=args.robot_width,
        resolution=args.res,
    )
    report = lint_path(poses, cfg)
    print(report.summary())
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
