"""ackermann_primitives 命令行入口"""
from __future__ import annotations

import argparse

from .boustrophedon import generate_lawnmower
from .primitives import assert_curvature_within, max_curvature, path_length
from .ros_path import write_path_csv


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ackermann_primitives")
    sub = parser.add_subparsers(dest="command", required=True)

    lawn = sub.add_parser("generate-lawnmower", help="generate K1 pure boustrophedon path")
    lawn.add_argument("--width", type=float, required=True)
    lawn.add_argument("--height", type=float, required=True)
    lawn.add_argument("--lane-spacing", type=float, required=True)
    lawn.add_argument("--turn-radius", type=float, required=True)
    lawn.add_argument("--margin", type=float, required=True)
    lawn.add_argument("--step", type=float, default=0.1)
    lawn.add_argument("--max-curvature", type=float, default=None)
    lawn.add_argument("--output", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "generate-lawnmower":
        poses = generate_lawnmower(
            width=args.width,
            height=args.height,
            lane_spacing=args.lane_spacing,
            turn_radius=args.turn_radius,
            margin=args.margin,
            step=args.step,
            max_curvature_allowed=args.max_curvature,
        )
        if args.max_curvature is not None:
            assert_curvature_within(poses, args.max_curvature, tolerance=1e-3)
        write_path_csv(poses, args.output)
        print(
            "generated "
            f"poses={len(poses)} "
            f"length={path_length(poses):.3f} "
            f"max_curvature={max_curvature(poses):.6f} "
            f"output={args.output}"
        )
        return 0

    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
