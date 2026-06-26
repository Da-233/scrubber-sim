"""V1 路径导出/CLI 测试"""
import csv

import pytest

from ackermann_primitives.boustrophedon import BoustrophedonError, generate_lawnmower
from ackermann_primitives.cli import main
from ackermann_primitives.primitives import Pose2D, max_curvature
from ackermann_primitives.ros_path import poses_to_rows, read_path_csv, write_path_csv


def test_poses_to_rows_preserves_xy_yaw():
    poses = [Pose2D(1.0, 2.0, 0.1), Pose2D(3.0, 4.0, -0.2)]

    assert poses_to_rows(poses) == [(1.0, 2.0, 0.1), (3.0, 4.0, -0.2)]


def test_write_and_read_path_csv_roundtrip(tmp_path):
    poses = [Pose2D(1.0, 2.0, 0.1), Pose2D(3.0, 4.0, -0.2)]
    out = tmp_path / "path.csv"

    write_path_csv(poses, out)
    loaded = read_path_csv(out)

    assert loaded == poses
    with open(out, newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0] == ["x", "y", "yaw"]


def test_read_path_csv_accepts_theta_column(tmp_path):
    out = tmp_path / "path.csv"
    out.write_text("x,y,theta\n1,2,0.5\n", encoding="utf-8")

    assert read_path_csv(out) == [Pose2D(1.0, 2.0, 0.5)]


def test_generate_lawnmower_cli_writes_curvature_limited_csv(tmp_path):
    out = tmp_path / "k1.csv"

    rc = main([
        "generate-lawnmower",
        "--width", "14",
        "--height", "12",
        "--lane-spacing", "4",
        "--turn-radius", "2",
        "--margin", "2",
        "--step", "0.2",
        "--max-curvature", "0.5",
        "--output", str(out),
    ])

    assert rc == 0
    poses = read_path_csv(out)
    assert len(poses) > 20
    assert max_curvature(poses) <= 0.5 + 1e-3


def test_cli_rejects_bad_lawnmower_params(tmp_path):
    out = tmp_path / "bad.csv"

    with pytest.raises(BoustrophedonError):
        main([
            "generate-lawnmower",
            "--width", "14",
            "--height", "12",
            "--lane-spacing", "1",
            "--turn-radius", "2",
            "--margin", "2",
            "--output", str(out),
        ])


def test_generated_csv_from_api_is_readable(tmp_path):
    poses = generate_lawnmower(
        width=14.0,
        height=12.0,
        lane_spacing=4.0,
        turn_radius=2.0,
        margin=2.0,
        step=0.2,
    )
    out = tmp_path / "path.csv"

    write_path_csv(poses, out)
    loaded = read_path_csv(out)

    assert loaded[0] == poses[0]
    assert loaded[-1] == poses[-1]
    assert len(loaded) == len(poses)
