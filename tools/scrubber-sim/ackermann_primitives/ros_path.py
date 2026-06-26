"""ROS Path 友好的路径导出工具

本模块不依赖 ROS 运行时，只把 Pose2D 转为通用行格式，方便：
- 写 CSV 给远程 runner 读取
- 后续转换成 nav_msgs/Path
"""
from __future__ import annotations

import csv
from pathlib import Path

from .primitives import Pose2D


PathRow = tuple[float, float, float]


def poses_to_rows(poses: list[Pose2D]) -> list[PathRow]:
    """Pose2D 列表转 CSV/ROS 友好的 (x, y, yaw) 行"""

    return [(float(p.x), float(p.y), float(p.yaw)) for p in poses]


def write_path_csv(poses: list[Pose2D], path: str | Path) -> None:
    """写出 x,y,yaw CSV"""

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        writer.writerows(poses_to_rows(poses))


def read_path_csv(path: str | Path) -> list[Pose2D]:
    """读取 x,y,yaw/theta CSV 为 Pose2D"""

    poses: list[Pose2D] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            yaw_key = "yaw" if "yaw" in row else "theta"
            poses.append(Pose2D(float(row["x"]), float(row["y"]), float(row[yaw_key])))
    if not poses:
        raise ValueError(f"empty path CSV: {path}")
    return poses
