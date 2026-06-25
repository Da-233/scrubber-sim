"""把 contour_coverage 的 Pose 序列转成 nav_msgs/Path 的 dict 表示

为什么用 dict 不直接出 ROS msg
============================
本包是纯算法库，不依赖 rclpy。远程跑的节点收到 dict 后一行 dict→msg 即可
（geometry_msgs/Quaternion(x,y,z,w) 直接展开），保持本地能跑单测的边界。

dict schema (与 nav_msgs/Path 字段一一对应)
==========================================
{
  "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
  "poses": [
    {
      "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
      "pose": {
        "position": {"x": ..., "y": ..., "z": 0.0},
        "orientation": {"x": 0, "y": 0, "z": sin(yaw/2), "w": cos(yaw/2)}
      }
    },
    ...
  ]
}

yaw=None 时填默认 quaternion (0,0,0,1) (朝 +x)。
"""
from __future__ import annotations
import math
from typing import Sequence

from .path_builder import Pose


def yaw_to_quaternion(yaw: float | None) -> dict:
    """Z 轴旋转 yaw -> quaternion(x,y,z,w)

    yaw=None 用默认朝向 (0,0,0,1)
    """
    if yaw is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    half = yaw / 2.0
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def poses_to_path_dict(
    poses: Sequence[Pose],
    frame_id: str = "map",
    stamp_sec: int = 0,
    stamp_nanosec: int = 0,
) -> dict:
    """List[Pose] -> nav_msgs/Path dict

    Args:
        poses: contour_coverage 输出的 (x, y, yaw_or_None) 序列
        frame_id: TF frame, 默认 "map"
        stamp_sec / stamp_nanosec: 时间戳 (0 = let ROS fill at send time)
    Returns:
        nav_msgs/Path 的 dict 表示, 远程节点直接展平成 msg
    """
    header = {
        "frame_id": frame_id,
        "stamp": {"sec": int(stamp_sec), "nanosec": int(stamp_nanosec)},
    }
    pose_list = []
    for (x, y, yaw) in poses:
        pose_list.append({
            "header": dict(header),
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": yaw_to_quaternion(yaw),
            },
        })
    return {"header": header, "poses": pose_list}
