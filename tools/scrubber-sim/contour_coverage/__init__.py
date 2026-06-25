"""contour_coverage — 同心轮廓覆盖路径生成器

输入: outer polygon + voids polygon list (项目惯例 Polygon = List[Point])
输出: 同心环 + 段间连接的 pose 序列, 喂 Nav2 FollowPath

详见 spec: projects/自动洗地机/03_软件架构/specs/2026-06-25-同心轮廓覆盖引擎-设计.md
"""

from .geometry import generate_concentric_rings, OffsetParams, Ring
from .path_builder import build_path, PathParams
from .ros_bridge import poses_to_path_dict, yaw_to_quaternion

__all__ = [
    "generate_concentric_rings", "OffsetParams", "Ring",
    "build_path", "PathParams",
    "poses_to_path_dict", "yaw_to_quaternion",
]
