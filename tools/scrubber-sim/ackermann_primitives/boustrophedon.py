"""K1 纯弓字覆盖路径生成器

第一版只做大房间、无障碍、水平扫线：
- 直线清扫段
- 端部半圆 U 掉头
- 不做外围贴边环
- 不做 K-turn / 原地转

关键约束：相邻扫线间距必须等于 2 * turn_radius。
这样端部连接就是标准半圆，路径曲率天然不超过 1 / turn_radius。
"""
from __future__ import annotations

from .primitives import Pose2D, assert_curvature_within, sample_line, sample_u_turn


class BoustrophedonError(ValueError):
    """弓字路径生成失败"""


def _append_without_duplicate(dst: list[Pose2D], src: list[Pose2D]) -> None:
    if not src:
        return
    if dst and dst[-1] == src[0]:
        dst.extend(src[1:])
    else:
        dst.extend(src)


def _validate_positive(name: str, value: float) -> None:
    if value <= 0.0:
        raise BoustrophedonError(f"{name} must be > 0, got {value}")


def generate_lawnmower(
    *,
    width: float,
    height: float,
    lane_spacing: float,
    turn_radius: float,
    margin: float,
    step: float = 0.1,
    max_curvature_allowed: float | None = None,
) -> list[Pose2D]:
    """生成大房间纯弓字路径

    坐标系以房间中心为原点，房间范围：
    - x ∈ [-width/2, width/2]
    - y ∈ [-height/2, height/2]

    路径约束：
    - 所有点都在 margin 内缩后的矩形内
    - 起点在左下角安全区，先向 +x 清扫
    - 相邻扫线用端部半圆连接
    - lane_spacing 必须等于 2*turn_radius

    Args:
        width: 房间宽度。
        height: 房间高度。
        lane_spacing: 相邻清扫线间距；K1 初版要求等于 2*turn_radius。
        turn_radius: U 掉头半径。
        margin: 离外墙安全距。
        step: 采样步长。
        max_curvature_allowed: 可选曲率上限；默认 1/turn_radius。

    Returns:
        Pose2D 列表。
    """

    for name, value in (
        ("width", width),
        ("height", height),
        ("lane_spacing", lane_spacing),
        ("turn_radius", turn_radius),
        ("step", step),
    ):
        _validate_positive(name, value)
    if margin < 0.0:
        raise BoustrophedonError(f"margin must be >= 0, got {margin}")

    expected_spacing = 2.0 * turn_radius
    if abs(lane_spacing - expected_spacing) > 1e-6:
        raise BoustrophedonError(
            "K1 initial generator requires lane_spacing == 2 * turn_radius; "
            f"got lane_spacing={lane_spacing}, turn_radius={turn_radius}"
        )

    x_min = -width / 2.0 + margin + turn_radius
    x_max = width / 2.0 - margin - turn_radius
    y_min = -height / 2.0 + margin
    y_max = height / 2.0 - margin

    usable_width = x_max - x_min
    usable_height = y_max - y_min
    if usable_width <= 0.0 or usable_height <= 0.0:
        raise BoustrophedonError("room too small after applying margin and turn radius")
    if usable_height < lane_spacing:
        raise BoustrophedonError(
            "room too small for at least one Ackermann U-turn lane change"
        )

    lane_count = int(usable_height // lane_spacing) + 1
    if lane_count < 2:
        raise BoustrophedonError("room too small for multiple sweeps")

    path: list[Pose2D] = []
    current = Pose2D(x_min, y_min, 0.0)

    for lane_idx in range(lane_count):
        going_right = lane_idx % 2 == 0
        line_length = usable_width
        line = sample_line(current, line_length, step=step)
        _append_without_duplicate(path, line)
        current = path[-1]

        if lane_idx == lane_count - 1:
            break

        # 半圆 U 掉头：
        # - 向右行驶到右端，左转半圆，y 增加 2R，车头朝 -x
        # - 向左行驶到左端，右转半圆，y 增加 2R，车头朝 +x
        direction = "left" if going_right else "right"
        turn = sample_u_turn(current, radius=turn_radius, step=step, direction=direction)
        _append_without_duplicate(path, turn)
        current = path[-1]

        if current.y > y_max + 1e-6:
            raise BoustrophedonError("generated path exceeds usable room height")

    allowed = max_curvature_allowed if max_curvature_allowed is not None else 1.0 / turn_radius
    try:
        assert_curvature_within(path, allowed, tolerance=1e-3)
    except ValueError as exc:
        raise BoustrophedonError(str(exc)) from exc

    return path
