"""path_linter — 阿卡曼路径可执行性闸门(薄护栏)

为什么有这个工具
================
洗地机踩过最贵的坑:覆盖/路径方案在本地"几何完美 + 单测全绿",一上远程
端到端实跑才卡死(P2 同心环内圈半径 < R_min、P3 贴墙凹角原地转、F2C swath
连接穿障)。单测验"几何对不对",验不了"三轮阿卡曼能不能真跟着走"。

这个 linter 把那些只有端到端才暴露的问题**左移到本地秒级**:路径生成出来
先过四项红绿灯,不达标根本不上远程。它是**薄护栏不是物理仿真**——只查
四件确定性的事,真实动力学/控制器交互仍由 gz 仿真负责。

四项检查
========
1. curvature   : 处处曲率 ≤ 1/R_min(车跟得动)        —— 复用 primitives
2. in_place_turn: 没有 v≈0 但转向的段(阿卡曼转不了)   —— 纯几何扫描
3. out_of_bounds: 整车 footprint 不扫出外墙(给了 outer 才查) —— 复用 coverage_meter
4. obstacle    : 整车 footprint 不压到障碍(给了 outer 才查)  —— 复用 coverage_meter

落地方式(自动闸门)
==================
- 每个产 path 的生成器(K1 generate_lawnmower 等)在测试里 `assert lint_path(...).ok`
- 远程部署脚本上传 path 前先跑,红灯拒绝 scp
见 `项目复盘_2026-06-26` 与记忆 feedback_process_fix_enforcement_hierarchy。

约定
====
- 坐标 m,yaw rad,yaw=0 朝 +x(与 primitives 一致)
- 整车外廓默认 1.4(纵向沿航向) x 1.0(横向),对齐采购清单 1400x1000mm
- 曲率/原地转是纯几何,不依赖 cv2;越界/穿障惰性 import coverage_meter
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from ackermann_primitives.primitives import Pose2D, normalize_angle

Point = tuple[float, float]
Polygon = list[Point]

# 整车外廓(m):1400(纵向沿航向) x 1000(横向),对齐采购清单
DEFAULT_ROBOT_LENGTH = 1.4
DEFAULT_ROBOT_WIDTH = 1.0
# 控制取的最小转弯半径(物理 R_min≈1.0,留 margin 取 1.2)
DEFAULT_R_MIN = 1.2


class PathNotExecutable(ValueError):
    """路径未通过可执行性闸门"""


@dataclass
class LintConfig:
    """linter 参数。

    outer 为 None 时只查曲率/原地转两项(越界/穿障跳过);给了 outer
    才做整车 footprint sweep 查越界/穿障。
    """

    r_min: float = DEFAULT_R_MIN
    curvature_tolerance: float = 1e-3      # 直线接圆弧处离散曲率会略超 1/R
    min_translation: float = 1e-3          # 段位移 < 此值视作"原地"
    inplace_yaw_thresh: float = 1e-3       # 原地段转向 > 此值判原地转
    # 边界 / 障碍(可选)
    outer: Polygon | None = None
    voids: Sequence[Polygon] = ()
    robot_length: float = DEFAULT_ROBOT_LENGTH
    robot_width: float = DEFAULT_ROBOT_WIDTH
    robot_offset_x: float = 0.0
    resolution: float = 0.05
    # 栅格化在边界处有 ±1 格量化噪声,小于此面积不算违规
    area_tolerance_m2: float = 0.02


@dataclass
class LintCheck:
    name: str
    ok: bool
    detail: str
    skipped: bool = False


@dataclass
class LintReport:
    checks: list[LintCheck] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    def failures(self) -> list[LintCheck]:
        return [c for c in self.checks if not c.ok]

    def summary(self) -> str:
        lines = [f"path_linter: {'PASS ✅' if self.ok else 'FAIL ❌'}"]
        for c in self.checks:
            if c.skipped:
                mark = "skip ⏭️"
            else:
                mark = "ok ✅" if c.ok else "FAIL ❌"
            lines.append(f"  [{mark}] {c.name}: {c.detail}")
        return "\n".join(lines)


def _as_xyyaw(p) -> tuple[float, float, float]:
    """接受 Pose2D 或 (x, y, yaw) 三元组。"""
    if isinstance(p, Pose2D):
        return p.x, p.y, p.yaw
    x, y, yaw = p[0], p[1], p[2]
    return float(x), float(y), float(yaw)


def _check_curvature_and_inplace(
    pts: list[tuple[float, float, float]], cfg: LintConfig
) -> tuple[LintCheck, LintCheck]:
    """单次扫描同时得到曲率与原地转两项检查。

    为什么合一次扫描:原地转段(ds≈0)曲率趋于无穷,若混进曲率检查会被
    数值放大成假尖峰,也会和原地转检查重复报。这里把每段先按位移分类——
    位移近零的交给原地转检查、不计曲率;其余正常算 |Δyaw|/Δs 曲率。
    """
    max_allowed = 1.0 / cfg.r_min if cfg.r_min > 0 else math.inf

    if len(pts) < 2:
        c = LintCheck("curvature", True, "路径点 < 2，无需检查")
        i = LintCheck("in_place_turn", True, "路径点 < 2，无需检查")
        return c, i

    max_kappa = 0.0
    max_kappa_idx = -1
    curv_violations = 0
    inplace_idx: list[int] = []

    for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        dyaw = abs(normalize_angle(b[2] - a[2]))
        if ds < cfg.min_translation:
            if dyaw > cfg.inplace_yaw_thresh:
                inplace_idx.append(k)
            continue
        kappa = dyaw / ds
        if kappa > max_kappa:
            max_kappa, max_kappa_idx = kappa, k
        if kappa > max_allowed + cfg.curvature_tolerance:
            curv_violations += 1

    implied_r = (1.0 / max_kappa) if max_kappa > 0 else math.inf
    if curv_violations == 0:
        curv = LintCheck(
            "curvature",
            True,
            f"max κ={max_kappa:.4f} (R≈{implied_r:.2f}m) ≤ 1/R_min={max_allowed:.4f} "
            f"(R_min={cfg.r_min}m)",
        )
    else:
        curv = LintCheck(
            "curvature",
            False,
            f"{curv_violations} 段曲率超限；max κ={max_kappa:.4f} "
            f"(R≈{implied_r:.2f}m < R_min={cfg.r_min}m) @ seg {max_kappa_idx}",
        )

    if not inplace_idx:
        inplace = LintCheck("in_place_turn", True, "无原地转段(阿卡曼可执行)")
    else:
        inplace = LintCheck(
            "in_place_turn",
            False,
            f"{len(inplace_idx)} 段 v≈0 仍转向(阿卡曼转不了)；首个 @ seg {inplace_idx[0]}",
        )
    return curv, inplace


def _check_area(
    pts: list[tuple[float, float, float]], cfg: LintConfig
) -> tuple[LintCheck, LintCheck]:
    """整车 footprint sweep 查越界/穿障,复用 coverage_meter。"""
    if cfg.outer is None or len(cfg.outer) < 3:
        b = LintCheck("out_of_bounds", True, "未提供 outer，跳过", skipped=True)
        o = LintCheck("obstacle", True, "未提供 outer，跳过", skipped=True)
        return b, o

    # 惰性 import:曲率/原地转纯几何不依赖 cv2,只有走到这才需要。
    # coverage_meter 是扁平模块目录(无 __init__.py),把它所在目录挂上
    # sys.path 后按扁平模块导入——与 tests/test_coverage_meter.py 同一约定,
    # 避免命名空间包 vs 扁平模块在不同测试顺序下相互遮蔽。
    import sys
    from pathlib import Path as _Path

    _cm_dir = _Path(__file__).resolve().parent.parent / "coverage_meter"
    if str(_cm_dir) not in sys.path:
        sys.path.insert(0, str(_cm_dir))
    from coverage_meter import Footprint, measure_coverage

    fp = Footprint(
        clean_width=cfg.robot_width,
        clean_length=cfg.robot_length,
        offset_x=cfg.robot_offset_x,
    )
    traj = [(x, y, yaw) for (x, y, yaw) in pts]
    res = measure_coverage(
        list(cfg.outer), list(cfg.voids), traj, fp, resolution=cfg.resolution
    )
    tol = cfg.area_tolerance_m2

    if res.overspray_m2 <= tol:
        bounds = LintCheck(
            "out_of_bounds", True, f"整车未扫出外墙(越界 {res.overspray_m2:.3f} m²)"
        )
    else:
        bounds = LintCheck(
            "out_of_bounds",
            False,
            f"整车扫出外墙 {res.overspray_m2:.3f} m²(掉头外鼓/margin 不足)",
        )

    if res.swept_obstacle_m2 <= tol:
        obst = LintCheck(
            "obstacle", True, f"整车未压障碍(扫障 {res.swept_obstacle_m2:.3f} m²)"
        )
    else:
        obst = LintCheck(
            "obstacle",
            False,
            f"整车压到障碍 {res.swept_obstacle_m2:.3f} m²(穿障/连接段切障)",
        )
    return bounds, obst


def lint_path(poses: Sequence, config: LintConfig | None = None) -> LintReport:
    """对路径跑四项可执行性检查,返回报告(不抛异常)。

    poses: list[Pose2D] 或 list[(x, y, yaw)]。
    config: None 时用默认(R_min=1.2,整车 1.4x1.0,不查边界)。
    """
    cfg = config or LintConfig()
    pts = [_as_xyyaw(p) for p in poses]

    curv, inplace = _check_curvature_and_inplace(pts, cfg)
    bounds, obst = _check_area(pts, cfg)
    return LintReport(checks=[curv, inplace, bounds, obst])


def assert_path_executable(poses: Sequence, config: LintConfig | None = None) -> None:
    """闸门断言:路径不可执行就抛 PathNotExecutable(带失败明细)。

    给生成器测试 / 部署脚本当硬闸门用,语义对齐 primitives.assert_curvature_within。
    """
    report = lint_path(poses, config)
    if not report.ok:
        raise PathNotExecutable(report.summary())
