#!/usr/bin/env python3
"""gen_coverage_goal — 从 area.yaml 生成 NavigateCompleteCoverage goal YAML (A4)

用途
====
M5.3 A4:把"外圈 + 障碍 inner voids"打成 NavigateCompleteCoverage 的 goal。
action 定义注释明确:polygons[0]=外圈, polygons[1..N]=internal voids。
本脚本只**生成 goal YAML 字符串**(打到 stdout),喂给已验证可用的
`ros2 action send_goal --feedback` CLI(M5.2 SUCCEEDED 用的就是这条路)。
不直接发 —— 复用 proven 路径,避免 rclpy action client 的 future/feedback 坑。

输入 area.yaml schema (两种都认)
================================
1. 自带 outer + voids (推荐,A7 手搭场景用):
     outer: [[x,y], ...]
     voids: [[[x,y],...], ...]
2. map_to_polygons 输出(只有 voids,需配 --outer):
     polygons:
       - points: [[x,y], ...]

卷绕约定 (F2C / GML)
====================
- 外圈强制 CCW(逆时针,外环规范)
- inner voids 强制 CW(顺时针,内环规范)
map_to_polygons 已出 CW voids,这里再兜一道保证。

用法
====
    python3 gen_coverage_goal.py area.yaml > /tmp/goal.yaml
    python3 gen_coverage_goal.py polys.yaml --outer "-3,-3;3,-3;3,3;-3,3"
    ros2 action send_goal --feedback /navigate_complete_coverage \
        opennav_coverage_msgs/action/NavigateCompleteCoverage \
        "$(python3 gen_coverage_goal.py area.yaml)"
"""
import argparse
import sys

import yaml


def signed_area(poly):
    """世界系(y朝上)有符号面积。CCW>0, CW<0。"""
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def force_winding(poly, ccw):
    """去掉闭合重复点后判向,按需反转,最后重新闭合。"""
    pts = [tuple(p[:2]) for p in poly]
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        raise ValueError(f"polygon 顶点不足 3 个: {poly}")
    a = signed_area(pts)
    is_ccw = a > 0
    if is_ccw != ccw:
        pts = list(reversed(pts))
    pts.append(pts[0])  # 闭合(F2C 要求首末点相同)
    return pts


def parse_outer_arg(s):
    out = []
    for pair in s.split(";"):
        pair = pair.strip()
        if not pair:
            continue
        x, y = pair.split(",")
        out.append((float(x), float(y)))
    return out


def load_area(path):
    data = yaml.safe_load(open(path))
    if "outer" in data:
        outer = [tuple(p[:2]) for p in data["outer"]]
        voids = [[tuple(p[:2]) for p in poly] for poly in data.get("voids", [])]
        return outer, voids
    if "polygons" in data:
        voids = [[tuple(p[:2]) for p in poly["points"]]
                 for poly in data["polygons"]]
        return None, voids
    raise ValueError(f"无法识别的 area yaml schema: {path}")


def poly_to_yaml(poly):
    pts = ",\n      ".join(
        f"{{x: {x:.4f}, y: {y:.4f}, z: 0.0}}" for x, y in poly
    )
    return "{points: [\n      " + pts + "\n    ]}"


def main():
    ap = argparse.ArgumentParser(description="生成 NavigateCompleteCoverage goal YAML")
    ap.add_argument("area", help="area.yaml (outer+voids 或 map_to_polygons 输出)")
    ap.add_argument("--outer", default=None,
                    help="外圈 'x0,y0;x1,y1;...'(area 无 outer 时必填/覆盖)")
    ap.add_argument("--frame", default="map", help="frame_id(默认 map)")
    args = ap.parse_args()

    outer, voids = load_area(args.area)
    if args.outer:
        outer = parse_outer_arg(args.outer)
    if not outer:
        print("错误: area 里没有 outer,必须用 --outer 提供外圈", file=sys.stderr)
        return 2

    outer = force_winding(outer, ccw=True)
    voids = [force_winding(v, ccw=False) for v in voids]

    polys_yaml = ",\n    ".join(
        poly_to_yaml(p) for p in [outer] + voids
    )
    goal = (
        "{\n"
        "  field_filepath: '',\n"
        f"  polygons: [\n    {polys_yaml}\n  ],\n"
        f"  frame_id: '{args.frame}',\n"
        "  behavior_tree: ''\n"
        "}"
    )
    print(goal)
    # 顶点数摘要打到 stderr(不污染 stdout 的 goal)
    print(f"[gen] outer {len(outer)-1} 点 + {len(voids)} voids "
          f"({', '.join(str(len(v)-1) for v in voids)} 点)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
