#!/usr/bin/env python3
"""run_contour_coverage — 生成同心环路径 + 直接 send_goal 给 RPP /follow_path

这个脚本在远程 chroot 内跑(本地无 rclpy 不能运行)。

数据流
======
    area.yaml
        │
        ▼
    generate_concentric_rings + build_path  (纯几何, 不涉 ROS)
        │
        ▼
    poses_to_path_dict  (扁平 dict)
        │
        ▼
    本脚本: dict → nav_msgs/Path msg → FollowPath ActionClient.send_goal
        │
        ▼
    RPP controller_server 拉机器人走

绕过 bt_navigator / opennav_coverage —— spec §6 "完全绕过 F2C/opennav_coverage,
自己出 Path 喂下游 RPP FollowPath (下游不动)"。

前置
====
- controller_server 已启动并 activate (lifecycle ACTIVE)
- /follow_path action server 已 ready
- TF map→odom→base_link 链已通 (slam_toolbox 或 map_server + amcl)
- 机器人起点在 area outer 内部 (用 ros2 service set_initial_pose 或 spawn 时摆好)

用法
====
    python3 run_contour_coverage.py \\
        --area /home/wmhn/disk2/scrubber_sim/area_7x7.yaml \\
        --spacing 0.55 --margin 0.6

退出码
======
    0  SUCCEEDED
    1  ABORTED / 中途 fail
    2  CANCELED
    3  send_goal 被拒
    4  action server 起不来 / 超时
"""
from __future__ import annotations
import argparse
import csv
import math
import sys
import time
from pathlib import Path

# 让脚本同时支持源码 in-place 跑 + 模块导入两种姿势
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from contour_coverage.geometry import generate_concentric_rings, OffsetParams
from contour_coverage.path_builder import build_path, PathParams
from contour_coverage.ros_bridge import poses_to_path_dict
from coverage_meter.coverage_meter import load_area

# --- ROS imports (仅远程才能 import) ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from nav_msgs.msg import Path as PathMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point as PointMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import FollowPath


def quaternion_to_yaw(q) -> float:
    """Z-axis yaw from quaternion (与 ros_bridge.yaw_to_quaternion 反向)"""
    # yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²)); 但平面运动 x=y=0
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def dict_to_path_msg(d: dict) -> PathMsg:
    """ros_bridge 的 dict 表示 → nav_msgs/Path msg

    ★ stamp 一律置 0: nav2 RPP 收到 stamp=0 的 path 用 "latest available
    transform", 避免与 controller 的 sim_time 做严格时间匹配 (盖墙钟会触发
    "Transform data too old", RPP 容错直接误判 Reached the goal)。
    """
    zero = TimeMsg()  # sec=0, nanosec=0
    msg = PathMsg()
    msg.header.frame_id = d["header"]["frame_id"]
    msg.header.stamp = zero

    for p in d["poses"]:
        ps = PoseStamped()
        ps.header.frame_id = p["header"]["frame_id"]
        ps.header.stamp = zero
        ps.pose.position = PointMsg(x=p["pose"]["position"]["x"],
                                     y=p["pose"]["position"]["y"],
                                     z=p["pose"]["position"]["z"])
        q = p["pose"]["orientation"]
        ps.pose.orientation = Quaternion(x=q["x"], y=q["y"], z=q["z"], w=q["w"])
        msg.poses.append(ps)
    return msg


class ContourCoverageRunner(Node):
    def __init__(self, action_name: str, controller_id: str, goal_checker_id: str,
                 log_traj_path: str | None = None, odom_topic: str = "/odom"):
        super().__init__(
            "contour_coverage_runner",
            # ★ 跟 controller 一致用 sim_time, 否则 feedback/TF 时间错位
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._action_name = action_name
        self._controller_id = controller_id
        self._goal_checker_id = goal_checker_id
        self._client = ActionClient(self, FollowPath, action_name)

        # feedback 节流: 每 N 个回调打一次
        self._feedback_count = 0
        self._feedback_throttle = 20

        self.result_code = None  # 4=SUCCEEDED 5=CANCELED 6=ABORTED (action_msgs/GoalStatus)
        self.done = False

        # 轨迹记录 (用于 verify_p2 量化)
        self._traj_file = None
        self._traj_writer = None
        if log_traj_path:
            self._traj_file = open(log_traj_path, "w", newline="")
            self._traj_writer = csv.writer(self._traj_file)
            self._traj_writer.writerow(["x", "y", "theta"])
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.get_logger().info(f"logging trajectory: {log_traj_path} (sub {odom_topic})")

    def wait_for_server(self, timeout_sec: float = 15.0) -> bool:
        self.get_logger().info(f"waiting for action server: {self._action_name} ...")
        ok = self._client.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().error(f"action server {self._action_name} not available")
        return ok

    def send_path(self, path_msg: PathMsg):
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self.get_logger().info(
            f"sending goal: {len(path_msg.poses)} poses, "
            f"controller_id={self._controller_id}, "
            f"goal_checker_id={self._goal_checker_id}"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb):
        self._feedback_count += 1
        if self._feedback_count % self._feedback_throttle != 0:
            return
        f = fb.feedback
        self.get_logger().info(
            f"[feedback] distance_to_goal={f.distance_to_goal:.2f}m "
            f"speed={f.speed:.2f}m/s"
        )

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("goal rejected by server")
            self.result_code = -1
            self.done = True
            return
        self.get_logger().info("goal accepted, waiting for result ...")
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        r = future.result()
        self.result_code = r.status  # 4=SUCCEEDED 5=CANCELED 6=ABORTED
        self.get_logger().info(f"result status={self.result_code}")
        self.done = True

    def _on_odom(self, msg: Odometry):
        if self._traj_writer is None:
            return
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._traj_writer.writerow([f"{p.x:.4f}", f"{p.y:.4f}", f"{yaw:.4f}"])

    def close_traj(self):
        if self._traj_file:
            self._traj_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--area", required=True, type=Path, help="area.yaml")
    parser.add_argument("--spacing", type=float, default=0.55)
    parser.add_argument("--margin", type=float, default=0.6)
    parser.add_argument("--densify-step", type=float, default=0.1)
    parser.add_argument("--inner-to-outer", action="store_true",
                        help="从内圈向外螺旋 (机器人中心 spawn 时用, 终点落外圈, "
                             "避免终点≈起点被 goal_checker 秒判到达)")
    parser.add_argument("--join", choices=["round", "mitre", "bevel"], default="round")
    parser.add_argument("--action", default="/follow_path", help="FollowPath action name")
    parser.add_argument("--controller-id", default="FollowPath",
                        help="controller plugin id (匹配 controller_server config)")
    parser.add_argument("--goal-checker-id", default="general_goal_checker",
                        help="goal checker id")
    parser.add_argument("--server-timeout", type=float, default=15.0)
    parser.add_argument("--action-timeout", type=float, default=600.0,
                        help="等待 action 完成的最长秒数")
    parser.add_argument("--log-traj", type=Path, default=None,
                        help="记录 /odom 到 CSV (x,y,theta), 给 verify_p2 用")
    parser.add_argument("--odom-topic", default="/odom",
                        help="轨迹订阅话题, 默认 /odom (wheel_odometry 输出)")
    args = parser.parse_args()

    # --- 几何计算 (纯本地, 失败立刻退出, 不连 ROS) ---
    outer, voids = load_area(args.area)
    print(f"[geo] area: outer={len(outer)}pts, voids={len(voids)}")

    rings = generate_concentric_rings(
        outer, voids,
        OffsetParams(spacing=args.spacing, safety_margin=args.margin, join_style=args.join),
    )
    print(f"[geo] rings: {len(rings)}")
    if not rings:
        print("[geo] ERROR: no rings generated (margin 过大? area 退化?)")
        return 4

    poses = build_path(rings, PathParams(densify_step=args.densify_step,
                                         inner_to_outer=args.inner_to_outer))
    print(f"[geo] poses: {len(poses)}")
    if not poses:
        print("[geo] ERROR: empty pose sequence")
        return 4

    path_dict = poses_to_path_dict(poses, frame_id="map")

    # --- ROS 连接 ---
    rclpy.init()
    node = ContourCoverageRunner(
        args.action, args.controller_id, args.goal_checker_id,
        log_traj_path=str(args.log_traj) if args.log_traj else None,
        odom_topic=args.odom_topic,
    )

    try:
        if not node.wait_for_server(args.server_timeout):
            return 4

        path_msg = dict_to_path_msg(path_dict)
        node.send_path(path_msg)

        start = time.time()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > args.action_timeout:
                node.get_logger().error(f"action timeout after {args.action_timeout}s")
                return 4

        # action_msgs/GoalStatus: 4=SUCCEEDED 5=CANCELED 6=ABORTED
        if node.result_code == 4:
            print("[ros] SUCCEEDED")
            return 0
        elif node.result_code == 6:
            print("[ros] ABORTED")
            return 1
        elif node.result_code == 5:
            print("[ros] CANCELED")
            return 2
        else:
            print(f"[ros] unexpected status={node.result_code}")
            return 3
    finally:
        node.close_traj()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
