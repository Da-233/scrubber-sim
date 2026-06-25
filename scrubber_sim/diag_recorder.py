#!/usr/bin/env python3
"""diag_recorder — A8 诊断录制器
录 /odom (车在 odom 系) + /plan (F2C 给 RPP 的 path) + map→base TF (车在 map 系)。
三路落 csv 便于离线对照:F2C 给的什么、RPP 跟到哪、map 系真位置。
"""
import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class DiagRec(Node):
    def __init__(self, outdir):
        super().__init__("diag_recorder")
        self.f_odom = open(f"{outdir}/odom.csv", "w", newline="")
        self.w_odom = csv.writer(self.f_odom)
        self.w_odom.writerow(["t", "x", "y", "theta"])

        self.f_map = open(f"{outdir}/map_pose.csv", "w", newline="")
        self.w_map = csv.writer(self.f_map)
        self.w_map.writerow(["t", "x", "y", "theta"])

        self.f_plan = open(f"{outdir}/plans.csv", "w", newline="")
        self.w_plan = csv.writer(self.f_plan)
        self.w_plan.writerow(["plan_idx", "pt_idx", "x", "y"])
        self.plan_idx = 0
        self.last_plan_len = -1

        self.create_subscription(Odometry, "/odom", self.cb_odom, 50)
        self.create_subscription(Path, "/plan", self.cb_plan, 10)
        # opennav_coverage 也可能用 /received_global_plan
        self.create_subscription(Path, "/received_global_plan",
                                 self.cb_plan_rcv, 10)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_timer(0.1, self.cb_tf)

        self.n_odom = 0
        self.n_map = 0

    def cb_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        yaw = yaw_from_q(m.pose.pose.orientation)
        self.w_odom.writerow([f"{t:.3f}", f"{p.x:.4f}", f"{p.y:.4f}",
                              f"{yaw:.4f}"])
        self.f_odom.flush()
        self.n_odom += 1

    def cb_tf(self):
        try:
            tr = self.tf_buf.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception:
            return
        t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        p = tr.transform.translation
        yaw = yaw_from_q(tr.transform.rotation)
        self.w_map.writerow([f"{t:.3f}", f"{p.x:.4f}", f"{p.y:.4f}",
                             f"{yaw:.4f}"])
        self.f_map.flush()
        self.n_map += 1

    def _write_plan(self, msg, tag):
        # 只写新 plan(长度变化才视为新一条;F2C+nav2 会高频重发同一份)
        if len(msg.poses) == self.last_plan_len:
            return
        self.last_plan_len = len(msg.poses)
        self.plan_idx += 1
        for i, ps in enumerate(msg.poses):
            self.w_plan.writerow([
                f"{self.plan_idx}_{tag}", i,
                f"{ps.pose.position.x:.4f}",
                f"{ps.pose.position.y:.4f}",
            ])
        self.f_plan.flush()
        self.get_logger().info(
            f"[plan#{self.plan_idx}/{tag}] {len(msg.poses)} 点"
        )

    def cb_plan(self, m):
        self._write_plan(m, "plan")

    def cb_plan_rcv(self, m):
        self._write_plan(m, "rcv")


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/diag"
    import os
    os.makedirs(outdir, exist_ok=True)
    rclpy.init()
    node = DiagRec(outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"odom={node.n_odom}, map_pose={node.n_map}, plans={node.plan_idx}"
        )
        for f in (node.f_odom, node.f_map, node.f_plan):
            f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
