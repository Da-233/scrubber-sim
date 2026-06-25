#!/usr/bin/env python3
"""odom_recorder — 录 /odom 到 csv(x,y,theta),给 coverage_meter 出覆盖图用。

用法: python3 odom_recorder.py /tmp/traj.csv
Ctrl-C / SIGTERM 停止,csv 实时 flush。
sim 里 odom≈map(起点重合、漂移小),出俯视图足够。
"""
import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomRecorder(Node):
    def __init__(self, path):
        super().__init__("odom_recorder")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["x", "y", "theta"])
        self.n = 0
        self.create_subscription(Odometry, "/odom", self.cb, 50)

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.w.writerow([f"{p.x:.4f}", f"{p.y:.4f}", f"{yaw:.4f}"])
        self.f.flush()
        self.n += 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/traj.csv"
    rclpy.init()
    node = OdomRecorder(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"录了 {node.n} 个点 → {path}")
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
