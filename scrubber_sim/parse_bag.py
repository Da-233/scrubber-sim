#!/usr/bin/env python3
# 解析 A8 bag，导出 coverage_plan / swaths / planning_field / field_boundary 到 csv
# 在 chroot 内跑（需 ros2 环境）
import sys, csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PolygonStamped

bag_path = sys.argv[1]
out_dir = sys.argv[2]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
    rosbag2_py.ConverterOptions("", ""),
)
type_map = {
    "/coverage_server/coverage_plan": ("Path", Path),
    "/coverage_server/swaths": ("Marker", Marker),
    "/coverage_server/planning_field": ("Poly", PolygonStamped),
    "/coverage_server/field_boundary": ("Poly", PolygonStamped),
}
done = set()
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic not in type_map or topic in done:
        continue
    kind, cls = type_map[topic]
    msg = deserialize_message(data, cls)
    fname = out_dir + "/" + topic.split("/")[-1] + ".csv"
    with open(fname, "w", newline="") as f:
        w = csv.writer(f)
        if kind == "Path":
            w.writerow(["x", "y"])
            for ps in msg.poses:
                w.writerow([ps.pose.position.x, ps.pose.position.y])
            print(f"{topic}: {len(msg.poses)} poses -> {fname}")
        elif kind == "Marker":
            w.writerow(["x", "y"])
            for p in msg.points:
                w.writerow([p.x, p.y])
            print(f"{topic}: {len(msg.points)} pts -> {fname}")
        elif kind == "Poly":
            w.writerow(["x", "y"])
            for p in msg.polygon.points:
                w.writerow([p.x, p.y])
            print(f"{topic}: {len(msg.polygon.points)} pts -> {fname}")
    done.add(topic)
    if len(done) == len(type_map):
        break
print("done:", sorted(done))
