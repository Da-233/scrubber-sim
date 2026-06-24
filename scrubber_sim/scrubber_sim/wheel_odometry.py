#!/usr/bin/env python3
"""
自研轮式里程计（替代 gz AckermannSteering 自算 odom，后者在三轮配置下失真）。

输入：/joint_states（后轮 velocity → 线速度，steering_joint position → 转向角）
模型：自行车模型（bicycle model）
    v = wheel_radius * 后轮平均角速度
    delta = steering_joint 角度
    omega = v * tan(delta) / wheel_base
    x += v*cos(theta)*dt;  y += v*sin(theta)*dt;  theta += omega*dt
输出：/odom（nav_msgs/Odometry）+ TF odom->base_footprint

这套与真车 STM32 里程计同源（轮速+转向角积分），仿真/真车通用。
参数与 URDF 一致：wheel_radius=0.15, wheel_base=1.0
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        # 参数（与 URDF 几何一致）
        self.declare_parameter("wheel_radius", 0.15)
        self.declare_parameter("wheel_base", 1.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("rear_left_joint", "rear_left_wheel_joint")
        self.declare_parameter("rear_right_joint", "rear_right_wheel_joint")
        self.declare_parameter("steering_joint", "steering_joint")
        self.declare_parameter("publish_tf", True)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rl_joint = self.get_parameter("rear_left_joint").value
        self.rr_joint = self.get_parameter("rear_right_joint").value
        self.steer_joint = self.get_parameter("steering_joint").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # 位姿状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"wheel_odometry started: r={self.wheel_radius} L={self.wheel_base}")

    def cb(self, msg: JointState):
        # 用消息自带时间戳（sim time）
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0.0 or dt > 1.0:
            return

        name2idx = {n: i for i, n in enumerate(msg.name)}
        try:
            wl = msg.velocity[name2idx[self.rl_joint]]
            wr = msg.velocity[name2idx[self.rr_joint]]
            delta = msg.position[name2idx[self.steer_joint]]
        except (KeyError, IndexError):
            return

        # 线速度 = 后轮平均角速度 * 轮半径
        v = (wl + wr) * 0.5 * self.wheel_radius
        # 自行车模型偏航率
        omega = v * math.tan(delta) / self.wheel_base

        # 积分（中点法对 theta）
        self.x += v * math.cos(self.theta + 0.5 * omega * dt) * dt
        self.y += v * math.sin(self.theta + 0.5 * omega * dt) * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q = yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        # 适度协方差
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
