#!/usr/bin/env python3
"""wall_follower_node — 反应式贴墙建图 ROS2 节点 (M1-Mode Phase 1.A)

这个脚本在远程 chroot 内跑(本地无 rclpy 不能运行, 只能 syntax check)。

数据流
======
    /scan (LaserScan) ──┐
                        ├─► scan callback
                        │      LaserScan → ScanData
                        │      compute_control(scan, params)   (纯算法层)
                        │      ┌── FOLLOW ────────────► 直接发 (v, omega)
                        │      ├── CONCAVE_CORNER ────► 直接发 (v, omega) (连续控制)
                        │      ├── LOST_WALL ─────────► 直接发 (v, omega) (连续控制)
                        │      └── BLOCKED ──────────► 进入恢复子状态机 (本节点新增)
                        │                                 后退→转向→直行→回 FOLLOW
                        ▼
                     /cmd_vel (Twist)
    /odom (Odometry) ──► odom callback
                            closure_tracker.update(x, y, yaw)   (纯算法层)
                            驱动恢复子状态机的里程/角度积分
                            is_closed() → 停车 + 存外墙轨迹 CSV + 退出 0
                            is_failed() → 停车 + 退出 1

恢复子状态机 (event=BLOCKED 触发, spec §5.1)
============================================
    NONE      正常态, 透传 compute_control
    STOP      ① 减速停 (一拍发零速)
    BACKUP    ② 后退 0.5m (按 odom 里程判断走够没)
    TURN      ③ 离墙 90° 转向 (贴右墙→左转; 按累计 yaw 判断转够没)
    FORWARD   ④ 沿新航向直行 1m (按 odom 里程)
                ⑤ 走完回到 NONE, compute_control 自然重新贴最近的墙

闭合 / 失败判据 (纯算法层 closure.py)
=====================================
    is_closed: 累计里程≥min + 回起点±pos_tol + 累计航向≥heading_tol + slam 闭环
               (slam 闭环仿真早期用 --assume-slam-closed 放宽, 默认 True)
    is_failed: 累计里程 > 200m 未闭合 → 进 M3-Mode

前置
====
- /scan (sensor_msgs/LaserScan) 在发, 单线雷达, beam 角 0 = 正前方
- /odom (nav_msgs/Odometry) 在发, 提供 base_link 的 pose
- /cmd_vel 被底盘 (或 gazebo diff_drive) 订阅

用法
====
    python3 wall_follower_node.py \\
        --side right \\
        --out-trajectory /tmp/m1_wall_trajectory.csv \\
        --d-target 0.6 --v-nominal 0.4 --w-pass 1.4

    # 关 slam 闸门 (要求几何 + slam 双闭环):
    python3 wall_follower_node.py --no-slam-gate

退出码
======
    0  LOOP CLOSED  外墙闭合, 轨迹 CSV 已落盘 (给 map_to_polygons / 当 outer 用)
    1  FAILED       里程超限未闭合, 进 M3-Mode 兜底
    2  外部中断 (Ctrl-C) / rclpy 提前关闭
    3  启动/参数异常 (CSV 打不开等)
"""
from __future__ import annotations
import argparse
import csv
import math
import sys
from enum import Enum
from pathlib import Path

# 让脚本同时支持源码 in-place 跑 + 模块导入两种姿势
# __file__ = .../wall_follower/scripts/wall_follower_node.py
# parent.parent.parent = .../scrubber-sim (仓库根, wall_follower 包的上一级)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from wall_follower.scan_utils import ScanData
from wall_follower.follower import (
    FollowerParams, WallEvent, compute_control,
)
from wall_follower.closure import ClosureParams, ClosureTracker

# --- ROS imports (仅远程才能 import) ---
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quaternion_to_yaw(q) -> float:
    """Z-axis yaw from quaternion (抄自 run_contour_coverage.py)"""
    # yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²)); 平面运动 x=y=0
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RecoveryState(Enum):
    """BLOCKED 后的多步恢复子状态机 (spec §5.1)

    转移由 odom 里程/角度积分驱动 (单帧 scan 决不了, 故放在节点层而非纯算法层)。
    """
    NONE = "none"          # 正常态: 透传 compute_control
    STOP = "stop"          # ① 减速停: 发一拍零速立即进 BACKUP
    BACKUP = "backup"      # ② 后退 0.5m: 退够距离进 TURN
    TURN = "turn"          # ③ 离墙转 90°: 累计 yaw 够进 FORWARD
    FORWARD = "forward"    # ④ 沿新航向直行 1m: 走够进 NONE (重新贴墙)


# 恢复动作的几何常量 (spec §5.1 给的固定值; 不暴露为 CLI 以免误调)
BACKUP_DIST = 0.5          # 后退距离 (m)
TURN_ANGLE = math.pi / 2   # 离墙转向角 (rad, 90°)
FORWARD_DIST = 1.0         # 离墙后直行距离 (m)
BACKUP_SPEED = 0.15        # 后退线速度 (m/s, 保守)
TURN_SPEED = 0.5           # 转向角速度 (rad/s)
FORWARD_SPEED = 0.3        # 直行线速度 (m/s)


class WallFollowerNode(Node):
    def __init__(
        self,
        side: str,
        follower_params: FollowerParams,
        closure_params: ClosureParams,
        scan_topic: str,
        odom_topic: str,
        cmd_topic: str,
        out_trajectory: str,
        assume_slam_closed: bool,
        control_rate: float,
    ):
        super().__init__("wall_follower_node")
        self._side = side
        self._fparams = follower_params
        self._assume_slam_closed = assume_slam_closed

        # --- 控制器 / 闭合追踪 (纯算法层) ---
        self._tracker = ClosureTracker(params=closure_params)

        # --- 恢复子状态机 ---
        self._recovery = RecoveryState.NONE
        # 进入某个里程/角度阶段时记下基准, 后续与当前值作差判断"走够没"
        self._rec_ref_dist = 0.0    # 进入阶段时的 cum_distance 快照
        self._rec_ref_heading = 0.0  # 进入阶段时的 cum_heading 快照

        # --- 最近一帧位姿缓存 (scan callback 要发零速 / 恢复速度都不需位姿,
        #     但里程判据由 odom callback 更新 tracker, scan 只读取 tracker 累计量) ---
        self._have_odom = False

        # --- 终止标志 (由 odom callback 置位, main loop 检测后退出) ---
        # 0=运行中, 0..=正常闭合, 1=失败兜底
        self.exit_code = None

        # --- 外墙轨迹 CSV (与 coverage_meter.load_trajectory 兼容: 列 x,y,theta + 表头) ---
        self._traj_path = out_trajectory
        try:
            self._traj_file = open(out_trajectory, "w", newline="")
        except OSError as e:
            self.get_logger().error(f"无法打开轨迹文件 {out_trajectory}: {e}")
            raise
        self._traj_writer = csv.writer(self._traj_file)
        self._traj_writer.writerow(["x", "y", "theta"])

        # --- pub / sub ---
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f"wall_follower 启动: side={side} scan={scan_topic} odom={odom_topic} "
            f"cmd={cmd_topic} traj={out_trajectory} "
            f"assume_slam_closed={assume_slam_closed} rate={control_rate}Hz"
        )

    # ------------------------------------------------------------------
    # /scan 回调: 反应控制 + 恢复子状态机的"动作执行"
    #   注意: 状态机的"转移判断"靠 odom 累计量, 但实际发什么速度在这里决定。
    #   scan 频率通常 >= odom 频率, 这里做控制可保证 cmd_vel 及时刷新。
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        if self.exit_code is not None:
            return  # 已判定终止, 不再发控制 (终速由 main 收尾发零)

        scan = ScanData(
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_max=msg.range_max if msg.range_max > 0 else 12.0,
        )

        # 恢复中: 由状态机出速度, 不看 compute_control
        if self._recovery != RecoveryState.NONE:
            self._publish(self._recovery_command())
            return

        # 正常态: 纯算法层单帧决策
        cmd = compute_control(scan, self._fparams)

        if cmd.event == WallEvent.BLOCKED:
            # 进入恢复序列: 先停 (STOP), 后续步骤由 odom 里程驱动
            self._enter_recovery()
            self._publish((0.0, 0.0))
            self.get_logger().warn(
                f"BLOCKED (passable={cmd.passable}) → 进入恢复序列"
            )
            return

        # FOLLOW / CONCAVE_CORNER / LOST_WALL 都是连续控制, 直接透传
        self._publish((cmd.v, cmd.omega))

    # ------------------------------------------------------------------
    # /odom 回调: 喂闭合追踪器 (累积里程/转角) + 记录轨迹 + 驱动恢复状态转移
    #   + 检测闭合 / 失败
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        if self.exit_code is not None:
            return

        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        x, y = p.x, p.y

        # 累积里程/转角 (纯算法层)
        self._tracker.update(x, y, yaw)
        # 记录外墙轨迹 (闭合后整条 CSV 当 outer 用)
        self._traj_writer.writerow([f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}"])

        self._have_odom = True

        # 恢复子状态机的转移判断 (依赖累计里程/角度)
        if self._recovery != RecoveryState.NONE:
            self._step_recovery()

        # --- 失败兜底优先于闭合 (里程超限即认输, 进 M3) ---
        if self._tracker.is_failed():
            self.get_logger().error(
                f"FAILED, enter M3 (cum_distance={self._tracker.cum_distance:.1f}m "
                f"> {self._tracker.params.max_distance}m 未闭合)"
            )
            self.exit_code = 1
            return

        # --- 闭合检测 ---
        if self._tracker.is_closed(slam_loop_closed=self._assume_slam_closed):
            d = self._tracker.dist_to_start()
            self.get_logger().info(
                f"LOOP CLOSED (cum_distance={self._tracker.cum_distance:.1f}m "
                f"cum_heading={math.degrees(self._tracker.cum_heading):.0f}° "
                f"dist_to_start={d:.2f}m) → 轨迹存 {self._traj_path}"
            )
            self.exit_code = 0
            return

    # ------------------------------------------------------------------
    # 恢复子状态机
    # ------------------------------------------------------------------
    def _enter_recovery(self):
        """从 NONE 进入 STOP (恢复序列第一步)"""
        self._recovery = RecoveryState.STOP
        self._snapshot()

    def _snapshot(self):
        """记下进入当前阶段时的里程/转角基准, 用于判断该阶段是否走够"""
        self._rec_ref_dist = self._tracker.cum_distance
        self._rec_ref_heading = self._tracker.cum_heading

    def _step_recovery(self):
        """在 odom 回调里推进恢复状态机的转移 (依赖累计里程/角度)

        各状态退出条件:
            STOP    → BACKUP   : 立即 (停只发一拍零速, 见 _recovery_command)
            BACKUP  → TURN     : 退够 BACKUP_DIST (cum_distance 增量 >= 0.5m)
            TURN    → FORWARD  : 转够 TURN_ANGLE  (cum_heading 增量 >= 90°)
            FORWARD → NONE     : 走够 FORWARD_DIST (cum_distance 增量 >= 1.0m)
        """
        d_inc = self._tracker.cum_distance - self._rec_ref_dist
        h_inc = self._tracker.cum_heading - self._rec_ref_heading

        if self._recovery == RecoveryState.STOP:
            # ① 停: 一进 odom 就转 BACKUP (零速由 scan callback 发过)
            self._recovery = RecoveryState.BACKUP
            self._snapshot()
            self.get_logger().info("恢复② 后退 0.5m")

        elif self._recovery == RecoveryState.BACKUP:
            # ② 后退: cum_distance 不分正负 (hypot), 退也算里程, 增量到 0.5m 即止
            if d_inc >= BACKUP_DIST:
                self._recovery = RecoveryState.TURN
                self._snapshot()
                self.get_logger().info("恢复③ 离墙转 90°")

        elif self._recovery == RecoveryState.TURN:
            # ③ 转向: cum_heading 累计转角增量到 90° 即止
            if h_inc >= TURN_ANGLE:
                self._recovery = RecoveryState.FORWARD
                self._snapshot()
                self.get_logger().info("恢复④ 沿新航向直行 1m")

        elif self._recovery == RecoveryState.FORWARD:
            # ④ 直行: 增量到 1.0m → 回 NONE, 下一帧 scan 自然重新贴最近的墙
            if d_inc >= FORWARD_DIST:
                self._recovery = RecoveryState.NONE
                self.get_logger().info("恢复⑤ 完成, 回到 FOLLOW 重新贴墙")

    def _recovery_command(self):
        """恢复中各状态对应的 (v, omega)

        转向方向: 贴右墙(side=right)要离墙 → 左转(omega>0); 贴左墙 → 右转(omega<0)。
        """
        turn_sign = 1.0 if self._side == "right" else -1.0
        if self._recovery == RecoveryState.STOP:
            return (0.0, 0.0)
        if self._recovery == RecoveryState.BACKUP:
            return (-BACKUP_SPEED, 0.0)
        if self._recovery == RecoveryState.TURN:
            return (0.0, turn_sign * TURN_SPEED)
        if self._recovery == RecoveryState.FORWARD:
            return (FORWARD_SPEED, 0.0)
        return (0.0, 0.0)

    # ------------------------------------------------------------------
    def _publish(self, vw):
        v, omega = vw
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self._cmd_pub.publish(msg)

    def stop_robot(self):
        """收尾: 连发几拍零速确保停下 (单拍可能丢)"""
        for _ in range(3):
            self._publish((0.0, 0.0))

    def close_traj(self):
        if self._traj_file:
            self._traj_file.flush()
            self._traj_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", choices=["right", "left"], default="right",
                        help="贴墙侧, 默认 right")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    # --assume-slam-closed 默认 True; 用 --no-slam-gate 关闭 (要求真 slam 闭环)
    parser.add_argument("--no-slam-gate", dest="assume_slam_closed",
                        action="store_false", default=True,
                        help="关闭 slam 闸门放宽 (默认开放宽=True; 加此flag则要求真 slam 闭环)")
    parser.add_argument("--out-trajectory", default="/tmp/m1_wall_trajectory.csv",
                        help="闭合后外墙轨迹 CSV (列 x,y,theta + 表头, 兼容 load_trajectory)")
    # FollowerParams 关键项
    parser.add_argument("--d-target", type=float, default=0.6, help="目标贴墙距 (m)")
    parser.add_argument("--v-nominal", type=float, default=0.4, help="巡航速度 (m/s)")
    parser.add_argument("--w-pass", type=float, default=1.4, help="最小可通行宽度 (m)")
    parser.add_argument("--control-rate", type=float, default=10.0,
                        help="控制频率 Hz (仅作 spin 超时/日志参考, 控制实际由 scan 回调触发)")
    args = parser.parse_args()

    fparams = FollowerParams(
        d_target=args.d_target,
        v_nominal=args.v_nominal,
        w_pass=args.w_pass,
        side=args.side,
    )
    cparams = ClosureParams()  # 用纯算法层默认 (pos_tol/heading_tol/max_distance/min_distance)

    rclpy.init()
    try:
        node = WallFollowerNode(
            side=args.side,
            follower_params=fparams,
            closure_params=cparams,
            scan_topic=args.scan_topic,
            odom_topic=args.odom_topic,
            cmd_topic=args.cmd_topic,
            out_trajectory=args.out_trajectory,
            assume_slam_closed=args.assume_slam_closed,
            control_rate=args.control_rate,
        )
    except OSError:
        rclpy.shutdown()
        return 3  # CSV 打不开等启动异常

    # spin 一拍超时取控制周期的倒数 (默认 0.1s @ 10Hz), 让退出判定及时
    spin_timeout = 1.0 / args.control_rate if args.control_rate > 0 else 0.1

    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=spin_timeout)
        # 退出前停车
        node.stop_robot()
        code = node.exit_code
        if code is None:
            # rclpy 被外部关闭 (shutdown) 而非自然闭合/失败
            code = 2
        return code
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().warn("KeyboardInterrupt, 停车退出")
        return 2
    finally:
        node.close_traj()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
