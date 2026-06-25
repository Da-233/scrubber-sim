#!/usr/bin/env bash
# m5_p3_clean.sh — P3 反应式贴墙建图 启栈 (chroot 内执行)
#
# 复用现成 slam.launch.py (= bringup[gz+urdf+wheel_odom+bridge] + slam_toolbox
# + lifecycle_manager), 正好是 P3 要的"无图 + slam 在线建图"。bridge 已桥
# /scan + /cmd_vel, wheel_odom 发 /odom + TF, 全对得上 wall_follower_node 默认 topic。
#
# 用法 (chroot 内):
#   bash m5_p3_clean.sh start    # 起 gz+slam (无 nav2 控制面)
#   bash m5_p3_clean.sh stop     # 清残留
#   bash m5_p3_clean.sh status   # 查 /scan /odom /map /tf
#
# 起好后另跑 wall_follower 贴墙建图 (见末尾提示 / RUNBOOK 第4步)。

set -eo pipefail
# 不开 -u: ROS setup.bash 用了 AMENT_TRACE_SETUP_FILES 等未定义变量

# ── chroot 内真实路径 (工作区 = /ws, 独立盘挂载, 非宿主 bind) ──
WS=/ws
PKG=${WS}/src/scrubber_sim
TOOLS=${WS}/tools/scrubber-sim
WALL_FOLLOWER=${TOOLS}/wall_follower/scripts/wall_follower_node.py
ROS_DISTRO=jazzy
WORLD=${WORLD:-room_7x7.sdf}
LOG_DIR=/tmp/p3_logs
mkdir -p "${LOG_DIR}"

load_env() {
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    [ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash
    set -u
    export PYTHONPATH="${PYTHONPATH:-}:${TOOLS}"
    export GZ_IP=127.0.0.1
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
}

# ── 残留清理 (红线: pkill -f 在 chroot 不可靠, 用 awk 精准 + 排除自身) ──
kill_residuals() {
    echo "[clean] killing residuals..."
    local pat='(ros2 launch|gz sim|gazebo|slam_toolbox|async_slam|wall_follower_node|wheel_odometry|robot_state_pub|parameter_bridge|ros_gz|lifecycle_manager|component_container)'
    local pids
    pids=$(ps -ef | awk -v p="${pat}" '$0 ~ p {print $2}' | grep -vx "$$" || true)
    [ -n "${pids}" ] && echo "${pids}" | xargs -r kill -9 2>/dev/null || true
    sleep 2
    rm -f /dev/shm/fastrtps* 2>/dev/null || true
    echo "[clean] done"
}

start_stack() {
    kill_residuals
    load_env
    echo "[start] ros2 launch scrubber_sim slam.launch.py world:=${WORLD} (nohup)..."
    nohup ros2 launch scrubber_sim slam.launch.py world:="${WORLD}" \
        > "${LOG_DIR}/slam_launch.log" 2>&1 &
    echo "[start] 等 gz/slam 就绪..."

    # poll /scan 有数据 (gz+bridge 起来了)
    for i in $(seq 1 30); do
        if timeout 3 ros2 topic echo /scan --once >/dev/null 2>&1; then
            echo "[start] /scan OK (i=$i)"; break
        fi
        sleep 2
    done
    # poll slam_toolbox active
    for i in $(seq 1 20); do
        if ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q active; then
            echo "[start] slam_toolbox ACTIVE (i=$i)"; break
        fi
        sleep 2
    done
    # poll TF map->odom (slam 在发图)
    for i in $(seq 1 15); do
        if ros2 run tf2_ros tf2_echo map odom --timeout 2 >/dev/null 2>&1; then
            echo "[start] TF map->odom OK (i=$i)"; break
        fi
        sleep 2
    done

    echo
    echo "[start] 栈就绪. 下一步跑 wall_follower 贴墙建图:"
    echo
    echo "  PYTHONPATH=${TOOLS} python3 ${WALL_FOLLOWER} \\"
    echo "      --side right --out-trajectory /tmp/m1_wall_trajectory.csv"
    echo
    echo "  退出码 0 = 闭合. 闭合后存图 + 一键生成 area.yaml:"
    echo "  ros2 run nav2_map_server map_saver_cli -f ${PKG}/maps/m1_built_map"
    echo "  PYTHONPATH=${TOOLS} python3 ${TOOLS}/wall_follower/scripts/build_area_from_map.py \\"
    echo "      --map ${PKG}/maps/m1_built_map.yaml --out ${PKG}/area_m1.yaml \\"
    echo "      --morph 0 --m1-distance <闭合里程>"
    echo "  (详见 RUNBOOK.md)"
}

show_status() {
    load_env
    echo "=== nodes ==="; ros2 node list 2>/dev/null
    echo "=== topics (关注 /scan /odom /cmd_vel /map /tf) ==="
    ros2 topic list 2>/dev/null | grep -E "scan|odom|cmd_vel|map|tf" || true
    echo "=== slam lifecycle ==="; ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "NOT FOUND"
    echo "=== /map 有数据? ==="; timeout 5 ros2 topic echo /map --once >/dev/null 2>&1 && echo "/map OK" || echo "/map 无数据"
}

case "${1:-}" in
    start)  start_stack ;;
    stop)   kill_residuals ;;
    status) show_status ;;
    *) echo "用法: $0 {start|stop|status}"; exit 1 ;;
esac
