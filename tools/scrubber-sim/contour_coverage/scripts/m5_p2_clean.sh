#!/usr/bin/env bash
# m5_p2_clean.sh — P2 同心轮廓覆盖 启栈 (chroot 内执行)
#
# 复用现成 m5.launch.py (全栈: bringup+slam+controller+planner+behavior
# +coverage_server+bt_navigator)。P2 只用其中的 controller_server (RPP FollowPath)
# + slam (提供 TF/map) + gz。多起的 coverage_server/bt_navigator 不发 goal 即闲置,无害。
#
# ★ 依赖 shapely (contour_coverage 几何核心)。chroot 内若未装:
#     sudo chroot ... apt install python3-shapely   (或离线 wheel)
#
# 用法 (chroot 内):
#   bash m5_p2_clean.sh start    # 起全栈
#   bash m5_p2_clean.sh stop     # 清残留
#   bash m5_p2_clean.sh status   # 查 controller / /follow_path
#
# 起好后跑 run_contour_coverage.py 发 path (见末尾提示 / RUNBOOK 第5步)。

set -eo pipefail

# ── chroot 内真实路径 (工作区 = /ws) ──
WS=/ws
PKG=${WS}/src/scrubber_sim
TOOLS=${WS}/tools/scrubber-sim
RUNNER=${TOOLS}/contour_coverage/scripts/run_contour_coverage.py
AREA_YAML=${PKG}/area_7x7.yaml
ROS_DISTRO=jazzy
WORLD=${WORLD:-room_7x7.sdf}
LOG_DIR=/tmp/p2_logs
mkdir -p "${LOG_DIR}"

load_env() {
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    [ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash
    set -u
    export PYTHONPATH="${PYTHONPATH:-}:${TOOLS}"
    # ★ F2C v1.2.1 坑 (m5.launch 起 coverage_server 用): /usr/local/lib 必须最前,
    #   否则 dlopen 拉到 apt v2.0 的 libFields2Cover.so 符号查不到 (issue_ld_library_path_vs_cache)
    export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
    export GZ_IP=127.0.0.1
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
}

kill_residuals() {
    echo "[clean] killing residuals..."
    local pat='(ros2 launch|gz sim|gazebo|slam_toolbox|async_slam|controller_server|planner_server|behavior_server|coverage_server|bt_navigator|wheel_odometry|robot_state_pub|parameter_bridge|ros_gz|lifecycle_manager|component_container|run_contour)'
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
    echo "[start] LD_LIBRARY_PATH head=$(echo ${LD_LIBRARY_PATH} | cut -d: -f1)"
    echo "[start] ros2 launch scrubber_sim m5.launch.py world:=${WORLD} (nohup)..."
    nohup ros2 launch scrubber_sim m5.launch.py world:="${WORLD}" \
        > "${LOG_DIR}/m5_launch.log" 2>&1 &

    echo "[start] 等 controller_server active (m5.launch 延迟 8s 起 nav2 控制面)..."
    for i in $(seq 1 30); do
        if ros2 lifecycle get /controller_server 2>/dev/null | grep -q active; then
            echo "[start] controller_server ACTIVE (i=$i)"; break
        fi
        sleep 2
    done
    for i in $(seq 1 15); do
        if ros2 action list 2>/dev/null | grep -q "/follow_path"; then
            echo "[start] /follow_path ready (i=$i)"; break
        fi
        sleep 2
    done

    echo
    echo "[start] 栈就绪. 下一步发 path 跑覆盖:"
    echo
    echo "  PYTHONPATH=${TOOLS} python3 ${RUNNER} \\"
    echo "      --area ${AREA_YAML} --spacing 0.55 --margin 0.6 \\"
    echo "      --log-traj /tmp/p2_logs/traj.csv"
    echo
    echo "  完成后 scp traj.csv 回本地, 跑 verify_p2.py 量化三判据 (详见 RUNBOOK.md)"
}

show_status() {
    load_env
    echo "=== nodes ==="; ros2 node list 2>/dev/null
    echo "=== /follow_path action ==="; ros2 action list 2>/dev/null | grep follow_path || echo "NOT FOUND"
    echo "=== controller lifecycle ==="; ros2 lifecycle get /controller_server 2>/dev/null || echo "NOT FOUND"
    echo "=== topics ==="; ros2 topic list 2>/dev/null | grep -E "scan|odom|cmd_vel|map|plan" || true
}

case "${1:-}" in
    start)  start_stack ;;
    stop)   kill_residuals ;;
    status) show_status ;;
    *) echo "用法: $0 {start|stop|status}"; exit 1 ;;
esac
