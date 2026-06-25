#!/usr/bin/env bash
# M5 弓字形全覆盖端到端跑通脚本（chroot 内执行）
#
# 用法：
#   bash m5_clean.sh start    # 启动整栈（gz+slam+nav2+coverage）
#   bash m5_clean.sh goal     # 等栈就绪后，发 NavigateCompleteCoverage 目标
#   bash m5_clean.sh stop     # 清理（含杀进程）
#   bash m5_clean.sh         # 显示帮助
#
# 设计要点：
# 1) **LD_LIBRARY_PATH 反超 cache** 是 M5.1 终极坑——必须在 source setup.bash 之后
#    强制把 /usr/local/lib 推到最前，否则会拉到 apt v2.0 的 libFields2Cover.so，
#    dlopen 时 v1.2.1 的 searchBestPath 符号查不到。
# 2) 测试纪律：发 action 用 send_goal --feedback 包 90s timeout（blocking），
#    禁 `pub -r N + timeout M` 组合。
# 3) start 子命令前台跑 launch，goal 子命令另开终端执行。
#
# 远程主机：wmhn (Jazzy chroot)

set -eo pipefail
# 注：不开 -u（ROS setup.bash 用了 AMENT_TRACE_SETUP_FILES 等未定义变量会直接退）

# ============ 环境装载 ============
WS=/home/wmhn/disk2/scrubber_sim/ros2_ws
ROS_DISTRO=jazzy

set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
# opennav_coverage 5 个包装在 /ws/install/（M5.0 步骤）
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
# scrubber_sim 装在 /ws/install/（chroot 路径），host 路径仅做 fallback
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi
set -u

# ★ 必须放在所有 setup.bash 之后：强 v1.2.1 优先于 apt v2.0
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# gz-sim 网络（chroot 跑 gz 必备）
export GZ_IP=127.0.0.1
# NVIDIA EGL
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

cmd=${1:-help}

case "${cmd}" in
    start)
        echo "[M5] LD_LIBRARY_PATH head=$(echo $LD_LIBRARY_PATH | cut -d: -f1)"
        echo "[M5] 启动 m5.launch.py (前台，Ctrl-C 退出)..."
        exec ros2 launch scrubber_sim m5.launch.py
        ;;

    goal)
        # M5.3 A4: 从 area.yaml 生成 goal(外圈 polygons[0] + inner voids polygons[1..N]),
        # 喂给已验证可用的 send_goal --feedback CLI。
        # AREA 环境变量切区域文件,默认 area_7x7.yaml(7×7房+2箱障)。
        # gen_coverage_goal.py 负责闭合 + 卷绕(外圈CCW/void CW)。
        AREA=${AREA:-area_7x7.yaml}
        TIMEOUT_S=${TIMEOUT_S:-180}
        SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        echo "[M5] 从 ${AREA} 生成 goal..."
        GOAL_YAML=$(python3 "${SELF_DIR}/gen_coverage_goal.py" "${SELF_DIR}/${AREA}")
        echo "[M5] 发 NavigateCompleteCoverage (timeout ${TIMEOUT_S}s)..."
        timeout "${TIMEOUT_S}" ros2 action send_goal --feedback \
            /navigate_complete_coverage \
            opennav_coverage_msgs/action/NavigateCompleteCoverage \
            "${GOAL_YAML}"
        ;;

    goal-mini)
        # M5.2 旧的硬编码防障小区(2.5×2.5,无 void),做回归对照用
        TIMEOUT_S=${TIMEOUT_S:-180}
        echo "[M5] 发 NavigateCompleteCoverage 防障小区(回归) (timeout ${TIMEOUT_S}s)..."
        timeout "${TIMEOUT_S}" ros2 action send_goal --feedback \
            /navigate_complete_coverage \
            opennav_coverage_msgs/action/NavigateCompleteCoverage \
"{
  field_filepath: '',
  polygons: [
    {points: [
      {x: -1.5, y: -1.5, z: 0.0},
      {x:  1.0, y: -1.5, z: 0.0},
      {x:  1.0, y:  1.0, z: 0.0},
      {x: -1.5, y:  1.0, z: 0.0},
      {x: -1.5, y: -1.5, z: 0.0}
    ]}
  ],
  frame_id: 'map',
  behavior_tree: ''
}"
        ;;

    diag)
        echo "[M5] 诊断信息："
        echo "--- libFields2Cover 加载路径检查 ---"
        ldconfig -p | grep -i fields2cover || true
        echo ""
        echo "--- 当前 LD_LIBRARY_PATH 头部 ---"
        echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | head -5
        echo ""
        echo "--- coverage_server 节点 ---"
        ros2 node list 2>/dev/null | grep -E "coverage|bt_navigator|controller|slam" || \
            echo "  (栈未启动)"
        echo ""
        echo "--- 关键 topic ---"
        ros2 topic list 2>/dev/null | grep -E "scan|odom|cmd_vel|map" | head -10 || true
        ;;

    stop)
        echo "[M5] 清理..."
        # 不要 -f 匹配 ssh 自己（远程杀进程自杀坑）
        pkill -f "ros2 launch scrubber_sim m5" 2>/dev/null || true
        pkill -f "component_container" 2>/dev/null || true
        pkill -f "slam_toolbox" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        sleep 1
        echo "[M5] done"
        ;;

    help|*)
        cat <<EOF
用法: bash m5_clean.sh {start|goal|diag|stop}

  start  前台启动 m5.launch.py（全栈），Ctrl-C 退出
  goal   发一次 NavigateCompleteCoverage（7×7 矩形 polygon，map 系，180s 超时）
         环境变量 TIMEOUT_S=120 可改超时
  diag   打印 F2C 加载路径 / 节点 / topic 诊断
  stop   清栈（pkill launch / container / slam / gz）

典型流程（两终端）：
  T1: bash m5_clean.sh start
  T2: bash m5_clean.sh diag    # 等到 coverage_server/bt_navigator 都出现
  T2: bash m5_clean.sh goal    # 发目标，看 feedback
  T1: Ctrl-C
  T2: bash m5_clean.sh stop
EOF
        ;;
esac
