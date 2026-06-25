#!/usr/bin/env bash
# A8 验证：min_turning_radius=0.3 重跑，录 coverage_plan/swaths/tf/map 判断 Dubins 穿箱假设
# 在 chroot 内执行。栈必须已经 start 就绪。
set -eo pipefail

WS=/home/wmhn/disk2/scrubber_sim/ros2_ws
ROS_DISTRO=jazzy
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f "${WS}/install/setup.bash" ] && source "${WS}/install/setup.bash"
set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
export GZ_IP=127.0.0.1

TS=$(date +%H%M%S)
OUT=/ws/a8_out
mkdir -p "${OUT}"
BAG="${OUT}/bag_${TS}"

echo "[A8] 启动 rosbag record -> ${BAG}"
ros2 bag record -o "${BAG}" \
    /coverage_server/coverage_plan \
    /coverage_server/swaths \
    /coverage_server/field_boundary \
    /coverage_server/planning_field \
    /received_global_plan \
    /plan \
    /tf /tf_static /map \
    > "${OUT}/bag_${TS}.log" 2>&1 &
BAGPID=$!
sleep 4

SELF_DIR=/ws/src/scrubber_sim
AREA=${AREA:-area_7x7.yaml}
echo "[A8] 从 ${AREA} 生成 goal..."
GOAL_YAML=$(python3 "${SELF_DIR}/gen_coverage_goal.py" "${SELF_DIR}/${AREA}")
echo "[A8] === GOAL YAML (head) ==="
echo "${GOAL_YAML}" | head -20

echo "[A8] 发 NavigateCompleteCoverage (timeout 45s, 只需录到 plan)..."
timeout 45 ros2 action send_goal --feedback \
    /navigate_complete_coverage \
    opennav_coverage_msgs/action/NavigateCompleteCoverage \
    "${GOAL_YAML}" 2>&1 | tee "${OUT}/goal_${TS}.log" || echo "[A8] send_goal 退出码非0 (timeout 或 reject)"

echo "[A8] === GOAL DONE, 停 bag ==="
sleep 2
kill ${BAGPID} 2>/dev/null || true
sleep 2
echo "[A8] BAG=${BAG}"
echo "[A8] OUT=${OUT}"
ls -la "${OUT}"
