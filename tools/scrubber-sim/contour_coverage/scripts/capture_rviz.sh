#!/usr/bin/env bash
# capture_rviz.sh — Xvfb + rviz2 离屏截图 (远程 chroot 内跑)
#
# 用于 P2 端到端跑完后, 给 RViz 出一张真图 (spec §7.2 用户偏好真实可视化)
#
# 前置:
#   - chroot 内装 xvfb + imagemagick: apt install xvfb imagemagick
#   - NVIDIA EGL 已就绪 (M0 阶段已通)
#   - RViz 配置文件 (.rviz) 存在并订阅了 Map / Path / TF / Odom
#
# 用法:
#   bash capture_rviz.sh <rviz_config.rviz> <output.png>
#   bash capture_rviz.sh ~/coverage.rviz /tmp/p2_rviz.png

set -e

RVIZ_CONFIG="${1:?需要 rviz 配置文件路径}"
OUTPUT_PNG="${2:?需要输出 PNG 路径}"
WAIT_SEC="${3:-12}"   # rviz 启动 + 订阅 + 渲染稳定的等待秒数

if ! command -v xvfb-run >/dev/null; then
    echo "ERROR: 缺 xvfb-run, 请 apt install xvfb"
    exit 1
fi
if ! command -v import >/dev/null; then
    echo "ERROR: 缺 imagemagick (import 命令), 请 apt install imagemagick"
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

# 启 Xvfb + rviz2 (后台)
echo "[capture] starting Xvfb + rviz2..."
xvfb-run -a -s "-screen 0 1920x1080x24" \
    rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true \
    > /tmp/rviz_capture.log 2>&1 &
RVIZ_PID=$!

# 等 rviz 起来并收到话题
echo "[capture] waiting ${WAIT_SEC}s for rviz to render..."
sleep "${WAIT_SEC}"

# 抓图 (root 窗口 = Xvfb 的虚拟屏)
DISPLAY=:99 import -window root "${OUTPUT_PNG}" || {
    # xvfb-run 用动态 DISPLAY, 上面写死 :99 不一定对; 改用循环找
    for d in 99 98 97 100 101; do
        if DISPLAY=:${d} import -window root "${OUTPUT_PNG}" 2>/dev/null; then
            echo "[capture] captured on DISPLAY=:${d}"
            break
        fi
    done
}

# 收尾
kill "${RVIZ_PID}" 2>/dev/null || true
wait "${RVIZ_PID}" 2>/dev/null || true

if [[ -s "${OUTPUT_PNG}" ]]; then
    echo "[capture] wrote ${OUTPUT_PNG} ($(stat -c%s "${OUTPUT_PNG}") bytes)"
else
    echo "[capture] FAILED: ${OUTPUT_PNG} 空 / 不存在"
    echo "[capture] 看 /tmp/rviz_capture.log:"
    tail -20 /tmp/rviz_capture.log
    exit 1
fi
