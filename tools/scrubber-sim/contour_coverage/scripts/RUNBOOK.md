# P2 端到端 Runbook

> contour_coverage 接入 m5 仿真栈, 跑通同心轮廓覆盖 + 三条判据量化
> 目标: 替代 F2C/opennav_coverage, 把 M5.3 "半通过" 烂尾收掉

---

## ★ 实测修正 (2026-06-25 实机核对, 以此为准)

| 项 | 真实路径 |
|----|---------|
| 进 chroot | `sudo chroot /home/wmhn/disk2/scrubber_sim/chroot_noble /bin/bash` (**不是 schroot**) |
| chroot 工作区 | `/ws` (独立盘挂载, 非宿主 bind) |
| 我的工具 | `/ws/tools/scrubber-sim` (= PYTHONPATH) |
| area_7x7.yaml | `/ws/src/scrubber_sim/area_7x7.yaml` |
| install | `source /ws/install/setup.bash` |

**起栈已简化**: 直接复用现成 `m5.launch.py` (controller_server 自带 RPP FollowPath)。`m5_p2_clean.sh start` 已封装, 多起的 coverage_server 闲置无害。

**依赖**: ✅ 全齐 (2026-06-25 实测)。chroot 内 shapely 2.0.3 (apt python3-shapely, 华为云源) + numpy 1.26.4 + cv2 4.6.0 + yaml 都有。P2 链路 chroot 内 import + 几何自检已通过 (4 rings/616 poses)。

**最短路径**:
```bash
sudo chroot /home/wmhn/disk2/scrubber_sim/chroot_noble /bin/bash
cd /ws/tools/scrubber-sim/contour_coverage/scripts
bash m5_p2_clean.sh start          # 起全栈, poll controller active + /follow_path
# 按 start 末尾提示跑 run_contour_coverage --log-traj
```

下方正文用 `/home/wmhn/disk2/scrubber_sim/...` 处一律换成 `/ws/...`。

---

## 0. 前置

- ✅ P1 几何核心已落地 (本地 commit `6bf15e9`)
- ✅ 远程 wmhn 主机 chroot_noble + ROS Jazzy 已就绪 (M5.2 已验证)
- ✅ Step 1~5 本地代码已落

远程登录:
```bash
ssh -p 53776 wmhn@frp-hen.com
```

---

## 1. 本地推代码到远程

```bash
# 本地
cd /Users/liyulin/claude-coding
git add tools/scrubber-sim/contour_coverage/ tools/scrubber-sim/tests/test_ros_bridge.py
git commit -m "P2: ros_bridge + remote runner + m5_p2_clean + verify_p2 + RUNBOOK"
git push

# 远程
cd /home/wmhn/disk2/scrubber_sim
git pull
chmod +x tools/scrubber-sim/contour_coverage/scripts/*.sh
chmod +x tools/scrubber-sim/contour_coverage/scripts/*.py
```

如果 `scrubber_sim` git 仓库不含 `tools/`, 用 scp:
```bash
scp -P 53776 -r tools/scrubber-sim/contour_coverage wmhn@frp-hen.com:/home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/
```

---

## 2. 进 chroot + 初始化

```bash
# 远程主机
sudo schroot -c noble -- bash

# chroot 内 (set -u 会与 ROS 冲突, 不开)
source /opt/ros/jazzy/setup.bash
cd /home/wmhn/disk2/scrubber_sim/ros2_ws
source install/setup.bash
export PYTHONPATH="${PYTHONPATH}:/home/wmhn/disk2/scrubber_sim/tools/scrubber-sim"

# NVIDIA GPU (M0 已通)
export GZ_IP=127.0.0.1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

---

## 3. 准备静态地图 (P2 不用 SLAM)

P2 用 area_7x7 + 静态 occupancy map. 若 `maps/room_7x7.yaml` + `.pgm` 已存在跳过.

否则一次性导出 (room_7x7 是程序生成的 ROS map): 在 chroot 内
```bash
# 用 nav2_map_saver 录一次 SLAM 出图, 或手写 yaml (room_7x7 边界已知)
# 这里假设场景固定, 直接用预录好的 maps/room_7x7.{yaml,pgm}
ls /home/wmhn/disk2/scrubber_sim/maps/room_7x7.*
```

如果没有, 用 slam_toolbox 跑一次 + 保存:
```bash
bash m5_clean.sh start    # 原 M5.2 脚本会起 SLAM
# 等 SLAM 建好 (手动遥控转一圈或自己飞), Ctrl+C 之后:
ros2 run nav2_map_server map_saver_cli -f /home/wmhn/disk2/scrubber_sim/maps/room_7x7
bash m5_clean.sh stop
```

---

## 4. 起 P2 栈

```bash
cd /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts
bash m5_p2_clean.sh start
```

观察输出, 等到看到 `[start] /follow_path ready` 才能进下一步.

**核对清单**:
```bash
bash m5_p2_clean.sh status
# 期望:
# - /controller_server 在 nodes 里
# - /follow_path 在 action 里
# - controller lifecycle = active
# - /odom /scan /tf /map 在 topics 里
```

机器人起点位置 (gz spawn 时 SDF 里写的) 必须在 area 内部. P2 不用 amcl, 信 wheel_odometry + gz 真值开局.

---

## 5. 发 goal + 跑覆盖

```bash
mkdir -p /tmp/p2_logs
python3 /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts/run_contour_coverage.py \
    --area /home/wmhn/disk2/scrubber_sim/area_7x7.yaml \
    --spacing 0.55 --margin 0.6 \
    --log-traj /tmp/p2_logs/traj.csv
```

观察:
- `[geo] rings: N` (N 应该 >= 4, 否则 area / margin 不对)
- `[geo] poses: M` (M 应该 ~400-500, M5.3 场景)
- `goal accepted`
- 每 ~2s 输出 `[feedback] distance_to_goal=... speed=...`
- 最后 `result status=4` (SUCCEEDED)

跑时间预计 ~3-5 分钟 (425 pose, RPP ~0.3 m/s).

**异常处理**:
- 卡在 `waiting for action server`: controller_server 没 activate, 看 `${LOG_DIR}/lifecycle_ctrl.log`
- `goal rejected`: 大多是 frame_id 错, 检查 `/tf` 有没有 `map`
- 中途 ABORTED (status=6): RPP 跟丢, 多半 path 密度问题或起点不在 outer 内; 看 `controller.log`

---

## 6. 拿回轨迹 + 本地量化

```bash
# 远程 → 本地
scp -P 53776 wmhn@frp-hen.com:/tmp/p2_logs/traj.csv /tmp/

# 本地
cd /Users/liyulin/claude-coding
python3 tools/scrubber-sim/contour_coverage/scripts/verify_p2.py \
    --traj /tmp/traj.csv \
    --area projects/自动洗地机/area_7x7.yaml \
    --out projects/自动洗地机/03_软件架构/figures/2026-06-25-P2同心轮廓覆盖_仿真验证.png
```

> ⚠️ `projects/自动洗地机/area_7x7.yaml` 本地不存在 (只远程有), 跑前先 scp 一份回来.

期望输出:
```
=== P2 判据 (spec §7.3) ===
  可清扫区面积: 23.04 m²  (因 area 而异)
  已扫面积:    XX.XX m²
  [1] 覆盖率: 9X.XX% (阈值 90%)  ✅
  [2] 扫障面积: 0.0000 m²  ✅
  [3] 越界面积: 0.0000 m²  ✅
=== P2 PASS (3/3) ===
```

三条都 ✅ = **P2 完成**.

---

## 7. (可选) RViz 截图

```bash
# 远程, P2 栈还在跑时另开一个 SSH session
cd /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts
bash capture_rviz.sh ~/coverage.rviz /tmp/p2_rviz.png

# 拉回本地
scp -P 53776 wmhn@frp-hen.com:/tmp/p2_rviz.png \
    /Users/liyulin/claude-coding/projects/自动洗地机/03_软件架构/figures/2026-06-25-P2-RViz实时执行.png
```

`coverage.rviz` 需要先手工配好一份 (Fixed Frame=map, 显示 /map /odom /tf /follow_path/_action/feedback 的 path).

---

## 8. 收尾

```bash
# 远程
bash m5_p2_clean.sh stop

# 本地
git add projects/自动洗地机/03_软件架构/figures/2026-06-25-P2*
git commit -m "P2 完成: contour_coverage 仿真验证 PASS (coverage XX%, swept=0, overspray=0)"
git push
```

更新日志 + 项目现状 + 必读清单 (本计划 Step 6, 实际执行时)

---

## 故障排查速查

| 症状 | 大概率原因 | 修法 |
|------|-----------|------|
| `/follow_path` action 不存在 | controller_server 没活 | `ros2 lifecycle set /controller_server activate` |
| `goal rejected` | path frame_id 不是 map | 改 `--frame-id` (或加参数) |
| feedback 一直 distance_to_goal=inf | path 是空的 | 几何输出为 0 ring, 检查 area.yaml |
| 走几步就 ABORTED | RPP 跟丢, max_allowed_time_to_collision 太严 | 临时加大 (但是 半通过坑) |
| 覆盖率 < 90% | 路径密度不够 / footprint 不对 | 加 --densify-step 0.05; 或 --clean-width 拉宽 |
| 扫障 > 0 | margin 不够大 | --margin 0.7 重跑 |
| 越界 > 0 | 起点不在 outer 内 / TF 飘 | 检查 spawn 位置 / wheel_odom 校准 |

## 红线 (来自项目记忆)

1. **pkill -f 不可靠** → 用 `ps awk + grep -vx "$$"` (`ops_kill_with_awk_not_pkill_f`)
2. **多栈混跑陷阱** → 每次启栈前必清残留 (`m5_p2_clean.sh stop`)
3. **FastRTPS shm 累积** → 清栈时 `rm /dev/shm/fastrtps*` (`ops_ros2_fastrtps_shm_buildup`)
4. **半通过陷阱** → `use_collision_detection` 不许关; 三条判据缺一就是失败 (`feedback_half_pass_is_not_pass`)
5. **诊断 nav2 先看 plan 形状** → 跑偏先 plot trajectory + path 对比, 别先调参 (`feedback_diagnose_nav2_plan_first`)
