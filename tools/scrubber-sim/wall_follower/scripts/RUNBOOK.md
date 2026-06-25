# P3 端到端 Runbook —— 无图场景 (反应式贴墙建图 → 同心环覆盖)

> M1-Mode (反应式贴外墙建图) 接入 m5 仿真栈, 跑通 **无图场景** 端到端:
> 开机无地图 → wall_follower 贴墙走一圈 → slam_toolbox 增量建图 →
> 闭合后 map_to_polygons 提外墙 polygon → 衔接 M2 (P2 已就绪) 同心环覆盖 → coverage_meter 量化
>
> 对应 spec §5.1 (M1-Mode Phase 1.A/1.B) + §7.2 场景 2 (无图场景)

---

## ★ 实测修正 (2026-06-25 实机核对, 以此为准, 覆盖下方正文里的旧路径)

远程环境实际布局 (本地已传代码 + md5 校验通过):

| 项 | 真实路径 |
|----|---------|
| 进 chroot | `sudo chroot /home/wmhn/disk2/scrubber_sim/chroot_noble /bin/bash` (**不是 schroot**, 没装) |
| chroot 工作区 | `/ws` (独立盘 /dev/sdc1 挂载, **非**宿主 bind; 宿主 `scrubber_sim/tools` chroot 内看不到) |
| 我的工具 | `/ws/tools/scrubber-sim` (= PYTHONPATH) |
| ROS 包 | `/ws/src/scrubber_sim/` (area_7x7.yaml, worlds/room_7x7.sdf, config/) |
| install | `source /ws/install/setup.bash` |

**起栈方式已简化**: 不用手动起一堆节点, 直接复用现成 `slam.launch.py` (= gz+urdf+wheel_odom+bridge+slam_toolbox)。`m5_p3_clean.sh start` 已封装。

**依赖**: P3 链路 (wall_follower 纯 math + build_area 用 cv2) **不需要 shapely**, chroot 已有 numpy/cv2/yaml, 可直接跑。(shapely 只 P2 需要。)

**最短路径**:
```bash
# 宿主 → 进 chroot
sudo chroot /home/wmhn/disk2/scrubber_sim/chroot_noble /bin/bash
# chroot 内
cd /ws/tools/scrubber-sim/wall_follower/scripts
bash m5_p3_clean.sh start          # 起 gz+slam, poll 就绪
# 按 start 末尾提示跑 wall_follower + build_area
```

下方正文 0~8 章是早先按"宿主 bind"假设写的, 路径用 `/home/wmhn/disk2/scrubber_sim/...` 处一律换成上表的 `/ws/...`。

---

## 0. 前置

- ✅ P2 同心环覆盖已就绪 (M2, 本地代码完成, 远程已验证三判据)
- ✅ wall_follower 纯算法层 (scan_utils/follower/closure) + 节点 (wall_follower_node.py) 已落地
- ✅ map_to_polygons (A2/A3 工具) 已就绪
- ✅ 远程 wmhn 主机 chroot_noble + ROS Jazzy + gz-sim Harmonic 已就绪 (M5.2 已验证)

远程登录:
```bash
ssh -p 53776 wmhn@frp-hen.com
```

P3 与 P2 的栈差异 (一句话): **P3 无图建图 → 去掉 map_server + nav2/controller, 换上 slam_toolbox 在线建图 + wall_follower 反应式直发 /cmd_vel**。

---

## 1. 本地推代码到远程

```bash
# 本地
cd /Users/liyulin/claude-coding
git add tools/scrubber-sim/wall_follower/
git commit -m "P3: wall_follower 节点 + m5_p3_clean + RUNBOOK (无图场景)"
git push

# 远程
cd /home/wmhn/disk2/scrubber_sim
git pull
chmod +x tools/scrubber-sim/wall_follower/scripts/*.sh
chmod +x tools/scrubber-sim/wall_follower/scripts/*.py
```

如果 `scrubber_sim` git 仓库不含 `tools/`, 用 scp (注意 rsync 拍平子目录坑, 用 -r 保持结构):
```bash
scp -P 53776 -r tools/scrubber-sim/wall_follower \
    wmhn@frp-hen.com:/home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/
```

> ⚠️ 远程实跑前核对路径: `/home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/` 是否就是 PYTHONPATH 期望的位置, wall_follower 包能否 `import wall_follower.follower` (它内部 sys.path.insert 到 scrubber-sim 根)。

---

## 2. 进 chroot + 初始化环境

```bash
# 远程主机
sudo schroot -c noble -- bash

# chroot 内 (set -u 会与 ROS setup.bash 冲突, 不开; 脚本内部已 set +u 包 source)
source /opt/ros/jazzy/setup.bash
cd /home/wmhn/disk2/scrubber_sim/ros2_ws
source install/setup.bash
export PYTHONPATH="${PYTHONPATH}:/home/wmhn/disk2/scrubber_sim/tools/scrubber-sim"

# NVIDIA GPU (M0 已通)
export GZ_IP=127.0.0.1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
```

> ⚠️ 远程实跑前核对路径: `slam_toolbox` 包是否装在 chroot 里 (`ros2 pkg list | grep slam_toolbox`)。没有就 `apt install ros-jazzy-slam-toolbox`。

---

## 3. 起 P3 栈 (空地图)

```bash
cd /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/wall_follower/scripts
bash m5_p3_clean.sh start
```

观察输出, 等到看到 `[start] /map topic present` + 末尾那段 "下一步" 提示才进下一步。
**这一步不会自动跑 wall_follower** —— 栈起好后机器人静止站在起点, slam 正在以当前视野建一小块图。

**核对清单**:
```bash
bash m5_p3_clean.sh status
# 期望:
# - slam_toolbox 节点在 nodes 里
# - /map /scan /odom /cmd_vel /tf 在 topics 里
# - /map 已有数据 (width/height/resolution 有值)
# - TF map->base_link 链通 (map->odom 由 slam 发, odom->base_link 由 wheel_odometry 发)
```

> ⚠️ 关键依赖: slam_toolbox 建图必须有 **odom->base_link 这条 TF**。它由 wheel_odometry 节点广播。
> 如果 status 里 TF map->base_link 不通, 八成是 wheel_odometry 没开 publish_tf —— 见故障排查表。

机器人起点 (gz spawn 时 SDF 里写的) 决定建图坐标原点。P3 不用 amcl, 信 wheel_odometry 开局。

---

## 4. 跑 wall_follower 贴墙建图 (M1-Mode Phase 1.A)

栈起好后, 在**同一个 chroot session** (环境变量都在) 手动跑:

```bash
python3 /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/wall_follower/scripts/wall_follower_node.py \
    --side right \
    --out-trajectory /tmp/m1_wall_trajectory.csv
```

机器人开始反应式贴右墙走。slam_toolbox 在后台随车增量建图。

观察 (节点 logger 输出):
- event 日志: `FOLLOW` (正常贴墙) / `CONCAVE_CORNER` / `LOST_WALL` (凸角找墙) / `BLOCKED` (前方堵 → 进恢复子状态机 STOP→BACKUP→TURN→FORWARD)
- 累计里程 / 累计航向 (闭合判据三要素: 回起点±0.5m + 航向≥350° + slam 闭环)
- slam.log 里出现 loop closure event

**等闭合 (阻塞前台, 自然退出)**:
- **退出码 0** = LOOP CLOSED, 外墙轨迹 CSV 已落盘 `/tmp/m1_wall_trajectory.csv` (列 x,y,theta + 表头)
- 退出码 1 = FAILED (里程 > 200m 未闭合 → spec 说进 M3-Mode 报警; 仿真里多半是贴墙参数 / 场景问题)
- 退出码 2 = Ctrl-C 中断
- 退出码 3 = 启动异常 (CSV 打不开等)

确认退出码:
```bash
echo "exit=$?"
```

> 仿真早期 slam 闭环可能不稳; 节点**默认 `assume_slam_closed=True`** (放宽: 只看几何回起点+航向, 不卡 slam 闭环 event)。
> 要严格双闭环加 `--no-slam-gate` (确认 slam 真报闭环了再用)。

跑时间预计 ~2-4 分钟 (7x7 房间外墙周长 ~28m, v_nominal 0.4 m/s)。

---

## 5. 闭合后存 slam 建好的地图

wall_follower 退 0 后 (栈仍在跑, slam 节点还活着), 立刻存图:

```bash
mkdir -p /home/wmhn/disk2/scrubber_sim/maps
ros2 run nav2_map_server map_saver_cli -f /home/wmhn/disk2/scrubber_sim/maps/m1_built_map
```

产出 `maps/m1_built_map.pgm` + `maps/m1_built_map.yaml`。

```bash
ls -la /home/wmhn/disk2/scrubber_sim/maps/m1_built_map.*
```

> ⚠️ map_saver 要在 slam 还在跑 (/map 在发) 时存; 先 stop 栈就没图存了。

---

## 6. SLAM map → area.yaml 一键衔接 ★关键步 (已自动化)

这一步把 slam 建的 occupancy map 转成 M2 需要的 `area.yaml` (outer + voids)。
**已用 `build_area_from_map.py` 自动化, 不再手工拆分** (见下方"为什么需要它")。

### 6.1 一键生成 (⚠️ 真 SLAM map 不开 morph)

```bash
cd /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/wall_follower/scripts

# --m1-distance 填第 4 步 wall_follower 闭合时打印的累计里程, 触发 Phase 1.B 周长合理性检查
python3 build_area_from_map.py \
    --map /home/wmhn/disk2/scrubber_sim/maps/m1_built_map.yaml \
    --out /tmp/p3_area.yaml \
    --morph 0 \
    --approx-eps 0.05 \
    --min-area 0.05 \
    --m1-distance <闭合里程>

# 输出会打印:
#   extracted N polygons (含外墙)
#   outer: Kpts, 面积 XX.XXm², 周长 XX.XXm
#   voids: M 个
#   周长检查: 外墙 XX vs M1 里程 XX, 相对误差 X.X% (容差 20%) ✅/❌
#   wrote /tmp/p3_area.yaml
```

退出码: 0=成功 / 1=提取失败(slam 没建好) / 2=周长检查不过(进 M3, area.yaml 仍写出供人工看)。

### 6.2 为什么需要它 (schema gap 背景)

`map_to_polygons.py` 默认 `include_outer_boundary=False` 只提内部障碍; 加 `--include-outer`
后外墙会进 `polygons` list 但**和障碍混在一起不区分**。而 `coverage_meter.load_area` 要的是
`outer:` + `voids:` 分开的 schema (只有 `polygons:` 时全当 voids、outer 返回空 → M2 直接废)。

`build_area_from_map.py` 内部: `extract_polygons(include_outer=True, morph=0)` → `area_builder.split_outer_voids`
(**最大面积 polygon = 外墙, 其余 = voids**) → 写 `outer/voids` schema。这套拆分逻辑有 13 个本地单测覆盖
(`tests/test_area_builder.py`)。

### 6.3 可视化核对 (可选)

```bash
# 想看一眼提对没, 单独跑 map_to_polygons 出图:
cd /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/map_to_polygons
python3 map_to_polygons.py \
    --map /home/wmhn/disk2/scrubber_sim/maps/m1_built_map.yaml \
    --output /tmp/p3_polygons_raw.yaml \
    --include-outer --morph 0 --approx-eps 0.05 --min-area 0.05 \
    --viz /tmp/p3_polygons.png
```

> ⚠️ 远程实跑前核对:
> - 7x7 空房间无障碍时 voids 应为 `[]`, 只有 outer。
> - 周长检查 ❌ (误差 > 20%): 多半 slam 没闭合好 / 外墙被噪声切断成几段 / approx-eps 太大, 回第 4 步重跑或调 eps。这正是 Phase 1.B 该拦下的"假外墙"。

---

## 7. 衔接 M2: 把 area.yaml 喂给 P2 跑同心环覆盖

area.yaml 组装好后, **栈别停** (gz + bridge + wheel_odom 还要给 M2 用), 但要换掉控制层:
wall_follower 已退出, slam 可以继续跑 (M2 差异检测要用) 或停掉。M2 走 Nav2 FollowPath, 所以
还需要起 P2 的 controller_server。

最简做法: **重起成 P2 栈**, 用刚提的 area.yaml (而非预录的 area_7x7):

```bash
# 用 P3 建的图当静态图喂 P2 (m1_built_map 替代 room_7x7)
# ⚠️ 远程实跑前核对: m5_p2_clean.sh 里 MAP_YAML 指向 maps/room_7x7.yaml,
#    改成 maps/m1_built_map.yaml 或临时 export 覆盖
bash /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts/m5_p2_clean.sh stop
# (上面会清掉 P3 残留; 注意它的 pattern 不含 wall_follower, 保险起见先跑 m5_p3_clean.sh stop)
bash m5_p3_clean.sh stop

# 起 P2 栈 (controller_server + map_server 喂 m1_built_map)
bash /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts/m5_p2_clean.sh start

# 用 P3 提的 area.yaml 发覆盖 goal
python3 /home/wmhn/disk2/scrubber_sim/tools/scrubber-sim/contour_coverage/scripts/run_contour_coverage.py \
    --area /tmp/p3_area.yaml \
    --spacing 0.55 --margin 0.6 \
    --log-traj /tmp/p3_coverage_traj.csv
```

观察: `[geo] area: outer=Npts, voids=M` —— outer 必须非空 (否则 area.yaml schema 没拆对, 回第 6 步)。

然后照 **P2 RUNBOOK 第 6 步** 拉回轨迹 + verify_p2.py 量化三判据 (覆盖率≥90% / 扫障=0 / 越界=0)。

---

## 8. M1→M2 数据衔接 (说清楚两个产物的用途)

P3 闭合后产出**两个东西**, 用途不同:

| 产物 | 来自 | 给谁用 | 作用 |
|------|------|--------|------|
| (a) `m1_wall_trajectory.csv` | wall_follower 的 **odom 轨迹** | **合理性检查** | 算 M1 累计里程 (轨迹周长) |
| (b) `m1_built_map.pgm/.yaml` | slam_toolbox 建的 **占据栅格图** | **→ map_to_polygons → area.yaml → M2** | 提外墙 outer + 障碍 voids, 是 M2 真正的几何输入 |

数据流:
```
                  ┌─ (a) odom 轨迹 CSV ──────────► 算累计里程 L_odom ─┐
wall_follower ────┤                                                  ├─► 合理性检查:
(贴墙一圈)        └─ slam 建图 (b) map.pgm ─► map_to_polygons         │   |外墙周长 - L_odom| / L_odom < 20% ?
                                              (--include-outer,       │   (spec §5.1 Phase 1.B)
                                               手工拆 outer/voids)    │
                                                     │                │
                                                     ▼                │
                                              area.yaml (outer+voids) ┘
                                                     │
                                                     ▼
                                              M2 同心环覆盖 (P2 run_contour_coverage)
                                                     │
                                                     ▼
                                              coverage_meter 三判据量化
```

**关键**: M2 真正吃的是 (b) 提出来的 **area.yaml**, 不是 (a) 轨迹。
(a) 只做一件事 —— 算 outer 周长 vs M1 累计里程的误差 (spec §5.1 Phase 1.B 合理性检查, 阈值 < 20%)。
误差超 20% 说明 slam 飘了 / 外墙提歪了, 不该进 M2, 该进 M3 兜底。

合理性检查可手算:
```bash
# (a) 累计里程 = 轨迹相邻点欧氏距离累加 (CSV 列 x,y,theta)
# 外墙周长 = area.yaml outer 多边形周长
# 两者比较, 误差 < 20% 才放行 M2
```
> 远程可写个一次性小脚本 (放 tmp/), 读两个文件算误差; 这里不强制, 量化时核一下即可。

---

## 故障排查速查

| 症状 | 大概率原因 | 修法 |
|------|-----------|------|
| `/map` 一直无数据 | slam 没收到 /scan 或 TF 不全 | `ros2 topic hz /scan`; 查 odom->base_link TF 是否在发 |
| TF map->base_link 不通 | wheel_odometry 没广播 odom->base_link TF | 开 wheel_odometry 的 publish_tf 参数 (⚠️ 远程核对节点是否发 TF) |
| slam 建图但全是噪点/墙断裂 | /scan frame_id 不对 / 雷达装偏 | 查 LaserScan.header.frame_id 与 URDF 里 laser link 一致 |
| wall_follower 撞墙 | d_target 太小 / Kp 太猛 | 调 `--d-target 0.7`; 看 follower.py 控制律日志 |
| wall_follower 原地打转不前进 | 一直 BLOCKED 进恢复 / w_pass 太大 | 调 `--w-pass 1.2` 放宽通行宽度判定 |
| 退出码 1 (里程超限未闭合) | 贴墙跟丢 / 凸角没找回墙 / 起点离墙太远 | 看 event 日志卡在哪个状态; 起点摆到离墙 0.6m 内 |
| 闭合误判 (走半圈就退 0) | 航向阈值/位置容差太松, 小空间转圈 | 默认 heading_tol 350° 已防; 若仍误判收紧 ClosureParams |
| 闭合不了 (绕回起点不退) | assume_slam_closed=True 但几何位置容差没满足 | 看节点日志报的 当前位置 vs 起点距离; 起点漂移则 wheel_odom 标定问题 |
| map_to_polygons 提不出外墙 | 没加 `--include-outer` | 加 `--include-outer` (P3 必须, 见第 6 步) |
| map_to_polygons 把墙吃没了 | 开了 morph | `--morph 0` (真 SLAM map 红线, 默认就 0) |
| M2 报 outer 为空 | area.yaml 是 map_to_polygons 原始 `polygons:` 格式 | 手工拆成 `outer:`+`voids:` schema (第 6.3) |
| 外墙周长 vs 里程误差 > 20% | slam 漂移 / 外墙提歪 | 不放行 M2; 重跑建图 或 调 approx-eps |

## 红线 (来自项目记忆, 照抄 P2 RUNBOOK)

1. **pkill -f 不可靠** → 用 `ps awk + grep -vx "$$"` (`ops_kill_with_awk_not_pkill_f`)
2. **多栈混跑陷阱** → 每次启栈前必清残留 (`m5_p3_clean.sh stop`); P3→P2 切换时两个 stop 都跑 (pattern 不同)
3. **FastRTPS shm 累积** → 清栈时 `rm /dev/shm/fastrtps*` (`ops_ros2_fastrtps_shm_buildup`)
4. **半通过陷阱** → 闭合判据三条 (位置/航向/slam 闭环) 缺一就不算闭合; 别为跑通关 slam 闸门当真通过 (`feedback_half_pass_is_not_pass`)
5. **诊断先看形状** → wall_follower 跑偏 / slam 飘, 先 plot odom 轨迹 + /map 截图对比, 别先调参 (`feedback_diagnose_nav2_plan_first`)

---

> ⚠️ 本 RUNBOOK 多处标 "远程实跑前核对路径" —— 本地无 ROS, 这些路径/包名/topic 名都未在远程实测,
> 第一次远程跑时逐处核对 (尤其 slam_toolbox 包名、wheel_odometry 是否发 TF、map_to_polygons 的 outer/voids 拆分)。
