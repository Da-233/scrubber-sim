# scrubber-sim 工具集

洗地机仿真项目的辅助工具，可在本地 / 远程独立运行（不依赖 ROS2 运行时）。

## 工具列表

### map_to_polygons (A2, M5.3)

SLAM map.pgm → 障碍 polygon list 提取工具，输出 F2C inner voids 用的 yaml。

```bash
# 安装依赖
pip install opencv-python-headless numpy pyyaml matplotlib

# 跑工具
python3 map_to_polygons/map_to_polygons.py \
    --map maps/room.yaml \
    --output maps/room_polygons.yaml \
    --viz /tmp/check.png        # 可选,出可视化对比图

# 跑测试 (15 个 case 全覆盖, ~0.1s)
cd .
python3 -m pytest tests/test_map_to_polygons.py -v
```

**关键参数**：
- `--min-area`: 过滤面积小于此值(m²)的轮廓 (默认 0.05)
- `--approx-eps`: 多边形简化容差(m) (默认 0.05)
- `--morph`: 形态学开运算核大小(px), 0=不做 (默认 3)
- `--include-outer`: 包含整张地图外边界(墙)，**默认 False**

**输出 yaml schema**:
```yaml
polygons:
  - points: [[x, y], ...]   # 世界坐标系,闭合(首末点相同),CW 方向
    area_m2: float
    vertices: int
source_map: <path>
resolution: <float>
```

**已知坑** (spec §3.1)：
1. 凹形障碍：`RETR_CCOMP` 跳过包住全图的外边界轮廓但保留所有真障碍
2. 贴墙障碍：会跟墙黏成一个 polygon，可通过 ROI 裁剪或建图分离
3. 简化过度：`approx_eps_m` 太大丢角；默认 0.05m (5cm)
4. 方向判定：`_ensure_cw_in_world` 强制 CW（F2C inner voids 约定）

### coverage_meter (A9, M5.3)

洗地机**实际覆盖率量化**工具 (footprint sweep)。把清扫工具矩形沿机器人
轨迹逐点旋转盖章栅格化,与"可清扫区(外圈 - 障碍 voids)"求交,算真实扫到
的面积占比 —— **不是用路径长度估**(路径长 ≠ 扫净面积)。M5.3 A8 验证
"覆盖率 ≥ 85%" 判据要用。

```bash
python3 coverage_meter/coverage_meter.py \
    --traj traj.csv \              # 轨迹 csv: x,y[,theta]
    --area area.yaml \            # 区域: outer + voids (或 map_to_polygons 输出)
    --clean-width 0.6 \          # 清扫横向宽 (默认 0.6, 对齐 F2C op_width)
    --clean-length 0.2 \         # 清扫纵向长 (默认 0.2)
    --res 0.05 \                 # 栅格分辨率 m/格
    --viz /tmp/coverage.png      # 可选: 绿=已扫 灰=漏扫 红=剐到障碍 蓝=轨迹

# 跑测试 (11 个 case)
python3 -m pytest tests/test_coverage_meter.py -v
```

**输出三个量**：
- `覆盖率` = 扫到的可清扫格 / 总可清扫格
- `扫障面积` = footprint 压到障碍上的面积 → **碰撞/剐蹭报警**
- `越界面积` = 扫到外圈外的面积 → **喷溅报警**

**关键设计**：
- 轨迹**自动补点**(相邻位姿间距 > 分辨率就插值),稀疏 waypoint 也不漏扫
- `theta` 缺失时由运动方向 `atan2` 推断(odom 一般有 yaw,可省)
- `--area` 兼容 `map_to_polygons` 输出(只有 `polygons:` 时当 voids,outer 用 `--outer` 给)
- 远程 A8:用 `ros2 topic echo /odom` 或小 recorder 把位姿落成 csv 再喂进来

**区域 yaml schema**:
```yaml
outer: [[x, y], ...]            # 清扫外圈
voids: [[[x, y], ...], ...]     # 障碍 inner voids(从可清扫区挖掉 + 扫到报警)
```

### contour_coverage (P1, M5.3 子系统#1)

同心轮廓覆盖路径生成器（M5.3 spec §5.2 实现）。

输入 polygon (outer + voids)，输出同心环 + 段间连接的 pose 序列，喂 Nav2 FollowPath。
几何上证明不穿障（任意环到障碍距离 ≥ safety_margin）。F2C 路线终结后的替代方案。

详见 `contour_coverage/README.md`。

## 测试 fixture

`tests/fixtures/` 下的 `.pgm`/`.yaml` 是 pytest 自动生成的合成 map，
首次跑测试后会出现，可手动 `rm` 重新生成。
