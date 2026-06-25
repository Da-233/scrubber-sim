# contour_coverage

自动洗地机同心轮廓覆盖路径生成器（M5.3 spec §5.2 实现）。

## 用法

```bash
python3 -m contour_coverage.cli --area path/to/area.yaml --spacing 0.55 --margin 0.6 --out path.json --preview
```

输入：area.yaml（项目惯例，coverage_meter.load_area 兼容）
输出：JSON pose 序列 + matplotlib 预览图

## 算法

详见 spec: `projects/自动洗地机/03_软件架构/specs/2026-06-25-同心轮廓覆盖引擎-设计.md` §5.2。
核心：Shapely buffer 递归内缩 + 段间 Nav2 ComputePathToPose 调用占位（P2 接入）。
