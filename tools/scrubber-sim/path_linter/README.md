# path_linter — 阿卡曼路径可执行性闸门

把"只有远程端到端实跑才暴露"的不可执行问题**左移到本地秒级**。
薄护栏,不是物理仿真——只查四件确定性的事,真实动力学仍由 gz 仿真负责。

## 为什么有它

洗地机踩过最贵的坑:覆盖/路径方案本地"几何完美 + 单测全绿",一上远程
就卡死(P2 同心环内圈 < R_min、P3 贴墙凹角原地转、F2C swath 连接穿障),
四套方案 ~16h 全推翻。单测验"几何对不对",验不了"三轮阿卡曼跟不跟得动"。

## 四项检查

| 项 | 查什么 | 实现 |
|----|--------|------|
| `curvature` | 处处曲率 ≤ 1/R_min | 复用 `ackermann_primitives` |
| `in_place_turn` | 无 v≈0 仍转向的段(阿卡曼转不了) | 纯几何扫描 |
| `out_of_bounds` | 整车 footprint 不扫出外墙 | 复用 `coverage_meter` |
| `obstacle` | 整车 footprint 不压障碍 | 复用 `coverage_meter` |

后两项要给 `outer`(房间外墙 polygon)才查,否则跳过。曲率/原地转是纯几何,
不依赖 cv2;越界/穿障惰性 import coverage_meter。

## 用法

代码里(生成器测试 / 部署前):
```python
from path_linter import lint_path, assert_path_executable, LintConfig

report = lint_path(poses, LintConfig(r_min=1.2, outer=room_outer))
if not report.ok:
    print(report.summary())

# 当硬闸门:不可执行就抛 PathNotExecutable
assert_path_executable(poses, LintConfig(r_min=1.2, outer=room_outer))
```

命令行(上传 path 到远程前):
```bash
python -m path_linter k1.csv --area area.yaml   # exit 1 = 有失败
# 部署脚本里: python -m path_linter k1.csv --area area.yaml || exit 1
```

## 落地方式(自动闸门 > 文档)

- K1 `generate_lawnmower` 等生成器在测试里 `assert_path_executable(...)` —— 见
  `tests/test_ackermann_boustrophedon.py::test_lawnmower_output_passes_path_linter_gate`
- 远程 `run.sh` 上传前跑 CLI,红灯拒绝 scp(待 remote/run.sh 落地后接入)

详见复盘 `projects/自动洗地机/03_软件架构/复盘_2026-06-26_开发流程.md`。

## 测试

`tests/test_path_linter.py`(10 项)+ K1 闸门测试 1 项。
