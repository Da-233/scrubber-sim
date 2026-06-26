"""阿卡曼路径可执行性闸门(薄护栏)。

把"只有端到端实跑才暴露"的不可执行问题左移到本地秒级:
曲率超 1/R_min、原地转、越界、穿障四项红绿灯。
"""

from .path_linter import (
    DEFAULT_R_MIN,
    DEFAULT_ROBOT_LENGTH,
    DEFAULT_ROBOT_WIDTH,
    LintCheck,
    LintConfig,
    LintReport,
    PathNotExecutable,
    assert_path_executable,
    lint_path,
)

__all__ = [
    "DEFAULT_R_MIN",
    "DEFAULT_ROBOT_LENGTH",
    "DEFAULT_ROBOT_WIDTH",
    "LintCheck",
    "LintConfig",
    "LintReport",
    "PathNotExecutable",
    "assert_path_executable",
    "lint_path",
]
