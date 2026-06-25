"""area_builder 单元测试 — M1→M2 polygon 拆分 + 周长检查"""
import math
import pytest

from wall_follower.area_builder import (
    polygon_area, polygon_perimeter, split_outer_voids,
    perimeter_check, to_area_yaml_dict,
)


def test_polygon_area_square():
    sq = [(0, 0), (2, 0), (2, 2), (0, 2)]
    assert polygon_area(sq) == pytest.approx(4.0)


def test_polygon_area_closed_same_as_open():
    """首末点重复不影响面积"""
    open_sq = [(0, 0), (2, 0), (2, 2), (0, 2)]
    closed_sq = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]
    assert polygon_area(open_sq) == pytest.approx(polygon_area(closed_sq))


def test_polygon_area_degenerate():
    assert polygon_area([(0, 0), (1, 1)]) == 0.0


def test_polygon_perimeter_square():
    sq = [(0, 0), (3, 0), (3, 3), (0, 3)]
    assert polygon_perimeter(sq) == pytest.approx(12.0)


def test_split_largest_is_outer():
    """大方框=外墙, 小方框=void"""
    big = [(0, 0), (10, 0), (10, 10), (0, 10)]
    small = [(2, 2), (3, 2), (3, 3), (2, 3)]
    outer, voids = split_outer_voids([small, big])
    assert outer == big
    assert voids == [small]


def test_split_multiple_voids():
    big = [(0, 0), (10, 0), (10, 10), (0, 10)]
    v1 = [(1, 1), (2, 1), (2, 2), (1, 2)]
    v2 = [(5, 5), (7, 5), (7, 7), (5, 7)]
    outer, voids = split_outer_voids([v1, big, v2])
    assert outer == big
    assert len(voids) == 2
    assert v1 in voids and v2 in voids


def test_split_empty():
    assert split_outer_voids([]) == ([], [])


def test_split_skips_degenerate():
    """不足 3 点的 polygon 被剔除"""
    big = [(0, 0), (4, 0), (4, 4), (0, 4)]
    degen = [(1, 1), (2, 2)]
    outer, voids = split_outer_voids([degen, big])
    assert outer == big
    assert voids == []


def test_perimeter_check_pass():
    """6×6 方房周长 24m, M1 里程 25m → 误差 4% < 20% 通过"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3)]
    ok, rel = perimeter_check(outer, m1_distance=25.0, tol=0.20)
    assert ok
    assert rel == pytest.approx(abs(24 - 25) / 25)


def test_perimeter_check_fail():
    """周长 24m vs 里程 50m → 误差 52% > 20% 不过"""
    outer = [(-3, -3), (3, -3), (3, 3), (-3, 3)]
    ok, rel = perimeter_check(outer, m1_distance=50.0, tol=0.20)
    assert not ok


def test_perimeter_check_zero_distance():
    outer = [(0, 0), (1, 0), (1, 1), (0, 1)]
    ok, rel = perimeter_check(outer, m1_distance=0.0)
    assert not ok
    assert math.isinf(rel)


def test_to_area_yaml_dict_schema():
    """输出 schema 与 coverage_meter.load_area 的 outer/voids 格式一致"""
    outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
    voids = [[(1, 1), (2, 1), (2, 2), (1, 2)]]
    d = to_area_yaml_dict(outer, voids)
    assert "outer" in d and "voids" in d
    assert d["outer"][0] == [0.0, 0.0]
    assert d["voids"][0][0] == [1.0, 1.0]
    # 全是 float (yaml 友好)
    assert all(isinstance(c, float) for p in d["outer"] for c in p)


def test_roundtrip_yaml_schema(tmp_path):
    """端到端: split → to_yaml_dict → yaml → 重解析还原 outer/voids

    验证产出的 yaml 命中 coverage_meter.load_area 的 'outer' 分支判据
    (load_area: if 'outer' in data → outer + voids), 不直接 import load_area
    以免 coverage_meter 的 sys.path 注入污染包式导入。
    """
    import yaml as _yaml

    big = [(0, 0), (10, 0), (10, 10), (0, 10)]
    small = [(3, 3), (5, 3), (5, 5), (3, 5)]
    outer, voids = split_outer_voids([small, big])
    d = to_area_yaml_dict(outer, voids)

    f = tmp_path / "area.yaml"
    f.write_text(_yaml.safe_dump(d))

    data = _yaml.safe_load(f.read_text())
    assert "outer" in data            # load_area 据此走 outer/voids 分支
    assert [tuple(p) for p in data["outer"]] == [tuple(p) for p in big]
    assert len(data["voids"]) == 1
