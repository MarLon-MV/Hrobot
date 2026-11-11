"""Tests de exploración por fronteras."""

from __future__ import annotations

import math

import pytest
from hrobot_core.domain.exploration import (
    ExplorationConfig,
    Frontier,
    GridMap,
    Point2D,
    find_frontiers,
    select_frontier,
)

CFG = ExplorationConfig(min_frontier_size=1)
CFG_SEL = ExplorationConfig()


def make_grid(rows: list[list[int]], res: float = 1.0, ox: float = 0.0, oy: float = 0.0) -> GridMap:
    return GridMap(
        width=len(rows[0]),
        height=len(rows),
        resolution=res,
        origin_x=ox,
        origin_y=oy,
        data=tuple(v for r in rows for v in r),
    )


def uniform(value: int, w: int = 10, h: int = 10) -> GridMap:
    return make_grid([[value] * w for _ in range(h)])


@pytest.mark.parametrize("value", [-1, 0, 100])
def test_uniform_maps_have_no_frontiers(value: int) -> None:
    assert find_frontiers(uniform(value), CFG) == []


def test_free_room_with_unknown_hole() -> None:
    rows = [[0] * 5 for _ in range(5)]
    rows[2][2] = -1
    grid = make_grid(rows, res=0.5, ox=1.0, oy=2.0)
    result = find_frontiers(grid, CFG)
    assert len(result) == 1
    assert result[0].size == 4
    assert result[0].centroid.x == pytest.approx(1.0 + 2.5 * 0.5)
    assert result[0].centroid.y == pytest.approx(2.0 + 2.5 * 0.5)


def test_two_separate_frontiers_and_small_group_discarded() -> None:
    rows = [[100] * 9 for _ in range(3)]
    rows[1][0] = -1
    rows[1][1] = 0  # grupo A: 1 celda
    rows[1][8] = -1
    for col in (5, 6, 7):
        rows[1][col] = 0  # grupo B: celdas (5..7,1); (7,1) toca (8,1) desconocido
        rows[0][col] = -1  # y las tres tocan desconocido por arriba
    assert len(find_frontiers(make_grid(rows), CFG)) == 2
    result = find_frontiers(make_grid(rows), ExplorationConfig(min_frontier_size=2))
    assert len(result) == 1
    assert result[0].size == 3
    assert find_frontiers(make_grid(rows), ExplorationConfig(min_frontier_size=4)) == []


def test_occupied_cell_between_free_and_unknown_is_not_frontier() -> None:
    assert find_frontiers(make_grid([[0, 100, -1]]), CFG) == []


def test_intermediate_value_is_not_free() -> None:
    assert find_frontiers(make_grid([[50, -1]]), CFG) == []


def test_diagonal_cells_group_by_8_connectivity() -> None:
    rows = [
        [0, -1, 100],
        [-1, 0, -1],
        [100, -1, 0],
    ]
    result = find_frontiers(make_grid(rows), CFG)
    assert len(result) == 1
    assert result[0].size == 3


def test_output_sorted_deterministically() -> None:
    rows = [
        [0, 0, 100, 0],
        [-1, -1, 100, -1],
    ]
    result = find_frontiers(make_grid(rows), CFG)
    assert [f.size for f in result] == [2, 1]
    assert result[0].centroid.x == pytest.approx(1.0)
    assert result[1].centroid.x == pytest.approx(3.5)


def test_large_map_no_recursion_error() -> None:
    n = 1000
    data = [0] * (n * n)
    for i in range(n):  # columna 0 desconocida: columna 1 es una frontera larga
        data[i * n] = -1
    grid = GridMap(n, n, 0.05, 0.0, 0.0, tuple(data))
    result = find_frontiers(grid, ExplorationConfig())
    assert len(result) == 1
    assert result[0].size == n


def test_input_not_mutated() -> None:
    grid = uniform(0)
    before = grid.data
    find_frontiers(grid, CFG)
    assert grid.data == before


@pytest.mark.parametrize(
    ("w", "h", "res", "data"),
    [
        (2, 2, 1.0, (0, 0, 0)),
        (2, 2, 0.0, (0, 0, 0, 0)),
        (2, 2, -0.1, (0, 0, 0, 0)),
        (2, 2, math.nan, (0, 0, 0, 0)),
        (0, 2, 1.0, ()),
        (2, 0, 1.0, ()),
    ],
)
def test_grid_validation(w: int, h: int, res: float, data: tuple[int, ...]) -> None:
    with pytest.raises(ValueError):
        GridMap(w, h, res, 0.0, 0.0, data)


def test_grid_rejects_non_finite_origin() -> None:
    with pytest.raises(ValueError):
        GridMap(1, 1, 1.0, math.inf, 0.0, (0,))


def fr(x: float, y: float, size: int = 10) -> Frontier:
    return Frontier(Point2D(x, y), size)


def test_select_nearest() -> None:
    result = select_frontier([fr(5, 0), fr(2, 0), fr(9, 0)], Point2D(0, 0), [], CFG_SEL)
    assert result == fr(2, 0)


def test_select_respects_blacklist() -> None:
    result = select_frontier([fr(2, 0), fr(5, 0)], Point2D(0, 0), [Point2D(2.2, 0)], CFG_SEL)
    assert result == fr(5, 0)


def test_select_respects_min_goal_distance() -> None:
    result = select_frontier([fr(0.3, 0), fr(5, 0)], Point2D(0, 0), [], CFG_SEL)
    assert result == fr(5, 0)


def test_select_none_cases() -> None:
    assert select_frontier([], Point2D(0, 0), [], CFG_SEL) is None
    assert select_frontier([fr(0.1, 0)], Point2D(0, 0), [], CFG_SEL) is None
    assert select_frontier([fr(2, 0)], Point2D(0, 0), [Point2D(2, 0)], CFG_SEL) is None


def test_select_tie_break_deterministic() -> None:
    robot = Point2D(0, 0)
    small, big_neg, big_pos = fr(0, 2, 5), fr(0, -2, 9), fr(2, 0, 9)
    # Misma distancia: gana mayor size; luego menor x.
    assert select_frontier([small, big_pos, big_neg], robot, [], CFG_SEL) == big_neg
    assert select_frontier([big_neg, big_pos, small], robot, [], CFG_SEL) == big_neg
    # Mismo size y x: gana menor y.
    up, down = fr(0, 2, 9), fr(0, -2, 9)
    assert select_frontier([up, down], robot, [], CFG_SEL) == down
    assert select_frontier([down, up], robot, [], CFG_SEL) == down


def test_select_skips_non_finite_frontier() -> None:
    result = select_frontier([fr(math.nan, 0), fr(3, 0)], Point2D(0, 0), [], CFG_SEL)
    assert result == fr(3, 0)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_select_rejects_non_finite_robot(bad: float) -> None:
    with pytest.raises(ValueError):
        select_frontier([fr(2, 0)], Point2D(bad, 0), [], CFG_SEL)
    with pytest.raises(ValueError):
        select_frontier([fr(2, 0)], Point2D(0, bad), [], CFG_SEL)


def test_select_rejects_non_finite_blacklist() -> None:
    with pytest.raises(ValueError):
        select_frontier([fr(2, 0)], Point2D(0, 0), [Point2D(math.nan, 0)], CFG_SEL)
