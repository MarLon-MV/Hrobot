"""Exploración por fronteras: detección, agrupación y selección (lógica pura).

Convenciones de la grilla (como ``nav_msgs/OccupancyGrid`` sin importarlo):
``-1`` desconocido, ``0..free_max`` libre, ``>= occupied_min`` ocupado.
Unidades en metros; coordenadas en el frame ``map``.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

UNKNOWN_CELL = -1
_NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NEIGHBORS_8 = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
)


@dataclass(frozen=True)
class GridMap:
    """Grilla de ocupación fila-mayor con origen en la esquina inferior izquierda."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width y height deben ser > 0")
        if not (math.isfinite(self.resolution) and self.resolution > 0):
            raise ValueError("resolution debe ser finita y > 0")
        if not (math.isfinite(self.origin_x) and math.isfinite(self.origin_y)):
            raise ValueError("el origen debe ser finito")
        if len(self.data) != self.width * self.height:
            raise ValueError("len(data) debe ser width*height")


@dataclass(frozen=True)
class Point2D:
    """Punto 2D en metros."""

    x: float
    y: float


@dataclass(frozen=True)
class Frontier:
    """Grupo de celdas frontera: centroide en metros y número de celdas."""

    centroid: Point2D
    size: int


@dataclass(frozen=True)
class ExplorationConfig:
    """Parámetros de la exploración."""

    min_frontier_size: int = 8
    free_max: int = 25
    occupied_min: int = 65
    blacklist_radius: float = 0.5
    min_goal_distance: float = 0.4


def _is_free(value: int, config: ExplorationConfig) -> bool:
    return 0 <= value <= config.free_max


def _frontier_mask(grid: GridMap, config: ExplorationConfig) -> list[bool]:
    """Marca celdas libres con al menos un vecino-4 desconocido."""
    w, h, data = grid.width, grid.height, grid.data
    mask = [False] * (w * h)
    for row in range(h):
        for col in range(w):
            idx = row * w + col
            if not _is_free(data[idx], config):
                continue
            for dc, dr in _NEIGHBORS_4:
                nc, nr = col + dc, row + dr
                if 0 <= nc < w and 0 <= nr < h and data[nr * w + nc] == UNKNOWN_CELL:
                    mask[idx] = True
                    break
    return mask


def _collect_group(
    start: int, mask: list[bool], visited: list[bool], width: int, height: int
) -> list[int]:
    """BFS iterativo con conectividad-8 sobre las celdas frontera."""
    group = [start]
    visited[start] = True
    queue: deque[int] = deque([start])
    while queue:
        idx = queue.popleft()
        row, col = divmod(idx, width)
        for dc, dr in _NEIGHBORS_8:
            nc, nr = col + dc, row + dr
            if not (0 <= nc < width and 0 <= nr < height):
                continue
            nidx = nr * width + nc
            if mask[nidx] and not visited[nidx]:
                visited[nidx] = True
                group.append(nidx)
                queue.append(nidx)
    return group


def _to_frontier(group: list[int], grid: GridMap) -> Frontier:
    size = len(group)
    sum_col = 0
    sum_row = 0
    for idx in group:
        row, col = divmod(idx, grid.width)
        sum_col += col
        sum_row += row
    x = grid.origin_x + (sum_col / size + 0.5) * grid.resolution
    y = grid.origin_y + (sum_row / size + 0.5) * grid.resolution
    return Frontier(centroid=Point2D(x, y), size=size)


def find_frontiers(grid: GridMap, config: ExplorationConfig) -> list[Frontier]:
    """Detecta y agrupa fronteras.

    Args:
        grid: mapa de ocupación.
        config: parámetros de exploración.

    Returns:
        Fronteras con ``size >= min_frontier_size``, ordenadas por ``size``
        descendente, luego ``x`` y ``y``.
    """
    mask = _frontier_mask(grid, config)
    visited = [False] * len(mask)
    frontiers: list[Frontier] = []
    for idx, is_frontier in enumerate(mask):
        if not is_frontier or visited[idx]:
            continue
        group = _collect_group(idx, mask, visited, grid.width, grid.height)
        if len(group) >= config.min_frontier_size:
            frontiers.append(_to_frontier(group, grid))
    frontiers.sort(key=lambda f: (-f.size, f.centroid.x, f.centroid.y))
    return frontiers


def _require_finite(point: Point2D, name: str) -> None:
    if not (math.isfinite(point.x) and math.isfinite(point.y)):
        raise ValueError(f"{name} contiene valores no finitos")


def select_frontier(
    frontiers: Sequence[Frontier],
    robot: Point2D,
    blacklist: Sequence[Point2D],
    config: ExplorationConfig,
) -> Frontier | None:
    """Elige la frontera válida más cercana al robot.

    Descarta fronteras cercanas a la lista negra o al propio robot.
    Desempate determinista: mayor ``size``, luego ``x``, luego ``y``.

    Raises:
        ValueError: si ``robot`` o la lista negra tienen valores no finitos.

    Returns:
        La frontera elegida o ``None`` si no queda ninguna candidata.
    """
    _require_finite(robot, "robot")
    for banned in blacklist:
        _require_finite(banned, "blacklist")

    best: Frontier | None = None
    best_key: tuple[float, int, float, float] | None = None
    for frontier in frontiers:
        c = frontier.centroid
        if not (math.isfinite(c.x) and math.isfinite(c.y)):
            continue
        dist = math.hypot(c.x - robot.x, c.y - robot.y)
        if dist < config.min_goal_distance:
            continue
        if any(math.hypot(c.x - b.x, c.y - b.y) < config.blacklist_radius for b in blacklist):
            continue
        key = (dist, -frontier.size, c.x, c.y)
        if best_key is None or key < best_key:
            best, best_key = frontier, key
    return best
