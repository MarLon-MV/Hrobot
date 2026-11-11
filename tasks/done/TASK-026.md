# TASK-026: Dominio de exploración por fronteras (detección, agrupación y selección)

- **Rama Git:** `feat/TASK-026`
- **Depende de:** TASK-025
- **Prioridad:** P1

## Contexto y objetivo de negocio
El robot ya mapea con `slam_toolbox` y navega con Nav2, pero un humano debe fijar cada objetivo. Para mapear de forma autónoma se implementará exploración *frontier-based*: una frontera es una celda libre del mapa adyacente a una celda desconocida. Esta tarea entrega solo la lógica **pura** (sin ROS, sin I/O); el nodo y el launch van en TASK-027.

## Alcance técnico
Crear en `domain` un módulo `exploration.py` con tipos inmutables y funciones puras. Convenciones de la grilla (iguales a `nav_msgs/OccupancyGrid`, pero sin importarlo): `-1` desconocido, `0..free_max` libre, `>= occupied_min` ocupado.

```python
# src/hrobot_core/hrobot_core/domain/exploration.py
@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float          # m/celda, > 0
    origin_x: float            # esquina inferior izquierda, en metros (frame map)
    origin_y: float
    data: tuple[int, ...]      # len == width*height, fila-mayor, valores -1..100

@dataclass(frozen=True)
class Point2D: x: float; y: float

@dataclass(frozen=True)
class Frontier:
    centroid: Point2D          # en metros (frame map)
    size: int                  # nº de celdas

@dataclass(frozen=True)
class ExplorationConfig:
    min_frontier_size: int = 8         # celdas; filtra ruido
    free_max: int = 25                 # celda libre si 0 <= v <= free_max
    occupied_min: int = 65
    blacklist_radius: float = 0.5      # m
    min_goal_distance: float = 0.4     # m; ignora fronteras ya "encima" del robot

def find_frontiers(grid: GridMap, config: ExplorationConfig) -> list[Frontier]: ...
def select_frontier(
    frontiers: Sequence[Frontier],
    robot: Point2D,
    blacklist: Sequence[Point2D],
    config: ExplorationConfig,
) -> Frontier | None: ...
```

Comportamiento:
- `GridMap` valida en `__post_init__` (`ValueError`): dimensiones > 0, `resolution > 0`, `len(data) == width*height`.
- `find_frontiers`: celda frontera = libre con al menos un vecino-4 desconocido. Se agrupan por conectividad-8 (BFS iterativo, **sin recursión**). Se descartan grupos con `size < min_frontier_size`. Centroide = media de las celdas del grupo, convertido a metros (centro de celda). Salida ordenada de forma determinista (por `size` desc, luego `x`, luego `y`).
- `select_frontier`: descarta fronteras a menos de `blacklist_radius` de algún punto de la lista negra y las que estén a menos de `min_goal_distance` del robot. Devuelve la de **menor distancia euclídea** al robot (desempate determinista: mayor `size`, luego `x`, luego `y`). Devuelve `None` si no queda ninguna.
- Sin NaN/inf: si `robot` o la lista negra contienen valores no finitos, `ValueError`.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/exploration.py`
- `src/hrobot_core/hrobot_core/domain/__init__.py`
- `tests/domain/test_exploration.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_exploration.py --cov=hrobot_core.domain.exploration --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Test: mapa totalmente desconocido, totalmente libre y totalmente ocupado ⇒ `[]`
- [ ] Test: sala libre con un hueco desconocido ⇒ una frontera con centroide y `size` esperados
- [ ] Test: dos fronteras separadas ⇒ dos `Frontier`; grupo menor a `min_frontier_size` se descarta
- [ ] Test: una celda ocupada entre libre y desconocido **no** genera frontera
- [ ] Test: `select_frontier` elige la más cercana, respeta lista negra y `min_goal_distance`, devuelve `None` sin candidatas y es determinista en empates
- [ ] Tests de validación: `GridMap` con `len(data)` incorrecto, `resolution <= 0`, robot con NaN/inf
- [ ] Un mapa de 1000x1000 se procesa sin recursión (test de rendimiento básico sin `RecursionError`)
- [ ] `git diff --name-only main...HEAD` solo lista archivos permitidos (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] `exploration.py` no importa `rclpy`, mensajes ROS, `services` ni `adapters`
- [ ] Modelos `frozen=True`; las funciones no mutan sus entradas
- [ ] Sin números mágicos fuera de `ExplorationConfig`; funciones <= 40 líneas, complejidad <= 10
- [ ] Commits con Conventional Commits y scope `(TASK-026)`

## Fuera de alcance
Nodo ROS, cliente de Nav2, launch, guardado del mapa (TASK-027); criterio por ganancia de información.
