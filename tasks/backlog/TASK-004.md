# TASK-004: Dominio — política de seguridad por LiDAR (reducción y parada)

- **Rama Git:** `feat/TASK-004`
- **Depende de:** TASK-002
- **Prioridad:** P0

## Contexto y objetivo de negocio
El robot circula en pasillos con personas y camillas. Una política determinista que convierta las lecturas del LiDAR en un factor de velocidad (0 = parar, 1 = libre) es el núcleo de la seguridad. Debe ser pura y exhaustivamente probada.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/domain/safety.py
@dataclass(frozen=True)
class SafetyConfig:
    stop_distance: float = 0.35      # m: por debajo ⇒ factor 0
    slow_distance: float = 1.00      # m: por debajo ⇒ reducción lineal
    front_half_angle: float = 0.6    # rad: semicono frontal evaluado
    stale_timeout: float = 0.5       # s: scan más viejo ⇒ parar

@dataclass(frozen=True)
class ScanData:
    ranges: Sequence[float]; angle_min: float; angle_increment: float
    range_min: float; range_max: float; age_s: float

def speed_factor(scan: ScanData, cfg: SafetyConfig) -> float: ...   # en [0.0, 1.0]
```
Reglas:
- Solo se consideran rayos dentro de `±front_half_angle` (ángulo normalizado respecto al frente).
- Clasificación de cada rayo dentro del cono frontal (convención REP-117):
  - **Obstáculo:** valor finito en `[range_min, range_max]` ⇒ distancia medida.
  - **Libre:** `+inf` ⇒ no hay obstáculo hasta `range_max` (cuenta como rayo válido y no limita la velocidad).
  - **Inválido (se ignora):** `NaN`, `-inf`, o valor finito fuera de `[range_min, range_max]`.
- La distancia efectiva es el mínimo de los rayos de tipo obstáculo; si solo hay rayos libres ⇒ factor **1.0**.
- Si en el cono frontal **no hay ningún rayo válido** (todos inválidos, o cono vacío / scan vacío) ⇒ **0.0** (fallo seguro).
- Documentar en el docstring que un rayo finito `< range_min` se trata como inválido (zona ciega del sensor) y cubrirlo con un test.
- `age_s > stale_timeout` ⇒ 0.0.
- Entre `stop_distance` y `slow_distance` el factor crece linealmente de 0 a 1.
- `SafetyConfig` inválida (`stop_distance >= slow_distance`, valores <= 0) ⇒ `ValueError`.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/safety.py`
- `tests/domain/__init__.py`
- `tests/domain/test_safety.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_safety.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_safety.py --cov=hrobot_core.domain.safety --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Tests: obstáculo exactamente en `stop_distance` y `slow_distance`, punto medio ⇒ 0.5, obstáculo fuera del cono no afecta, scan vacío ⇒ 0.0, todo `NaN` ⇒ 0.0, todo `+inf` ⇒ 1.0, mezcla `NaN` + `+inf` ⇒ 1.0, mezcla `NaN` + obstáculo ⇒ factor del obstáculo, `-inf` solo ⇒ 0.0, rayo finito `< range_min` o `> range_max` ignorado, scan viejo ⇒ 0.0, config inválida
- [ ] No hay imports de `rclpy` ni mensajes ROS en el módulo

## Fuera de alcance
Publicación en `/cmd_vel`, nodo ROS, parámetros dinámicos.
