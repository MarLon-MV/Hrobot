# TASK-003: Dominio — geometría 2D (Pose2D, yaw ↔ cuaternión, normalización de ángulos)

- **Rama Git:** `feat/TASK-003`
- **Depende de:** TASK-002
- **Prioridad:** P0

## Contexto y objetivo de negocio
La navegación, la entrega en waypoints y la comprobación de llegada requieren operaciones de pose 2D correctas. Errores de envoltura de ángulo (±π) son una fuente clásica de giros erráticos del robot. Se implementa en `domain` como lógica pura, sin ROS.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/domain/geometry.py
@dataclass(frozen=True)
class Pose2D:
    x: float; y: float; yaw: float   # metros, metros, radianes (REP-103)

def normalize_angle(a: float) -> float: ...            # resultado en (-π, π]
def angle_diff(target: float, current: float) -> float: ...  # diferencia más corta con signo
def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]: ...   # (x, y, z, w)
def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float: ...
def distance(a: Pose2D, b: Pose2D) -> float: ...
def reached(current: Pose2D, goal: Pose2D, xy_tol: float, yaw_tol: float) -> bool: ...
```
- Rechazar con `ValueError` entradas `NaN`/`inf` y tolerancias negativas.
- Solo `math` de la librería estándar.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/geometry.py`
- `tests/domain/__init__.py`
- `tests/domain/test_geometry.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_geometry.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_geometry.py --cov=hrobot_core.domain.geometry --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Tests de borde: `normalize_angle(π)==π`, `normalize_angle(-π)==π`, `normalize_angle(3π)`, `angle_diff(-3.1, 3.1)` ≈ 0.083, ida y vuelta yaw→quat→yaw, `NaN`, `inf`, tolerancias negativas
- [ ] `grep -rE "import (rclpy|geometry_msgs)" src/hrobot_core/hrobot_core/domain` no produce coincidencias

## Fuera de alcance
Transformaciones 3D, conversión a mensajes ROS.
