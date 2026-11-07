# TASK-005: Dominio — máquina de estados de la misión de entrega

- **Rama Git:** `feat/TASK-005`
- **Depende de:** TASK-003
- **Prioridad:** P1

## Contexto y objetivo de negocio
Cada entrega de medicamentos sigue un ciclo controlado: salir de base, navegar a destino, esperar confirmación humana de recepción y regresar. Una máquina de estados explícita evita estados ambiguos y garantiza que el robot no dé por entregado nada sin confirmación.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/domain/mission.py
class MissionState(Enum):
    IDLE; NAVIGATING_TO_DESTINATION; AWAITING_CONFIRMATION
    RETURNING_TO_BASE; EMERGENCY_STOP; FAILED

class MissionEvent(Enum):
    START; GOAL_REACHED; NAV_FAILED; DELIVERY_CONFIRMED
    CONFIRMATION_TIMEOUT; EMERGENCY; RESET

@dataclass(frozen=True)
class Mission:
    mission_id: str; destination: Pose2D; base: Pose2D
    state: MissionState = MissionState.IDLE
    resume_state: MissionState | None = None

def transition(m: Mission, event: MissionEvent) -> Mission: ...   # puro; devuelve nueva Mission
```
Transiciones válidas:
- `IDLE --START--> NAVIGATING_TO_DESTINATION`
- `NAVIGATING_TO_DESTINATION --GOAL_REACHED--> AWAITING_CONFIRMATION`
- `AWAITING_CONFIRMATION --DELIVERY_CONFIRMED--> RETURNING_TO_BASE`
- `AWAITING_CONFIRMATION --CONFIRMATION_TIMEOUT--> RETURNING_TO_BASE` (marcar sin entrega confirmada: agregar campo `delivered: bool`, `False` en este caso)
- `RETURNING_TO_BASE --GOAL_REACHED--> IDLE`
- Cualquier estado excepto `IDLE`/`FAILED` `--NAV_FAILED--> FAILED`
- Cualquier estado `--EMERGENCY--> EMERGENCY_STOP` (guarda `resume_state`)
- `EMERGENCY_STOP --RESET--> IDLE` (nunca reanuda sola la misión)
- Cualquier otra combinación ⇒ `InvalidTransition(Exception)` con estado y evento en el mensaje.
- `Mission` es inmutable; `transition` no muta.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/mission.py`
- `tests/domain/__init__.py`
- `tests/domain/test_mission.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_mission.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_mission.py --cov=hrobot_core.domain.mission --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check .`, `ruff format --check .` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Test parametrizado que recorre **todas** las combinaciones estado×evento y verifica que solo las transiciones listadas son válidas
- [ ] Test del ciclo feliz completo con `delivered == True`, y del timeout con `delivered == False`
- [ ] Test: `EMERGENCY_STOP` + `GOAL_REACHED`/`START` ⇒ `InvalidTransition`; `RESET` ⇒ `IDLE`
- [ ] Test de inmutabilidad (la Mission original no cambia)

## Fuera de alcance
Integración con Nav2, persistencia, interfaz de usuario de confirmación.
