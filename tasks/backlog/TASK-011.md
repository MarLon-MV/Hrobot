# TASK-011: Misión — blindar el enclavamiento de EMERGENCY_STOP y permitir RESET desde FAILED

- **Rama Git:** `feat/TASK-011`
- **Depende de:** TASK-005
- **Prioridad:** P0

## Contexto y objetivo de negocio
La revisión de TASK-005 encontró dos defectos en `transition`:
1. `NAV_FAILED` desde `EMERGENCY_STOP` lleva a `FAILED`, abandonando el estado de emergencia **sin un `RESET` explícito**. Esto viola el enclavamiento exigido en `architecture_rules.md` §5 ("solo un reset explícito lo libera"). Es un caso realista: tras una parada de emergencia, Nav2 cancela el objetivo y puede reportar fallo o cancelación con retraso.
2. `FAILED` no tiene salida: `RESET` lanza `InvalidTransition`, y la única forma de limpiar una misión fallida es simular una emergencia.

## Alcance técnico
Modificar `src/hrobot_core/hrobot_core/domain/mission.py` y sus tests (sin cambiar la API pública: `Mission`, `MissionState`, `MissionEvent`, `InvalidTransition`, `transition`).

Reglas nuevas:
- **Invariante de enclavamiento:** desde `EMERGENCY_STOP`, el **único** evento que cambia el estado es `RESET` (⇒ `IDLE`). Cualquier otro evento no cambia el estado:
  - `NAV_FAILED` y `EMERGENCY` ⇒ se ignoran y se devuelve la misma `Mission` sin cambios (conserva `resume_state`). Son eventos tardíos legítimos y no deben lanzar excepción.
  - `START`, `GOAL_REACHED`, `DELIVERY_CONFIRMED`, `CONFIRMATION_TIMEOUT` ⇒ siguen lanzando `InvalidTransition`.
- `FAILED --RESET--> IDLE`, con `delivered=False` y `resume_state=None`.
- `FAILED --EMERGENCY--> EMERGENCY_STOP` se mantiene.
- `NAV_FAILED` desde `IDLE` y desde `FAILED` se mantiene como hasta ahora (`InvalidTransition`).
- Actualizar el docstring del módulo y de `transition` con la tabla de transiciones completa y la invariante de enclavamiento.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/mission.py`
- `tests/domain/test_mission.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_mission.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_mission.py --cov=hrobot_core.domain.mission --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Test de la invariante: para **cada** evento de `MissionEvent` aplicado a una misión en `EMERGENCY_STOP`, o bien el estado resultante sigue siendo `EMERGENCY_STOP`, o bien el evento es `RESET` y el resultado es `IDLE`, o bien se lanza `InvalidTransition`. Ningún otro desenlace es válido.
- [ ] Test: `START → EMERGENCY → NAV_FAILED` deja la misión en `EMERGENCY_STOP` con el mismo `resume_state` (`NAVIGATING_TO_DESTINATION`)
- [ ] Test: `START → EMERGENCY → NAV_FAILED → RESET` termina en `IDLE`
- [ ] Test: `START → NAV_FAILED → RESET` termina en `IDLE` con `delivered=False` y `resume_state=None`
- [ ] Test: `FAILED + EMERGENCY ⇒ EMERGENCY_STOP`; `FAILED + START/GOAL_REACHED/NAV_FAILED ⇒ InvalidTransition`
- [ ] Test parametrizado estado×evento actualizado con la tabla nueva completa (cada combinación tiene una expectativa explícita)
- [ ] Los tests existentes de TASK-005 siguen pasando; si alguno depende de `EMERGENCY_STOP + NAV_FAILED ⇒ FAILED` o de `FAILED + RESET ⇒ InvalidTransition`, se corrige su expectativa con un comentario `# TASK-011` que lo explique
- [ ] El módulo sigue sin importar `rclpy` ni mensajes ROS

## Fuera de alcance
Integración con Nav2, publicación de eventos, persistencia, cambios en `geometry.py` o `safety.py`.
