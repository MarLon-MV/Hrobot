        # PR: TASK-011: Misión — blindar el enclavamiento de EMERGENCY_STOP y permitir RESET desde FAILED

        - **Rama:** `feat/TASK-011` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        92f861b fix(TASK-011): lock EMERGENCY_STOP interlock and allow RESET from FAILED
314212d chore(TASK-011): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/mission.py | 30 +++++++++++--
 tasks/{backlog => in_progress}/TASK-011.md    |  0
 tests/domain/test_mission.py                  | 61 ++++++++++++++++++++++++++-
 3 files changed, 87 insertions(+), 4 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté en `feat/TASK-011` los comandos del DoD y todos terminan con código 0:
- `pytest -q tests/domain/test_mission.py`: 67 passed.
- Cobertura de `domain.mission`: 100 % (mínimo exigido 98 %).
- `ruff check`, `ruff format --check` y `mypy --strict`: sin incidencias.

El diff solo toca `mission.py` y `test_mission.py`; el cambio de `tasks/` es el movimiento del orquestador. `mission.py` no importa `rclpy` ni mensajes ROS, y `EMERGENCY_STOP` solo sale con `RESET`.

Notas (no bloqueantes):
- `test_emergency_stop_interlock_invariant` solo parte de `EMERGENCY` desde `NAVIGATING_TO_DESTINATION`. El `resume_state` de los demás estados queda cubierto por la tabla parametrizada y por `test_failed_then_emergency_then_reset_clears_resume_state`.
- `test_emergency_stop_interlock_invariant` usa `try/except: return`, así que no distingue lanzar de ignorar. Esa distinción la cubren los tests explícitos de `NAV_FAILED` y de eventos de confirmación.
