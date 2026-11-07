        # PR: TASK-005: Dominio — máquina de estados de la misión de entrega

        - **Rama:** `feat/TASK-005` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        7915a4b test(TASK-005): cover mission transitions, immutability and emergency
124fd03 feat(TASK-005): add delivery mission state machine
0279b14 chore(TASK-005): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/mission.py |  94 ++++++++++++++++++++++
 tasks/{backlog => in_progress}/TASK-005.md    |   0
 tests/domain/test_mission.py                  | 107 ++++++++++++++++++++++++++
 3 files changed, 201 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté estos comandos y todos pasaron:
- `pytest -q tests/domain/test_mission.py`: 51 tests pasan.
- La misma suite con `--cov=hrobot_core.domain.mission --cov-fail-under=98`: cobertura del 100 %.
- `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core`: sin errores.
- El `grep` de capas sobre `domain` y `services` no dio coincidencias.
- El diff solo toca `mission.py`, `test_mission.py` y el movimiento de la tarea a `in_progress`.

No ejecuté la suite completa con `--cov-fail-under=85`. El sandbox bloqueó ese comando y solo pude lanzar los del DoD de esta tarea.

Notas (no bloqueantes):
- `NAV_FAILED` desde `EMERGENCY_STOP` lleva a `FAILED`. Es lo que pide la spec ("cualquier estado excepto IDLE/FAILED"). Para salir de ahí hay que pasar por `EMERGENCY` y luego `RESET`, así que nada se reanuda solo.
- `EMERGENCY` desde `EMERGENCY_STOP` es idempotente y conserva el `resume_state` original. La spec no lo precisa, pero el test lo cubre.
