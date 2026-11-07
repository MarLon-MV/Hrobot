        # PR: TASK-015: Supervisor de seguridad — lógica pura de limitación de velocidad

        - **Rama:** `feat/TASK-015` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 3

        ## Commits
        ```
        4409f9b test(TASK-015): isolate backwards clock and cover rule priorities
41f1ff9 test(TASK-015): cover safety supervisor rules, ramp and bench mode
e9928aa feat(TASK-015): add safety supervisor speed limiting logic
15b412e chore(TASK-015): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/services/safety_supervisor.py      | 242 ++++++++++
 tasks/{backlog => in_progress}/TASK-015.md         |   0
 tests/services/__init__.py                         |   0
 tests/services/test_safety_supervisor.py           | 492 +++++++++++++++++++++
 4 files changed, 734 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté los comandos del DoD y todos pasaron:
- Los 76 tests de `tests/services/test_safety_supervisor.py` pasan. La cobertura de `safety_supervisor.py` es del 100 %, con el umbral del 98 % cumplido.
- `ruff format --check src tests`, `ruff check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan sin errores.
- El `grep` de imports de ROS y `serial` en `domain` y `services` no da coincidencias. El diff solo toca los 3 archivos permitidos y el movimiento de la tarea que hace el orquestador.

Notas (no bloqueantes):
- `step` mide `dt` desde el máximo `now_s` visto, no desde el paso anterior como dice la regla 6. Con el reloj retrocedido la rampa sube menos de lo que dice la spec. Es más conservador y un test lo cubre, pero conviene documentarlo en la spec.
- El oráculo del test de propiedad lee atributos privados (`_mission`, `_cmd`, `_emergency_latched`). Acopla el test a la implementación, y solo comprueba la parada cuando fallan las precondiciones.
- El comando del DoD con `2>&1 | tail` lo bloqueó el sandbox. Ejecuté los mismos comandos sin tubería.
