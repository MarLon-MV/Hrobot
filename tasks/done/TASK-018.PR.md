        # PR: TASK-018: Supervisor — reiniciar la rampa de velocidad tras una pérdida de supervisión

        - **Rama:** `feat/TASK-018` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 2

        ## Commits
        ```
        9c58d32 feat(TASK-018): restart speed ramp after supervision loss
29e9bf7 chore(TASK-018): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/services/safety_supervisor.py      |  19 ++-
 tasks/{backlog => in_progress}/TASK-018.md         |   0
 tests/services/test_safety_supervisor.py           | 144 ++++++++++++++++++++-
 3 files changed, 160 insertions(+), 3 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté los comandos del DoD y todos salieron con código 0:
- **Tests:** `pytest` de `test_safety_supervisor.py` (94 passed) y la suite completa (407 passed, cobertura 100 %). La cobertura de `safety_supervisor.py` es 100 % con umbral 98.
- **Calidad:** `ruff format --check`, `ruff check` y `mypy --strict` pasan sin errores.
- **Alcance y capas:** el diff solo toca los dos archivos permitidos más el movimiento de la tarea. El `grep` de imports ROS en `domain` y `services` no da coincidencias.

Notas (no bloqueantes):
- El test del reloj que retrocede usa la secuencia `1.0, 1.2, 1.0, 1.3` en lugar de la `1.0, 0.5, 1.1` del ejemplo de la spec. Cubre el mismo comportamiento: el hueco se mide contra el máximo visto y no hay reinicio ni subida.
- El test existente `test_clock_going_backwards_no_exception_and_no_rise` se ajustó con `max_step_gap_s=1.0` y un comentario `# TASK-018`. Eso está permitido por la spec porque no debilita ninguna aserción.
