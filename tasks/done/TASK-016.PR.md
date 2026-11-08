        # PR: TASK-016: Nodo ROS 2 del supervisor de seguridad (único publicador de `/cmd_vel`)

        - **Rama:** `feat/TASK-016` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        f5cda08 test(TASK-016): add supervisor node tests with ros fakes
25faf17 feat(TASK-016): add supervisor node as sole cmd_vel publisher
560819d chore(TASK-016): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/adapters/supervisor_node.py        | 229 ++++++++++++++++
 src/hrobot_core/setup.py                           |   6 +-
 tasks/{backlog => in_progress}/TASK-016.md         |   0
 tests/adapters/test_supervisor_node.py             | 304 +++++++++++++++++++++
 4 files changed, 538 insertions(+), 1 deletion(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q --cov=hrobot_core --cov-fail-under=85`: 436 tests pasan y la cobertura total es 100 %, con `supervisor_node.py` también al 100 %. `ruff format --check`, `ruff check` y `mypy --strict` pasan sin errores. El `grep` de capas no encuentra importaciones de ROS en `domain` ni en `services`. El diff solo toca los archivos permitidos más el movimiento de la tarea.

Notas (no bloqueantes):
- No ejecuté por separado los comandos del DoD de `tests/adapters/test_supervisor_node.py` ni `tests/test_architecture.py`. La suite completa los incluye y pasa.
- Una excepción en `_on_scan` (por ejemplo, un scan malformado) terminaría `spin`. El `finally` de `main` publicaría ceros, así que falla de forma segura.
- Quedan pendientes en hardware, como indica la tarea: la API real de `rclpy` frente a los fakes, `Twist` frente a `TwistStamped` en el Create 3, y la unicidad del publicador de `/cmd_vel` con `ros2 topic info -v`.
