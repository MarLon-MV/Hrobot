        # PR: TASK-027: Gestor y nodo de exploración autónoma (núcleo)

        - **Rama:** `feat/TASK-027` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        d67d316 test(TASK-027): cover exploration node adapter
4205032 feat(TASK-027): add exploration node adapter
768b9ce feat(TASK-027): add exploration manager state machine
19a0ef9 chore(TASK-027): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/adapters/exploration_node.py       | 373 ++++++++++++
 src/hrobot_core/hrobot_core/adapters/names.py      |   5 +
 .../hrobot_core/services/exploration_manager.py    | 192 +++++++
 src/hrobot_core/package.xml                        |   5 +
 src/hrobot_core/setup.py                           |   1 +
 tasks/{backlog => in_progress}/TASK-027.md         |   0
 tests/adapters/test_exploration_node.py            | 625 +++++++++++++++++++++
 tests/services/test_exploration_manager.py         | 261 +++++++++
 8 files changed, 1462 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté la suite completa con cobertura (692 tests OK, 100 % de cobertura total y de `exploration_manager`), `ruff format --check`, `ruff check` y `mypy --strict`, todos sin errores. `git diff --name-only main...HEAD` solo lista archivos permitidos más el movimiento de la tarea. El `grep` de capas no encuentra imports de ROS en `domain` ni `services`, no hay `cmd_vel` ni `sleep` en el nodo, y los commits usan Conventional Commits con scope `(TASK-027)`.

No pude ejecutar el comando de cobertura del manager con el prefijo `PYTHONPATH=` porque el entorno me lo bloqueó. Lo cubre la ejecución completa, donde el manager marca 100 %.

Notas (no bloqueantes):
- `colcon build` no se pudo verificar: el DoD lo exige solo si hay ROS 2 instalado, y los tests usan fakes de rclpy, Nav2 y tf2. Falta validación humana con ROS 2 real, y así lo declara el docstring de los tests.
- `lookup_transform(..., Time())` toma la última TF sin comprobar su antigüedad, así que una pose caducada se aceptaría. Conviene revisarlo en TASK-029.
- El guardado de mapa bloquea el callback del temporizador hasta `map_save_timeout_s`, y `~/stop` no puede ejecutarse mientras tanto porque comparten grupo de callbacks. Es aceptable, pero queda documentado.
- `map_save_name` se pasa tal cual a `map_url`. Es un parámetro del operador, pero no se valida contra rutas con `..` o absolutas.
