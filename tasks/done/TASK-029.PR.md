        # PR: TASK-029: Condiciones de seguridad del Create 3 en la exploración (dock, secuestro, batería, peligros)

        - **Rama:** `feat/TASK-029` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 3

        ## Commits
        ```
        13faa2d fix(TASK-029): lock robot status samples in exploration node
248da32 fix(TASK-029): serialize manager calls and avoid double failure count
a62b5f1 fix(TASK-029): cancel pending undock on stop and timeout
4ed0942 fix(TASK-029): expire stale create3 status and add generation guard to undock
0745337 feat(TASK-029): add create3 safety topics, undock client and params to exploration node
6b8741f feat(TASK-029): add dock, kidnap, battery and hazard handling to exploration manager
bb5822c chore(TASK-029): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/config/exploration_params.yaml  |   5 +
 .../hrobot_core/adapters/exploration_node.py       | 182 +++++++-
 src/hrobot_core/hrobot_core/adapters/names.py      |   7 +
 .../hrobot_core/services/exploration_manager.py    | 272 ++++++++++-
 src/hrobot_core/package.xml                        |   2 +
 tasks/{backlog => in_progress}/TASK-029.md         |   0
 tests/adapters/test_exploration_node.py            | 344 +++++++++++++-
 tests/config/test_exploration_config.py            |  31 +-
 tests/services/test_exploration_manager.py         | 511 ++++++++++++++++++---
 9 files changed, 1269 insertions(+), 85 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest` completo con cobertura: 798 pasan, cobertura total 100 % (mínimo 85 %). El módulo `exploration_manager.py` tiene 100 % (mínimo 95 %). `ruff format --check`, `ruff check` y `mypy --strict` salen limpios. El grep de capas no da coincidencias, el diff solo toca archivos permitidos y los commits siguen Conventional Commits con scope `(TASK-029)`. No pude ejecutar `colcon build` en este entorno, ni nada que requiera el robot.

Notas (no bloqueantes):
- Los umbrales `undock_timeout_s` y `min_battery_pct` están marcados `TODO(verify)` en el YAML. La política ante datos desconocidos es conservadora, según lo exigido: no se explora con dock desconocido, y secuestro desconocido pausa.
- `on_hazard` pasa la pose actual del robot, no la del contacto real. Es lo que pide la spec, pero conviene validarlo en el robot.
- `test_hazard_cancels_goal_and_blacklists_contact` del adapter no comprueba la lista negra. Esa verificación solo está en los tests del gestor.
