        # PR: TASK-007: Configuración Nav2 / AMCL / slam_toolbox con validación automática

        - **Rama:** `feat/TASK-007` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        ea1926e feat(TASK-007): add nav2 and slam_toolbox configs with coherence validator
ae5df94 chore(TASK-007): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/config/nav2_params.yaml         | 125 ++++++++
 src/hrobot_bringup/config/slam_toolbox_params.yaml |  13 +
 tasks/{backlog => in_progress}/TASK-007.md         |   0
 tests/config/__init__.py                           |   0
 tests/config/test_nav_configs.py                   | 322 +++++++++++++++++++++
 5 files changed, 460 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q` completo (212 passed, 22 de ellos en `tests/config/test_nav_configs.py`), `ruff format --check`, `ruff check` y `mypy --strict`, y todos terminan con código 0. El grep de capas no da coincidencias. El diff solo toca los 4 archivos permitidos más el movimiento de la tarea que hace el orquestador.

Notas (no bloqueantes):
- `_check_costmaps` en `tests/config/test_nav_configs.py` compara `res != slam_res` sin exigir que existan. Si `resolution` faltara a la vez en slam_toolbox y en el costmap, `validate` no lo reportaría, y `test_resolution_matches` pasaría con `None == None`. Conviene exigir valor numérico (o 0.05).
- `min_vel_y` y `max_vel_y` (0.0 según la spec) y `velocity_smoother.max_velocity[2]` no se validan en `validate`. La spec no lo exige.
- Los YAML no se validaron contra una instalación real de Nav2. Es la verificación humana de la spec, y los parámetros dudosos llevan `TODO(verify)`.
