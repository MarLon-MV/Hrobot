        # PR: TASK-022: Controlador Regulated Pure Pursuit y ruta del BT seguro desde el launch

        - **Rama:** `feat/TASK-022` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        aa04d93 feat(TASK-022): switch to regulated pure pursuit and inject safe BT path from launch
f5c0816 chore(TASK-022): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/config/nav2_params.yaml     | 33 ++++++----
 src/hrobot_bringup/launch/navigation.launch.py |  8 ++-
 tasks/{backlog => in_progress}/TASK-022.md     |  0
 tests/config/test_nav_configs.py               | 83 +++++++++++++++++---------
 tests/launch/test_navigation_launch.py         | 40 +++++++++++++
 5 files changed, 123 insertions(+), 41 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `PYTHONPATH=src/hrobot_core pytest -q`: 571 passed. `ruff format --check`, `ruff check` y `mypy --strict` salieron limpios. No pude ejecutar `py_compile` porque el comando se bloqueó por permisos. En su lugar leí `navigation.launch.py` y es sintácticamente válido, y pytest lo carga sin error. Con grep comprobé que el YAML ya no contiene `dwb` ni `min_vel_x`/`max_vel_x`/`max_vel_theta`. También comprobé que `controller_plugins: ["FollowPath"]` se conserva y que `default_nav_to_pose_bt_xml` sigue vacío con un comentario que remite al launch. El diff toca solo los 4 archivos permitidos más el movimiento de la tarea, sin cambios de interfaces ni de remapeos.

Notas (no bloqueantes):
- `_check_controller` solo rechaza plugins que contengan "dwb". Un plugin ausente o distinto de RPP pasaría ese validador, aunque `test_velocity_limits` sí exige el plugin RPP exacto.
- El `py_compile` del DoD queda sin ejecutar por mi parte.
- Los valores de RPP quedan marcados `TODO(verify)`: no están validados en el robot.
