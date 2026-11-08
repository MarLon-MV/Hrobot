        # PR: TASK-020: Launch de navegación (Nav2 con salida a `/cmd_vel_nav`)

        - **Rama:** `feat/TASK-020` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 3

        ## Commits
        ```
        bb74a39 fix(TASK-020): apply normalized nav topic and assert node wiring in launch tests
9b75df5 fix(TASK-020): reject raw controller topic, normalize topic and test launch wiring
c022ca2 feat(TASK-020): add navigation launch with nav2 cmd_vel_nav remaps
24efa88 chore(TASK-020): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/launch/navigation.launch.py | 133 ++++++++
 tasks/{backlog => in_progress}/TASK-020.md     |   0
 tests/launch/test_navigation_launch.py         | 424 +++++++++++++++++++++++++
 3 files changed, 557 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q tests/launch/test_navigation_launch.py` (33 passed), `ruff format --check src tests` (limpio) y `ruff check src tests` (limpio). No pude ejecutar `py_compile` porque el sandbox lo bloqueó. El diff solo toca los dos archivos permitidos más el movimiento de la tarea. El `grep` de la cadena de velocidad en `nav2_params.yaml` y `names.py` es coherente: los `node_names` coinciden con los ejecutables lanzados y `/cmd_vel_nav` coincide con `NAV_CMD_VEL_TOPIC`.

Notas (no bloqueantes):
- `py_compile` no se ejecutó. El DoD lo exige, así que conviene correrlo a mano.
- `_check_topic` y el efecto de `SetLaunchConfiguration` solo se verifican por análisis estático (AST). Sin ROS no se puede comprobar en ejecución que el remapeo use el valor normalizado.
- Los nombres de tópico por defecto de `controller_server` y `velocity_smoother` (`cmd_vel`, `cmd_vel_smoothed`) siguen sin verificar. Queda pendiente la verificación humana con `ros2 topic info /cmd_vel -v` en la RPi5, como indica el comentario `# VERIFY`.
