        # PR: TASK-023: Namespace del Create 3 (`/r2`) en cmd_vel y odom

        - **Rama:** `feat/TASK-023` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        944ae96 feat(TASK-023): namespace create3 cmd_vel and odom
444f381 chore(TASK-023): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/config/nav2_params.yaml         |  4 +-
 src/hrobot_bringup/launch/navigation.launch.py     | 45 ++++++++++++++-----
 src/hrobot_bringup/launch/safety.launch.py         | 52 ++++++++++++++--------
 src/hrobot_core/hrobot_core/adapters/names.py      | 22 +++++++++
 .../hrobot_core/adapters/supervisor_node.py        | 17 ++++++-
 tasks/{backlog => in_progress}/TASK-023.md         |  0
 tests/adapters/test_names.py                       | 27 +++++++++++
 tests/adapters/test_supervisor_node.py             | 21 ++++++++-
 tests/launch/test_launch_files.py                  |  9 ++++
 tests/launch/test_navigation_launch.py             | 32 +++++++++++++
 10 files changed, 196 insertions(+), 33 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté pytest con cobertura (594 passed, 100%), `ruff format --check`, `ruff check` y `mypy --strict`; los tres pasaron sin errores. El grep de capas sobre `domain` y `services` no dio coincidencias. No pude ejecutar `py_compile` sobre los dos launch porque el entorno pidió aprobación. Los tests de launch parsean ambos archivos con `ast`, así que la sintaxis está comprobada de forma indirecta.

Notas (no bloqueantes):
- `controller_server` probablemente sigue suscrito a `/odom`. Su `odom_topic` no está en el YAML y, hasta donde recuerdo, en Jazzy declara su propio `odom_topic` con default `odom`. Esa lectura es de memoria y no la verifiqué. La tarea solo pide sobrescribir `bt_navigator` y `velocity_smoother`, así que no es defecto de este diff, pero con hardware real RPP podría no recibir odometría. Conviene una tarea de seguimiento.
- `validate_topic` en `supervisor_node.py` acepta `cmd_vel_topic=/cmd_vel_nav`, que es `NAV_CMD_VEL_TOPIC`. El supervisor se suscribiría a su propia salida, pero hoy solo se puede dar ese valor por un parámetro explícito.
- `test_check_topic_forbids_namespaced_cmd_vel` y `test_safety_declares_create3_namespace_and_passes_cmd_vel_topic` comparan cadenas de texto del fuente. Solo detectan que el código esté escrito, no que se comporte bien. La tarea pedía tests estáticos, por eso no bloquea.
- `validate_nav_cmd_vel_topic` repite el literal `("/cmd_vel",)` en su default y en `FORBIDDEN_CMD_VEL`.
