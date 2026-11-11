        # PR: TASK-025: Eliminar `velocity_smoother` (aceleración ya limitada por supervisor y Create 3)

        - **Rama:** `feat/TASK-025` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        0808009 test(TASK-025): update nav launch and config tests for smoother removal
a59adb7 feat(TASK-025): remove velocity_smoother from nav launch, params and docs
cefcd27 chore(TASK-025): move task to in_progress
        ```

        ## Cambios
        ```
        pendientes/01-verificacion-ros-hardware.md     |   6 +-
 src/README.md                                  |   4 +-
 src/hrobot_bringup/config/nav2_params.yaml     |  19 +---
 src/hrobot_bringup/launch/navigation.launch.py |  19 +---
 tasks/{backlog => in_progress}/TASK-025.md     |   0
 tests/config/test_nav_configs.py               | 125 ++++---------------------
 tests/launch/test_navigation_launch.py         |  38 ++++----
 7 files changed, 48 insertions(+), 163 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté pytest con cobertura (601 passed, cobertura 100 %), `ruff format --check`, `ruff check`, `mypy --strict` y `py_compile` de `navigation.launch.py`. Todos terminaron en OK. El diff solo toca archivos permitidos más el movimiento de la tarea, y los 3 commits usan el scope `(TASK-025)`.

Verifiqué en `navigation.launch.py` y `nav2_params.yaml`:
- `controller_server` remapea `cmd_vel` a `nav_cmd_vel_topic`.
- `bt_navigator` conserva el override de `odom_topic`.
- Los remapeos de TF se mantienen en todos los nodos.
- `_check_topic` sigue rechazando `/cmd_vel` y `<ns>/cmd_vel`.
- `FollowPath.desired_linear_vel` es 0.3.

Notas (no bloqueantes):
- `pendientes/03-calibracion.md` (líneas 14 y 19) todavía menciona `velocity_smoother`, lo que contradice el ítem "no quedan referencias" del checklist. El archivo no está en "Archivos permitidos", así que es un defecto de la spec; conviene una tarea de seguimiento.
- Los criterios de verificación en el robot (publishers de `/cmd_vel_nav` y `/r2/cmd_vel`, arranque y parada sin brusquedad) siguen pendientes de validación humana. El diff no los declara validados.
