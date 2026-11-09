        # PR: TASK-024: Remapear TF (`/tf`, `/tf_static`) al namespace del Create 3 (`/r2`)

        - **Rama:** `feat/TASK-024` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        932baf6 feat(TASK-024): remap tf and tf_static to create3 namespace
f4ad4ef chore(TASK-024): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/launch/mapping.launch.py    | 43 +++++++++++++++++---
 src/hrobot_bringup/launch/navigation.launch.py | 21 ++++++++--
 src/hrobot_bringup/launch/rviz.launch.py       | 29 ++++++++++++--
 src/hrobot_bringup/launch/sensors.launch.py    | 21 +++++++++-
 src/hrobot_core/hrobot_core/adapters/names.py  |  2 +
 tasks/{backlog => in_progress}/TASK-024.md     |  0
 tests/adapters/test_names.py                   | 18 +++++++++
 tests/launch/test_launch_files.py              | 54 ++++++++++++++++++++++++++
 tests/launch/test_navigation_launch.py         | 39 +++++++++++++++++++
 9 files changed, 214 insertions(+), 13 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q` (609 passed), `ruff format --check` y `ruff check` (limpios), `mypy --strict` (sin errores) y la búsqueda de imports de ROS en `domain` y `services` (0 coincidencias). Revisé a mano que ningún `Node(...)` quede sin remapear: `ld19_node` y `safety` quedan fuera según la tarea, y el resto está cubierto. El diff solo toca archivos permitidos, más el movimiento de la tarea que hace el orquestador.

Notas (no bloqueantes):
- No pude ejecutar `python3 -m py_compile` sobre los cuatro launch porque el entorno pidió aprobación y no se concedió. La sintaxis queda cubierta indirectamente, porque los tests hacen `ast.parse` de esos archivos y pasan. Conviene correr ese comando una vez de forma manual.
- Los tests de launch son estáticos (AST y texto). Ninguno comprueba que `SetRemap` dentro de `GroupAction` llegue a slam_toolbox en ejecución, ni que `create3_namespace:=/` dé `/tf`. Esto último solo está cubierto a nivel de `namespaced()`. Ambas cosas necesitan verificación humana en el robot, como indica la tarea.
