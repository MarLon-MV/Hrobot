        # PR: TASK-009: Launch files de bringup (sensor, supervisor, mapeo, RViz)

        - **Rama:** `feat/TASK-009` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        220a3ed test(TASK-009): add static launch file checks
7334afa feat(TASK-009): add bringup launch files and glob-based setup install
889ba03 chore(TASK-009): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/launch/mapping.launch.py |  39 ++++++
 src/hrobot_bringup/launch/rviz.launch.py    |  24 ++++
 src/hrobot_bringup/launch/safety.launch.py  |  41 ++++++
 src/hrobot_bringup/launch/sensors.launch.py |  75 +++++++++++
 src/hrobot_bringup/setup.py                 |  11 +-
 tasks/{backlog => in_progress}/TASK-009.md  |   0
 tests/launch/__init__.py                    |   0
 tests/launch/test_launch_files.py           | 199 ++++++++++++++++++++++++++++
 8 files changed, 388 insertions(+), 1 deletion(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q --cov=hrobot_core --cov-fail-under=85` (502 passed, cobertura 100 %), `pytest tests/launch/test_launch_files.py` (25 passed), `ruff format --check`, `ruff check` y `mypy --strict` (todos limpios). El grep de capas sobre `domain` y `services` no dio coincidencias, y el diff solo toca archivos permitidos más el movimiento de la tarea hecho por el orquestador.

Notas (no bloqueantes):
- No pude ejecutar `python3 -m py_compile <launch>` por archivo: el entorno pidió aprobación y la sesión no es interactiva. Lo cubre `test_compiles`, que hace `compile()` de los cuatro archivos y pasó, pero conviene que un humano corra esos cuatro comandos.
- No se verificó `ros2 launch ... --show-args`, que el DoD marca como opcional, ni el arranque en un robot real. Los tests son solo análisis estático, y su docstring lo dice.
- `test_setup_installs_required_dirs` comprueba `glob` y `RESOURCE_DIRS` en `setup.py` buscando texto, y solo extrae la lista de directorios con `ast`. No valida el prefijo `share/hrobot_bringup/`.
- `_files_in` en `setup.py` usa rutas relativas al directorio actual. Con `colcon` no hay problema, pero un `pip install` lanzado desde otro directorio dejaría `data_files` vacío sin avisar.
- `test_executables_exist_as_console_scripts` repite una comprobación en su último bucle.
