        # PR: TASK-028: Launch y configuración del modo exploración (bringup)

        - **Rama:** `feat/TASK-028` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        6f0e306 test(TASK-028): cover exploration config and launch; docs: exploration mode
ff716d4 feat(TASK-028): add exploration launch and params
5c5e2f8 feat(TASK-028): add use_slam mode to navigation launch
fc6340f chore(TASK-028): move task to in_progress
        ```

        ## Cambios
        ```
        src/README.md                                     |  15 +++
 src/hrobot_bringup/config/exploration_params.yaml |  24 ++++
 src/hrobot_bringup/launch/exploration.launch.py   | 108 +++++++++++++++++
 src/hrobot_bringup/launch/navigation.launch.py    |  23 +++-
 tasks/{backlog => in_progress}/TASK-028.md        |   0
 tests/config/test_exploration_config.py           |  83 +++++++++++++
 tests/launch/test_exploration_launch.py           | 137 ++++++++++++++++++++++
 7 files changed, 386 insertions(+), 4 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Comandos ejecutados: `python3 -m pytest -q` (715 passed), `ruff format --check` y `ruff check` (limpios), `mypy --strict` (sin errores en 16 archivos). El grep de capas sobre `domain` y `services` no da coincidencias, y `git diff --name-only main...HEAD` solo lista archivos permitidos más el movimiento de la tarea. `py_compile` y la variante con `PYTHONPATH=src/hrobot_core` no pude ejecutarlos (el entorno pidió aprobación), pero los tests ya parsean `exploration.launch.py` con `ast`, así que la sintaxis es válida.

Notas (no bloqueantes):
- `test_no_amcl_nor_static_map_server` repite `"nav2_amcl"` en su tupla.
- `test_navigation_slam_mode_skips_localization_and_includes` cuenta la cadena `condition=not_slam` (==4), lo que es frágil ante refactors.
- Ninguno de los tests de launch ejecuta el launch, así que el modo `use_slam` y el guardado del mapa solo se comprueban de forma estática; la verificación en el robot sigue pendiente (TASK-029).
