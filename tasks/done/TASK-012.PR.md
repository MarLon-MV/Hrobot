        # PR: TASK-012: Infraestructura para nodos ROS 2 testeables sin ROS instalado

        - **Rama:** `feat/TASK-012` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        a201aa9 test(TASK-012): add static architecture tests
efbf309 test(TASK-012): add ros and serial fakes with fixture
dce4ee1 feat(TASK-012): add topic and frame name constants
96353fb chore(TASK-012): add mypy overrides for ros imports
ef2159d chore(TASK-012): move task to in_progress
        ```

        ## Cambios
        ```
        pyproject.toml                                |  10 +
 src/hrobot_core/hrobot_core/adapters/names.py |  15 ++
 tasks/{backlog => in_progress}/TASK-012.md    |   0
 tests/adapters/test_names.py                  |  21 ++
 tests/conftest.py                             |  18 ++
 tests/ros_fakes.py                            | 370 ++++++++++++++++++++++++++
 tests/test_architecture.py                    | 190 +++++++++++++
 tests/test_ros_fakes.py                       | 168 ++++++++++++
 8 files changed, 792 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté estos comandos:
- `pytest -q --cov=hrobot_core --cov-fail-under=85`: 255 passed, cobertura 100 %.
- `ruff format --check src tests` y `ruff check src tests`: sin errores.
- `mypy --strict src/hrobot_core/hrobot_core`: "Success: no issues found in 9 source files".
- `grep` de imports de ROS en `domain/` y `services/`: sin coincidencias.
- `grep` de `[[tool.mypy.overrides]]` en `pyproject.toml`: 2 coincidencias.
- El diff solo toca los archivos permitidos, más el movimiento de la spec a `in_progress`.

No pude leer el código de salida de mypy: el entorno bloqueó las líneas de comandos que lo habrían mostrado. Pasó sin errores de tipos, así que asumo que es 0, pero no lo verifiqué.

Notas (no bloqueantes):
- El override `module = ["hrobot_core.adapters.*_node"]` (`pyproject.toml`, línea 41 aprox.) no funciona. Mypy lo rechaza ("Patterns must be fully-qualified module names, optionally with '*' in some components") y lo ignora, y lo imprime en cada ejecución. Es el patrón literal de la spec, así que es un fallo de la spec. Los nodos de TASK-014/016 que hereden de `Node` con `--strict` probablemente fallarán con `disallow_subclassing_any`. Conviene que el Manager lo corrija en la spec, por ejemplo con `hrobot_core.adapters.*`.
- `test_only_supervisor_publishes_cmd_vel` exime a cualquier archivo llamado `supervisor_node.py` en cualquier carpeta, no solo `adapters/supervisor_node.py`. Es un hueco menor.
- `test_violation_scan_reports_offending_file` no ejercita `_violations` sobre archivos reales.
- Los detectores no ven imports dinámicos (`importlib`, `__import__`).
