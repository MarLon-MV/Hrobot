        # PR: TASK-019: Test de higiene del repositorio (sintaxis, marcadores de conflicto, entry points)

        - **Rama:** `feat/TASK-019` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        4411142 test(TASK-019): add repo hygiene tests
c9d20ef chore(TASK-019): move task to in_progress
        ```

        ## Cambios
        ```
        tasks/{backlog => in_progress}/TASK-019.md |   0
 tests/test_repo_hygiene.py                 | 219 +++++++++++++++++++++++++++++
 2 files changed, 219 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q tests/test_repo_hygiene.py` (14 passed), `ruff format --check src tests` (37 archivos ya formateados) y `ruff check src tests` (sin hallazgos). El diff solo toca `tests/test_repo_hygiene.py` y el movimiento de la tarea. `hrobot_core/setup.py` declara 2 `console_scripts` reales, así que el test de entry points no pasa en vacío. No ejecuté `mypy --strict`, que solo cubre `src/` y no el archivo nuevo, ni la suite completa ni la cobertura.

Notas (no bloqueantes):
- `rglob("*")` recorre directorios excluidos como `.git` y `.venv` antes de filtrarlos. Es solo más lento.
- `_console_scripts` usa `ast.literal_eval`, que lanza `ValueError` si algún `entry_points` deja de ser literal. El test fallaría en ese caso en vez de avisar con un mensaje claro.
- No se contemplan entradas con extras tipo `name = mod:func [extra]`.
