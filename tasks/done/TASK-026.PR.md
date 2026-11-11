        # PR: TASK-026: Dominio de exploración por fronteras (detección, agrupación y selección)

        - **Rama:** `feat/TASK-026` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        9ad299b feat(TASK-026): add frontier exploration domain
0ae695b chore(TASK-026): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/exploration.py | 198 ++++++++++++++++++++++
 tasks/{backlog => in_progress}/TASK-026.md        |   0
 tests/domain/test_exploration.py                  | 188 ++++++++++++++++++++
 3 files changed, 386 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté estos comandos y todos pasaron: `pytest -q` (629 passed), el pytest de `test_exploration.py` con cobertura (28 passed, 100 % en `exploration.py`, mínimo exigido 95 %), `ruff format --check`, `ruff check` y `mypy --strict`. El diff solo toca `exploration.py`, su test y el movimiento de la tarea. La búsqueda de imports ROS solo da coincidencias en `adapters/`, y `domain` y `services` quedan limpios.

Notas (no bloqueantes):
- No hay tests de los límites exactos de `blacklist_radius` y `min_goal_distance`; el código usa `<` (estricto) y un cambio a `<=` no fallaría ningún test.
- `GridMap` no valida que los valores de `data` estén en -1..100. La spec no lo exige.
