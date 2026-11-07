        # PR: TASK-004: Dominio — política de seguridad por LiDAR (reducción y parada)

        - **Rama:** `feat/TASK-004` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 3

        ## Commits
        ```
        f94fc83 test(TASK-004): cover zero/negative angle increment and negative range_min
8bd466f fix(TASK-004): return stop factor when ray angle overflows
939778b feat(TASK-004): add lidar safety speed-factor policy
36d283c chore(TASK-004): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/safety.py | 129 +++++++++++++++++++
 tasks/{backlog => in_progress}/TASK-004.md   |   0
 tests/domain/test_safety.py                  | 180 +++++++++++++++++++++++++++
 3 files changed, 309 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q tests/domain/test_safety.py` (43 passed) y la variante con `--cov=hrobot_core.domain.safety --cov-fail-under=98` (100 %, 43 passed). `ruff format --check`, `ruff check` y `mypy --strict` pasaron sin errores. El `grep` de imports ROS en `domain` y `services` no dio coincidencias, y el diff solo toca `safety.py`, `test_safety.py` y el movimiento de la tarea hecho por el orquestador.

Notas (no bloqueantes):
- No pude ejecutar una comprobación manual de `normalize_angle` con ángulos finitos muy grandes (p. ej. 1e300), porque el comando requería aprobación. Ese caso no tiene test dedicado. El desbordamiento a `inf` sí está cubierto y devuelve 0.0.
- Con `range_min` negativo o cero, una distancia ≤ 0 se trata como obstáculo y detiene el robot. Es fallo seguro y tiene test, pero la spec no lo define.
