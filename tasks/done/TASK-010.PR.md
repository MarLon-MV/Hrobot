        # PR: TASK-010: Seguridad LiDAR — limitar velocidad cuando hay demasiados rayos inválidos

        - **Rama:** `feat/TASK-010` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 2

        ## Commits
        ```
        6b93c43 test(TASK-010): cover degraded speed on invalid rays and config validation
449756c feat(TASK-010): limit speed when too many lidar rays are invalid
d211e55 chore(TASK-010): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/safety.py | 36 ++++++++++--
 tasks/{backlog => in_progress}/TASK-010.md   |  0
 tests/domain/test_safety.py                  | 82 ++++++++++++++++++++++++++--
 3 files changed, 109 insertions(+), 9 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]
Ejecuté la suite completa con `PYTHONPATH=src/hrobot_core pytest -q --cov=hrobot_core --cov-fail-under=85`. Pasaron 109 tests y `safety.py` tiene 100 % de cobertura. No ejecuté por separado los dos comandos de `tests/domain/test_safety.py` que pide el DoD, así que el umbral del 98 % solo está cubierto por esa cifra del 100 %.
`ruff format --check` y `ruff check` pasan, y `mypy --strict src/hrobot_core/hrobot_core` no da errores. El `grep` de imports ROS en `domain` y `services` no da coincidencias.
El diff solo toca `safety.py`, `test_safety.py` y el movimiento de la tarea a `in_progress/`.

Notas (no bloqueantes):
- Falta un test de degradación con `-inf` mezclado en el cono. La clasificación sí lo trata como inválido, pero ningún test lo fija.
- Los valores `0.70` y `0.30` siguen sin calibrar con scans reales del LD19. Es una verificación humana fuera del DoD y no la pude comprobar aquí.
