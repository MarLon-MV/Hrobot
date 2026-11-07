        # PR: TASK-017: Seguridad LiDAR — limitar velocidad ante tramos contiguos de rayos inválidos

        - **Rama:** `feat/TASK-017` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 2

        ## Commits
        ```
        fa8d137 test(TASK-017): cover contiguous invalid run rule
5885f3c feat(TASK-017): degrade speed on contiguous invalid ray runs
e5047fb chore(TASK-017): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/safety.py |  53 ++++++++-
 tasks/{backlog => in_progress}/TASK-017.md   |   0
 tests/domain/test_safety.py                  | 160 ++++++++++++++++++++++++++-
 3 files changed, 207 insertions(+), 6 deletions(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté estos comandos y todos pasaron:
- `pytest -q tests/domain/test_safety.py`: 93 passed.
- El mismo archivo con `--cov=hrobot_core.domain.safety --cov-fail-under=98`: cobertura 100 %.
- `ruff format --check src tests`, `ruff check src tests` y `mypy --strict src/hrobot_core/hrobot_core`: sin errores.
- El grep de importaciones ROS sobre `src/hrobot_core/hrobot_core`: sin coincidencias.
- El diff solo toca `safety.py`, `test_safety.py` y el movimiento de la tarea hecho por el orquestador.

Notas (no bloqueantes):
- No ejecuté la suite completa del repo, solo `test_safety.py`. Los cambios de código están limitados a `safety.py` y su test.
- Cambiaron las aserciones de `test_valid_ratio_thresholds` (29 y 30 NaN pasan de 1.0 a 0.30). Está justificado con un comentario `# TASK-017`, y `test_valid_ratio_thresholds_scattered` conserva el umbral de la fracción.
- El umbral de 8° sigue sin calibrar con scans reales y los efectos operativos de la tarea no se pueden validar en simulación. Quedan en la "Verificación humana" de la spec.
