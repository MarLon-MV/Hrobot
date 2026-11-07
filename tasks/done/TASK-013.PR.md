        # PR: TASK-013: Constructor de scans de rejilla fija a partir de las tramas del LD19

        - **Rama:** `feat/TASK-013` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        de1a47f feat(TASK-013): add fixed-grid scan builder for ld19 packets
0987df9 chore(TASK-013): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/adapters/scan_builder.py           | 149 ++++++++++++++
 tasks/{backlog => in_progress}/TASK-013.md         |   0
 tests/adapters/test_scan_builder.py                | 214 +++++++++++++++++++++
 3 files changed, 363 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté la suite completa con cobertura (284 passed, 100 %), el DoD de `test_scan_builder.py` con `--cov-fail-under=95` (29 passed, `scan_builder.py` 100 %), `ruff format --check`, `ruff check` y `mypy --strict` (todos limpios). El `grep` de capas sobre `domain` y `services` no dio coincidencias, y el diff solo toca los dos archivos permitidos más el movimiento de la tarea.

Notas (no bloqueantes):
- `ScanBuilderConfig` no valida el tipo de `range_min`, `range_max` ni `max_points_per_revolution`. Un `str` en un rango lanzaría `TypeError` en lugar de `ValueError`, y un `float` en `max_points_per_revolution` se aceptaría.
- `test_end_to_end_anti_mirror` comprueba `any(abs(i-270)<=1)` sobre los bins cercanos, no `ranges[270]` directamente. El espejado queda cubierto igualmente por `ranges[90] == 3.0`.
- `ScanBuilder` mantiene estado mutable sin protección. El adaptador ROS futuro debe llamarlo desde un único callback.
