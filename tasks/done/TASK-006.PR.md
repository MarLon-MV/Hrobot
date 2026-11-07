        # PR: TASK-006: Adapter — parser de tramas del LiDAR LD19

        - **Rama:** `feat/TASK-006` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        e762265 test(TASK-006): add LD19 parser tests
fe35d16 feat(TASK-006): add LD19 frame parser
a45004e chore(TASK-006): move task to in_progress
        ```

        ## Cambios
        ```
        .../hrobot_core/adapters/ld19_parser.py            | 134 ++++++++++++++++++
 tasks/{backlog => in_progress}/TASK-006.md         |   0
 tests/adapters/__init__.py                         |   0
 tests/adapters/test_ld19_parser.py                 | 151 +++++++++++++++++++++
 4 files changed, 285 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté pytest del módulo (14 pasan, cobertura 100% con umbral 95%) y la suite completa (190 pasan, cobertura 100% con umbral 85%). `ruff format --check`, `ruff check` y `mypy --strict` pasan sin errores. El grep de imports prohibidos (`rclpy`, `serial`, mensajes ROS) no da coincidencias en `src/hrobot_core/hrobot_core`. El diff solo toca los 3 archivos permitidos, más el movimiento de la tarea que hace el orquestador. `git diff --stat` lo confirma.

Notas (no bloqueantes):
- `_decode_frame` no valida que `start_angle` y `end_angle` estén en [0, 36000). Con CRC válido y un valor mayor, el ángulo se reduce con módulo sin avisar. El spec no lo exige.
- `test_valid_frame_inside_corrupt_frame` corrompe `bad[20]`, pero solo usa `bad[:10]`, así que esa corrupción no interviene. El caso sigue siendo válido (trama truncada con header falso seguida de una trama real) y `crc_errors == 1` se cumple. El test no demuestra lo que su nombre sugiere.
- `test_no_forbidden_imports` busca el texto `import rclpy`/`import serial`. No detectaría `from rclpy import ...`.
- Las pruebas con hardware real (CRC en vivo, scan sin espejar en RViz) quedan para la verificación humana del spec. No las validé ni las doy por validadas.
