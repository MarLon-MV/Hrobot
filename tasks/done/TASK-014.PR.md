        # PR: TASK-014: Nodo ROS 2 del LiDAR LD19 (lectura serie → `/scan`)

        - **Rama:** `feat/TASK-014` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        962a0fd feat(TASK-014): add ld19 ros 2 node publishing /scan
e87600c chore(TASK-014): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/adapters/ld19_node.py | 193 ++++++++++++++++
 src/hrobot_core/package.xml                       |   1 +
 src/hrobot_core/setup.py                          |   6 +-
 tasks/{backlog => in_progress}/TASK-014.md        |   0
 tests/adapters/test_ld19_node.py                  | 263 ++++++++++++++++++++++
 5 files changed, 462 insertions(+), 1 deletion(-)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté estos comandos y todos terminaron con código 0:
- La suite completa con cobertura: 422 tests pasan, 100 % de cobertura total y 100 % en `ld19_node.py`.
- Los comandos del DoD de la tarea: `pytest tests/adapters/test_ld19_node.py --cov-fail-under=90`, `ruff format --check`, `ruff check`, `mypy --strict` y los dos `grep` de `setup.py` y `package.xml`.
- El `grep` de capas sobre `domain` y `services`: sin coincidencias.

El diff solo toca los archivos permitidos más el movimiento de la tarea a `in_progress`, y no publica en `/cmd_vel` ni en `.msg/.srv/.action`.

Notas (no bloqueantes):
- Si una lectura devuelve varias vueltas completas a la vez, todas se sellan con el mismo `now_ns` (`ld19_node.py:_process`). Es poco probable con el flujo normal del LD19, pero el estampado sería impreciso tras un retraso.
- `reconnect_period_s`, `read_period_s` o `stats_period_s` ≤ 0 no se validan y fallarían en `create_timer`.
- Pasar un entero a un parámetro float por CLI (`-p range_max:=12`) puede dar `InvalidParameterTypeException` en `rclpy` real, porque el valor por defecto se declara como float. Los fakes no lo detectan.
- Ningún test comprueba que `serial.Serial` se abre con `timeout=0`.
- Verificación humana pendiente en la RPi5 con el LD19: frecuencia de `/scan`, orientación sin espejado y API real de `rclpy` y `pyserial`.
