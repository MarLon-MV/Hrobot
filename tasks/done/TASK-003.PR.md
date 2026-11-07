        # PR: TASK-003: Dominio — geometría 2D (Pose2D, yaw ↔ cuaternión, normalización de ángulos)

        - **Rama:** `feat/TASK-003` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        a2e15fd feat(TASK-003): add 2D geometry domain module with tests
d677b91 chore(TASK-003): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_core/hrobot_core/domain/geometry.py | 106 ++++++++++++++++++
 tasks/{backlog => in_progress}/TASK-003.md     |   0
 tests/domain/__init__.py                       |   0
 tests/domain/test_geometry.py                  | 143 +++++++++++++++++++++++++
 4 files changed, 249 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté pytest (45 pasan, cobertura 100 %, incluido el comando exacto del DoD con `--cov-fail-under=95`), `ruff format --check`, `ruff check` y `mypy --strict`. Todos terminan sin errores. El grep de imports de ROS en `domain` y `services` no da coincidencias. El diff solo toca los archivos permitidos, más el movimiento de la tarea hecho por el orquestador.

Notas (no bloqueantes):
- `quaternion_to_yaw` acepta cuaterniones no unitarios sin normalizarlos. Con componentes finitos muy grandes (~1e200) el cuadrado desborda a `inf` y devuelve un yaw silenciosamente incorrecto. Con componentes muy pequeños (~1e-200) la norma al cuadrado da 0 y rechaza un cuaternión válido. Conviene validar o normalizar con `math.hypot` si algún adaptador pasa cuaterniones sin normalizar.
- Los tests no cubren esos cuaterniones no unitarios ni los valores extremos.
