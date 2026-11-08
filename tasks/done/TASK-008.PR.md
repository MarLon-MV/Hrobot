        # PR: TASK-008: Configuración de RViz2 para supervisión y calibración

        - **Rama:** `feat/TASK-008` → `main` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** 1

        ## Commits
        ```
        14d2c9a test(TASK-008): validate rviz config against nav2 and slam params
b18450a feat(TASK-008): add rviz2 nav supervision config
6763bec chore(TASK-008): move task to in_progress
        ```

        ## Cambios
        ```
        src/hrobot_bringup/rviz/hrobot_nav.rviz    | 111 ++++++++++++++
 tasks/{backlog => in_progress}/TASK-008.md |   0
 tests/config/test_rviz_config.py           | 236 +++++++++++++++++++++++++++++
 3 files changed, 347 insertions(+)
        ```

        ## Veredicto del Reviewer
        Veredicto: [APROBADO]

Ejecuté `pytest -q tests/config/test_rviz_config.py` (12 passed), la suite completa (463 passed), `ruff format --check` y `ruff check` (limpios) y `mypy --strict` (sin errores). El diff solo toca los archivos permitidos más el movimiento de la tarea hecho por el orquestador. Los commits siguen Conventional Commits, y el `grep` de capas no aplica porque no hay cambios en `domain` ni `services`.

Notas (no bloqueantes):
- No pude verificar que RViz2 cargue el `.rviz`; hace falta ROS 2 y no está instalado aquí. El docstring del test ya lo declara.
- Según mi conocimiento de Nav2 (Humble en adelante), AMCL publica `/particle_cloud` como `nav2_msgs/ParticleCloud`, y su RViz usa `nav2_rviz_plugins/ParticleCloud`. Un display `PoseArray` en ese tópico podría no dibujar las partículas. No lo comprobé contra una instalación de Nav2. El DoD pide explícitamente `PoseArray`, así que el Coder cumplió la tarea. Conviene verificarlo con hardware o ROS y, si se confirma, corregir la especificación en una tarea posterior.
- Falta un test negativo para la incoherencia de `Fixed Frame` frente a `amcl.global_frame_id` y `global_costmap.global_frame`. Solo se prueba que `Fixed Frame` sea `odom`. El validador sí cubre ese caso.
