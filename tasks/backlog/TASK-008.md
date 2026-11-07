# TASK-008: Configuración de RViz2 para supervisión y calibración

- **Rama Git:** `feat/TASK-008`
- **Depende de:** TASK-007
- **Prioridad:** P1

## Contexto y objetivo de negocio
Durante el mapeo, la calibración de AMCL y la puesta a punto de Nav2, el equipo necesita ver tf2, el scan del LD19, el mapa, los costmaps, las partículas de AMCL y el plan global/local en una vista preconfigurada y reproducible. Una configuración `.rviz` versionada evita montarla a mano en cada sesión y asegura que los tópicos y frames coincidan con la configuración de Nav2.

## Alcance técnico
Crear `src/hrobot_bringup/rviz/hrobot_nav.rviz` (formato YAML de RViz2) con:
- `Fixed Frame: map`.
- Displays: `TF`; `LaserScan` en `/scan` (QoS `Best Effort`); `Map` en `/map`; `Map` para `/local_costmap/costmap` y `/global_costmap/costmap`; `PoseArray` en `/particle_cloud` (partículas AMCL); `Path` en `/plan` y `/local_plan`; `Pose` en `/goal_pose` (opcional).
- Tools: `MoveCamera`, `Select`, `SetInitialPose` (publica en `/initialpose`), `SetGoal` (publica en `/goal_pose`).
- Vista `TopDownOrtho` centrada en el robot.

Test de coherencia en `tests/config/test_rviz_config.py`: carga el `.rviz` y el `nav2_params.yaml`/`slam_toolbox_params.yaml` y valida contra ellos.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/rviz/hrobot_nav.rviz`
- `tests/config/__init__.py`
- `tests/config/test_rviz_config.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config/test_rviz_config.py` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] El test verifica: el `.rviz` parsea como YAML; `Fixed Frame` es `map` y coincide con `global_frame` de Nav2; existe un display `TF`, un `LaserScan` con tópico `/scan` (igual al `scan_topic` de la config), un `PoseArray` en `/particle_cloud`, un `Path` en `/plan`, dos displays `Map` para los costmaps; las tools `SetInitialPose` y `SetGoal` están presentes
- [ ] Test negativo: si se cambia el tópico de scan en memoria, el validador detecta la incoherencia

## Fuera de alcance
Lanzar RViz (TASK-009), paneles personalizados, plugins propios, visualización de la misión de entrega.
