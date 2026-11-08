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
- `tests/config/__init__.py` (ya existe; no hace falta modificarlo)
- `tests/config/test_rviz_config.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config/test_rviz_config.py` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] El test verifica: el `.rviz` parsea como YAML; `Fixed Frame` es `map` y coincide con `amcl.ros__parameters.global_frame_id` y con `global_costmap.global_costmap.ros__parameters.global_frame` de `nav2_params.yaml` (NO con el `global_frame` del local_costmap, que es `odom` y es correcto); existe un display `TF`, un `LaserScan` con tópico `/scan` (igual a `amcl.ros__parameters.scan_topic` y a `slam_toolbox.ros__parameters.scan_topic`), un `PoseArray` en `/particle_cloud`, un `Path` en `/plan`, dos displays `Map` para los costmaps; las tools `SetInitialPose` y `SetGoal` están presentes
- [ ] Test negativo: si se cambia el tópico de scan en memoria, el validador detecta la incoherencia

## Notas
- Los displays de costmap deben usar los tópicos `/local_costmap/costmap` y `/global_costmap/costmap`; el test los comprueba por tópico, no por nombre de display.
- Usar `yaml.safe_load` (PyYAML ya está en requirements-dev). Las claves de nav2 se leen con rutas explícitas; si faltan, el test falla con mensaje claro.

## Fuera de alcance
Lanzar RViz (TASK-009), paneles personalizados, plugins propios, visualización de la misión de entrega.
