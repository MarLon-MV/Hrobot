# TASK-007: Configuración Nav2 / AMCL / slam_toolbox con validación automática

- **Rama Git:** `feat/TASK-007`
- **Depende de:** TASK-002
- **Prioridad:** P1

## Contexto y objetivo de negocio
Una inconsistencia de frames o tópicos entre Create 3, el LD19 y Nav2 (p. ej. `scan` vs `/scan`, `base_link` vs `base_footprint`) provoca fallos silenciosos de navegación. Se versionan los YAML base y un test que garantiza su coherencia.

## Alcance técnico
- `src/hrobot_bringup/config/nav2_params.yaml`: parámetros de `amcl`, `bt_navigator`, `controller_server` (DWB o MPPI), `planner_server`, `local_costmap`, `global_costmap` con `robot_radius` configurado (Create 3 ≈ 0.17 m), `scan_topic: /scan`, `odom_topic: /odom`, `base_frame_id: base_link`, `odom_frame_id: odom`, `global_frame: map`, y velocidades máximas conservadoras (`max_vel_x <= 0.3`).
- `src/hrobot_bringup/config/slam_toolbox_params.yaml`: modo `mapping`, `scan_topic: /scan`, mismos frames, `do_loop_closing: true`, `resolution: 0.05`.
- `tests/config/test_nav_configs.py`: carga ambos YAML y valida coherencia.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/config/nav2_params.yaml`
- `src/hrobot_bringup/config/slam_toolbox_params.yaml`
- `tests/config/__init__.py`
- `tests/config/test_nav_configs.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config/test_nav_configs.py` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] El test verifica: los YAML parsean; todos los `*_frame` coinciden (`map`, `odom`, `base_link`); el tópico de scan es `/scan` en AMCL, costmaps y slam_toolbox; `robot_radius` en (0.10, 0.30); velocidad lineal máxima <= 0.3 m/s; `do_loop_closing` es `true`; la resolución del mapa coincide en SLAM y costmaps
- [ ] Un test negativo muestra que el validador detecta un frame inconsistente (usando un dict modificado en memoria)

## Fuera de alcance
Launch files, mapas, calibración con el robot real, ajuste fino (tuning) de controladores.
