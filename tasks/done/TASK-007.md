# TASK-007: Configuración Nav2 / AMCL / slam_toolbox con validación automática

- **Rama Git:** `feat/TASK-007`
- **Depende de:** TASK-002
- **Prioridad:** P1

## Contexto y objetivo de negocio
Una inconsistencia de frames o tópicos entre Create 3, el LD19 y Nav2 (p. ej. `scan` vs `/scan`, `base_link` vs `base_footprint`) provoca fallos silenciosos de navegación. Se versionan los YAML base y un test que garantiza su coherencia y que los límites de velocidad sean conservadores (robot en un hospital).

**Aviso importante:** el Coder no tiene acceso a la instalación de Nav2 ni a la documentación en línea. Los YAML se redactan con los parámetros de Nav2 (ROS 2 Jazzy) que conozca con seguridad; **no inventar nombres de parámetros**. Todo parámetro del que no esté seguro se omite (Nav2 usará su valor por defecto) o se marca con un comentario `# TODO(verify)`. Que el test de coherencia pase **no** demuestra que Nav2 arranque con estos archivos: eso lo valida un humano (ver "Verificación humana").

## Decisiones de diseño (cerradas, no reinterpretar)
- **Controlador local: DWB** (`dwb_core::DWBLocalPlanner`), no MPPI. Es más ligero para la Raspberry Pi 5 y expone `max_vel_x`/`max_vel_theta`, que son los que valida el test.
- **Robot diferencial, sin marcha atrás:** `min_vel_x: 0.0` (el robot no retrocede a ciegas) y `min_vel_y: 0.0`, `max_vel_y: 0.0`.
- **Frames (REP-105):** `map → odom → base_link → laser`. `odom → base_link` lo publica el Create 3. Los frames por rol son:

| Rol | Valor | Dónde aparece (clave) |
|---|---|---|
| mapa global | `map` | `amcl.global_frame_id`, `bt_navigator.global_frame`, `global_costmap.global_frame`, `slam_toolbox.map_frame` |
| odometría | `odom` | `amcl.odom_frame_id`, `local_costmap.global_frame`, `slam_toolbox.odom_frame` |
| base del robot | `base_link` | `amcl.base_frame_id`, `bt_navigator.robot_base_frame`, `local_costmap.robot_base_frame`, `global_costmap.robot_base_frame`, `slam_toolbox.base_frame` |

  Nota: el **costmap local se define en `odom`** (convención de Nav2: ventana deslizante sin saltos de AMCL), no en `map`.
- **Tópicos:** scan `/scan`, odometría `/odom`.
- **Estructura YAML de Nav2:** cada nodo es una clave de primer nivel con `ros__parameters`; los costmaps van anidados (`local_costmap: {local_costmap: {ros__parameters: ...}}`, igual para `global_costmap`).

## Alcance técnico
- `src/hrobot_bringup/config/nav2_params.yaml` con secciones para: `amcl`, `bt_navigator`, `controller_server` (plugin `FollowPath` = DWB), `planner_server`, `local_costmap`, `global_costmap`, `map_server` (con `yaml_filename: ""`), `velocity_smoother` y los `lifecycle_manager` necesarios. Todos con `use_sim_time: false`.
  - `amcl`: `scan_topic: /scan`, frames según la tabla, `robot_model_type: nav2_amcl::DifferentialMotionModel`.
  - Costmaps: `robot_radius: 0.17` (Create 3), `resolution: 0.05`, capa de obstáculos con fuente `scan` (`topic: /scan`, `data_type: LaserScan`), capa de inflación con `inflation_radius` **mayor** que `robot_radius`.
  - Límites de velocidad: `controller_server.FollowPath.max_vel_x <= 0.3` (m/s), `max_vel_theta <= 1.0` (rad/s), `min_vel_x == 0.0`; y `velocity_smoother.max_velocity` con el mismo tope lineal `[0.3, 0.0, 1.0]`.
- `src/hrobot_bringup/config/slam_toolbox_params.yaml`: `slam_toolbox: ros__parameters:` con `mode: mapping`, `scan_topic: /scan`, frames según la tabla, `do_loop_closing: true`, `resolution: 0.05`, `max_laser_range` no mayor que 12.0 (alcance nominal del LD19; **verificar con el datasheet**).
- `tests/config/test_nav_configs.py`: carga ambos YAML con `yaml.safe_load` y contiene una función `validate(nav2, slam) -> list[str]` que devuelve la lista de incoherencias (vacía si todo es correcto). Los tests la invocan con los YAML reales y con copias modificadas en memoria.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/config/nav2_params.yaml`
- `src/hrobot_bringup/config/slam_toolbox_params.yaml`
- `tests/config/__init__.py`
- `tests/config/test_nav_configs.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config/test_nav_configs.py` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] El test verifica sobre los YAML reales (cada punto con su propia aserción): ambos archivos parsean y tienen la estructura `ros__parameters`; todos los frames de la tabla de roles coinciden con su valor (`map`, `odom`, `base_link`), incluyendo que `local_costmap.global_frame == "odom"`; el tópico de scan es `/scan` en AMCL, ambos costmaps y slam_toolbox; `robot_radius` en (0.10, 0.30) y `inflation_radius > robot_radius`; `max_vel_x <= 0.3`, `max_vel_theta <= 1.0`, `min_vel_x == 0.0`, y `velocity_smoother.max_velocity[0] <= 0.3`; `do_loop_closing` es `true`; la resolución coincide en slam_toolbox y en ambos costmaps; ningún `use_sim_time` es `true`
- [ ] Tests negativos (un caso por cada uno, sobre copias en memoria): frame inconsistente (`base_footprint` en un solo sitio) ⇒ `validate` lo reporta; tópico de scan distinto ⇒ lo reporta; `max_vel_x = 0.5` ⇒ lo reporta; `min_vel_x = -0.1` ⇒ lo reporta; `local_costmap.global_frame = "map"` ⇒ lo reporta; `inflation_radius < robot_radius` ⇒ lo reporta
- [ ] Todo parámetro del que el Coder no esté seguro lleva un comentario `# TODO(verify)` o se omite; el archivo no incluye valores inventados

## Verificación humana (fuera del DoD)
- En un equipo con ROS 2 Jazzy y Nav2 instalados: `ros2 launch nav2_bringup bringup_launch.py params_file:=<ruta> map:=<mapa.yaml>` debe arrancar sin errores de parámetros ni nodos que no activen (lifecycle). Comparar con el `nav2_params.yaml` por defecto de `nav2_bringup` y completar lo que falte.
- Comprobar el nombre del tópico y el tipo de `/cmd_vel` y `/odom` del Create 3 con `ros2 topic list -t`, y si el Create 3 usa un espacio de nombres.
- Calibrar `robot_radius`, `inflation_radius` y los límites de velocidad con el robot real.
- Probar con el robot en un entorno controlado a baja velocidad antes de operar en pasillos.

## Fuera de alcance
Launch files (TASK-009), mapas, remapeo de `/cmd_vel` (el supervisor de seguridad será el único que publique `/cmd_vel`; ver TASK-009), calibración con el robot real, ajuste fino de controladores, Collision Monitor de Nav2.
