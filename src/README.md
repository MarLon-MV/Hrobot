# Workspace `src/`

Workspace ROS 2 Jazzy (C++17) para que un robot **iRobot Create 3 + Raspberry Pi 5 + LiDAR LD19**
explore y mapee entornos desconocidos de forma autónoma (slam_toolbox + Nav2 + exploración por fronteras).

> Estado: el código se desarrolló sin ROS 2 instalado. Los algoritmos puros tienen tests ejecutados;
> los nodos y los launch solo se validaron con stubs. **El primer `colcon build` y arranque reales
> están pendientes** (ver [Pendientes de la primera ejecución](#pendientes-de-la-primera-ejecución)).

## Paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| [`create3_bot_interfaces`](create3_bot_interfaces) | ament_cmake | Acción `ExploreMap`, mensaje `FrontierInfo`, servicio `SaveExplorationMap`. Sin código. |
| [`create3_bot_description`](create3_bot_description) | ament_cmake | URDF/xacro (`base_link` radio 0.171 m → `laser_frame`) y config de RViz. |
| [`create3_autonomous_explorer`](create3_autonomous_explorer) | ament_cmake | Lib pura `${PROJECT_NAME}_lib`, `explorer_node` y `map_manager_node`. |
| [`create3_bot_bringup`](create3_bot_bringup) | ament_cmake | Solo `config/` y `launch/`. |

Cada paquete tiene su propio README con el detalle (explorer y bringup).

### Dependencias entre paquetes (sin ciclos)

```text
create3_bot_interfaces ◄── create3_autonomous_explorer ◄──┐
                                                           ├── create3_bot_bringup
create3_bot_description ◄──────────────────────────────────┘
```

Futuros paquetes (`mission_manager`, `web_bridge`...) deben depender de `create3_bot_interfaces`,
nunca del explorer.

## Arquitectura

```text
  LD19 ──/scan──► slam_toolbox ──/map, TF map→odom──┐
  Create 3 ──/odom, TF odom→base_link──────────────┤
                                                    ▼
  ExploreMap (acción) ─► explorer_node ──/navigate_to_pose──► Nav2 ──/cmd_vel──► Create 3
                           │  ▲                                  ▲
                           │  └── /map (transient_local) ────────┘
                           ├──► /frontiers (MarkerArray) ──► RViz
                           └──► save_exploration_map ─► map_manager_node
                                                          ├─► /map_saver/save_map   (nav2 map_saver: .yaml/.pgm)
                                                          └─► /slam_toolbox/serialize_map (.posegraph/.data)
```

- **`FrontierDetector`** (BFS 8-conectado) y **`GoalEvaluator`** (puntuación por tamaño, distancia y
  ángulo + lista negra) son lógica pura: no incluyen `rclcpp` ni mensajes ROS y operan solo con
  `types.hpp`. La conversión ROS ↔ tipos puros ocurre en `explorer_node`.
- **`explorer_node`**: action server `explore_map`, suscripción a `/map` con QoS `transient_local`
  protegida por `std::mutex`, pose por TF2 `map→base_link`, cliente de `navigate_to_pose`, recuperación
  por rotación si se atasca y publicación de `/frontiers`.
- **`map_manager_node`**: guarda el mapa y el posegraph (permite ampliarlo después).

## Preparar la Raspberry Pi

Requisito: Ubuntu con ROS 2 Jazzy ya instalado (`/opt/ros/jazzy`). El script no instala ROS.

```bash
git clone https://github.com/MarLon-MV/Hrobot.git && cd Hrobot
scripts/setup_pi.sh            # herramientas, driver LD19, permisos, rosdep y colcon build
source install/setup.bash
```

Opciones: `--skip-udev`, `--skip-build`; variables `LDLIDAR_REF` (rama/tag/commit del driver,
por defecto `master`), `JOBS` (workers de colcon, por defecto 2) y `LDLIDAR_REPO`. Es idempotente.
Más detalle en [`scripts/setup_pi.sh`](../scripts/setup_pi.sh).

Qué hace el script:

1. Instala `git`, `colcon` y `rosdep`, e inicializa `rosdep`.
2. Clona o actualiza el driver oficial del LD19 en `src/ldlidar_stl_ros2` (ignorado por git).
3. Añade tu usuario al grupo `dialout` e instala una regla udev (CP2102, `10c4:ea60`) que crea
   el symlink `/dev/ldlidar`. Tras la primera vez, cierra sesión y vuelve a entrar.
4. Ejecuta `rosdep install` (sin `-r`: se detiene si falta una dependencia) y `colcon build --symlink-install`.

## Uso rápido

```bash
# Compilar a mano (si no usas el script), en un entorno con ROS 2 Jazzy
cd <raíz del repo>
colcon build --packages-select create3_bot_interfaces create3_bot_description \
  create3_autonomous_explorer create3_bot_bringup
source install/setup.bash

# Tests
colcon test --packages-select create3_autonomous_explorer && colcon test-result --verbose

# Lanzar todo (slam + Nav2 + explorer + map_manager + RViz)
ros2 launch create3_bot_bringup master_explore.launch.py lidar_port:=/dev/ldlidar
# Otros argumentos: lidar_baudrate (230400), rviz:=true|false, use_sim_time, maps_dir, map, slam_lifecycle

# Iniciar la exploración
ros2 action send_goal /explore_map create3_bot_interfaces/action/ExploreMap \
  "{start_exploration: true, max_duration_sec: 0.0}" --feedback

# Guardar el mapa manualmente (el explorer ya guarda al terminar)
ros2 service call /save_exploration_map create3_bot_interfaces/srv/SaveExplorationMap "{map_name: sala1}"

# Ampliar un mapa guardado
ros2 launch create3_bot_bringup master_explore.launch.py map:=sala1
```

Los mapas se guardan en `~/.create3_maps` (`maps_dir`). Más detalle en los README de
[explorer](create3_autonomous_explorer/README.md) y [bringup](create3_bot_bringup/README.md).

## Dependencias externas

- ROS 2 Jazzy, Nav2 (`nav2_bringup`, `nav2_map_server`...), `slam_toolbox`, `robot_state_publisher`, `xacro`, `rviz2`.
- Driver del LD19: **`ldlidar_stl_ros2`** (repo oficial de LDRobot, `ldrobotSensorTeam/ldlidar_stl_ros2`,
  rama `master`; soporta LD06/LD19/STL-27L). No está en rosdep: `scripts/setup_pi.sh` lo clona en
  `src/ldlidar_stl_ros2`. El repo declara soporte de ROS 2 Foxy y posteriores; Jazzy no figura
  explícitamente.
- Create 3 accesible por red (micro-ROS) con el mismo `ROS_DOMAIN_ID` y RMW que la RPi5.

## Convenciones

- C++17, `-Wall -Wextra -Wpedantic` sin warnings, TDD con GoogleTest. Reglas completas en
  [`.claude/.agents/coder.md`](../.claude/.agents/coder.md).
- Nombres: `PascalCase` (tipos), `snake_case` (funciones y variables), `nombre_miembro_` (atributos privados).
- Parámetros `double` en YAML siempre con decimales (`1.0`, no `1`).

## Pendientes de la primera ejecución

1. Compilar con ROS 2 real: las firmas de `rclcpp`/`rclcpp_action` y los campos de
   `nav2_msgs/SaveMap` y `slam_toolbox/SerializePoseGraph` se escribieron de memoria.
2. Arrancar Nav2 y revisar avisos de parámetros desconocidos en `nav2_params.yaml`.
3. Confirmar si `async_slam_toolbox_node` es un nodo de ciclo de vida (si no, `slam_lifecycle:=false`).
4. Driver LD19: sentido de giro (`laser_scan_dir`) y, sobre todo, la **QoS de `/scan`**: si el driver publica
   *best effort* y slam_toolbox o el costmap de Nav2 piden *reliable*, no recibirán datos
   (`ros2 topic info /scan -v`). Puerto (`/dev/ldlidar`) y baudrate (230400) ya están configurados.
   `scripts/setup_pi.sh` y su regla udev no se han probado en la Pi.
5. Medir en el robot la posición real del LiDAR (`laser_x/y/z` en el xacro; `laser_z` = 0.12 m es un valor supuesto).
6. Sustituir el maintainer `dev@example.com` de los `package.xml`.
