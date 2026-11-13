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
| [`create3_autonomous_explorer`](create3_autonomous_explorer) | ament_cmake | Lib pura `${PROJECT_NAME}_lib`, `explorer_node`, `map_manager_node` y `odom_relay_node`. |
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
  (nombres relativos al namespace del robot: con el defecto `r2`, `scan` es `/r2/scan`, etc.)
  LD19 ──scan──► slam_toolbox ──map, TF map→odom──┐
  Create 3 ──odom, TF odom→base_link─────────────┤
                                                  ▼
  ExploreMap (acción) ─► explorer_node ──navigate_to_pose──► Nav2 ──cmd_vel──► Create 3
                           │  ▲                                ▲
                           │  └── map (transient_local) ───────┘
                           ├──► frontiers (MarkerArray) ──► RViz
                           └──► save_exploration_map ─► map_manager_node
                                                          ├─► map_saver/save_map   (nav2 map_saver: .yaml/.pgm)
                                                          └─► slam_toolbox/serialize_map (.posegraph/.data)
```

### Namespace del robot (`robot_namespace`)

El Create 3 vive en el namespace `/r2` (publica `/r2/odom`, `/r2/tf`, `/r2/tf_static`; escucha
`/r2/cmd_vel`) y **todo el stack cuelga del mismo namespace**. Se elige con el argumento
`robot_namespace:=<nombre>` (los tres launch lo aceptan); por defecto es la variable de entorno
`CREATE3_ROBOT_NAMESPACE` si existe y, si no, `r2`. Vacio (`robot_namespace:=''`) = sin namespace
(simulacion). El namespace se aplica una sola vez: `hardware_base` y `navigation` lo aplican a sus nodos y
`master_explore` solo a los suyos (slam_toolbox, explorer, map_manager, rviz2), nunca a los `include`.
TF se remapea a `<ns>/tf` y `<ns>/tf_static`; los `frame_id` (`base_link`, `odom`, `map`, `laser_frame`) no
llevan prefijo. Los YAML se reescriben al arrancar con `nav2_common` (`RewrittenYaml`, `root_key=<ns>`).

| Elemento | `r2` (por defecto) | Vacio |
|---|---|---|
| Escaneo LD19 | `/r2/scan` | `/scan` |
| Mapa / fronteras | `/r2/map`, `/r2/frontiers` | `/map`, `/frontiers` |
| Odometria / velocidad (Create 3) | `/r2/odom` (BEST_EFFORT), `/r2/cmd_vel` | `/odom`, `/cmd_vel` |
| Odometria para Nav2 (rele RELIABLE) | `/r2/odom_reliable` | `/odom_reliable` |
| TF | `/r2/tf`, `/r2/tf_static` | `/tf`, `/tf_static` |
| Accion de exploracion | `/r2/explore_map` | `/explore_map` |
| Guardar mapa | `/r2/save_exploration_map` | `/save_exploration_map` |
| Nav2 | `/r2/navigate_to_pose`, `/r2/map_saver/save_map` | `/navigate_to_pose`, `/map_saver/save_map` |
| slam_toolbox | `/r2/slam_toolbox/serialize_map` | `/slam_toolbox/serialize_map` |
| Nodos | `/r2/explorer_node`, `/r2/odom_relay_node`, `/r2/controller_server`... | `/explorer_node`, `/controller_server`... |

Cambiar de robot (otro nombre, p. ej. `r3`): `scripts/set_robot.sh r3` (guarda
`CREATE3_ROBOT_NAMESPACE` en `~/.config/create3_bot/robot.env` y lo carga desde `~/.bashrc`; `--show`,
`--check` y `--unset` disponibles) o, solo para una ejecucion, `robot_namespace:=r3`. **El namespace del
propio Create 3 se cambia en su interfaz web (Application > ROS 2); el script no lo modifica.**

- **`FrontierDetector`** (BFS 8-conectado) y **`GoalEvaluator`** (puntuación por tamaño, distancia y
  ángulo + lista negra) son lógica pura: no incluyen `rclcpp` ni mensajes ROS y operan solo con
  `types.hpp`. La conversión ROS ↔ tipos puros ocurre en `explorer_node`.
- **`explorer_node`**: action server `explore_map`, suscripción a `map` con QoS `transient_local`
  protegida por `std::mutex`, pose por TF2 `map→base_link`, cliente de `navigate_to_pose`, recuperación
  por rotación si se atasca y publicación de `frontiers`.
- **`map_manager_node`**: guarda el mapa y el posegraph (permite ampliarlo después).
- **`odom_relay_node`**: republica `/r2/odom` (BEST_EFFORT, Create 3) como `/r2/odom_reliable` (RELIABLE) porque Nav2 suscribe RELIABLE y no conectaria con el Create 3. Comprobar: `ros2 topic info /r2/odom_reliable -v` (RELIABLE, publicador `odom_relay_node`, suscriptores de Nav2).

## Preparar la Raspberry Pi

Requisito: Ubuntu con ROS 2 Jazzy ya instalado (`/opt/ros/jazzy`). El script no instala ROS.

```bash
git clone https://github.com/MarLon-MV/Hrobot.git && cd Hrobot
scripts/setup_pi.sh            # herramientas, driver LD19, permisos, rosdep y colcon build
source install/setup.bash
```

Opciones: `--skip-udev` (omite grupo `dialout` y regla udev; no habrá `/dev/ldlidar`, usa `lidar_port:=/dev/ttyUSB0`),
`--skip-build`, `--skip-net` (no toca `ROS_DOMAIN_ID`/RMW), `--domain-id N`, `--rmw <nombre>`; variables `LDLIDAR_REF` (rama/tag/commit del driver,
por defecto `master`), `JOBS` (workers de colcon, por defecto 2) y `LDLIDAR_REPO`. Es idempotente.
Más detalle en [`scripts/setup_pi.sh`](../scripts/setup_pi.sh).

Qué hace el script:

1. Instala `git`, `colcon` y `rosdep`, e inicializa `rosdep`.
2. Driver del LD19: si ya hay un `ldlidar_stl_ros2` en el entorno (otro workspace compilado y cargado),
   lo **reutiliza** y excluye la copia de `src/` del build (`--force-driver` para compilarlo aquí). Si no,
   clona el repo oficial en `src/ldlidar_stl_ros2` (ignorado por git) y lo compila con
   `-include pthread.h`, porque el driver no incluye esa cabecera y falla con el GCC de Ubuntu 24.04.
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
# Otros argumentos: robot_namespace:=r2 (defecto: $CREATE3_ROBOT_NAMESPACE o r2; vacio = sin namespace),
# lidar_baudrate (230400), rviz:=true|false, use_sim_time, maps_dir, map, slam_lifecycle

# Elegir el robot una vez para todas las terminales (no cambia el namespace del Create 3)
scripts/set_robot.sh r3 && scripts/set_robot.sh --check   # tambien --show y --unset

# Iniciar la exploración
ros2 action send_goal /r2/explore_map create3_bot_interfaces/action/ExploreMap \
  "{start_exploration: true, max_duration_sec: 0.0}" --feedback

# Guardar el mapa manualmente (el explorer ya guarda al terminar)
ros2 service call /r2/save_exploration_map create3_bot_interfaces/srv/SaveExplorationMap "{map_name: sala1}"

# Ampliar un mapa guardado
ros2 launch create3_bot_bringup master_explore.launch.py map:=sala1
```

Los mapas se guardan en `~/.create3_maps` (`maps_dir`). Más detalle en los README de
[explorer](create3_autonomous_explorer/README.md) y [bringup](create3_bot_bringup/README.md).

## RViz en otro equipo (portátil Ubuntu)

Lanza la Pi con `rviz:=false` y abre RViz en un PC con ROS 2 Jazzy y `rviz2`, en la misma red:

```bash
scripts/rviz_remote.sh                 # namespace por defecto: $CREATE3_ROBOT_NAMESPACE o r2
scripts/rviz_remote.sh -n r3           # otro robot
scripts/rviz_remote.sh --dry-run       # muestra la config y el comando sin abrir RViz
```

El script sustituye el marcador `/<robot_namespace>` del `.rviz` por el namespace y remapea
`/tf` y `/tf_static` a `/<ns>/tf` y `/<ns>/tf_static`. Requisitos en el PC: mismo `ROS_DOMAIN_ID` y RMW
que la Pi, **hora sincronizada con la Pi** (un desfase de unos segundos rompe el TF en RViz) y red sin
bloqueo de multicast.

## Dependencias externas

- ROS 2 Jazzy, Nav2 (`nav2_bringup`, `nav2_map_server`...), `slam_toolbox`, `robot_state_publisher`, `xacro`, `rviz2`.
- Driver del LD19: **`ldlidar_stl_ros2`** (repo oficial de LDRobot, `ldrobotSensorTeam/ldlidar_stl_ros2`,
  rama `master`; soporta LD06/LD19/STL-27L). No está en rosdep: `scripts/setup_pi.sh` lo clona en
  `src/ldlidar_stl_ros2`. El repo declara soporte de ROS 2 Foxy y posteriores; Jazzy no figura
  explícitamente.
- `nav2_common` (en `nav2_bringup`; `RewrittenYaml`/`ReplaceString`).
- Create 3 accesible por red (micro-ROS) con el mismo `ROS_DOMAIN_ID` y RMW que la RPi5.

## Convenciones

- C++17, `-Wall -Wextra -Wpedantic` sin warnings, TDD con GoogleTest. Reglas completas en
  [`.claude/.agents/coder.md`](../.claude/.agents/coder.md).
- Nombres: `PascalCase` (tipos), `snake_case` (funciones y variables), `nombre_miembro_` (atributos privados).
- Parámetros `double` en YAML siempre con decimales (`1.0`, no `1`).

## Pendientes de la primera ejecución

0. Con namespace (T-08): comprobar que `/r2/tf` lleva `odom→base_link` del Create 3, que el `TransformListener`
   de `explorer_node` (remap absoluto `/tf:=/r2/tf`) ve `map→base_link` y que `RewrittenYaml` carga los YAML sin avisos.
1. Compilar con ROS 2 real: las firmas de `rclcpp`/`rclcpp_action` y los campos de
   `nav2_msgs/SaveMap` y `slam_toolbox/SerializePoseGraph` se escribieron de memoria.
2. Arrancar Nav2 y revisar avisos de parámetros desconocidos en `nav2_params.yaml`.
3. Confirmar si `async_slam_toolbox_node` es un nodo de ciclo de vida (si no, `slam_lifecycle:=false`).
4. Driver LD19: sentido de giro (`laser_scan_dir`) y, sobre todo, la **QoS de `/r2/scan`**: si el driver publica
   *best effort* y slam_toolbox o el costmap de Nav2 piden *reliable*, no recibirán datos
   (`ros2 topic info /r2/scan -v`). Puerto (`/dev/ldlidar`) y baudrate (230400) ya están configurados.
   `scripts/setup_pi.sh` y su regla udev no se han probado en la Pi.
5. Medir en el robot la posición real del LiDAR (`laser_x/y/z` en el xacro; `laser_z` = 0.12 m es un valor supuesto).
6. Sustituir el maintainer `dev@example.com` de los `package.xml`.
