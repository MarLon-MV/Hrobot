# create3_bot_bringup

Solo `config/` y `launch/` (sin nodos propios) para el robot Create 3 + RPi5 + LiDAR LD19.

## Lanzar

```bash
ros2 launch create3_bot_bringup master_explore.launch.py
# Argumentos: robot_namespace:=r2  use_sim_time:=false  rviz:=true|false  maps_dir:=~/.create3_maps
#             map:=<nombre>  slam_lifecycle:=true|false  lidar_port:=/dev/ttyUSB0  lidar_baudrate:=230400
ros2 launch create3_bot_bringup master_explore.launch.py robot_namespace:=r3   # otro robot
ros2 launch create3_bot_bringup master_explore.launch.py robot_namespace:=''   # sin namespace (simulacion)
```

## Namespace del robot

El Create 3 usa el namespace `/r2` (`/r2/odom`, `/r2/tf`, `/r2/tf_static`, `/r2/cmd_vel`) y todo el stack
cuelga de el. `robot_namespace` (en los tres launch) vale por defecto `$CREATE3_ROBOT_NAMESPACE` si esta
definida y, si no, `r2`; vacio = sin namespace. `scripts/set_robot.sh r3` (o `--show`, `--check`, `--unset`)
guarda la variable para todas las terminales; **no cambia el namespace del Create 3** (interfaz web >
Application > ROS 2).

Diseno (el namespace se aplica **una sola vez**, nunca `/r2/r2/...`):
`hardware_base` y `navigation` aplican `PushRosNamespace` a sus propios nodos; `master_explore` lo aplica
solo a los suyos (slam_toolbox, `map_manager_node`, `explorer_node`, `rviz2`) y los `include` quedan fuera de
ese grupo, recibiendo `robot_namespace`. Ayudantes comunes en `launch/create3_namespace.py`.

- **TF**: remapeo `/tf -> tf` y `/tf_static -> tf_static` en cada nodo con TF (robot_state_publisher, Nav2,
  slam_toolbox, rviz2), asi leen/publican `/r2/tf` y `/r2/tf_static`. En `explorer_node` el destino es absoluto
  (`/tf:=/r2/tf`) porque su `tf2_ros::TransformListener` crea un nodo interno en el namespace raiz que solo ve
  remapeos globales. Los `frame_id` no llevan prefijo.
- **YAML**: con namespace se reescriben al arrancar con `nav2_common` `RewrittenYaml(root_key=<ns>)`
  (nav2, slam, explorer); sin namespace se usan tal cual. Los topics/servicios de los YAML son relativos, **salvo los de los costmaps**
  (ver abajo).
- **Costmaps de Nav2**: son nodos anidados (`/r2/local_costmap/local_costmap`, namespace `/r2/local_costmap`), asi que un
  `topic: scan` relativo daria `/r2/local_costmap/scan` en vez de `/r2/scan` (el costmap local no veia el LiDAR).
  En `nav2_params.yaml` los topics de los costmaps (`scan.topic` de ambos `obstacle_layer`, `static_layer.map_topic`) son
  absolutos con el marcador `/<robot_namespace>/...`, que `navigation.launch.py` sustituye con `ReplaceString` (helper
  `substitute_namespace_markers`) **antes** de `RewrittenYaml`: `/r2/scan`, `/r2/map` o, sin namespace, `/scan`, `/map`.
  La auditoria de nombres relativos esta en el comentario inicial de `nav2_params.yaml`.
- **RViz**: `exploration.rviz` usa `/<robot_namespace>/...`; `ReplaceString` lo sustituye por `/r2/...` o,
  sin namespace, por `/...` (sin barra doble).

| Nombre | `robot_namespace:=r2` | `robot_namespace:=''` |
|---|---|---|
| LiDAR / mapa / fronteras | `/r2/scan`, `/r2/map`, `/r2/frontiers` | `/scan`, `/map`, `/frontiers` |
| Create 3 | `/r2/odom` (BEST_EFFORT), `/r2/cmd_vel`, `/r2/tf`, `/r2/tf_static` | `/odom`, `/cmd_vel`, `/tf`, `/tf_static` |
| Odometria para Nav2 (rele) | `/r2/odom_reliable` | `/odom_reliable` |
| Exploracion | `/r2/explore_map`, `/r2/save_exploration_map` | `/explore_map`, `/save_exploration_map` |
| Nav2 | `/r2/navigate_to_pose`, `/r2/map_saver/save_map` | `/navigate_to_pose`, `/map_saver/save_map` |
| slam_toolbox | `/r2/slam_toolbox/serialize_map` | `/slam_toolbox/serialize_map` |
| Nodos | `/r2/explorer_node`, `/r2/slam_toolbox`, ... | `/explorer_node`, `/slam_toolbox`, ... |

Lanza: `robot_state_publisher` (xacro de `create3_bot_description`) + driver LD19 (`hardware_base`),
Nav2 sin amcl/map_server + `map_saver_server` + `odom_relay_node` (`navigation`), `slam_toolbox` (`async_slam_toolbox_node`),
`explorer_node`, `map_manager_node` y `rviz2` con `create3_bot_description/rviz/exploration.rviz`.
Launch individuales: `hardware_base.launch.py` (`lidar_port`, `lidar_baudrate`) y
`navigation.launch.py` (`params_file`, `explorer_params_file`, `autostart`); ambos aceptan `robot_namespace`.

**Rele de odometria (`odom_relay_node`)**: el Create 3 publica `/r2/odom` BEST_EFFORT (depth 1) y Nav2
(`controller_server`, `bt_navigator`) suscribe RELIABLE, por lo que no conectan. `navigation.launch.py` lanza
`odom_relay_node` bajo el mismo namespace y `use_sim_time`; sus parametros salen de la seccion `odom_relay_node:`
de `explorer_params.yaml` (reescrita con `RewrittenYaml(root_key=<ns>)`) y republica `odom` como `odom_reliable`
(RELIABLE). `nav2_params.yaml` fija `odom_topic: odom_reliable` en `bt_navigator` y `controller_server`.
Comprobacion: `ros2 topic info /r2/odom_reliable -v` debe mostrar `RELIABLE`, publicador `odom_relay_node` y como
suscriptores `controller_server` y `bt_navigator`.

## Disparar la exploracion

La accion se llama `explore_map` (relativa al namespace: `/r2/explore_map`; `/explore_map` sin namespace):

```bash
ros2 action send_goal /r2/explore_map create3_bot_interfaces/action/ExploreMap \
  "{start_exploration: true, max_duration_sec: 0.0}" --feedback
```

`max_duration_sec: 0.0` = sin limite. Al terminar, `explorer_node` guarda el mapa
(`save_map_on_finish`, nombre `map_name` de `config/explorer_params.yaml`).

## Guardar un mapa

```bash
ros2 service call /r2/save_exploration_map create3_bot_interfaces/srv/SaveExplorationMap "{map_name: sala1}"
```

Genera en `maps_dir`: `sala1.yaml/.pgm` (map_saver) y `sala1.posegraph/.data` (slam_toolbox).

## Ampliar un mapa

```bash
ros2 launch create3_bot_bringup master_explore.launch.py map:=sala1
```

slam_toolbox arranca con `map_file_name: <maps_dir>/sala1` (sin extension), `map_start_at_dock: true`
y modo mapping. Si `sala1.posegraph`/`.data` no existen, el launch aborta con un `RuntimeError`
explicito (no arranca un mapa nuevo en silencio). Importante: `map_start_at_dock` supone que el robot
arranca en el dock/pose de origen del mapa; si no, hay que usar `map_start_pose` (pendiente).

## Parametros

| Archivo | Contenido |
|---|---|
| `config/slam_params.yaml` | slam_toolbox online_async, `max_laser_range: 12.0` (limite del LD19) |
| `config/nav2_params.yaml` | Nav2 sin amcl/map_server; RPP (0.25 m/s), NavFn con `allow_unknown`, `robot_radius: 0.18`, `inflation_radius: 0.40` |
| `config/explorer_params.yaml` | `explorer_node`, `map_manager_node` y `odom_relay_node` (doubles con decimales; `save_timeout_sec` 30.0 > 4 x `service_timeout_sec` 5.0) |

## Supuestos y pendientes de hardware (no verificado: nada se ha lanzado contra ROS 2)

- **Driver LD19**: se asume `ldlidar_stl_ros2` (driver oficial de LDRobot), ejecutable
  `ldlidar_stl_ros2_node`, parametros `topic_name`, `frame_id`, `port_name`, `port_baudrate`
  (`product_name: LDLiDAR_LD19`). No esta en rosdep: clonarlo y compilarlo en el workspace. Si se usa
  otro driver, editar `hardware_base.launch.py` y `package.xml`.
- **Puerto serie**: por defecto `/dev/ttyUSB0`, 230400 baudios (confirmar; crear regla udev y permisos
  `dialout`). `angle_crop_*` desactivado; `laser_scan_dir` puede requerir invertirse segun el montaje.
- **Create 3**: no se lanza aqui. Se conecta por red; debe compartir `ROS_DOMAIN_ID` y RMW con la RPi5.
  Se asume el namespace `r2` (o el de `robot_namespace`): `/r2/odom`, `/r2/cmd_vel` y TF `odom -> base_link`
  en `/r2/tf`, publicados por el Create 3. Sin verificar contra el robot real.
- **TF**: `laser_frame` cuelga de `base_link` con `laser_z: 0.12` m (xacro, ajustable); medir la altura/offset
  reales. `odom -> base_link` lo da el Create 3, `map -> odom` slam_toolbox.
- **slam_toolbox lifecycle**: en Jazzy `async_slam_toolbox_node` es un nodo lifecycle; el launch hace
  configure+activate. Si la version instalada no lo es, usar `slam_lifecycle:=false`.
- **Nav2 Jazzy**: parametros escritos de memoria (sin `nav2_bringup` instalado); revisar al primer arranque
  los logs de `controller_server`/`bt_navigator`. Sin `velocity_smoother`: `cmd_vel` (`/r2/cmd_vel`) directo.
- Namespace: sin verificar con ROS 2 real: el `RewrittenYaml` con `root_key`, que `slam_toolbox` exponga
  `slam_toolbox/serialize_map` bajo el namespace y que el remapeo de TF de `explorer_node` funcione.
- Sin verificar con ROS 2 real: que `ReplaceString` -> `RewrittenYaml` encadenen como se asume y que el costmap local reciba `/r2/scan` (ver `ros2 topic info /r2/scan`; no debe aparecer `/r2/local_costmap/scan`). Riesgo abierto: el remapeo relativo `/tf:=tf` en controller_server/planner_server podria resolverse en el namespace anidado de los costmaps (`/r2/local_costmap/tf`); si el costmap no encuentra TF, usar remapeo absoluto `/tf:=/r2/tf`.
- Sin verificar con ROS 2 real: que `/r2/odom_reliable` conecte con los suscriptores RELIABLE de Nav2 y que el Create 3 siga publicando `/r2/odom` BEST_EFFORT.
- La QoS de `/r2/scan` del driver (best effort vs reliable) debe ser compatible con slam_toolbox y las capas de costmap.
