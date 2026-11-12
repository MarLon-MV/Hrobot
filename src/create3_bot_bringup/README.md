# create3_bot_bringup

Solo `config/` y `launch/` (sin nodos propios) para el robot Create 3 + RPi5 + LiDAR LD19.

## Lanzar

```bash
ros2 launch create3_bot_bringup master_explore.launch.py
# Argumentos: use_sim_time:=false  rviz:=true|false  maps_dir:=~/.create3_maps  map:=<nombre>
#             slam_lifecycle:=true|false  (lifecycle de slam_toolbox, ver Supuestos)
```

Lanza: `robot_state_publisher` (xacro de `create3_bot_description`) + driver LD19 (`hardware_base`),
Nav2 sin amcl/map_server + `map_saver_server` (`navigation`), `slam_toolbox` (`async_slam_toolbox_node`),
`explorer_node`, `map_manager_node` y `rviz2` con `create3_bot_description/rviz/exploration.rviz`.
Launch individuales: `hardware_base.launch.py` (`lidar_port`, `lidar_baudrate`) y
`navigation.launch.py` (`params_file`, `autostart`).

## Disparar la exploracion

La accion se llama `explore_map` (relativa al nodo; `/explore_map` sin namespace):

```bash
ros2 action send_goal /explore_map create3_bot_interfaces/action/ExploreMap \
  "{start_exploration: true, max_duration_sec: 0.0}" --feedback
```

`max_duration_sec: 0.0` = sin limite. Al terminar, `explorer_node` guarda el mapa
(`save_map_on_finish`, nombre `map_name` de `config/explorer_params.yaml`).

## Guardar un mapa

```bash
ros2 service call /save_exploration_map create3_bot_interfaces/srv/SaveExplorationMap "{map_name: sala1}"
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
| `config/explorer_params.yaml` | `explorer_node` y `map_manager_node` (doubles con decimales; `save_timeout_sec` 30.0 > 4 x `service_timeout_sec` 5.0) |

## Supuestos y pendientes de hardware (no verificado: nada se ha lanzado contra ROS 2)

- **Driver LD19**: se asume `ldlidar_stl_ros2` (driver oficial de LDRobot), ejecutable
  `ldlidar_stl_ros2_node`, parametros `topic_name`, `frame_id`, `port_name`, `port_baudrate`
  (`product_name: LDLiDAR_LD19`). No esta en rosdep: clonarlo y compilarlo en el workspace. Si se usa
  otro driver, editar `hardware_base.launch.py` y `package.xml`.
- **Puerto serie**: por defecto `/dev/ttyUSB0`, 230400 baudios (confirmar; crear regla udev y permisos
  `dialout`). `angle_crop_*` desactivado; `laser_scan_dir` puede requerir invertirse segun el montaje.
- **Create 3**: no se lanza aqui. Se conecta por red; debe compartir `ROS_DOMAIN_ID` y RMW con la RPi5.
  Se asume SIN namespace: `/odom`, `/cmd_vel` y TF `odom -> base_link` publicados por el Create 3. Con
  namespace hay que remapear `cmd_vel`, `odom` y `tf`.
- **TF**: `laser_frame` cuelga de `base_link` con `laser_z: 0.12` m (xacro, ajustable); medir la altura/offset
  reales. `odom -> base_link` lo da el Create 3, `map -> odom` slam_toolbox.
- **slam_toolbox lifecycle**: en Jazzy `async_slam_toolbox_node` es un nodo lifecycle; el launch hace
  configure+activate. Si la version instalada no lo es, usar `slam_lifecycle:=false`.
- **Nav2 Jazzy**: parametros escritos de memoria (sin `nav2_bringup` instalado); revisar al primer arranque
  los logs de `controller_server`/`bt_navigator`. Sin `velocity_smoother`: `/cmd_vel` directo.
- La QoS de `/scan` del driver (best effort vs reliable) debe ser compatible con slam_toolbox y las capas de costmap.
