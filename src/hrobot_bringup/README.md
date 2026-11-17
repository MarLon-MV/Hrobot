# hrobot_bringup

`config/` y `launch/` para el robot **real** (Create 3 + RPi5 + LiDAR LD19), más dos scripts Python auxiliares
(`scan_resampler.py`, `odom_relay.py`; no hay C++). Mapea con slam_toolbox moviendo el robot por teleop y navega sobre
el mapa guardado con AMCL + Nav2.

| Launch / script | Contenido |
|---|---|
| `mapping.launch.py` | Robot real: `hardware_base` + `slam`. |
| `navigation.launch.py` | Robot real: `hardware_base` + `nav` + relé de odom. |
| `hardware_base.launch.py` | `robot_state_publisher` (xacro, `sim:=false`) + driver LD19 (publica `scan_raw`) + `scan_resampler` (publica `scan`). El Create 3 no se lanza aquí. |
| `slam.launch.py` | `slam_toolbox` + RViz (`mapping.rviz`). Lo reutiliza `hrobot_simulation`. |
| `nav.launch.py` | `map_server` + `amcl` + Nav2 + RViz (`navigation.rviz`), sin hardware. Lo reutiliza la simulación. |
| `hrobot_namespace.py` | Módulo auxiliar de namespace (no es un launch). |
| `scripts/scan_resampler.py` | `scan_raw` -> `scan` con 504 rayos fijos. |
| `scripts/odom_relay.py` | `odom` (BEST_EFFORT) -> `odom_reliable`, para Nav2. |

## 1. Preparar la Pi (una vez)

Desde la raíz del workspace en la Pi:

```bash
src/scripts/setup_pi.sh [--domain-id N] [--rmw rmw_fastrtps_cpp|rmw_cyclonedds_cpp] [--skip-net]
```

Instala dependencias y el driver LD19, crea la regla udev `/dev/ldlidar`, **fija `ROS_DOMAIN_ID` y
`RMW_IMPLEMENTATION`** (`~/.config/hrobot/ros_net.env`, cargado desde `~/.bashrc`) y compila. Después abre una terminal
nueva y `source install/setup.bash`.

Pi, laptop y Create 3 deben compartir `ROS_DOMAIN_ID` y RMW. El Create 3 los fija en su interfaz web
(Application > ROS 2); reinicia su aplicación tras cambiarlos. Comprueba también que los relojes de Pi y Create 3
estén en hora (`date -u`); sin RTC/NTP la Pi arranca con la hora del último apagado.

## 2. Preparar la laptop (si vas a ver RViz desde ella)

Necesita este repo y `ros-jazzy-rviz2` (no hace falta compilar). Exporta los mismos valores que la Pi:

```bash
export ROS_DOMAIN_ID=..  RMW_IMPLEMENTATION=..     # o pásalos con --domain / --rmw al script
ros2 topic list                                     # con el robot en marcha debes ver /r2/scan
```

## 3. Antes de mapear: arrancar desde el dock

El frame `map` nace en la pose del robot al arrancar el mapeo; si cada vez empieza en un sitio distinto, el modelo de
RViz no coincide con el real. Para tener una referencia repetible (y la que exige ampliar un mapa, que usa
`map_start_at_dock`):

1. Pon el robot **en el dock** y no lo muevas hasta que RViz muestre el mapa.
2. Opcional, si el Create 3 lleva tiempo encendido y su odometría ya se movió, ponla a cero **antes** de lanzar el
   mapeo (no con slam_toolbox corriendo):
   ```bash
   ros2 service call /r2/reset_pose irobot_create_msgs/srv/ResetPose "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```
   Necesita `ros-jazzy-irobot-create-msgs` en la máquina que lo llama (`sudo apt update && sudo apt install
   ros-jazzy-irobot-create-msgs`); sin él da «The passed service type is invalid». Probado: el Create 3 responde sin error.
3. Con el robot en el dock, el modelo de RViz debe apuntar igual que el real. Si no, queda un desfase fijo: es el yaw
   de `laser_joint` en `hrobot_ld19.urdf.xacro` (o `laser_scan_dir` del driver), no un problema del dock.

## 4. Mapear

```bash
# Pi, terminal 1 (rviz:=false si lo vas a ver desde la laptop)
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar rviz:=false
# Pi, terminal 2: mover el robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
# Laptop: ver el mapeo (o abre RViz en la Pi con rviz:=true, el valor por defecto)
src/scripts/rviz_remote.sh [mapping|navigation] [--namespace NS] [--domain ID] [--rmw NAME]
```

`rviz_remote.sh` sustituye `/<robot_namespace>` en el `.rviz` y remapea `/tf` y `/tf_static`; no fija nada de forma
permanente.

Argumentos de `mapping.launch.py`: `robot_namespace:=r2  use_sim_time:=false  rviz:=true|false  maps_dir:=~/.create3_maps
map:=<nombre>  slam_lifecycle:=true|false  lidar_port:=/dev/ttyUSB0  lidar_baudrate:=230400  laser_scan_dir:=true|false`
(`laser_scan_dir:=false` si el scan gira al revés que el robot). Para otro robot,
`robot_namespace:=r3`; sin namespace, `robot_namespace:=''`.

## 5. Guardar el mapa

Con el mapeo aún corriendo (Pi, terminal 3):

```bash
src/scripts/save_map.sh nombre_del_mapa [--maps-dir DIR] [--namespace NS] [--force]
```

Genera `maps/<nombre>.{posegraph,data,yaml,pgm}` llamando a los servicios `serialize_map` y `save_map` de
slam_toolbox; no sobrescribe sin `--force` y comprueba que existan los cuatro archivos. Corre en la máquina donde corre
slam_toolbox.

<details>
<summary>Equivalente manual</summary>

```bash
WS=$PWD   # los mapas quedan en <workspace>/maps
ros2 service call /r2/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$WS/maps/sala1'}"
ros2 service call /r2/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$WS/maps/sala1'}}"
```

Los nombres de los campos están escritos de memoria: verifícalos con `ros2 interface show`.
</details>

## 6. Ampliar un mapa guardado (opcional)

Para el mapeo, pon el robot en el dock/origen del mapa y relanza el mapeo con el mapa cargado:

```bash
ros2 launch hrobot_bringup mapping.launch.py map:=sala1 maps_dir:=$PWD/maps
```

slam_toolbox carga `<maps_dir>/<nombre>.posegraph` (`map_start_at_dock: true`, modo mapping). Si el pose graph no existe
el launch aborta con un `RuntimeError` (no arranca un mapa nuevo en silencio). Las rutas deben ser absolutas (sin `~`).
Después vuelve al paso 5 para guardar.

## 7. Navegar sobre el mapa guardado

Para el mapeo antes (Ctrl+C): mapping y navigation a la vez publican `map -> odom`.

```bash
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$PWD/maps/sala1.yaml lidar_port:=/dev/ldlidar
```

- En RViz: "2D Pose Estimate" (pose inicial) y luego "2D Goal Pose" (topic `goal_pose`). La pose inicial también se
  puede dar con `set_initial_pose:=true initial_x:=.. initial_y:=.. initial_yaw:=..`. Nav2 publica velocidades en
  `cmd_vel` (`/r2/cmd_vel`).
- `nav.launch.py`: `map_server` + `amcl` (publica `map -> odom`) + Nav2 (controller RPP, planner Navfn, smoother,
  behaviors, bt_navigator) + RViz. Config en `config/nav2_params.yaml`. `navigation.launch.py` añade `hardware_base` y
  activa el relé de odom.
- `odom_relay.py`: el Create 3 publica `/r2/odom` BEST_EFFORT y Nav2 suscribe RELIABLE (no empareja); el relé publica
  `odom_reliable`, que leen `controller_server` y `bt_navigator`. Comprueba con
  `ros2 topic info /r2/odom_reliable -v`.

## Referencia

### Namespace del robot

`robot_namespace` vale por defecto `$CREATE3_ROBOT_NAMESPACE` si existe y, si no, `r2`; vacío = sin namespace.
Se aplica **una sola vez**: cada launch hace `PushRosNamespace` solo de los nodos que crea y `mapping` no envuelve los
`include` (nunca `/r2/r2/...`).

- **TF**: remapeo `/tf -> tf`, `/tf_static -> tf_static` (el Create 3 publica `/r2/tf`). Los `frame_id` no llevan prefijo.
- **YAML**: con namespace se reescribe con `RewrittenYaml(root_key=<ns>)`; sin namespace se usa tal cual.
- **slam_toolbox** publica `/map` absoluto: se remapea a `/<ns>/map`.
- **RViz**: `mapping.rviz` usa `/<robot_namespace>/...`, sustituido con `ReplaceString`.

| Nombre | `r2` | vacío |
|---|---|---|
| LiDAR / mapa | `/r2/scan`, `/r2/map` | `/scan`, `/map` |
| Create 3 | `/r2/odom`, `/r2/cmd_vel`, `/r2/tf`, `/r2/tf_static` | `/odom`, `/cmd_vel`, `/tf`, `/tf_static` |
| slam_toolbox | `/r2/slam_toolbox/serialize_map`, `/r2/slam_toolbox/save_map` | sin prefijo |

### Parámetros

`config/slam_params.yaml`: slam_toolbox online_async, `max_laser_range: 12.0` (límite del LD19).

### Supuestos y pendientes de hardware (nada verificado contra ROS 2 real)

- **Driver LD19**: `ldlidar_stl_ros2` (ejecutable `ldlidar_stl_ros2_node`, `product_name: LDLiDAR_LD19`). Si usas otro,
  edita `hardware_base.launch.py` y `package.xml`. Puerto por defecto `/dev/ttyUSB0`, 230400 baudios; `laser_scan_dir`
  puede requerir invertirse.
- **`scan_resampler.py`**: el LD19 da 500-504 rayos por vuelta y slam_toolbox avisa `LaserRangeScan contains N range
  readings, expected 504`. El driver publica `scan_raw` y el nodo republica `scan` con 504 rayos fijos (0..2π, vecino
  más cercano; parámetro `num_beams`). Comprueba que el aviso desaparezca y que `ros2 topic hz /r2/scan` siga en ~10 Hz.
- QoS de `/r2/scan_raw` del driver (best effort vs reliable) debe ser compatible con el suscriptor del resampler
  (`ros2 topic info /r2/scan_raw -v`).
- **Create 3**: se conecta por red (ver paso 1) y publica `odom -> base_link` en `/r2/tf`.
- **TF**: `laser_frame` cuelga de `base_link` con `laser_z: 0.12` m (medir el real). `map -> odom` lo da slam_toolbox.
- **slam_toolbox lifecycle**: en Jazzy `async_slam_toolbox_node` es lifecycle; si no lo es, `slam_lifecycle:=false`.
