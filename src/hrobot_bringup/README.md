# hrobot_bringup

Solo `config/` y `launch/` (sin nodos propios) para el robot **real**: Create 3 + RPi5 + LiDAR LD19, mapeo
con slam_toolbox y movimiento por teleop.

## Lanzar

```bash
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar
# Argumentos: robot_namespace:=r2  use_sim_time:=false  rviz:=true|false  maps_dir:=~/.create3_maps
#             map:=<nombre>  slam_lifecycle:=true|false  lidar_port:=/dev/ttyUSB0  lidar_baudrate:=230400
ros2 launch hrobot_bringup mapping.launch.py robot_namespace:=r3   # otro robot
ros2 launch hrobot_bringup mapping.launch.py robot_namespace:=''   # sin namespace

# Mover el robot (otra terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
```

| Launch | Contenido |
|---|---|
| `mapping.launch.py` | Robot real: `hardware_base` + `slam`. |
| `hardware_base.launch.py` | `robot_state_publisher` (xacro, `sim:=false`) + driver LD19 (publica `scan_raw`) + `scan_resampler` (publica `scan`). El Create 3 no se lanza aquí. |
| `slam.launch.py` | `slam_toolbox` + RViz (`mapping.rviz`). Lo reutiliza `hrobot_simulation`. |
| `hrobot_namespace.py` | Módulo auxiliar de namespace (no es un launch). |

## Namespace del robot

`robot_namespace` vale por defecto `$CREATE3_ROBOT_NAMESPACE` si existe y, si no, `r2`; vacío = sin namespace.
Se aplica **una sola vez**: cada launch hace `PushRosNamespace` solo de los nodos que crea y `mapping` no
envuelve los `include` (nunca `/r2/r2/...`).

- **TF**: remapeo `/tf -> tf`, `/tf_static -> tf_static` (el Create 3 publica `/r2/tf`). Los `frame_id` no llevan prefijo.
- **YAML**: con namespace se reescribe con `RewrittenYaml(root_key=<ns>)`; sin namespace se usa tal cual.
- **slam_toolbox** publica `/map` absoluto: se remapea a `/<ns>/map`.
- **RViz**: `mapping.rviz` usa `/<robot_namespace>/...`, sustituido con `ReplaceString`.

| Nombre | `r2` | vacío |
|---|---|---|
| LiDAR / mapa | `/r2/scan`, `/r2/map` | `/scan`, `/map` |
| Create 3 | `/r2/odom`, `/r2/cmd_vel`, `/r2/tf`, `/r2/tf_static` | `/odom`, `/cmd_vel`, `/tf`, `/tf_static` |
| slam_toolbox | `/r2/slam_toolbox/serialize_map`, `/r2/slam_toolbox/save_map` | sin prefijo |

## Red ROS 2 y scripts auxiliares

Pi, laptop y Create 3 deben compartir `ROS_DOMAIN_ID` y RMW (`rmw_fastrtps_cpp` o `rmw_cyclonedds_cpp`). El Create 3 los
fija en su interfaz web (Application > ROS 2); reinicia su aplicación tras cambiarlos. Scripts en `src/scripts/`:

| Script | Dónde | Para qué |
|---|---|---|
| `setup_pi.sh [--domain-id N] [--rmw NAME] [--skip-net]` | Pi (una vez) | Instala dependencias y driver LD19, regla udev `/dev/ldlidar`, **fija `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION`** (`~/.config/hrobot/ros_net.env`, cargado desde `~/.bashrc`) y compila. |
| `rviz_remote.sh [mapping\|navigation] [--namespace NS] [--domain ID] [--rmw NAME]` | Laptop | Abre RViz contra el robot real: sustituye `/<robot_namespace>` en el `.rviz` y remapea `/tf` y `/tf_static`. No fija nada de forma permanente. |
| `save_map.sh <nombre>` | Donde corre slam_toolbox (Pi) | Guarda el mapa (ver más abajo). |

Con la Pi ya configurada, en la laptop exporta los mismos valores (`export ROS_DOMAIN_ID=.. RMW_IMPLEMENTATION=..`) o
pásalos con `--domain`/`--rmw`. Comprueba con `ros2 topic list` que ves `/r2/scan`.

### Ver el mapeo desde la laptop

```bash
# Pi: mapear sin RViz local
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar rviz:=false
# Laptop (necesita este repo y ros-jazzy-rviz2; no hace falta compilar)
src/scripts/rviz_remote.sh                 # o: rviz_remote.sh navigation
```

## Flujo completo en el robot real: mapear -> guardar -> navegar

Desde la raíz del workspace (`source install/setup.bash`):

```bash
# 1. Mapear (terminal 1) y mover el robot con teleop (terminal 2)
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel

# 2. Guardar (terminal 3, con el mapeo aún corriendo). Genera maps/sala1.{posegraph,data,yaml,pgm}
src/scripts/save_map.sh nombre_del_mapa

# 3. Parar el mapeo (Ctrl+C) y navegar sobre el mapa guardado
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$PWD/maps/sala1.yaml lidar_port:=/dev/ldlidar
#    En RViz: "2D Pose Estimate" (pose inicial) y luego "2D Goal Pose".
```

`save_map.sh <nombre> [--maps-dir DIR] [--namespace NS] [--force]` llama a los servicios `serialize_map` y `save_map`
de slam_toolbox, no sobrescribe sin `--force` y comprueba que los cuatro archivos existan. No corras mapping y
navigation a la vez: ambos publican `map -> odom`.

## Calibrar la orientación: arrancar siempre desde el dock

El frame `map` nace en la pose del robot al arrancar el mapeo, así que si cada vez empieza en un sitio distinto el
modelo de RViz no coincide con el real. Para tener una referencia repetible (y la que exige ampliar un mapa con
`map:=...`, que usa `map_start_at_dock`):

1. Pon el robot **en el dock** antes de lanzar `mapping.launch.py` (no lo muevas hasta que RViz muestre el mapa).
2. Opcional, si el Create 3 lleva tiempo encendido y su odometría ya se movió, ponla a cero **antes** de lanzar el mapeo:
   ```bash
   ros2 service call /r2/reset_pose irobot_create_msgs/srv/ResetPose "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```
   (Verifica los campos con `ros2 interface show irobot_create_msgs/srv/ResetPose`. No lo llames con slam_toolbox corriendo.)
3. Con el robot en el dock, el modelo de RViz debe apuntar igual que el real. Si no, queda un desfase fijo: es el yaw de
   `laser_joint` en `hrobot_ld19.urdf.xacro` (o `laser_scan_dir` del driver), no un problema del dock.

## Guardar y ampliar un mapa (manual)

Con slam_toolbox corriendo (ajusta el namespace; los nombres son rutas en la máquina donde corre slam_toolbox):

```bash
# Desde la raíz del workspace; los mapas quedan en <workspace>/maps
WS=$PWD
# Pose graph (permite ampliar el mapa después): genera sala1.posegraph y sala1.data
ros2 service call /r2/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$WS/maps/sala1'}"
# Imagen .pgm/.yaml (para usar con otras herramientas)
ros2 service call /r2/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$WS/maps/sala1'}}"

# Ampliar un mapa guardado (el robot debe arrancar en el dock/origen del mapa)
ros2 launch hrobot_bringup mapping.launch.py map:=sala1 maps_dir:=$WS/maps
```

Con `map:=<nombre>` slam_toolbox carga `<maps_dir>/<nombre>.posegraph` (`map_start_at_dock: true`, modo mapping). Si el
pose graph no existe el launch aborta con un `RuntimeError` (no arranca un mapa nuevo en silencio). Las rutas deben ser
absolutas (sin `~`).

## Parámetros

`config/slam_params.yaml`: slam_toolbox online_async, `max_laser_range: 12.0` (límite del LD19).

## Supuestos y pendientes de hardware (nada verificado contra ROS 2 real)

- **Driver LD19**: `ldlidar_stl_ros2` (ejecutable `ldlidar_stl_ros2_node`, `product_name: LDLiDAR_LD19`). Si usas otro,
  edita `hardware_base.launch.py` y `package.xml`. Puerto por defecto `/dev/ttyUSB0`, 230400 baudios; `laser_scan_dir`
  puede requerir invertirse.
- **Create 3**: se conecta por red (ver «Red ROS 2 y scripts auxiliares») y publica `odom -> base_link` en `/r2/tf`.
- **TF**: `laser_frame` cuelga de `base_link` con `laser_z: 0.12` m (medir el real). `map -> odom` lo da slam_toolbox.
- **slam_toolbox lifecycle**: en Jazzy `async_slam_toolbox_node` es lifecycle; si no lo es, `slam_lifecycle:=false`.
- Los nombres de los campos de `SerializePoseGraph`/`SaveMap` de arriba están escritos de memoria: verifícalos con
  `ros2 interface show`.
- `scripts/scan_resampler.py`: el LD19 da 500-504 rayos por vuelta y slam_toolbox avisa `LaserRangeScan contains N range
  readings, expected 504`. El driver publica `scan_raw` y el nodo republica `scan` con 504 rayos fijos (0..2π, vecino más
  cercano; parámetro `num_beams`). Comprueba que el aviso desaparezca y que `ros2 topic hz /r2/scan` siga en ~10 Hz.
- QoS de `/r2/scan_raw` del driver (best effort vs reliable) debe ser compatible con slam_toolbox.

## Navegación autónoma (AMCL + Nav2)

Sobre un mapa ya guardado (imagen `.yaml`/`.pgm` de `save_map`, ver arriba). No corre slam_toolbox a la vez.

```bash
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$WS/maps/sala1.yaml lidar_port:=/dev/ldlidar
```

- `nav.launch.py` (sin hardware, lo comparte la simulación): `map_server` + `amcl` (publica `map -> odom`) + Nav2
  (controller RPP, planner Navfn, smoother, behaviors, bt_navigator) + RViz (`navigation.rviz`).
  Config en `config/nav2_params.yaml`. `navigation.launch.py` añade `hardware_base` y activa el relé de odom.
- Pose inicial: "2D Pose Estimate" en RViz, o `set_initial_pose:=true initial_x:=.. initial_y:=.. initial_yaw:=..`.
  Meta: "2D Goal Pose" en RViz (topic `goal_pose`). Nav2 publica velocidades en `cmd_vel` (`/r2/cmd_vel`).
- `scripts/odom_relay.py`: el Create 3 publica `/r2/odom` BEST_EFFORT y Nav2 suscribe RELIABLE (no empareja); el relé
  publica `odom_reliable`, que leen `controller_server` y `bt_navigator`. Comprueba con `ros2 topic info /r2/odom_reliable -v`.
  Excepción consciente a "solo config/launch": es un script Python, no hay C++.
