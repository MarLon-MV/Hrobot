# Workspace `src/`

Workspace ROS 2 Jazzy para un robot **iRobot Create 3 + Raspberry Pi 5 + LiDAR LD19** que mapea entornos
con **slam_toolbox** (teleoperación) y navega de forma autónoma sobre el mapa guardado con **AMCL + Nav2** (metas por RViz,
sin exploración autónoma). Antes de probar con
el robot real se simula en **Gazebo Harmonic**.

> Estado: nada de esto se ha compilado ni lanzado todavía contra ROS 2 real (se escribió sin ROS 2
> instalado). El primer `colcon build`, la simulación y el arranque en el robot están pendientes.

## Paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| [`create3_bot_description`](create3_bot_description) | ament_cmake | URDF/xacro (`base_link` → `laser_frame`; con `sim:=true` añade ruedas y plugins de Gazebo) y config de RViz. |
| [`create3_bot_bringup`](create3_bot_bringup) | ament_cmake | Solo `config/` y `launch/` para el robot real: `mapping`, `hardware_base`, `slam`. |
| [`create3_bot_simulation`](create3_bot_simulation) | ament_cmake | Gazebo: mundo, puente `ros_gz_bridge` y `sim.launch.py`. Solo para el PC de desarrollo. |

```text
create3_bot_description ◄── create3_bot_bringup ◄── create3_bot_simulation
        ▲                                                    │
        └────────────────────────────────────────────────────┘
```

## Arquitectura

```text
Robot real                                   Simulación (Gazebo, sin namespace)
  LD19 ──scan──► slam_toolbox ──► map          gz gpu_lidar ──/scan──► slam_toolbox ──► map
  Create 3 ──odom, TF odom→base_link──┘        gz DiffDrive ──/odom, /tf──┘
  teleop ──cmd_vel──► Create 3                 teleop ──/cmd_vel──► gz DiffDrive
```

Los mismos `slam.launch.py`, `slam_params.yaml` y `mapping.rviz` sirven para ambos mundos; solo cambia
quién publica `scan`, `odom` y TF.

## Uso rápido

```bash
# PC de desarrollo (simulación): ROS 2 Jazzy + Gazebo Harmonic + ros_gz
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-common ros-jazzy-navigation2 ros-jazzy-xacro
colcon build --symlink-install --packages-select create3_bot_description create3_bot_bringup create3_bot_simulation
source install/setup.bash
ros2 launch create3_bot_simulation sim.launch.py                 # terminal 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard             # terminal 2 (mueve el robot)

# Navegación autónoma (AMCL + Nav2) sobre un mapa guardado (maps/mapa1.yaml; ver bringup para guardarlo)
ros2 launch create3_bot_simulation sim.launch.py nav:=true map_yaml:=$PWD/maps/mapa1.yaml
# En RViz: "2D Goal Pose" para mandar el robot a un punto (AMCL ya arranca en la pose de spawn)

# Robot real (Raspberry Pi), ver scripts/setup_pi.sh
ros2 launch create3_bot_bringup mapping.launch.py lidar_port:=/dev/ldlidar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
ros2 launch create3_bot_bringup navigation.launch.py map_yaml:=$PWD/maps/mapa1.yaml lidar_port:=/dev/ldlidar   # navegar (2D Pose Estimate en RViz)
```

Más detalle en los README de [bringup](create3_bot_bringup/README.md) y
[simulation](create3_bot_simulation/README.md). Guardado de mapas: ver bringup.

## Namespace del robot (`robot_namespace`)

El Create 3 vive en el namespace `/r2` (`/r2/odom`, `/r2/tf`, `/r2/cmd_vel`...) y el stack real cuelga del
mismo. `robot_namespace:=<nombre>` (por defecto `$CREATE3_ROBOT_NAMESPACE` o `r2`; vacío = sin namespace).
`scripts/set_robot.sh r3` lo guarda para todas las terminales; el namespace del propio Create 3 se cambia
en su interfaz web (Application > ROS 2). **La simulación corre sin namespace.**

## RViz en otro equipo

`scripts/rviz_remote.sh [-n r2] [--dry-run]` abre RViz con `mapping.rviz` en un PC de la misma red
(mismo `ROS_DOMAIN_ID` y RMW, hora sincronizada con la Pi). Lanza la Pi con `rviz:=false`.

## Dependencias externas

- ROS 2 Jazzy, `slam_toolbox`, Nav2 (`navigation2`: AMCL, map_server, controller/planner/bt_navigator), `robot_state_publisher`, `xacro`, `rviz2`, `teleop_twist_keyboard`,
  `nav2_common` (solo `RewrittenYaml`/`ReplaceString` para los launch).
- Simulación: Gazebo Harmonic (`ros-jazzy-ros-gz`: `ros_gz_sim`, `ros_gz_bridge`).
- Driver del LD19: `ldlidar_stl_ros2` (LDRobot), no está en rosdep: `scripts/setup_pi.sh` lo clona en
  `src/ldlidar_stl_ros2` (ignorado por git).
- Create 3 accesible por red con el mismo `ROS_DOMAIN_ID` y RMW que la RPi5.

## Pendientes de la primera ejecución

1. `colcon build` y `sim.launch.py` en el PC: comprobar que el robot aparece en Gazebo, que `ros2 topic echo /scan`
   y `/odom` responden y que `ros2 run tf2_tools view_frames` muestra `map → odom → base_link → laser_frame`.
2. Confirmar si `async_slam_toolbox_node` es lifecycle en la versión instalada (si no, `slam_lifecycle:=false`).
3. Robot real: QoS de `/r2/scan` del driver (best effort vs reliable, `ros2 topic info /r2/scan -v`), sentido de
   giro (`laser_scan_dir`), y `/r2/odom` del Create 3 (BEST_EFFORT; slam_toolbox solo usa TF).
4. Medir la posición real del LiDAR (`laser_x/y/z` en el xacro; `laser_z` = 0.12 m es supuesto).
5. Sustituir el maintainer `dev@example.com` de los `package.xml`.
