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
| [`hrobot_description`](hrobot_description) | ament_cmake | URDF/xacro (`base_link` → `laser_frame`; con `sim:=true` añade ruedas y plugins de Gazebo) y config de RViz. |
| [`hrobot_bringup`](hrobot_bringup) | ament_cmake | Solo `config/` y `launch/` para el robot real: `mapping`, `hardware_base`, `slam`. |
| [`hrobot_simulation`](hrobot_simulation) | ament_cmake | Gazebo: mundo, puente `ros_gz_bridge` y `sim.launch.py`. Solo para el PC de desarrollo. |

```text
hrobot_description ◄── hrobot_bringup ◄── hrobot_simulation
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
colcon build --symlink-install --packages-select hrobot_description hrobot_bringup hrobot_simulation
source install/setup.bash
ros2 launch hrobot_simulation sim.launch.py                 # terminal 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard             # terminal 2 (mueve el robot)

# Navegación autónoma (AMCL + Nav2) sobre un mapa guardado (src/maps/mapa1.yaml; ver bringup para guardarlo)
ros2 launch hrobot_simulation sim.launch.py nav:=true map_yaml:=$PWD/src/maps/mapa1.yaml
# En RViz: "2D Goal Pose" para mandar el robot a un punto (AMCL ya arranca en la pose de spawn)

# Robot real (Raspberry Pi), ver src/scripts/setup_pi.sh
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$PWD/src/maps/mapa1.yaml lidar_port:=/dev/ldlidar   # navegar (AMCL arranca en el dock, origen del mapa)
```

Más detalle en los README de [bringup](hrobot_bringup/README.md) y
[simulation](hrobot_simulation/README.md). Guardado de mapas: ver bringup.

## Namespace del robot (`robot_namespace`)

El Create 3 vive en el namespace `/r2` (`/r2/odom`, `/r2/tf`, `/r2/cmd_vel`...) y el stack real cuelga del
mismo. `robot_namespace:=<nombre>` (por defecto `$CREATE3_ROBOT_NAMESPACE` o `r2`; vacío = sin namespace).
Exporta `CREATE3_ROBOT_NAMESPACE=r3` para fijarlo; el namespace del propio Create 3 se cambia
en su interfaz web (Application > ROS 2). **La simulación corre sin namespace.**

## Dependencias externas

- ROS 2 Jazzy, `slam_toolbox`, Nav2 (`navigation2`: AMCL, map_server, controller/planner/bt_navigator), `robot_state_publisher`, `xacro`, `rviz2`, `teleop_twist_keyboard`,
  `nav2_common` (solo `RewrittenYaml`/`ReplaceString` para los launch).
- Simulación: Gazebo Harmonic (`ros-jazzy-ros-gz`: `ros_gz_sim`, `ros_gz_bridge`).
- Driver del LD19: `ldlidar_stl_ros2` (LDRobot), no está en rosdep: `src/scripts/setup_pi.sh` lo clona en
  `src/ldlidar_stl_ros2` (ignorado por git).
- Create 3 accesible por red con el mismo `ROS_DOMAIN_ID` y RMW que la RPi5.

