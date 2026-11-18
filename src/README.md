# Workspace `src/`

ROS 2 Jazzy-workspace voor een robot met **iRobot Create 3 + Raspberry Pi 5 + SLAMTEC RPLIDAR C1 (360° DTOF-laserscanner)** die
omgevingen in kaart brengt met **slam_toolbox** (teleoperatie) en autonoom navigeert op de opgeslagen kaart met **AMCL + Nav2**
(doelen via RViz, geen autonome exploratie). Voordat er met de echte robot wordt getest, wordt er gesimuleerd in **Gazebo Harmonic**.

> Status: niets hiervan is al gebouwd of gestart tegen echt ROS 2 (het is geschreven zonder geïnstalleerd ROS 2). De eerste
> `colcon build`, de simulatie en het opstarten op de robot staan nog open. De overstap van de LD19 naar de RPLIDAR C1 is
> evenmin op hardware getest.

## Pakketten

| Pakket | Type | Inhoud |
|---|---|---|
| [`hrobot_description`](hrobot_description) | ament_cmake | URDF/xacro (`base_link` → `laser_frame`; met `sim:=true` komen wielen en Gazebo-plugins erbij) en RViz-configuratie. |
| [`hrobot_bringup`](hrobot_bringup) | ament_cmake | Alleen `config/` en `launch/` voor de echte robot: `mapping`, `hardware_base`, `slam`, `nav`, `navigation`. |
| [`hrobot_simulation`](hrobot_simulation) | ament_cmake | Gazebo: wereld, `ros_gz_bridge`-brug en `sim.launch.py`. Alleen voor de ontwikkel-pc. |

```text
hrobot_description ◄── hrobot_bringup ◄── hrobot_simulation
        ▲                                                    │
        └────────────────────────────────────────────────────┘
```

## Architectuur

```text
Echte robot                                  Simulatie (Gazebo, zonder namespace)
  RPLIDAR C1 ──scan──► slam_toolbox ──► map    gz gpu_lidar ──/scan──► slam_toolbox ──► map
  Create 3 ──odom, TF odom→base_link──┘        gz DiffDrive ──/odom, /tf──┘
  teleop ──cmd_vel──► Create 3                 teleop ──/cmd_vel──► gz DiffDrive
```

Dezelfde `slam.launch.py`, `slam_params.yaml` en `mapping.rviz` dienen voor beide werelden; alleen wie `scan`, `odom`
en TF publiceert verschilt.

## Snelstart

```bash
# Ontwikkel-pc (simulatie): ROS 2 Jazzy + Gazebo Harmonic + ros_gz
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-common ros-jazzy-navigation2 ros-jazzy-xacro
colcon build --symlink-install --packages-select hrobot_description hrobot_bringup hrobot_simulation
source install/setup.bash
ros2 launch hrobot_simulation sim.launch.py                 # terminal 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard             # terminal 2 (beweegt de robot)

# Autonome navigatie (AMCL + Nav2) op een opgeslagen kaart (src/maps/mapa1.yaml; zie bringup om op te slaan)
ros2 launch hrobot_simulation sim.launch.py nav:=true map_yaml:=$PWD/src/maps/mapa1.yaml
# In RViz: "2D Goal Pose" om de robot naar een punt te sturen (AMCL start al op de spawn-pose)

# Echte robot (Raspberry Pi), zie src/scripts/setup_pi.sh
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/rplidar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$PWD/src/maps/mapa1.yaml lidar_port:=/dev/rplidar   # navigeren (AMCL start in het dock, oorsprong van de kaart)
```

Meer details in de README's van [bringup](hrobot_bringup/README.md) en
[simulation](hrobot_simulation/README.md). Kaarten opslaan: zie bringup.

## Namespace van de robot (`robot_namespace`)

De Create 3 leeft in de namespace `/r2` (`/r2/odom`, `/r2/tf`, `/r2/cmd_vel`...) en de echte stack hangt eronder.
`robot_namespace:=<naam>` (standaard `$CREATE3_ROBOT_NAMESPACE` of `r2`; leeg = geen namespace).
Exporteer `CREATE3_ROBOT_NAMESPACE=r3` om hem vast te zetten; de namespace van de Create 3 zelf wijzig je
in zijn webinterface (Application > ROS 2). **De simulatie draait zonder namespace.**

## Externe afhankelijkheden

- ROS 2 Jazzy, `slam_toolbox`, Nav2 (`navigation2`: AMCL, map_server, controller/planner/bt_navigator), `robot_state_publisher`, `xacro`, `rviz2`, `teleop_twist_keyboard`,
  `nav2_common` (alleen `RewrittenYaml`/`ReplaceString` voor de launches).
- Simulatie: Gazebo Harmonic (`ros-jazzy-ros-gz`: `ros_gz_sim`, `ros_gz_bridge`).
- Driver van de RPLIDAR C1: `sllidar_ros2` (Slamtec), staat niet in rosdep: `src/scripts/setup_pi.sh` kloont hem in
  `src/sllidar_ros2` (genegeerd door git).
- Create 3 bereikbaar via het netwerk met dezelfde `ROS_DOMAIN_ID` en RMW als de RPi5.

