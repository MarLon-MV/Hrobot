# hrobot_simulation

Simulatie in **Gazebo Harmonic** (gz-sim 8, `ros_gz`, ROS 2 Jazzy) van de Create 3 + RPLIDAR C1 om het kaarten met
teleop te testen voordat de echte robot wordt gebruikt. Alleen `launch/`, `config/` en `worlds/`.

## Vereisten

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-common ros-jazzy-navigation2 ros-jazzy-xacro
```

## Gebruik

```bash
colcon build --symlink-install --packages-select hrobot_description hrobot_bringup hrobot_simulation
source install/setup.bash

ros2 launch hrobot_simulation sim.launch.py                       # Gazebo + robot + slam + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard                   # andere terminal: i/j/l/,/k
```

Argumenten van `sim.launch.py`: `world` (standaard `worlds/room.sdf`), `headless:=true` (Gazebo zonder GUI),
`rviz:=false`, `x`, `y`, `yaw` (spawn-pose), `maps_dir`, `map`, `slam_lifecycle`.
Autonome navigatie: `nav:=true map_yaml:=<absoluut pad .yaml>` start map_server + AMCL + Nav2 in plaats van
slam_toolbox; AMCL start op de spawn-pose (`x`/`y`/`yaw`), die moet overeenkomen met de oorsprong van de opgeslagen kaart
(een kaart gemaakt met de standaard spawn begint op 0,0,0). Stuur doelen met "2D Goal Pose" van RViz.
Kaarten opslaan/uitbreiden: net als op de echte robot (zie bringup), zonder namespace
(`/slam_toolbox/serialize_map`).

## Wat er wordt gesimuleerd

| Element | Hoe |
|---|---|
| Basis | URDF `hrobot_rplidar_c1.urdf.xacro` met `sim:=true`: basis, 2 wielen (afstand 0,233 m, straal 0,03575 m) en 1 wrijvingsloos zwenkwiel vooraan. Visueel met de officiële meshes van iRobot (`hrobot_description/meshes`). |
| Beweging | plugin `gz-sim-diff-drive-system`: `cmd_vel` → `odom` + TF `odom → base_link` (frames `odom`/`base_link`). |
| LiDAR | `gpu_lidar` in `laser_frame`: 360°, 360 samples, 10 Hz, 0,05–12 m, Gauss-ruis 1 cm. Publiceert `/scan`. |
| Brug | `config/gz_bridge.yaml`: `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan`. |
| Wereld | `worlds/room.sdf`: kamer van 8×8 m met tussenwand en deuropening, dozen en een pilaar. |

Alles draait met `use_sim_time:=true` en **zonder namespace** (`/cmd_vel`, `/scan`, `/map`...). De echte robot gebruikt standaard `/r2/...`;
om te vergelijken, start je de echte met `robot_namespace:=''`.

## Verschillen sim vs echt (om niet verrast te worden)

- De odometrie van Gazebo is bijna perfect; die van de echte Create 3 heeft drift. slam_toolbox corrigeert met scan matching,
  maar stel zorgvuldig af voordat je het gesimuleerde resultaat vertrouwt.
- De echte Create 3 beperkt snelheden en heeft veiligheidssensoren (afgrond, bumper) die niet worden gesimuleerd.
- De echte RPLIDAR C1 heeft ruis, reflecties en verloren punten (vooral bij zwarte of spiegelende oppervlakken) die de gesimuleerde sensor niet reproduceert.
- Als `gpu_lidar` faalt omdat er geen GPU/OpenGL is, probeer `headless:=true` (gebruikt `--headless-rendering`) of wijzig de
  rendering-engine naar `ogre` in `worlds/room.sdf`.

## Niet geverifieerd

Niet gebouwd of gestart (er is geen ROS 2 op de ontwikkelmachine). Punten om bij de eerste start te controleren:
namen van de brugtopics (`gz topic -l`), dat `<gz_frame_id>` `frame_id: laser_frame` op `/scan` laat staan, dat de
robot niet wegzakt of slipt op de vloer, en dat de `JointStatePublisher`-plugin de wielen publiceert.
