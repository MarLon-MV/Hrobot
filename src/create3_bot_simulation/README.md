# create3_bot_simulation

Simulación en **Gazebo Harmonic** (gz-sim 8, `ros_gz`, ROS 2 Jazzy) del Create 3 + LD19 para probar el mapeo con
teleop antes de usar el robot real. Solo `launch/`, `config/` y `worlds/`.

## Requisitos

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-common ros-jazzy-xacro
```

## Uso

```bash
colcon build --symlink-install --packages-select create3_bot_description create3_bot_bringup create3_bot_simulation
source install/setup.bash

ros2 launch create3_bot_simulation sim.launch.py                       # Gazebo + robot + slam + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard                   # otra terminal: i/j/l/,/k
```

Argumentos de `sim.launch.py`: `world` (por defecto `worlds/room.sdf`), `headless:=true` (Gazebo sin GUI),
`rviz:=false`, `x`, `y`, `yaw` (pose de spawn), `maps_dir`, `map`, `slam_lifecycle`.
Guardar/ampliar mapas: igual que en el robot real (ver bringup), sin namespace
(`/slam_toolbox/serialize_map`).

## Qué se simula

| Elemento | Cómo |
|---|---|
| Base | URDF `create3_ld19.urdf.xacro` con `sim:=true`: base, 2 ruedas (separación 0.233 m, radio 0.03575 m) y 1 caster delantero sin fricción. Visual con los meshes oficiales de iRobot (`create3_bot_description/meshes`). |
| Movimiento | plugin `gz-sim-diff-drive-system`: `cmd_vel` → `odom` + TF `odom → base_link` (frames `odom`/`base_link`). |
| LiDAR | `gpu_lidar` en `laser_frame`: 360°, 450 muestras, 10 Hz, 0.02–12 m, ruido gaussiano 1 cm. Publica `/scan`. |
| Puente | `config/gz_bridge.yaml`: `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan`. |
| Mundo | `worlds/room.sdf`: sala 8×8 m con tabique y puerta, cajas y un pilar. |

Todo corre con `use_sim_time:=true` y **sin namespace** (`/cmd_vel`, `/scan`, `/map`...). El robot real usa `/r2/...`
por defecto; para comparar, lanza el real con `robot_namespace:=''`.

## Diferencias sim vs real (para no sorprenderse)

- La odometría de Gazebo es casi perfecta; la del Create 3 real tiene deriva. slam_toolbox corrige con scan matching,
  pero ajusta con cuidado antes de fiarte del resultado simulado.
- El Create 3 real limita velocidades y tiene sensores de seguridad (acantilado, parachoques) que no se simulan.
- El LD19 real tiene ruido, reflejos y puntos perdidos que el sensor simulado no reproduce.
- Si `gpu_lidar` falla por no tener GPU/OpenGL, prueba `headless:=true` (usa `--headless-rendering`) o cambia el motor
  de render a `ogre` en `worlds/room.sdf`.

## Sin verificar

No se ha compilado ni lanzado (no hay ROS 2 en la máquina de desarrollo). Puntos a comprobar al primer arranque:
nombres de tópicos del puente (`gz topic -l`), que `<gz_frame_id>` deje `frame_id: laser_frame` en `/scan`, que el
robot no se hunda ni derrape en el suelo, y que el plugin `JointStatePublisher` publique las ruedas.
