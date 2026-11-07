# TASK-009: Launch files de bringup (sensor, mapeo, navegación, RViz)

- **Rama Git:** `feat/TASK-009`
- **Depende de:** TASK-007, TASK-008
- **Prioridad:** P1

## Contexto y objetivo de negocio
Hoy no hay forma reproducible de arrancar el robot. Se necesitan launch files separados por modo de operación. En la práctica la RPi5 corre sin pantalla (headless), por lo que RViz se lanza en un PC aparte conectado a la misma red ROS 2.

## Alcance técnico
En `src/hrobot_bringup/launch/`:
- `sensors.launch.py`: driver del Youyeetoo FHL-LD19 (paquete externo; ver nota), con `lidar_port` (default `/dev/ttyUSB0`), `baudrate=230400`, `frame_id=laser`, topic `/scan` y `static_transform_publisher` `base_link → laser` con offsets como argumentos (`laser_x`, `laser_y`, `laser_z`, `laser_yaw`).
- `mapping.launch.py`: incluye `sensors.launch.py` y `slam_toolbox` (`online_async`) con `slam_toolbox_params.yaml`.
- `navigation.launch.py`: incluye `sensors.launch.py`, `nav2_bringup` (`localization` + `navigation`) con `nav2_params.yaml` y argumento `map` (ruta al `.yaml` del mapa, obligatorio).
- `rviz.launch.py`: lanza `rviz2` con `rviz/hrobot_nav.rviz`; solo debe usarse en un equipo con pantalla.
- Todos aceptan `use_sim_time` (default `false`). Las rutas a YAML/rviz se resuelven con `get_package_share_directory('hrobot_bringup')`, nunca rutas absolutas.
- `ROS_DOMAIN_ID` y RMW **no** se fijan en los launch (se configuran en el entorno).

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/launch/sensors.launch.py`
- `src/hrobot_bringup/launch/mapping.launch.py`
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/launch/rviz.launch.py`
- `src/hrobot_bringup/setup.py`
- `tests/launch/__init__.py`
- `tests/launch/test_launch_files.py`

**Nota sobre el driver (a verificar por un humano antes de la tarea):** la wiki de Youyeetoo usa `ldlidar_stl_ros` (flujo `roslaunch`, ROS 1); para ROS 2 existe la comunidad `Myzhar/ldrobot-lidar-ros2` (`ros2 launch ldlidar_node ldlidar.launch.py`, `lidar.model: LDLiDAR_LD19`), documentada para Humble: **comprobar compatibilidad con Jazzy**. Algunos usuarios reportan que el driver emite 0° y 360° repetidos y un número variable de lecturas por scan, lo que hace que `slam_toolbox` descarte scans. Si el driver elegido no es compatible, el Coder debe detenerse con `BLOCKED:` y no inventar nombres de paquete; la alternativa es un nodo propio en `adapters` basado en TASK-006.

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/launch/test_launch_files.py` termina con código 0 (el test **no requiere ROS instalado**: usa `ast` para análisis estático)
- [ ] `ruff check .`, `ruff format --check .` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/*.launch.py` termina con código 0
- [ ] El test verifica por análisis estático: cada archivo define `generate_launch_description`; los cuatro declaran `use_sim_time`; `navigation.launch.py` declara el argumento `map`; `sensors.launch.py` declara `lidar_port` y los offsets del láser; los archivos de configuración referenciados existen en `src/hrobot_bringup/{config,rviz}/`; ningún launch contiene rutas absolutas (`/home/`, `/Users/`) ni fija `ROS_DOMAIN_ID`
- [ ] `setup.py` instala `launch/`, `config/`, `rviz/` y `maps/` en `share/hrobot_bringup`
- [ ] (Opcional, si hay ROS 2) `ros2 launch hrobot_bringup navigation.launch.py --show-args` termina con código 0

## Fuera de alcance
Ejecutar con el robot real, ajuste de offsets del LiDAR (calibración humana), nodos de misión.
