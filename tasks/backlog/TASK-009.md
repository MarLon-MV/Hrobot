# TASK-009: Launch files de bringup (sensor, supervisor, mapeo, RViz)

- **Rama Git:** `feat/TASK-009`
- **Depende de:** TASK-008, TASK-014, TASK-016
- **Prioridad:** P1

## Contexto y objetivo de negocio
Hoy no hay forma reproducible de arrancar el robot. Se necesitan launch files separados por modo de operación, usando los **nodos propios** `ld19_node` (TASK-014) y `supervisor_node` (TASK-016), ya en `hrobot_core`. La RPi5 corre sin pantalla (headless): RViz se lanza en un PC aparte en la misma red ROS 2.

## Alcance técnico
En `src/hrobot_bringup/launch/`:
- `sensors.launch.py`: lanza `Node(package='hrobot_core', executable='ld19_node')` con parámetros `port` (argumento `lidar_port`, default `/dev/ttyUSB0`), `baudrate` 230400, `frame_id` `laser`, `scan_topic` `/scan` (nombres reales de los parámetros: ver `ld19_node.py::_declare_parameters`), más `static_transform_publisher` `base_link → laser` con argumentos `laser_x`, `laser_y`, `laser_z`, `laser_yaw` (default 0.0; calibración humana).
- `safety.launch.py`: lanza `Node(package='hrobot_core', executable='supervisor_node')`. Argumento `require_mission_status` (default `true`; con `true` el robot NO se mueve hasta que exista un nodo de misión que publique estado de navegación; `false` es modo banco, solo pruebas controladas, y el launch debe documentarlo en la descripción del argumento). No sobrescribir los demás parámetros de seguridad (se usan los defaults del nodo).
- `mapping.launch.py`: incluye `sensors.launch.py`, `safety.launch.py` y `slam_toolbox` (`online_async`) con `slam_toolbox_params.yaml`.
- `navigation.launch.py` NO forma parte de esta tarea (ver TASK-020); no crearlo ni mencionarlo como pendiente en los tests.
- `rviz.launch.py`: lanza `rviz2` con `rviz/hrobot_nav.rviz`; solo para un equipo con pantalla.
- Todos aceptan `use_sim_time` (default `false`). Rutas a YAML/rviz con `get_package_share_directory('hrobot_bringup')`, nunca absolutas.
- `ROS_DOMAIN_ID` y RMW **no** se fijan en los launch.
- `setup.py` de `hrobot_bringup`: instalar `launch/*.py`, `config/*`, `rviz/*`, `maps/*` con `glob` (sin depender de `.gitkeep`, que puede desaparecer cuando haya archivos reales); `maps/` puede quedar vacío sin romper el build.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/launch/sensors.launch.py`
- `src/hrobot_bringup/launch/safety.launch.py`
- `src/hrobot_bringup/launch/mapping.launch.py`
- `src/hrobot_bringup/launch/rviz.launch.py`
- `src/hrobot_bringup/setup.py`
- `tests/launch/__init__.py`
- `tests/launch/test_launch_files.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/launch/test_launch_files.py` termina con código 0 (el test **no requiere ROS instalado**: usa `ast` para análisis estático)
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/sensors.launch.py` termina con código 0 (y análogos para los otros tres launch, un comando por archivo)
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] El test verifica por análisis estático: cada archivo define `generate_launch_description`; los cuatro declaran `use_sim_time`; `sensors.launch.py` declara `lidar_port` y los cuatro offsets; `safety.launch.py` declara `require_mission_status` con default `'true'`; los launch referencian `ld19_node` y `supervisor_node`, cuyos `executable` existen como `console_scripts` en `hrobot_core/setup.py`; los parámetros pasados a `ld19_node` existen en sus defaults; los archivos de configuración referenciados existen; ningún launch contiene rutas absolutas (`/home/`, `/Users/`) ni fija `ROS_DOMAIN_ID`
- [ ] `setup.py` de `hrobot_bringup` instala `launch`, `config`, `rviz` y `maps` en `share/hrobot_bringup` (verificación estructural: extraer la lista de directorios de `setup.py` con `ast`, no solo buscar cadenas de texto)
- [ ] (Opcional, si hay ROS 2) `ros2 launch hrobot_bringup sensors.launch.py --show-args` termina con código 0

## Fuera de alcance
Ejecutar con el robot real, calibración de offsets del LiDAR, nodos de misión, `navigation.launch.py` (TASK-020).
