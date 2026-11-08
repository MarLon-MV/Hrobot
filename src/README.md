# Workspace ROS 2 (`src/`)

Robot móvil autónomo (AMR) para reparto de medicación en hospital: **iRobot Create 3** (base diferencial) + **Raspberry Pi 5** + **LiDAR 2D Youyeetoo FHL-LD19**. ROS 2 **Jazzy**, Python (`rclpy`). Navegación con Nav2 + AMCL sobre mapa estático; mapeo con `slam_toolbox`.

> Estado: la lógica pura está probada con tests (571), pero **nada se ha ejecutado aún en ROS real ni en el robot**. Ver [`../pendientes/`](../pendientes/README.md).

## Paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| [`hrobot_interfaces`](hrobot_interfaces/) | ament_cmake | Mensajes propios. Hoy: `msg/MissionStatus.msg` (`mission_id`, `state`, `delivered`). |
| [`hrobot_core`](hrobot_core/) | ament_python | Código del robot, en tres capas (abajo). |
| [`hrobot_bringup`](hrobot_bringup/) | ament_python | Solo `launch/`, `config/`, `rviz/`, `maps/`. Sin lógica. |

### Capas de `hrobot_core`

| Capa | Reglas | Módulos |
|---|---|---|
| `domain/` | Python puro: sin ROS, sin I/O, sin importar `services`/`adapters`. | `geometry`, `safety` (factor de velocidad según LiDAR), `mission` (máquina de estados) |
| `services/` | Lógica de aplicación, también pura. | `safety_supervisor` (limita la velocidad: rampa, sin marcha atrás, parada de emergencia) |
| `adapters/` | **Único** sitio que importa `rclpy`, `serial` o mensajes ROS. | `ld19_parser`, `scan_builder` (puros), `ld19_node`, `supervisor_node`, `names` (tópicos y frames) |

## Flujo de datos y seguridad

```
LD19 (UART 230400) → ld19_node → /scan ─┬─→ AMCL / slam_toolbox / costmaps (Nav2)
                                         └─→ supervisor_node
Nav2 (controller → velocity_smoother) → /cmd_vel_nav → supervisor_node → /cmd_vel → Create 3
/hrobot/mission_status (estado de misión) ────────────→ supervisor_node
```

Reglas que no se rompen:
- **Solo `supervisor_node` publica `/cmd_vel`.** Nav2 publica en `/cmd_vel_nav`; `navigation.launch.py` rechaza `nav_cmd_vel_topic:=/cmd_vel`.
- El supervisor mueve el robot solo en `NAVIGATING_TO_DESTINATION` / `RETURNING_TO_BASE` (modo banco: `require_mission_status:=false`, solo para pruebas controladas). Sin scan válido o con scan viejo (>0,5 s) la velocidad es 0. Parada a 0,35 m, reducción hasta 1,0 m. Nunca marcha atrás.
- `EMERGENCY_STOP` queda enclavado: solo una orden `RESET` lo libera.
- Nav2 sin recuperaciones (`BackUp`/`Spin`/`Wait`) y sin `behavior_server`: si falla, la misión pasa a `NAV_FAILED`.

## Configuración (`hrobot_bringup`)

| Archivo | Para qué |
|---|---|
| `config/nav2_params.yaml` | Parámetros de Nav2 (AMCL, Regulated Pure Pursuit, costmaps, velocity_smoother). Muchos valores llevan `# TODO(verify)`. |
| `config/slam_toolbox_params.yaml` | Mapeo online asíncrono. |
| `config/bt_navigate_no_recovery.xml` | Árbol de comportamiento sin recuperaciones; lo inyecta `navigation.launch.py`. |
| `rviz/hrobot_nav.rviz` | Vista de supervisión (TF, scan, mapa, costmaps, partículas AMCL, planes). |
| `launch/` | `sensors`, `safety`, `mapping`, `navigation`, `rviz` (ver abajo). |

## Cómo se usa

```bash
# Compilar (en la RPi5 / equipo con ROS 2 Jazzy)
colcon build --packages-select hrobot_interfaces hrobot_core hrobot_bringup
source install/setup.bash

# Solo sensor + supervisor
ros2 launch hrobot_bringup sensors.launch.py lidar_port:=/dev/ttyUSB0
ros2 launch hrobot_bringup safety.launch.py

# Mapear
ros2 launch hrobot_bringup mapping.launch.py

# Navegar sobre un mapa
ros2 launch hrobot_bringup navigation.launch.py map:=/ruta/al/mapa.yaml

# En un PC con pantalla, misma red ROS 2 (la RPi5 va sin pantalla)
ros2 launch hrobot_bringup rviz.launch.py
```

`ROS_DOMAIN_ID` y el RMW **no** se fijan en los launch: se configuran en el entorno. Offsets del LiDAR respecto a `base_link`: argumentos `laser_x/y/z/yaw` de `sensors.launch.py` (calibración humana).

## Tests y calidad (desde la raíz del repo, sin ROS)

```bash
PYTHONPATH=src/hrobot_core pytest -q
ruff check src tests && ruff format --check src tests
mypy --strict src/hrobot_core/hrobot_core
```

Los tests usan dobles de `rclpy`/`serial` (`tests/ros_fakes.py`): **no demuestran** compatibilidad real de QoS, tipos de mensaje ni comportamiento ante SIGINT.

## Más información
- Convenciones técnicas: [`../.claude/architecture_rules.md`](../.claude/architecture_rules.md)
- Flujo multiagente y tareas: [`../CLAUDE.md`](../CLAUDE.md), `../tasks/`
- Lo que falta por hacer: [`../pendientes/README.md`](../pendientes/README.md)
