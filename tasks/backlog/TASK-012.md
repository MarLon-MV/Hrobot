# TASK-012: Infraestructura para nodos ROS 2 testeables sin ROS instalado

- **Rama Git:** `feat/TASK-012`
- **Depende de:** TASK-002
- **Prioridad:** P1

## Contexto y objetivo de negocio
Los agentes y el equipo de desarrollo trabajan en máquinas sin ROS 2 (macOS). `mypy --strict` falla al importar `rclpy`, y `pytest` no puede importar módulos que lo usen. Antes de escribir nodos (LiDAR, supervisor de seguridad, misión) hace falta: (1) configuración de `mypy` que tolere los imports de ROS ausentes, (2) constantes únicas de tópicos y frames, (3) un entorno de **fakes** que simule la API mínima de ROS y de `pyserial` para probar la lógica de los nodos, y (4) tests estáticos que hagan cumplir la arquitectura (capas y la regla de que solo el supervisor publica `/cmd_vel`).

**Límite honesto de los fakes:** simulan *nuestro uso* de la API, no la API real. Un nodo puede pasar todos sus tests con fakes y fallar en ROS real. Los fakes no sustituyen la verificación humana en un equipo con ROS 2 Jazzy.

## Alcance técnico

### 1. `pyproject.toml`
Añadir a la configuración de mypy:
```toml
[[tool.mypy.overrides]]
module = ["rclpy", "rclpy.*", "sensor_msgs", "sensor_msgs.*", "geometry_msgs", "geometry_msgs.*",
          "std_msgs", "std_msgs.*", "nav2_msgs", "nav2_msgs.*", "hrobot_interfaces", "hrobot_interfaces.*",
          "serial", "serial.*"]
ignore_missing_imports = true

[[tool.mypy.overrides]]
module = ["hrobot_core.adapters.*_node"]
disallow_subclassing_any = false
```
No se relaja `strict` en ningún otro módulo. No se tocan las demás secciones.

### 2. `src/hrobot_core/hrobot_core/adapters/names.py` (solo constantes, sin imports de ROS)
```python
SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"               # solo lo publica supervisor_node
NAV_CMD_VEL_TOPIC = "/cmd_vel_nav"       # salida de Nav2, entrada del supervisor
MISSION_STATUS_TOPIC = "/hrobot/mission_status"
FRAME_MAP = "map"; FRAME_ODOM = "odom"; FRAME_BASE = "base_link"; FRAME_LASER = "laser"
LD19_DEFAULT_PORT = "/dev/ttyUSB0"
LD19_BAUDRATE = 230400
```

### 3. Fakes en `tests/ros_fakes.py` + fixture en `tests/conftest.py`
Un fixture `ros_fakes` (alcance función) que instala módulos falsos en `sys.modules` (con `monkeypatch.setitem`) y **purga de `sys.modules` cualquier módulo `hrobot_core.adapters.*_node` antes y después del test**, para que cada test importe el nodo con los fakes activos y nunca quede un módulo cacheado con fakes viejos. Devuelve un objeto `FakeRos` con helpers. API mínima a simular (solo lo que usarán los nodos de TASK-014/016):
- `rclpy`: `init(args=None)`, `shutdown()`, `spin(node)` (no bloquea: ejecuta los timers pendientes una vez), `ok()`.
- `rclpy.node.Node`: `__init__(node_name)`; `declare_parameter(name, default)` y `get_parameter(name)` (devuelve objeto con `.value`; los valores se pueden sobreescribir desde el test antes de crear el nodo con `FakeRos.set_params({...})`); `create_publisher(msg_type, topic, qos)` ⇒ `FakePublisher` con lista `.published`; `create_subscription(msg_type, topic, callback, qos)`; `create_timer(period_s, callback)` ⇒ `FakeTimer` con `.cancel()`; `get_logger()` ⇒ logger que registra `(nivel, mensaje)` en `.records`; `get_clock().now()` ⇒ objeto con `.nanoseconds` y `.to_msg()`; `destroy_node()`.
- `rclpy.qos`: `qos_profile_sensor_data`, `QoSProfile(depth=..., reliability=..., durability=...)`, `ReliabilityPolicy`, `DurabilityPolicy`.
- Mensajes con los campos que usaremos: `sensor_msgs.msg.LaserScan` (`header.stamp`, `header.frame_id`, `angle_min`, `angle_max`, `angle_increment`, `time_increment`, `scan_time`, `range_min`, `range_max`, `ranges`, `intensities`), `geometry_msgs.msg.Twist` (`linear.x/y/z`, `angular.x/y/z`), `hrobot_interfaces.msg.MissionStatus` (`mission_id`, `state`, `delivered`).
- `serial.Serial(port, baudrate, timeout=0)`: `read(n)` devuelve bytes de una cola configurable, `in_waiting`, `is_open`, `close()`; permite inyectar fallos (`SerialException` al abrir o al leer) y datos desde el test. `serial.SerialException`.
- Helpers de `FakeRos`: `advance(seconds)` (avanza el reloj falso), `fire_timers()` (ejecuta timers vencidos), `deliver(topic, msg)` (entrega a las suscripciones de ese tópico), `published(topic)` (mensajes publicados), `logs(level=None)`.

### 4. Tests estáticos `tests/test_architecture.py`
Usando `ast` (sin importar los módulos), sobre `src/hrobot_core/hrobot_core/`:
- `domain/` y `services/` no importan `rclpy`, `serial`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `nav2_msgs`, `hrobot_interfaces`.
- En `adapters/`, solo los archivos `*_node.py` pueden importar `rclpy` y mensajes ROS; solo `*_node.py` puede importar `serial`.
- Solo `adapters/supervisor_node.py` puede crear un publisher cuyo tópico sea `CMD_VEL_TOPIC` (nombre) o el literal `"/cmd_vel"`; ningún otro archivo del paquete.
- Ningún archivo del paquete usa `time.sleep` ni `print(`.
- Cada detector se prueba también con **fuentes sintéticas que lo violan** (cadenas de código en el test) para demostrar que detecta la violación, no solo que hoy no hay ninguna.

## Archivos permitidos de modificar/crear
- `pyproject.toml`
- `src/hrobot_core/hrobot_core/adapters/names.py`
- `tests/conftest.py`
- `tests/ros_fakes.py`
- `tests/test_ros_fakes.py`
- `tests/test_architecture.py`
- `tests/adapters/test_names.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/test_ros_fakes.py tests/test_architecture.py tests/adapters/test_names.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` (suite completa) termina con código 0 sin romper tests existentes
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] `tests/test_ros_fakes.py` prueba los fakes: parámetros con valor por defecto y sobreescritos, publisher que registra mensajes, suscripción que recibe `deliver`, timer que se dispara según el reloj falso, logger que registra niveles, `Serial` con lectura en cola y con fallo inyectado, y que el fixture purga los módulos `*_node` entre tests (importar un módulo ficticio `*_node` de prueba en un test y comprobar que no sobrevive al siguiente)
- [ ] `tests/adapters/test_names.py` comprueba los valores exactos de las constantes y que `CMD_VEL_TOPIC != NAV_CMD_VEL_TOPIC`
- [ ] `tests/test_architecture.py` pasa sobre el código actual y, para cada una de sus reglas, incluye un caso sintético que viola la regla y es detectado
- [ ] `grep -nE "^\[\[tool.mypy.overrides\]\]" pyproject.toml` muestra al menos 2 coincidencias

## Verificación humana (fuera del DoD)
- En un equipo con ROS 2 Jazzy, ejecutar `mypy --strict src/hrobot_core/hrobot_core` con ROS cargado: con ROS presente los overrides de `ignore_missing_imports` ya no aplican y pueden aparecer errores de tipos reales que no se ven en macOS.
- Contrastar que la API que simulan los fakes (firmas de `create_publisher`, `declare_parameter`, etc.) coincide con `rclpy` de Jazzy.

## Fuera de alcance
Los nodos en sí, launch files, `setup.py`/`package.xml`, simulación completa de ROS (grafos, DDS, ciclo de vida).
