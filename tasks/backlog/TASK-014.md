# TASK-014: Nodo ROS 2 del LiDAR LD19 (lectura serie → `/scan`)

- **Rama Git:** `feat/TASK-014`
- **Depende de:** TASK-012, TASK-013
- **Prioridad:** P1

## Contexto y objetivo de negocio
Nodo propio que lee el UART del LD19, decodifica con `Ld19Parser` (TASK-006), construye scans de rejilla fija con `ScanBuilder` (TASK-013) y publica `sensor_msgs/LaserScan` en `/scan`. Es la alternativa al driver de la comunidad (cuya compatibilidad con Jazzy y cuyos scans de longitud variable no están verificados). El nodo es una capa **delgada**: toda la lógica de decodificación y de rejilla ya está probada en los módulos puros.

## Alcance técnico
Crear `src/hrobot_core/hrobot_core/adapters/ld19_node.py` con la clase `Ld19Node(Node)` y la función `main(args=None)`.

- **Parámetros** (todos con `declare_parameter`, tipo y valor por defecto; usar las constantes de `adapters/names.py`):

| Parámetro | Defecto | Descripción |
|---|---|---|
| `port` | `LD19_DEFAULT_PORT` | puerto serie |
| `baudrate` | `LD19_BAUDRATE` | 230400 |
| `frame_id` | `FRAME_LASER` | frame del mensaje |
| `scan_topic` | `SCAN_TOPIC` | tópico de salida |
| `bins` | 360 | rayos por scan |
| `range_min` | 0.02 | m |
| `range_max` | 12.0 | m |
| `read_period_s` | 0.01 | periodo del timer de lectura |
| `reconnect_period_s` | 2.0 | espera entre reintentos de apertura |
| `stats_period_s` | 10.0 | periodo del log de estadísticas |

- **Publicación:** `create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)`. Cada `ScanFrame` se convierte a `LaserScan`: `header.frame_id = frame_id`; `header.stamp` = (instante de cierre de la vuelta − `scan_time_s`), es decir, el inicio de la vuelta, en reloj del nodo; `angle_min/angle_max/angle_increment/range_min/range_max/scan_time` copiados; `time_increment = scan_time_s / bins`; `ranges` e `intensities` como listas de `float`. El mensaje siempre tiene `bins` lecturas.
- **Lectura:** un timer cada `read_period_s` llama a `serial.read(4096)` (puerto abierto con `timeout=0`, no bloqueante), pasa los bytes a `parser.feed`, cada paquete a `builder.add_packet` y publica los scans completos. **Prohibido** `time.sleep` y cualquier bucle bloqueante dentro de callbacks.
- **Robustez serie:**
  - Si la apertura falla (`serial.SerialException` u `OSError`): log de error (una vez por intento), no lanzar excepción, reintentar tras `reconnect_period_s` con un timer.
  - Si `read` lanza excepción durante la operación: cerrar el puerto, reiniciar `parser` y `builder` (`builder.reset()`, parser nuevo), loguear, y volver al modo de reintento.
  - Si no llegan bytes en `3 × reconnect_period_s`: log de advertencia (una vez hasta que vuelvan datos).
- **Estadísticas:** cada `stats_period_s` loguear `crc_errors`, `dropped_revolutions` y número de scans publicados en ese periodo.
- **Parámetros inválidos** (p. ej. `bins` fuera de rango): `ScanBuilderConfig` lanza `ValueError`; el nodo **no** lo captura: debe fallar al arrancar.
- `main`: `rclpy.init`, crear nodo, `rclpy.spin`, y en `finally` destruir el nodo, cerrar el puerto y `rclpy.shutdown` si `rclpy.ok()`.
- Registrar el ejecutable en `src/hrobot_core/setup.py`: `"ld19_node = hrobot_core.adapters.ld19_node:main"`, y añadir `<exec_depend>python3-serial</exec_depend>` a `src/hrobot_core/package.xml`.
- No publica en `/cmd_vel` ni en ningún otro tópico de movimiento.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/ld19_node.py`
- `src/hrobot_core/setup.py`
- `src/hrobot_core/package.xml`
- `tests/adapters/test_ld19_node.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/adapters/test_ld19_node.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/adapters/test_ld19_node.py --cov=hrobot_core.adapters.ld19_node --cov-fail-under=90 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Los tests usan el fixture `ros_fakes` de TASK-012 (sin ROS real) y verifican: con un flujo serie sintético de varias vueltas completas se publican `LaserScan` y **todos** tienen `len(ranges) == bins` (incluyendo vueltas con distinto número de puntos); `frame_id`, `angle_min`, `angle_increment` y `range_max` correctos; `header.stamp` coincide con el inicio de la vuelta; el publisher usa `qos_profile_sensor_data` y el tópico `scan_topic`
- [ ] Test de fallo de apertura: no se lanza excepción, se registra un error y se reintenta tras `reconnect_period_s` (usando `advance` del reloj falso); tras reabrir con éxito se publican scans
- [ ] Test de fallo de lectura a mitad de flujo: el puerto se cierra, el estado del parser/builder se reinicia (una trama partida anterior al fallo no contamina el scan posterior) y se reintenta
- [ ] Test de advertencia por falta de datos (una sola vez) y de log de estadísticas con `crc_errors` tras inyectar una trama con CRC corrupto
- [ ] Test de parámetro inválido (`bins = 10`) ⇒ la construcción del nodo lanza `ValueError`
- [ ] Test estático (`ast`) sobre `ld19_node.py`: sin `time.sleep`, sin `print(`, sin `create_publisher` hacia `/cmd_vel`
- [ ] `grep -q "ld19_node" src/hrobot_core/setup.py` y `grep -q "python3-serial" src/hrobot_core/package.xml` terminan con código 0

## Verificación humana (fuera del DoD)
- En la RPi5 con el FHL-LD19 conectado: `ros2 run hrobot_core ld19_node --ros-args -p port:=/dev/ttyUSB0`; comprobar `ros2 topic hz /scan` ≈ 10 Hz, `ros2 topic echo /scan --once` con 360 lecturas, y la visualización en RViz (sin espejado, obstáculo a la derecha a la derecha).
- Permisos del dispositivo (`dialout`/regla udev) y que el nombre del puerto es estable.
- Confirmar que la API real de `rclpy` y `pyserial` coincide con lo que simulan los fakes.

## Fuera de alcance
Launch files (TASK-009), TF estático `base_link → laser`, control PWM/velocidad del motor del LD19, filtrado de datos, diagnósticos ROS (`diagnostic_msgs`).
