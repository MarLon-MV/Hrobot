# TASK-016: Nodo ROS 2 del supervisor de seguridad (único publicador de `/cmd_vel`)

- **Rama Git:** `feat/TASK-016`
- **Depende de:** TASK-012, TASK-015
- **Prioridad:** P0

## Contexto y objetivo de negocio
Envuelve `SafetySupervisor` (TASK-015) en un nodo `rclpy`. Es el **único** nodo que publica `/cmd_vel`: recibe las órdenes de Nav2 en `/cmd_vel_nav`, el scan en `/scan` y el estado de misión, y publica a ritmo fijo la velocidad ya limitada. Publicar a ritmo fijo (incluso ceros) garantiza que, si algo deja de llegar, el robot recibe órdenes explícitas de parada en pocas decenas de milisegundos. La capa es delgada: la decisión está en el servicio puro.

## Alcance técnico
Crear `src/hrobot_core/hrobot_core/adapters/supervisor_node.py` con la clase `SupervisorNode(Node)` y `main(args=None)`.

- **Parámetros** (`declare_parameter` con tipo y defecto; nombres y defectos iguales a `SupervisorConfig` y `SafetyConfig`): `max_linear`, `max_angular`, `cmd_timeout_s`, `mission_timeout_s`, `require_mission_status`, `max_rise_per_s`, `stop_distance`, `slow_distance`, `front_half_angle`, `stale_timeout`, `min_valid_ratio`, `degraded_factor`; más `publish_rate_hz` (defecto 20.0, debe estar en `(0, 100]`). Los parámetros inválidos hacen que el constructor lance `ValueError` (el nodo **falla al arrancar**; nunca se sustituyen silenciosamente por valores por defecto).
- **Suscripciones** (constantes de `adapters/names.py`):
  - `NAV_CMD_VEL_TOPIC`, `Twist`, QoS fiable de profundidad 1 ⇒ `supervisor.on_command(VelocityCommand(linear.x, angular.z), now)`.
  - `SCAN_TOPIC`, `LaserScan`, `qos_profile_sensor_data` ⇒ `supervisor.on_scan(list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, now)`.
  - `MISSION_STATUS_TOPIC`, `hrobot_interfaces.msg.MissionStatus`, QoS fiable + `TRANSIENT_LOCAL` de profundidad 1 (para recibir el último estado al arrancar; el publicador de misión debe usar la misma QoS) ⇒ se convierte `msg.state` a `MissionState` por nombre; un nombre desconocido se trata como `None` (equivale a "sin estado válido") con un log de advertencia.
- **Publicación:** `create_publisher(Twist, CMD_VEL_TOPIC, <QoS fiable profundidad 1>)`. Un timer a `publish_rate_hz` llama a `supervisor.step(now)` y publica **siempre** un `Twist` (solo `linear.x` y `angular.z` distintos de cero; el resto, 0.0). Si el timer no recibe nada nuevo, igualmente publica (con ceros cuando corresponda).
- **Reloj:** `now_s = get_clock().now().nanoseconds / 1e9`.
- **Logs:** registrar el `StopReason` solo cuando **cambia** (no en cada tick) y a nivel `warn` cuando pasa a un motivo distinto de `NONE`/`RAMP_LIMITED`; registrar `reverse_blocked` cuando se activa.
- **Parada al cerrar:** en `main`, en el bloque `finally`, publicar un `Twist` de ceros antes de destruir el nodo y de llamar a `rclpy.shutdown()` (si `rclpy.ok()`).
- Es el **único** módulo del paquete que crea un publisher hacia `CMD_VEL_TOPIC` (lo comprueba `tests/test_architecture.py` de TASK-012).
- Registrar el ejecutable en `src/hrobot_core/setup.py`: `"supervisor_node = hrobot_core.adapters.supervisor_node:main"`.
- Sin `time.sleep`, sin `print`.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/supervisor_node.py`
- `src/hrobot_core/setup.py`
- `tests/adapters/test_supervisor_node.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/adapters/test_supervisor_node.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/adapters/test_supervisor_node.py --cov=hrobot_core.adapters.supervisor_node --cov-fail-under=90 -q` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/test_architecture.py` termina con código 0 (incluye la regla de único publicador de `/cmd_vel`)
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Tests con el fixture `ros_fakes` (sin ROS real):
  - Sin ninguna entrada recibida, el timer publica `Twist` con ceros en cada tick (y se publica a la frecuencia configurada)
  - Flujo feliz: estado de misión `NAVIGATING_TO_DESTINATION`, orden `(0.2, 0.5)` en `/cmd_vel_nav`, scan libre; tras varios ticks la salida crece de forma progresiva hasta `(0.2, 0.5)`
  - Obstáculo a `stop_distance` en el scan ⇒ el siguiente tick publica ceros
  - Orden de Nav2 que deja de llegar (avanzando el reloj falso más de `cmd_timeout_s`) ⇒ ceros
  - Estado `EMERGENCY_STOP` ⇒ ceros aunque lleguen órdenes y scans válidos; al llegar `NAVIGATING_TO_DESTINATION` vuelve a moverse con rampa
  - Estado de misión con nombre desconocido ⇒ trata como sin estado válido y se loguea advertencia
  - Marcha atrás pedida (`linear.x < 0`) ⇒ `linear.x == 0.0` publicado
  - Parámetro inválido (`max_linear = -1.0`, `publish_rate_hz = 0.0` o `500.0`) ⇒ `ValueError` al construir el nodo
  - Parámetros sobreescritos (`max_linear = 0.1`) ⇒ se respetan en la salida
  - `main` publica ceros en el `finally` al terminar (simulando excepción durante `spin`)
  - El `StopReason` se loguea solo al cambiar, no en cada tick
  - Tipos de mensaje (los mensajes reales de `rclpy` rechazan tipos incorrectos con `AssertionError`, los fakes no): todo `Twist` publicado tiene sus seis campos (`linear.x/y/z`, `angular.x/y/z`) de tipo `float` exacto (también los ceros: `0.0`, nunca `0`)
  - Exactamente **un** publisher a `/cmd_vel` y las QoS de las tres suscripciones son las especificadas (incluida `TRANSIENT_LOCAL` para el estado de misión)
- [ ] Test estático (`ast`) de `supervisor_node.py`: sin `time.sleep`, sin `print(`, exactamente un `create_publisher` hacia `CMD_VEL_TOPIC`
- [ ] `grep -q "supervisor_node" src/hrobot_core/setup.py` termina con código 0

## Verificación humana (fuera del DoD)
- Revisión humana obligatoria del diff completo antes del merge.
- Con ROS 2 Jazzy: `ros2 topic hz /cmd_vel` ≈ `publish_rate_hz`; con el robot **elevado**, comprobar que `ros2 topic pub /cmd_vel_nav ...` mueve las ruedas solo con estado de misión de movimiento y scan válido, y que cortar cualquiera de las tres entradas detiene el robot en < 0.5 s.
- Comprobar que **ningún otro** nodo publica en `/cmd_vel` (`ros2 topic info /cmd_vel -v` debe mostrar un único publicador) y que Nav2 está remapeado a `/cmd_vel_nav` (TASK-009).
- Confirmar que la API real de `rclpy` coincide con los fakes y que el tipo de mensaje que espera el Create 3 en `/cmd_vel` (`Twist` o `TwistStamped`) coincide con el publicado.
- Confirmar cómo se comporta el Create 3 ante la falta de órdenes (su propio tiempo de espera) como segunda barrera.

## Fuera de alcance
Nodo de misión que publica `MissionStatus`, launch files (TASK-009), lectura de odometría, diagnósticos (`diagnostic_msgs`), botón físico de emergencia.
