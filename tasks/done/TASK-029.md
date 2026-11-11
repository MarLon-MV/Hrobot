# TASK-029: Condiciones de seguridad del Create 3 en la exploración (dock, secuestro, batería, peligros)

- **Rama Git:** `feat/TASK-029`
- **Depende de:** TASK-028
- **Prioridad:** P0

## Contexto y objetivo de negocio
El Create 3 expone estado propio relevante para explorar sin riesgo: `dock_status`, `kidnap_status`, `battery_state`, `hazard_detection` y la acción `undock`. Sin usarlos, el robot podría intentar moverse estando en la base, seguir mapeando tras ser levantado (corrompe el SLAM), quedarse sin batería o repetir un objetivo que provoca choques. Esta tarea extiende el gestor y el nodo de TASK-027 con esas condiciones. Tópicos y acciones se resuelven bajo `<create3_namespace>` (por defecto `/r2`).

## Alcance técnico
1. **`services/exploration_manager.py`**: nuevos estados `UNDOCKING` y `PAUSED` (`IDLE -> UNDOCKING -> EXPLORING <-> PAUSED -> DONE | FAILED`), puerto `DockPort.undock()` y tipo inmutable `RobotStatus` (`docked: bool | None`, `kidnapped: bool | None`, `battery_pct: float | None`; `None` = dato desconocido). `step(grid, robot, status)` pasa a recibir `RobotStatus` (actualizar los tests de TASK-027). Reglas:
   - **Dock:** en `start`, si `docked` es `True` ⇒ `UNDOCKING` y `DockPort.undock()`; no se envía objetivo hasta confirmar `docked == False`. Si `undock` falla o excede `undock_timeout_s` ⇒ `FAILED`. Si `docked` es `None`, no se explora: espera hasta `status_timeout_s` y luego `FAILED`.
   - **Secuestro:** `kidnapped == True` en `EXPLORING` ⇒ cancela el objetivo y pasa a `PAUSED` (sin contar fallos ni lista negra). Reanuda solo tras `kidnap_clear_cycles` ciclos consecutivos con `kidnapped == False` (default 3). `kidnapped == None` se trata como secuestro (conservador).
   - **Batería:** `battery_pct < min_battery_pct` (default 20.0) ⇒ cancela el objetivo, guarda el mapa parcial y termina en `DONE` con motivo `LOW_BATTERY`. Dato `None`/NaN/inf = desconocido: advertencia, sin abortar.
   - **Peligros:** `on_hazard(Point2D | None)` recibe la pose aproximada del contacto. Añade el punto a la lista negra, cancela el objetivo en curso y cuenta como fallo consecutivo.
2. **`adapters/exploration_node.py`**: con tipos de `irobot_create_msgs` (solo en esta capa) y tópicos bajo `<create3_namespace>`:
   - `dock_status` (`is_docked`), `kidnap_status` (`is_kidnapped`) y `battery_state` (`percentage` 0..1 convertido a %), con `SensorDataQoS` (best effort), agregados en un `RobotStatus`.
   - `hazard_detection` (`HazardDetectionVector`): ante `BUMP`, `CLIFF` o `STUCK` llama a `on_hazard` con la pose actual; ignora `BACKUP_LIMIT`, `VIRTUAL_WALL` y `OBJECT_PROXIMITY`.
   - Action client `undock` (`irobot_create_msgs/action/Undock`) que implementa `DockPort`.
   - Parámetros nuevos declarados: `create3_namespace`, `undock_timeout_s`, `status_timeout_s`, `kidnap_clear_cycles`, `min_battery_pct`. El estado publicado incluye el motivo de fin.
3. **`adapters/names.py`**: `DOCK_STATUS_TOPIC`, `KIDNAP_STATUS_TOPIC`, `BATTERY_STATE_TOPIC`, `HAZARD_DETECTION_TOPIC`, `UNDOCK_ACTION` (se resuelven con `namespaced`).
4. **`package.xml` de `hrobot_core`**: `exec_depend` de `irobot_create_msgs` y `sensor_msgs`.
5. **`exploration_params.yaml`**: los parámetros nuevos con tipos y rangos válidos.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/services/exploration_manager.py`
- `src/hrobot_core/hrobot_core/adapters/exploration_node.py`
- `src/hrobot_core/hrobot_core/adapters/names.py`
- `src/hrobot_core/package.xml`
- `src/hrobot_bringup/config/exploration_params.yaml`
- `tests/services/test_exploration_manager.py`
- `tests/adapters/test_exploration_node.py`
- `tests/config/test_exploration_config.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q --cov=hrobot_core --cov-fail-under=85` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/services/test_exploration_manager.py --cov=hrobot_core.services.exploration_manager --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Tests de dock: `docked=True` ⇒ `UNDOCKING` y ningún objetivo hasta `docked=False`; `undock` fallido o con timeout ⇒ `FAILED`; `docked=None` más allá de `status_timeout_s` ⇒ `FAILED`
- [ ] Tests de secuestro: cancela y pasa a `PAUSED` sin contar fallos; reanuda solo tras `kidnap_clear_cycles` ciclos limpios consecutivos; un ciclo con `True` (o `None`) reinicia la cuenta
- [ ] Tests de batería: bajo el umbral ⇒ guarda mapa parcial y `DONE(LOW_BATTERY)`; exactamente igual al umbral no dispara; `None`/NaN/inf ⇒ desconocido sin abortar
- [ ] Tests de peligros: `on_hazard` cancela, lista negra el punto y suma un fallo; varios seguidos llevan a `FAILED`; `on_hazard(None)` no revienta
- [ ] Tests del adapter sin ROS real: `BUMP/CLIFF/STUCK` invocan `on_hazard` y `BACKUP_LIMIT/VIRTUAL_WALL/OBJECT_PROXIMITY` no; `percentage` 0..1 ⇒ %; los tópicos usan `namespaced(create3_namespace, ...)`
- [ ] Test: `exploration_params.yaml` declara los parámetros nuevos y coincide con los del nodo
- [ ] `git diff --name-only main...HEAD` solo lista archivos permitidos (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] Ante dato desconocido de dock/secuestro la política es conservadora (no explorar / pausar), nunca asumir "seguro"
- [ ] Solo se invoca `undock` del Create 3; no se usan `drive_*`, `wall_follow` ni `navigate_to_position`, ni se publica en `cmd_vel`
- [ ] `irobot_create_msgs` solo se importa en `adapters`; `services` sigue sin `rclpy`
- [ ] Watchdog y parada por obstáculo del supervisor intactos
- [ ] Commits con Conventional Commits y scope `(TASK-029)`

## Verificación humana en el robot (los agentes no pueden)
- En la base: `ros2 topic echo /r2/dock_status` muestra `is_docked: true`; al iniciar, ejecuta `undock` y solo entonces recibe objetivos.
- Levantar el robot a mano: `/r2/kidnap_status` pasa a true y queda `PAUSED`; al soltarlo reanuda tras unos ciclos (si el mapa se corrompió, reiniciar el mapeo).
- Bump suave contra un obstáculo: actúa el reflejo del firmware, la zona queda en lista negra y se elige otra frontera.
- `ros2 param get /r2/motion_control safety_override` y `reflexes_enabled`: confirmar que los reflejos de bump/cliff siguen activos.
- Batería baja (o umbral simulado): se guarda el mapa parcial y termina con `LOW_BATTERY`.
- Zona despejada, ruedas en el aire primero y luego suelo: `ros2 service call /exploration/start std_srvs/srv/Trigger`; comprobar que frena ante un obstáculo repentino (supervisor prevalece) y que al terminar se genera el `.yaml`/`.pgm` en `hrobot_bringup/maps/` y carga bien en `navigation.launch.py`.
- Ajustar `robot_radius`/`inflation_radius` si no pasa por puertas y reportarlo.

## Fuera de alcance
Relocalización automática tras secuestro, volver a la base al terminar o con batería baja, criterio por ganancia de información, cambios en `supervisor_node`/`safety_supervisor` o en parámetros del Create 3.
