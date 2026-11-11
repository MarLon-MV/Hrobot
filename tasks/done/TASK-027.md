# TASK-027: Gestor y nodo de exploración autónoma (núcleo)

- **Rama Git:** `feat/TASK-027`
- **Depende de:** TASK-026
- **Prioridad:** P1

## Contexto y objetivo de negocio
Con la lógica pura de TASK-026, falta conectarla a Nav2: leer `/map` de `slam_toolbox`, elegir la siguiente frontera, enviarla a Nav2 (`NavigateToPose`) y, al no quedar fronteras, guardar el mapa. Esta tarea entrega el gestor (sin ROS) y el nodo delgado. El launch va en TASK-028 y las condiciones de seguridad del Create 3 (dock, secuestro, batería, peligros) en TASK-029; **no se debe probar en el robot real hasta TASK-029**. La seguridad no cambia: Nav2 publica en `/cmd_vel_nav` y solo `supervisor_node` publica el `cmd_vel` del robot.

## Alcance técnico
1. **`services/exploration_manager.py`** (sin rclpy): máquina de estados `IDLE -> EXPLORING -> DONE | FAILED` con puertos `Protocol`: `NavigatorPort` (`send_goal(Point2D)`, `cancel()`) y `MapSaverPort` (`save(name) -> bool`). Método `step(grid: GridMap, robot: Point2D)` por ciclo, más `on_goal_result(success: bool)`, `start()` y `stop()`. Usa `find_frontiers`/`select_frontier` de TASK-026. Reglas:
   - Al fallar o cancelarse un objetivo, añade la frontera a la lista negra y elige otra.
   - Tras `max_consecutive_failures` fallos seguidos (default 5) ⇒ `FAILED`; un éxito reinicia la cuenta.
   - Sin fronteras válidas durante `no_frontier_confirmations` ciclos consecutivos (default 3, evita falsos fin por mapas transitorios) ⇒ `MapSaverPort.save`; si guarda bien ⇒ `DONE`, si no ⇒ `FAILED`.
   - Un solo objetivo activo a la vez; no reenvía el mismo objetivo mientras esté en curso.
   - `stop()` cancela el objetivo y vuelve a `IDLE`, desde cualquier estado. `start()` solo es válido desde `IDLE`, `DONE` o `FAILED` (reinicia lista negra y contadores).
   - Expone `state` y `reason` (texto corto del motivo de fin) como tipos propios (`Enum`/`str`).
2. **`adapters/exploration_node.py`**: nodo `exploration_node` delgado. Suscribe `/map` (QoS transient_local, reliable), obtiene la pose del robot por `tf2` (`map -> base_link`), traduce `OccupancyGrid -> GridMap`, usa el action client `navigate_to_pose` (implementa `NavigatorPort`) y llama al servicio de guardado de mapa de `nav2_map_server` (implementa `MapSaverPort`). Temporizador de ciclo a `exploration_period_s`. Parámetros declarados con tipo y default: `min_frontier_size`, `blacklist_radius`, `min_goal_distance`, `exploration_period_s`, `max_consecutive_failures`, `no_frontier_confirmations`, `map_save_name`, `use_sim_time`. Servicios `std_srvs/Trigger` `start` y `stop`. Publica el estado y el motivo en `/hrobot/exploration_status` (`std_msgs/String`). Sin `time.sleep` en callbacks. Si falta TF o `/map`, el ciclo no hace nada y registra una advertencia limitada (sin spam).
3. **`adapters/names.py`**: constantes `MAP_TOPIC`, `EXPLORATION_STATUS_TOPIC`, `NAVIGATE_TO_POSE_ACTION`.
4. **`setup.py` y `package.xml` de `hrobot_core`**: entry point `exploration_node` y `exec_depend` de `nav2_msgs`, `nav_msgs`, `std_srvs`, `tf2_ros`, `geometry_msgs` si faltan.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/services/exploration_manager.py`
- `src/hrobot_core/hrobot_core/services/__init__.py`
- `src/hrobot_core/hrobot_core/adapters/exploration_node.py`
- `src/hrobot_core/hrobot_core/adapters/names.py`
- `src/hrobot_core/setup.py`
- `src/hrobot_core/package.xml`
- `tests/services/test_exploration_manager.py`
- `tests/adapters/test_exploration_node.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q --cov=hrobot_core --cov-fail-under=85` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/services/test_exploration_manager.py --cov=hrobot_core.services.exploration_manager --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Tests de `ExplorationManager` con puertos falsos: flujo feliz hasta `DONE` con guardado; fallo de objetivo ⇒ lista negra y siguiente frontera; `max_consecutive_failures` ⇒ `FAILED` y éxito reinicia la cuenta; confirmaciones de fin (un ciclo con fronteras reinicia la cuenta); guardado fallido ⇒ `FAILED`; no reenvía objetivo en curso; `stop()` cancela desde cualquier estado; `start()` inválido en `EXPLORING`
- [ ] Test del adapter sin ROS real (conversión `OccupancyGrid -> GridMap` en una función pura extraíble, y declaración de parámetros con sus defaults)
- [ ] Si hay ROS 2 instalado: `colcon build --packages-select hrobot_interfaces hrobot_core hrobot_bringup` termina con código 0
- [ ] `git diff --name-only main...HEAD` solo lista archivos permitidos (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] `services/exploration_manager.py` no importa `rclpy` ni `adapters`; solo `adapters` toca ROS
- [ ] El nodo nunca publica en `/cmd_vel` ni `<ns>/cmd_vel`; el objetivo va solo por la acción de Nav2
- [ ] Sin números mágicos en el nodo (todo parámetro declarado); sin `time.sleep`; sin `except:` desnudo; `use_sim_time` soportado
- [ ] Commits con Conventional Commits y scope `(TASK-027)`

## Fuera de alcance
Launch y config de bringup (TASK-028); dock, secuestro, batería y peligros (TASK-029); criterio por ganancia de información.
