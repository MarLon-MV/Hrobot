# TASK-020: Launch de navegación (Nav2 con salida a `/cmd_vel_nav`)

- **Rama Git:** `feat/TASK-020`
- **Depende de:** TASK-009
- **Prioridad:** P1

## Contexto y objetivo de negocio
Falta `navigation.launch.py`. Regla de seguridad: solo `supervisor_node` publica `/cmd_vel`; Nav2 debe publicar su salida final en `/cmd_vel_nav` (`NAV_CMD_VEL_TOPIC` en `adapters/names.py`). Como no se puede verificar `nav2_bringup/navigation_launch.py` (sin ROS en el entorno de desarrollo), **no se usa**: se lanzan los nodos de Nav2 uno a uno con remapeos explícitos, coherentes con `lifecycle_manager_navigation.node_names` de `nav2_params.yaml`.

## Alcance técnico
Crear `src/hrobot_bringup/launch/navigation.launch.py` (usar los archivos `sensors.launch.py` y `safety.launch.py` ya existentes como referencia de estilo):
- Argumentos: `use_sim_time` (default `false`), `map` (ruta al `.yaml` del mapa, **sin default**, obligatorio), `nav_cmd_vel_topic` (default `/cmd_vel_nav`), `params_file` (default `nav2_params.yaml` del share de `hrobot_bringup`).
- Incluye (`IncludeLaunchDescription`) `sensors.launch.py` y `safety.launch.py`.
- Localización: nodos `nav2_map_server/map_server` (parámetro `yaml_filename` = argumento `map`), `nav2_amcl/amcl`, y `nav2_lifecycle_manager/lifecycle_manager` con nombre `lifecycle_manager_localization`.
- Navegación: nodos `nav2_controller/controller_server`, `nav2_planner/planner_server`, `nav2_velocity_smoother/velocity_smoother`, `nav2_bt_navigator/bt_navigator`, y `nav2_lifecycle_manager/lifecycle_manager` con nombre `lifecycle_manager_navigation`. Todos con `parameters=[params_file, {'use_sim_time': ...}]`.
- **Cadena de velocidad (remapeos obligatorios):** `controller_server`: `cmd_vel` → `cmd_vel_nav_raw`; `velocity_smoother`: entrada `cmd_vel` → `cmd_vel_nav_raw` y salida `cmd_vel_smoothed` → valor de `nav_cmd_vel_topic`. Ningún nodo de Nav2 debe publicar ni suscribirse a `/cmd_vel`. Estos nombres de tópico por defecto de `controller_server` y `velocity_smoother` son los habituales en Humble/Jazzy pero **no se pueden verificar aquí**: dejar un comentario `# VERIFY (human): ros2 topic info /cmd_vel -v debe mostrar solo supervisor_node como publisher`.
- **Rechazo de `/cmd_vel`:** si `nav_cmd_vel_topic` resuelve a `/cmd_vel` o `cmd_vel`, el launch debe fallar con un error claro (`OpaqueFunction` que lance `RuntimeError`, o equivalente). Implementar la comprobación en una función pura `validate_nav_cmd_vel_topic(topic: str) -> str` definida a nivel de módulo (testeable sin ROS) y usarla desde el `OpaqueFunction`.
- Los nodos con `/tf` y `/tf_static` no se remapean. `ROS_DOMAIN_ID` y RMW no se fijan. Sin rutas absolutas.
- Si algo del alcance no es realizable, emitir `BLOCKED:` explícito con el motivo; **no omitir archivos en silencio**.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/launch/navigation.launch.py`
- `tests/launch/test_navigation_launch.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/launch/test_navigation_launch.py` termina con código 0 (sin ROS instalado; análisis con `ast`, y `validate_nav_cmd_vel_topic` se prueba extrayendo la función con `ast`/`exec` aislado, no importando `launch`)
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/navigation.launch.py` termina con código 0
- [ ] El test verifica: define `generate_launch_description`; declara `use_sim_time`, `map` (sin `default_value`), `nav_cmd_vel_topic` (default `/cmd_vel_nav`) y `params_file`; referencia los siete nodos Nav2 indicados y los dos lifecycle managers; contiene los remapeos de la cadena de velocidad; no contiene la cadena `"/cmd_vel"` como destino de un remapeo; los `node_names` de `nav2_params.yaml` para navegación coinciden con los ejecutables lanzados (`controller_server`, `planner_server`, `velocity_smoother`, `bt_navigator`)
- [ ] `validate_nav_cmd_vel_topic`: acepta `/cmd_vel_nav` y `cmd_vel_nav`; rechaza `/cmd_vel`, `cmd_vel`, `//cmd_vel`, `/cmd_vel/` y la cadena vacía (tests parametrizados)

## Fuera de alcance
Ejecutar Nav2 real, completar `nav2_params.yaml` (behavior_server, smoother_server, waypoint_follower), verificación de nombres de tópico (humana, en la RPi5).
