# TASK-024: Remapear TF (`/tf`, `/tf_static`) al namespace del Create 3 (`/r2`)

- **Rama Git:** `feat/TASK-024`
- **Depende de:** TASK-023
- **Prioridad:** P0

## Contexto y objetivo de negocio
Verificado en el robot real (`ros2 topic echo /r2/tf --once`): el Create 3 (namespace `/r2`) publica `odom -> base_link` y `odom -> base_footprint` en **`/r2/tf`** (frames **sin prefijo**: `odom`, `base_link`). Su TF estático sale por `/r2/tf_static`. Hoy AMCL, Nav2, slam_toolbox y RViz escuchan `/tf` y `/tf_static`, por lo que no ven `odom -> base_link` y fallan con "frame odom does not exist". Además, el TF estático `base_link -> laser` de `sensors.launch.py` se publica en `/tf_static`, que los consumidores (una vez remapeados) ya no leerían.

Solución: remapear `tf` y `tf_static` a `<create3_namespace>/tf` y `<create3_namespace>/tf_static` en **todos** los nodos que consumen o publican TF, reutilizando el argumento `create3_namespace` (default `/r2`) de TASK-023. Los frames NO cambian (`FRAME_ODOM`, `FRAME_BASE` ya coinciden).

## Alcance técnico
1. `sensors.launch.py`: argumento `create3_namespace` (default `/r2`); el `static_transform_publisher` (`base_link_to_laser`) se remapea `tf -> <ns>/tf` y `tf_static -> <ns>/tf_static` (así el láser queda junto al resto del árbol TF del robot). `ld19_node` no usa TF: no tocar.
2. `navigation.launch.py`: los nodos Nav2/AMCL (`map_server` incluido por uniformidad, `amcl`, `controller_server`, `planner_server`, `velocity_smoother`, `bt_navigator`) y los `lifecycle_manager` reciben remapeos `tf` y `tf_static`. Seguir el patrón de TASK-023 (`SetLaunchConfiguration` en un `OpaqueFunction` + `LaunchConfiguration` en los remapeos) y **conservar** los remapeos de `cmd_vel` existentes. Pasar `create3_namespace` al include de `sensors.launch.py` (ya se pasa).
3. `mapping.launch.py`: argumento `create3_namespace` (default `/r2`); pasarlo al include de `sensors`/`safety`; aplicar los remapeos de TF a slam_toolbox. `online_async_launch.py` de slam_toolbox no acepta remapeos como argumento: envolver el `IncludeLaunchDescription` en un `GroupAction` con `SetRemap` (de `launch_ros.actions`) para `tf` y `tf_static`. No reescribir slam_toolbox como `Node` (es un nodo lifecycle gestionado por su launch).
4. `rviz.launch.py`: argumento `create3_namespace` (default `/r2`) y remapeos `tf`/`tf_static` en `rviz2`.
5. Helper compartido sin duplicar lógica: si ayuda, añadir a `adapters/names.py` las constantes `TF_TOPIC = "/tf"` y `TF_STATIC_TOPIC = "/tf_static"` (se compone con `namespaced`).
6. No cambiar `nav2_params.yaml`, `slam_toolbox_params.yaml`, frames ni la lógica de seguridad.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/names.py`
- `src/hrobot_bringup/launch/sensors.launch.py`
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/launch/mapping.launch.py`
- `src/hrobot_bringup/launch/rviz.launch.py`
- `tests/adapters/test_names.py`
- `tests/launch/test_launch_files.py`
- `tests/launch/test_navigation_launch.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] `python3 -m py_compile` sobre `sensors`, `navigation`, `mapping` y `rviz` `.launch.py` termina con código 0
- [ ] Tests estáticos: los cuatro launch declaran `create3_namespace` con default `/r2`; cada uno remapea `tf` y `tf_static` (en `navigation` cubre amcl, controller, planner, smoother, bt_navigator y lifecycle managers; en `sensors` el `static_transform_publisher`)
- [ ] `namespaced("/r2", TF_TOPIC) == "/r2/tf"` y `namespaced("/r2", TF_STATIC_TOPIC) == "/r2/tf_static"` (si se añaden las constantes)
- [ ] Los remapeos de `cmd_vel` de `navigation.launch.py` y las pruebas de TASK-020/022/023 siguen pasando sin modificar sus aserciones

## Checklist de verificación para el Reviewer
- [ ] Ningún nodo que use TF queda sin remapear (revisar los `Node(...)` de los cuatro launch)
- [ ] No se alteran frames (`odom`, `base_link`, `laser`) ni YAML
- [ ] Con `create3_namespace:=/` (o vacío) el comportamiento equivale al actual (`/tf`, `/tf_static`)
- [ ] Commits con Conventional Commits y scope `(TASK-024)`

## Verificación humana en el robot (los agentes no pueden)
- `ros2 run tf2_ros tf2_echo odom base_link` con `--ros-args -r tf:=/r2/tf -r tf_static:=/r2/tf_static` muestra la transformada.
- `ros2 run tf2_tools view_frames` (mismos remapeos): árbol `map -> odom -> base_link -> laser`.
- **QoS de odometría:** `/r2/odom` es **BEST_EFFORT** (depth 1). Nav2 (`velocity_smoother`, `bt_navigator`) podría suscribirse RELIABLE y no recibir nada. Comprobar con `ros2 topic info /r2/odom -v` (ver QoS del suscriptor) y que `velocity_smoother` no avise de falta de odometría. Si hay incompatibilidad, abrir una tarea aparte (p. ej. relay de odom con QoS compatible); **no** se resuelve aquí.

## Fuera de alcance
Remapeo de `/scan`, QoS de odom, tipo de `cmd_vel`, fijar `ROS_DOMAIN_ID`/RMW en launch, cambios en `ld19_node`.
