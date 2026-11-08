# TASK-022: Controlador Regulated Pure Pursuit y ruta del BT seguro desde el launch

- **Rama Git:** `feat/TASK-022`
- **Depende de:** TASK-020, TASK-021
- **Prioridad:** P1

## Contexto y objetivo de negocio
Dos defectos detectados al revisar TASK-021:
1. `bt_navigator.default_nav_to_pose_bt_xml` está vacío, así que Nav2 cargaría su árbol por defecto (con `BackUp`/`Spin` y dependencia de un `behavior_server` que no se lanza) en lugar de `config/bt_navigate_no_recovery.xml`.
2. `controller_server` usa `dwb_core::DWBLocalPlanner` sin `critics` ni límites de aceleración: DWB no se configuraría. Se sustituye por **Regulated Pure Pursuit** (ligero para la RPi5, sin marcha atrás con `allow_reversing: false`, adecuado para un robot lento de pasillo).

## Alcance técnico
1. `navigation.launch.py`: al nodo `bt_navigator` añadirle el parámetro `default_nav_to_pose_bt_xml` con la ruta `bringup / "config" / "bt_navigate_no_recovery.xml"` (derivada de `get_package_share_directory('hrobot_bringup')`, nunca absoluta en el código). Debe ir **después** del `params_file` en la lista de parámetros para sobrescribir el valor vacío del YAML. No cambiar nada más del launch (remapeos, validación de `nav_cmd_vel_topic`, etc.).
2. `nav2_params.yaml`:
   - `controller_server.ros__parameters.FollowPath`: sustituir el bloque DWB por
     `plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"` con:
     `desired_linear_vel: 0.3`, `lookahead_dist: 0.6`, `min_lookahead_dist: 0.3`, `max_lookahead_dist: 0.9`, `lookahead_time: 1.5`, `rotate_to_heading_angular_vel: 0.8`, `transform_tolerance: 0.1`, `use_velocity_scaled_lookahead_dist: false`, `min_approach_linear_velocity: 0.05`, `approach_velocity_scaling_dist: 0.6`, `use_collision_detection: true`, `max_allowed_time_to_collision_up_to_carrot: 1.0`, `use_regulated_linear_velocity_scaling: true`, `regulated_linear_scaling_min_radius: 0.9`, `regulated_linear_scaling_min_speed: 0.1`, `use_rotate_to_heading: true`, `allow_reversing: false`, `max_angular_accel: 3.2`. Marcar todo con `# TODO(verify)` (valores de la documentación de Nav2, no validados en el robot). Conservar `controller_plugins: ["FollowPath"]`.
   - `rotate_to_heading_angular_vel` ≤ `velocity_smoother.max_velocity[2]` (1.0).
   - No tocar el resto del YAML.
3. Actualizar los validadores/tests existentes que asumen DWB (`min_vel_x`, `max_vel_x`, `max_vel_theta`, `min_vel_y`, `max_vel_y` en `FollowPath`): sustituirlos por comprobaciones equivalentes para RPP — `allow_reversing is False`, `0 < desired_linear_vel <= 0.3`, `0 < rotate_to_heading_angular_vel <= 1.0`, `use_collision_detection is True` — manteniendo los tests negativos (cada condición debe tener su test que la viole). Si se elimina un test por quedar obsoleto, sustituirlo por su equivalente RPP.
4. Tests nuevos en `tests/launch/test_navigation_launch.py`: el nodo `bt_navigator` recibe `default_nav_to_pose_bt_xml` desde el share de `hrobot_bringup`, el nombre del archivo es `bt_navigate_no_recovery.xml`, y el archivo existe en `src/hrobot_bringup/config/`; el parámetro va después del `params_file`.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/config/nav2_params.yaml`
- `tests/config/test_nav_configs.py`
- `tests/launch/test_navigation_launch.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config tests/launch` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/navigation.launch.py` termina con código 0
- [ ] El YAML ya no contiene `dwb_core` ni claves `min_vel_x`/`max_vel_x`/`max_vel_theta` en `FollowPath`; `default_nav_to_pose_bt_xml` sigue vacío en el YAML con un comentario que remite al launch (la ruta se inyecta desde `navigation.launch.py`)
- [ ] Tests negativos: `allow_reversing: true`, `desired_linear_vel` > 0.3, `use_collision_detection: false` y `rotate_to_heading_angular_vel` > 1.0 son detectados por el validador

## Fuera de alcance
Calibración de RPP en el robot real, `behavior_server`, cambios en los remapeos de velocidad.
