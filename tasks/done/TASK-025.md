# TASK-025: Eliminar `velocity_smoother` (aceleración ya limitada por supervisor y Create 3)

- **Rama Git:** `feat/TASK-025`
- **Depende de:** TASK-024
- **Prioridad:** P2

## Contexto y objetivo de negocio
Hoy la velocidad pasa por tres limitadores de aceleración en cadena: `velocity_smoother` de Nav2 (`max_accel`/`max_decel`), la rampa de `supervisor_node` (`max_rise_per_s`, sobre el factor de seguridad) y `wheel_accel_limit` del firmware del Create 3 (`/r2/motion_control`). El smoother usa `feedback: OPEN_LOOP`, así que no aporta información de odometría, y sus `max_accel`/`max_decel` son valores sin verificar (`TODO(verify)`). Además es el único nodo de Nav2 que consume `/r2/odom` (que es BEST_EFFORT, ver TASK-024), y `bt_navigator` ya la recibe aparte.

Objetivo: simplificar la cadena a `controller_server -> /cmd_vel_nav -> supervisor_node -> /r2/cmd_vel`, sin cambiar la garantía de que solo `supervisor_node` publica en `<create3_namespace>/cmd_vel`.

## Alcance técnico
1. `navigation.launch.py`: quitar el nodo `velocity_smoother`; el remapeo de salida de `controller_server` pasa a `("cmd_vel", LaunchConfiguration("nav_cmd_vel_topic"))`. Eliminar `CONTROLLER_RAW_TOPIC` y la validación que lo menciona. Mantener la validación que impide `nav_cmd_vel_topic` = `/cmd_vel` o `<ns>/cmd_vel`. Quitar `velocity_smoother` de `node_names` del `lifecycle_manager_navigation` si aparece.
2. `nav2_params.yaml`: eliminar la sección `velocity_smoother`. Conservar los límites de `FollowPath` (`desired_linear_vel`, etc.) <= `max_linear`/`max_angular` del supervisor.
3. Actualizar `src/README.md` (diagrama de la cadena de velocidad) y `pendientes/01-verificacion-ros-hardware.md` (sección B: el publisher de `/cmd_vel_nav` pasa a ser `controller_server`).
4. Ajustar tests que referencian el smoother sin debilitar las garantías: la cadena sigue validada (solo `supervisor_node` publica `cmd_vel` del robot; Nav2 publica en `nav_cmd_vel_topic`), `max_velocity` ya no se comprueba en el smoother (las comprobaciones de velocidad máxima pasan a `FollowPath`/supervisor).

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/config/nav2_params.yaml`
- `src/README.md`
- `pendientes/01-verificacion-ros-hardware.md`
- `tests/launch/test_navigation_launch.py`
- `tests/config/test_nav_configs.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/navigation.launch.py` termina con código 0
- [ ] Test: `navigation.launch.py` no contiene ningún nodo `velocity_smoother`, y `controller_server` remapea `cmd_vel` a `nav_cmd_vel_topic`
- [ ] Test: `nav_cmd_vel_topic:=/cmd_vel` y `:=/r2/cmd_vel` siguen rechazados
- [ ] Test: `nav2_params.yaml` no tiene sección `velocity_smoother` y `FollowPath.desired_linear_vel <= 0.3` (límite del supervisor)
- [ ] `git diff --name-only main...HEAD` solo lista archivos permitidos (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] No quedan referencias a `velocity_smoother`, `cmd_vel_smoothed` ni `CONTROLLER_RAW_TOPIC` (salvo `docs/nav2_jazzy_defaults.yaml`, que es referencia)
- [ ] `bt_navigator` conserva el override de `odom_topic` a `<ns>/odom`
- [ ] Los remapeos de TF de TASK-024 se conservan en todos los nodos
- [ ] Commits con Conventional Commits y scope `(TASK-025)`

## Verificación humana en el robot (los agentes no pueden)
- `ros2 topic info /cmd_vel_nav -v`: publisher = `controller_server`; `ros2 topic info /r2/cmd_vel -v`: único publisher = `supervisor_node`.
- Banco (ruedas en el aire) y luego suelo: el arranque/parada no es brusco. Si lo fuera, ajustar `max_rise_per_s` del supervisor o `wheel_accel_limit` del Create 3 en vez de reintroducir el smoother, y reportarlo.

## Fuera de alcance
Cambios en `supervisor_node`, en `safety_supervisor` o en parámetros del Create 3; `behavior_server` y recuperaciones.
