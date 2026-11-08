# TASK-021: Completar `nav2_params.yaml` (Jazzy) con BT sin recuperaciones ni marcha atrás

- **Rama Git:** `feat/TASK-021`
- **Depende de:** TASK-020
- **Prioridad:** P1

## Prerrequisito HUMANO (antes de lanzar esta tarea)
Copiar al repo, desde una máquina con ROS 2 Jazzy, los defaults de Nav2 **sin editarlos**:
`cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml docs/nav2_jazzy_defaults.yaml`
y, del mismo sitio, el BT por defecto:
`cp /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml docs/nav2_jazzy_default_bt.xml`
(commitearlos en `main`). Si `docs/nav2_jazzy_defaults.yaml` no existe, el Coder debe detenerse con `BLOCKED:`; **no inventar valores de parámetros de Nav2**.

## Contexto y objetivo de negocio
`config/nav2_params.yaml` es un esqueleto: faltan parámetros obligatorios (p. ej. del `velocity_smoother`: `max_accel`, `max_decel`, `smoothing_frequency`, `feedback`; del `controller_server`, `planner_server` y `bt_navigator`). Sin ellos Nav2 no configura sus nodos. Restricciones de seguridad del proyecto:
- El robot **no se mueve marcha atrás** (el supervisor pone el lineal negativo a 0). Nada de `BackUp`, y `Spin`/`Wait` solo vía `behavior_server`, que además publica `cmd_vel` por su cuenta y `navigation.launch.py` (TASK-020) **no lo lanza**. Por eso se usa un árbol de comportamiento propio **sin recuperaciones**.
- Solo `supervisor_node` publica `/cmd_vel`.

## Alcance técnico
1. Completar `src/hrobot_bringup/config/nav2_params.yaml` tomando los valores del archivo de defaults de Jazzy para las secciones existentes (`amcl`, `bt_navigator`, `controller_server`, `planner_server`, `local_costmap`, `global_costmap`, `map_server`, `velocity_smoother`, lifecycle managers). Conservar los valores ya fijados por el proyecto (frames `map`/`odom`/`base_link`, `/scan`, `/odom`, límites de velocidad, footprint/radio si existen). Marcar con `# TODO(verify)` todo valor tomado del default que dependa del robot real (aceleraciones, radio, tolerancias).
2. `velocity_smoother`: `min_velocity[0] == 0.0`, `max_velocity[1] == 0.0` (sin lateral), `min_velocity[1] == 0.0`; `max_velocity[0] <= 0.3`; `max_decel[0]` negativa; `odom_topic: /odom`; `feedback: "OPEN_LOOP"` salvo que los defaults indiquen otra cosa razonable.
3. `controller_server`: sin plugins que permitan marcha atrás (en DWB/RPP o el controlador elegido: `min_vel_x >= 0`, `min_vel_y == 0`, `vy_samples` irrelevante; en RPP, `allow_reversing: false`). Reutilizar el controlador ya configurado en el esqueleto; no cambiarlo.
4. Crear `src/hrobot_bringup/config/bt_navigate_no_recovery.xml`: árbol mínimo derivado del BT por defecto de Jazzy **eliminando** `RecoveryNode`, `Spin`, `BackUp`, `Wait`, `ClearEntireCostmap`, dejando `PipelineSequence` con `RateController` + `ComputePathToPose` y `FollowPath`. Si falla, el árbol falla (la misión pasa a `NAV_FAILED`; no hay reintento automático).
5. `bt_navigator`: `default_nav_to_pose_bt_xml` apuntando al XML anterior. El YAML no puede llevar rutas absolutas ni de `/home`/`/Users`; usar el truco soportado por Nav2 (`default_nav_to_pose_bt_xml: ""` y que `navigation.launch.py` lo pase con `RewrittenYaml`/parámetro con ruta al share) **solo si se amplía el alcance**; en esta tarea, dejar `default_nav_to_pose_bt_xml` con un marcador explícito `# TODO(human): ruta al XML instalado (share/hrobot_bringup/config/bt_navigate_no_recovery.xml)` y documentarlo en el resumen. No inventar rutas.
6. `lifecycle_manager_navigation.node_names` debe coincidir exactamente con los nodos lanzados por `navigation.launch.py`; no añadir `behavior_server`, `smoother_server` ni `waypoint_follower`.

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/config/nav2_params.yaml`
- `src/hrobot_bringup/config/bt_navigate_no_recovery.xml`
- `tests/config/test_nav_configs.py`
- `tests/config/test_bt_xml.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/config` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] Tests nuevos en `test_nav_configs.py` (validador existente ampliado, con tests negativos): `velocity_smoother.min_velocity[0] == 0.0`, `max_velocity[1] == 0.0`, `max_velocity[0] <= 0.3`; presentes y numéricos `max_accel`, `max_decel`, `smoothing_frequency`, `feedback`; el controlador no permite marcha atrás; los `node_names` del lifecycle manager de navegación son exactamente `controller_server`, `planner_server`, `velocity_smoother`, `bt_navigator` y cada uno tiene su sección en el YAML; el YAML no contiene `cmd_vel` apuntando a `/cmd_vel` ni rutas absolutas
- [ ] `test_bt_xml.py`: el XML parsea (`xml.etree`); no contiene `BackUp`, `Spin`, `Wait`, `RecoveryNode`; contiene `ComputePathToPose` y `FollowPath`
- [ ] Si faltan los archivos de defaults en `docs/`, el Coder emite `BLOCKED:` y no modifica nada

## Fuera de alcance
Ejecutar Nav2 real, afinar costmaps/inflación/velocidades (calibración humana en la RPi5), pasar la ruta del BT desde el launch, `behavior_server`, `waypoint_follower`.
