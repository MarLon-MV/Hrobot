# TASK-023: Namespace del Create 3 (`/r2`) en cmd_vel y odom

- **Rama Git:** `feat/TASK-023`
- **Depende de:** TASK-020, TASK-022
- **Prioridad:** P1

## Contexto y objetivo de negocio
El Create 3 corre con `ROS_DOMAIN_ID=21`, `rmw_cyclonedds_cpp` y namespace **`/r2`**: acepta velocidades en `/r2/cmd_vel` y publica odometría en `/r2/odom`. Hoy `supervisor_node` publica en `/cmd_vel` (constante fija) y Nav2 usa `/odom`, por lo que el robot no recibiría órdenes ni odometría. El entorno (`ROS_DOMAIN_ID`, RMW) sigue **sin** fijarse en los launch. **No** se usa `ROS_NAMESPACE`: el namespace del Create 3 se pasa como argumento.

## Alcance técnico
1. `adapters/names.py`: mantener `CMD_VEL_TOPIC = "/cmd_vel"` y `NAV_CMD_VEL_TOPIC` como valores por defecto; añadir `DEFAULT_CREATE3_NAMESPACE = "/r2"` y una función pura `namespaced(namespace: str, topic: str) -> str` (une sin dobles barras; `namespaced("/r2", "/cmd_vel") == "/r2/cmd_vel"`; namespace vacío o `/` devuelve el tópico tal cual; rechaza espacios).
2. `supervisor_node.py`: nuevo parámetro `cmd_vel_topic` (str, default `CMD_VEL_TOPIC`), validado (no vacío, absoluto); el publisher lo usa. No cambiar el resto de la lógica de seguridad. El suscriptor `NAV_CMD_VEL_TOPIC` queda igual.
3. `safety.launch.py`: argumento `create3_namespace` (default `/r2`) y paso a `supervisor_node` de `cmd_vel_topic = namespaced(create3_namespace, "/cmd_vel")`. Importar la función de `hrobot_core.adapters.names` (los launch ya dependen de `hrobot_core`).
4. `navigation.launch.py`: argumento `create3_namespace` (default `/r2`) y sobrescritura del parámetro `odom_topic` de `bt_navigator` y `velocity_smoother` con `namespaced(create3_namespace, "/odom")` (después de `params_file`). **`validate_nav_cmd_vel_topic` debe rechazar también `/<create3_namespace>/cmd_vel`** (la entrada real del Create 3): firma `validate_nav_cmd_vel_topic(topic, forbidden=("/cmd_vel",))` compatible con los tests actuales, y el `OpaqueFunction` pasa ambos valores prohibidos.
5. `nav2_params.yaml`: dejar `odom_topic: /odom` (el launch lo sobrescribe); añadir comentario. No tocar nada más.
6. **TF queda fuera de esta tarea**: se decidirá tras verificar si el Create 3 publica `/r2/tf`.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/names.py`
- `src/hrobot_core/hrobot_core/adapters/supervisor_node.py`
- `src/hrobot_bringup/launch/safety.launch.py`
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/config/nav2_params.yaml`
- `tests/adapters/test_names.py`
- `tests/adapters/test_supervisor_node.py`
- `tests/launch/test_launch_files.py`
- `tests/launch/test_navigation_launch.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/safety.launch.py` y `python3 -m py_compile src/hrobot_bringup/launch/navigation.launch.py` terminan con código 0
- [ ] Tests de `namespaced`: `("/r2","/cmd_vel")→"/r2/cmd_vel"`, `("r2","/cmd_vel")→"/r2/cmd_vel"`, `("/r2/","cmd_vel")→"/r2/cmd_vel"`, `("","/cmd_vel")→"/cmd_vel"`, `("/","/odom")→"/odom"`, namespace con espacios → `ValueError`
- [ ] Test del nodo: con `cmd_vel_topic=/r2/cmd_vel` el publisher de Twist se crea en `/r2/cmd_vel` y **no** existe publisher en `/cmd_vel`; con el parámetro vacío o relativo el nodo falla al arrancar; el default sigue siendo `/cmd_vel`
- [ ] Tests de launch (estáticos): ambos declaran `create3_namespace` con default `/r2`; `navigation.launch.py` sobrescribe `odom_topic` tras `params_file`; `validate_nav_cmd_vel_topic("/r2/cmd_vel", forbidden=("/cmd_vel","/r2/cmd_vel"))` lanza `ValueError`, y las llamadas existentes con un solo argumento siguen funcionando
- [ ] Regresión de seguridad: los tests existentes del supervisor (rampa, emergencia, sin marcha atrás) siguen pasando sin modificar sus aserciones

## Fuera de alcance
Remapeo de `/tf` y `/tf_static`, namespace de slam_toolbox/RViz, fijar `ROS_DOMAIN_ID`/RMW en launch, tipo de mensaje de `cmd_vel` (Twist vs TwistStamped).
