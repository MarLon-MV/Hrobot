# TASK-028: Launch y configuración del modo exploración (bringup)

- **Rama Git:** `feat/TASK-028`
- **Depende de:** TASK-027
- **Prioridad:** P1

## Contexto y objetivo de negocio
El nodo de exploración (TASK-027) necesita un launch que levante todo el modo "mapear explorando": sensores, seguridad, `slam_toolbox`, Nav2 sin AMCL ni mapa estático, el nodo de exploración y el guardado del mapa. Esta tarea solo toca `hrobot_bringup` y documentación; no hay lógica de negocio.

## Alcance técnico
1. **`config/exploration_params.yaml`**: parámetros de `exploration_node` (los de TASK-027) con valores iniciales razonables y comentarios `TODO(verify)` donde dependan del robot real.
2. **`launch/exploration.launch.py`**: incluye `mapping.launch.py` (sensores, seguridad, `slam_toolbox`) y la parte de Nav2 de `navigation.launch.py` en modo SLAM (sin `amcl` ni `map_server` estático; `slam_toolbox` es la fuente de `map -> odom`). Si `navigation.launch.py` no permite ese modo, añadir un argumento (p. ej. `use_slam:=true`) sin romper su comportamiento por defecto. Lanza `exploration_node` con el YAML y `map_saver_server` con su lifecycle manager. Reutiliza los argumentos existentes (`use_sim_time`, `create3_namespace`, `nav_cmd_vel_topic`) y los remapeos de TF de TASK-024. Ruta de guardado de mapas: `hrobot_bringup/maps/`.
3. **`setup.py` de `hrobot_bringup`**: instalar el nuevo launch/config si hace falta.
4. **`src/README.md`**: sección del modo exploración (cómo lanzar, servicios `start`/`stop`, dónde se guarda el mapa, advertencia de no usar en el robot hasta TASK-029).

## Archivos permitidos de modificar/crear
- `src/hrobot_bringup/config/exploration_params.yaml`
- `src/hrobot_bringup/launch/exploration.launch.py`
- `src/hrobot_bringup/launch/navigation.launch.py`
- `src/hrobot_bringup/setup.py`
- `src/README.md`
- `tests/config/test_exploration_config.py`
- `tests/launch/test_exploration_launch.py`
- `tests/launch/test_navigation_launch.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] `python3 -m py_compile src/hrobot_bringup/launch/exploration.launch.py` termina con código 0
- [ ] Test: `exploration_params.yaml` declara todos los parámetros del nodo con tipos y rangos válidos, y los nombres coinciden con los declarados en `exploration_node`
- [ ] Test: `exploration.launch.py` lanza `exploration_node` y `map_saver_server`, **no** lanza `amcl` ni `map_server` estático, y ningún nodo propio publica en `<ns>/cmd_vel` salvo `supervisor_node`
- [ ] Test: los remapeos de TF de TASK-024 y el `odom_topic` de `bt_navigator` se conservan; `navigation.launch.py` sin `use_slam` se comporta igual que antes (tests existentes siguen pasando)
- [ ] `git diff --name-only main...HEAD` solo lista archivos permitidos (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] Sin lógica de negocio en `hrobot_bringup`
- [ ] `nav_cmd_vel_topic` sigue rechazando `/cmd_vel` y `<ns>/cmd_vel`
- [ ] Sin referencias a `velocity_smoother`
- [ ] Commits con Conventional Commits y scope `(TASK-028)`

## Verificación humana en el robot (los agentes no pueden)
Solo después de TASK-029 (dock/secuestro/batería/peligros). Antes, únicamente `ros2 launch hrobot_bringup exploration.launch.py` sin movimiento y comprobar en RViz que `/map`, TF y costmaps aparecen.

## Fuera de alcance
Cambios en el nodo o el gestor de exploración (TASK-027/029), en `supervisor_node` o en parámetros del Create 3.
