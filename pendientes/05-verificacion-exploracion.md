# 05 · Verificación de la exploración autónoma (robot real)

Guía para validar TASK-026..029 con el robot. Hazla **en orden** y no pases a la fase siguiente si falla una comprobación. Requiere haber completado antes `01-verificacion-ros-hardware.md` (secciones A–D): sin cadena de velocidad y TF verificadas, esto no tiene sentido.

Entorno común (RPi5, Create 3 y PC con RViz): `ROS_DOMAIN_ID=21`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, namespace del Create 3 `/r2`.

## Antes de empezar (seguridad)
- Zona despejada de al menos 3x3 m, sin escaleras ni desniveles (el Create 3 tiene sensores de cliff, pero no los pruebes por accidente).
- Una persona junto al robot en todo momento, con acceso físico para cogerlo.
- Parada de emergencia lista **antes** de mover nada:
  1. `ros2 service call /exploration_node/stop std_srvs/srv/Trigger` (para la exploración y cancela el objetivo).
  2. `ros2 service call /r2/e_stop irobot_create_msgs/srv/EStop "{e_stop_on: true}"` (frena el Create 3). Confirma el campo con `ros2 interface show irobot_create_msgs/srv/EStop`.
  3. Último recurso: matar el launch con Ctrl+C. Comprueba antes (punto D de la guía 01) que el robot frena si desaparece `/r2/cmd_vel`.
- Batería del robot por encima de 50 %.

## Fase 0 · Interfaces (sin movimiento)
Los tipos de mensaje de `irobot_create_msgs` se escribieron sin ROS instalado; confírmalos:
- [ ] `ros2 interface show irobot_create_msgs/msg/DockStatus` tiene `is_docked`.
- [ ] `ros2 interface show irobot_create_msgs/msg/KidnapStatus` tiene `is_kidnapped`.
- [ ] `ros2 interface show irobot_create_msgs/msg/HazardDetectionVector` y `HazardDetection`: anota los valores de `type` (`BUMP`, `CLIFF`, `STUCK`, `BACKUP_LIMIT`, `VIRTUAL_WALL`, `OBJECT_PROXIMITY`...) y compáralos con los que usa `exploration_node.py`. Si falta alguno o su valor numérico difiere, abrir tarea.
- [ ] `ros2 interface show irobot_create_msgs/action/Undock`.
- [ ] `ros2 interface show nav2_msgs/srv/SaveMap`.
- [ ] `ros2 topic echo /r2/dock_status --once`, `/r2/kidnap_status --once`, `/r2/battery_state --once` devuelven datos. Anota el QoS con `ros2 topic info -v` (el nodo se suscribe con `SensorDataQoS`, best effort; debe coincidir con el publisher).
- [ ] `battery_state.percentage` está en 0..1 (no 0..100). El nodo lo multiplica por 100.

## Fase 1 · Arranque sin movimiento
```bash
ros2 launch hrobot_bringup exploration.launch.py
```
- [ ] Arranca sin errores. Revisa advertencias de TF o lifecycle en los primeros 30 s.
- [ ] `ros2 node list` incluye `exploration_node`, `map_saver`, `lifecycle_manager_map_saver`, `slam_toolbox`, `supervisor_node`, `controller_server`, `bt_navigator`.
- [ ] **No** hay `amcl` ni `map_server` (modo SLAM).
- [ ] `ros2 lifecycle get /map_saver` devuelve `active`. Todos los nodos de Nav2 en `active`.
- [ ] `ros2 service list | grep save_map` muestra `/map_saver/save_map`; `ros2 service type /map_saver/save_map` devuelve `nav2_msgs/srv/SaveMap`. (El nombre salió de memoria del Coder; si no coincide, cambiar `MAP_SAVER_SERVICE` en `names.py`).
- [ ] `ros2 service list | grep exploration_node` muestra `/exploration_node/start` y `/exploration_node/stop`.
- [ ] `ros2 topic echo /hrobot/exploration_status --once`: estado `IDLE`.
- [ ] `ros2 topic info /r2/cmd_vel -v`: un único publisher, `supervisor_node`. `exploration_node` **no** aparece.
- [ ] En RViz (PC): `/map` crece con el scan, TF `map → odom → base_link → laser` completa, costmaps visibles.
- [ ] Parámetros cargados: `ros2 param get /exploration_node min_battery_pct` devuelve `20.0`.
- [ ] Sin llamar a `start`, el robot **no se mueve**.

## Fase 2 · Dock y undock (ruedas en el aire)
Pon el robot sobre un soporte con las ruedas libres y **en la base**.
- [ ] `ros2 topic echo /r2/dock_status` muestra `is_docked: true`.
- [ ] `ros2 service call /exploration_node/start std_srvs/srv/Trigger`: estado `UNDOCKING`, el robot ejecuta undock.
- [ ] No se envía ningún objetivo a Nav2 hasta que `is_docked` pasa a `false`.
- [ ] Tras el undock, estado `EXPLORING`.
- [ ] Prueba de fallo: `stop`, vuelve a la base, tapa/desconecta el dato o impide el undock (p. ej. sujeta el robot); debe acabar en `FAILED` tras `undock_timeout_s` (30 s), no quedarse colgado.
- [ ] Prueba de dato desconocido: con `dock_status` sin publicar, el robot **no** debe explorar; pasa a `FAILED` tras `status_timeout_s` (10 s).

## Fase 3 · Secuestro, peligros y batería (ruedas en el aire)
- [ ] **Secuestro:** con `EXPLORING`, levanta el robot. `ros2 topic echo /r2/kidnap_status` pasa a `is_kidnapped: true`; el estado pasa a `PAUSED` y se cancela el objetivo. Al dejarlo en el suelo, reanuda solo tras `kidnap_clear_cycles` (3) ciclos limpios. **Anota** si el mapa se corrompió tras levantarlo: si es así, el procedimiento es reiniciar el mapeo.
- [ ] **Reflejos del firmware:** `ros2 param get /r2/motion_control safety_override` y `reflexes_enabled`. `REFLEX_BUMP` y `REFLEX_CLIFF` deben seguir activos durante la exploración.
- [ ] **Bump:** presiona suavemente el parachoques. Debe verse un `BUMP` en `/r2/hazard_detection`, el objetivo se cancela y el punto entra en la lista negra (la exploración elige otra frontera). Cuenta como un único fallo, no dos.
- [ ] **Cliff:** simula con la mano bajo un sensor frontal (sin acercarte a un borde real). Debe tratarse igual que un bump.
- [ ] **Batería baja:** sin esperar a que se agote, baja el umbral para la prueba: `ros2 param set /exploration_node min_battery_pct 99.0`. Debe guardar el mapa parcial y terminar con motivo `LOW_BATTERY`. Restablece `20.0` al terminar.

## Fase 4 · Exploración en suelo
Primero con velocidad reducida, en un recinto pequeño y cerrado con obstáculos fijos.
- [ ] `ros2 service call /exploration_node/start std_srvs/srv/Trigger` desde el robot en el suelo, fuera de la base.
- [ ] En RViz el robot recibe objetivos hacia fronteras y el mapa crece.
- [ ] **El supervisor prevalece:** coloca un obstáculo repentino frente al robot. Debe frenar por la parada del supervisor, no por el reflejo del firmware (revisa los logs).
- [ ] **Watchdog:** mata `ld19_node` mientras explora. El robot se detiene en menos de 1 s y la exploración no manda objetivos inalcanzables.
- [ ] `ros2 topic echo /hrobot/exploration_status` refleja estado y motivo coherentes.
- [ ] Al quedarse sin fronteras (3 ciclos): estado `DONE`, y aparecen `map.yaml` y `map.pgm` en la ruta de `map_save_path` (por defecto `hrobot_bringup/maps/map`, en el directorio de instalación de `share`).
- [ ] El `map.yaml` guardado se carga en `navigation.launch.py` (modo AMCL, con `map:=<ruta>`) y el robot se localiza.
- [ ] `stop` en mitad de la exploración detiene el robot y vuelve a `IDLE`.

## Fase 5 · Ajustes y registro
Anota qué valores cambias (y por qué) en `03-calibracion.md`:
- `robot_radius` e `inflation_radius` del costmap si el robot no pasa por puertas.
- `min_frontier_size` según el ruido real del LD19 (si persigue fronteras fantasma, subirlo).
- `blacklist_radius` y `min_goal_distance` según pasillos y tolerancia del controlador.
- `max_consecutive_failures` si aborta demasiado pronto.
- `undock_timeout_s`, `map_save_timeout_s`, `min_battery_pct` (marcados con `TODO(verify)` en `exploration_params.yaml`).

## Si algo falla
| Síntoma | Causa probable | Dónde mirar |
|---|---|---|
| Se queda en `UNDOCKING`/`FAILED` al inicio | `dock_status` no llega (QoS o namespace) | `ros2 topic info -v /r2/dock_status`, `create3_namespace` |
| Nunca explora, estado `PAUSED` | `kidnap_status` sin datos se trata como secuestro (conservador) | `ros2 topic echo /r2/kidnap_status` |
| `DONE` pero no hay mapa en disco | nombre del servicio de guardado | `ros2 service list \| grep save_map` |
| El robot no recibe objetivos | falta `navigate_to_pose` o TF `map → base_link` | `ros2 action list`, `ros2 run tf2_ros tf2_echo map base_link` |
| Objetivos fallan siempre | costmaps con `inflation_radius` excesivo o mapa sin cerrar | RViz, costmap global |

Cuando un punto se resuelva, bórralo de aquí o conviértelo en una `tasks/backlog/TASK-XXX.md` si hay que cambiar código.
