# 01 · Verificación en ROS 2 Jazzy y hardware real

Requiere RPi5 (o PC) con ROS 2 Jazzy, y para los últimos puntos el Create 3 y el LD19. Marca cada punto al completarlo y anota el resultado.

## A. Compilación y arranque (sin robot)
- [ ] `colcon build --packages-select hrobot_interfaces hrobot_core hrobot_bringup` sin errores; `rosdep install` cubre `python3-serial`.
- [ ] `ros2 launch hrobot_bringup navigation.launch.py --show-args` lista `map`, `nav_cmd_vel_topic`, `params_file`, `use_sim_time`.
- [ ] `ros2 launch ... mapping.launch.py` arranca: confirmar que `slam_toolbox` en Jazzy acepta `slam_params_file` con `online_async_launch.py`.
- [ ] `ros2 run hrobot_core ld19_node` y `supervisor_node` arrancan y declaran sus parámetros.
- [ ] `ros2 launch hrobot_bringup rviz.launch.py` abre `hrobot_nav.rviz` sin displays en rojo ni errores de plugin.

## B. Cadena de velocidad (crítico para seguridad)
- [ ] `ros2 topic info /cmd_vel -v`: **un único publisher, `supervisor_node`**. Ningún nodo de Nav2.
- [ ] `ros2 topic info /cmd_vel_nav -v`: publisher = `velocity_smoother`.
- [ ] Confirmar que `controller_server` publica `cmd_vel` y `velocity_smoother` consume `cmd_vel` / publica `cmd_vel_smoothed` (nombres por defecto asumidos en `navigation.launch.py`).
- [ ] `navigation.launch.py ... nav_cmd_vel_topic:=/cmd_vel` debe fallar con error claro.
- [ ] Nav2 llega a `active` los 4 nodos de navegación + `map_server`/`amcl` (`ros2 lifecycle get ...`). Si `velocity_smoother`, RPP o `bt_navigator` fallan al configurar, anotar el parámetro que falta.
- [ ] `bt_navigator` carga `bt_navigate_no_recovery.xml` (no el árbol por defecto) y no pide `behavior_server`.

## C. Interfaces con el Create 3
- [ ] Tipo real de `cmd_vel` del Create 3 en Jazzy: `geometry_msgs/Twist` o `TwistStamped`. `supervisor_node` publica `Twist`; si hace falta `TwistStamped`, abrir tarea.
- [ ] Qué hace el Create 3 si deja de recibir `/cmd_vel` (timeout del firmware). Debe frenar; si no, el robot sigue con la última orden.
- [ ] Topics de odometría (`/odom`) y TF `odom → base_link` publicados por el Create 3 con los nombres esperados.
- [ ] RMW compatible con el Create 3 (Fast DDS / Cyclone) y `ROS_DOMAIN_ID` consistente entre RPi5, Create 3 y PC con RViz.

## D. Parada y fallos
- [ ] **Ctrl+C en `supervisor_node`**: ¿publica ceros antes de morir? (con `rclpy` real el contexto puede invalidarse antes; los tests con dobles no lo prueban). Si no, hace falta otra estrategia (p. ej. depender del timeout del Create 3 o un watchdog externo).
- [ ] Matar `ld19_node` con el robot en marcha (banco, ruedas en el aire): velocidad → 0 en < 1 s.
- [ ] Desconectar el USB del LD19 en marcha: el nodo reconecta al reenchufar y el supervisor no reanuda hasta tener scan válido.
- [ ] Matar `supervisor_node`: confirmar qué hace el robot (depende del punto C).

## E. QoS y mensajes
- [ ] Suscripción a `/hrobot/mission_status`: el supervisor usa `TRANSIENT_LOCAL`; un publisher `VOLATILE` sería incompatible. El futuro nodo de misión debe publicar `RELIABLE + TRANSIENT_LOCAL`.
- [ ] `/scan` (`ld19_node` con `qos_profile_sensor_data`) lo consumen bien AMCL, slam_toolbox y los costmaps (suelen pedir `Best Effort`/`Reliable` según versión).
- [ ] `ros2 topic hz /scan` ≈ 10 Hz estables; sin NaN problemáticos para slam_toolbox (el builder usa NaN para "sin dato").

## F. LD19
- [ ] Regla udev / permisos de `/dev/ttyUSB0` (grupo `dialout`) o nombre estable por `by-id`.
- [ ] Ángulos: comprobar con un obstáculo conocido que izquierda/derecha no están invertidas (el LD19 es horario; el parser convierte a antihorario REP-103).
- [ ] Offsets `laser_x/y/z/yaw` medidos y aplicados.
