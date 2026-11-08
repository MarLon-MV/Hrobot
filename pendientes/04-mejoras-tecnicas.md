# 04 · Mejoras técnicas (pueden hacerlas los agentes)

Candidatas a tarea en `tasks/backlog/`. Ninguna es bloqueante.

- [ ] **Reconexión/robustez del supervisor**: decidir con los resultados de `01-D` cómo garantizar la parada si el proceso muere (watchdog, depender del timeout del Create 3).
- [ ] **Dependencias declaradas**: revisar `package.xml` de los tres paquetes (`exec_depend`: `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav2_*`, `slam_toolbox`, `tf2_ros`, `rviz2`, `python3-serial`) y que `rosdep` las resuelva.
- [ ] **Tipo de `cmd_vel`**: si el Create 3 exige `TwistStamped`, adaptar `supervisor_node` (parametrizable).
- [ ] **Preservar `delivered` entre `RESET`** en `domain/mission.py` (para auditoría).
- [ ] **Registro de diagnósticos**: `diagnostic_updater` o tópico `/hrobot/diagnostics` con estado del supervisor (factor de velocidad, motivo de reducción, edad del scan).
- [ ] **Test de integración de la cadena** `ld19_parser → scan_builder → safety_supervisor` con grabaciones reales del LD19 (rosbag/serial dump), cuando existan.
- [ ] **CI**: ejecutar `pytest`, `ruff`, `mypy` y el test de higiene en cada push (hoy solo se ejecutan en local / por el orquestador).
- [ ] **Perfil `colcon test`/`launch_testing`** para humo de los launch cuando haya ROS disponible en CI.
- [ ] **Documentar** en `src/README.md` el procedimiento de despliegue en la RPi5 (servicio systemd, variables de entorno, arranque automático) cuando se defina.
