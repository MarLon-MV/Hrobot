# 03 · Calibración con el robot real

Todos estos valores son **provisionales**; ninguno se ha medido en el robot.

## Seguridad (`domain/safety.py`, parámetros de `supervisor_node`)
| Parámetro | Valor actual | Qué ajustar / riesgo |
|---|---|---|
| `stop_distance` / `slow_distance` | 0,35 m / 1,0 m | Frenado real del Create 3 con carga; margen según velocidad máxima. |
| `min_valid_ratio` / `degraded_factor` | 0,70 / 0,30 | Con cristal, superficies negras o espejos habrá muchas lecturas inválidas: medir cuántas veces se activa la reducción en pasillos reales. |
| `max_invalid_run_rad` | 8° | Un hueco contiguo ≥ 8° reduce la velocidad. Objetos finos (<8°) sin eco pueden no detectarse: probar con patas de silla/carros. |
| `front_half_angle` | ±0,6 rad | Cono frontal vigilado; revisar giros y esquinas. |
| `stale_timeout` | 0,5 s | Tolerancia a retrasos del scan. |
| `max_rise_per_s`, `max_step_gap_s` | 1,0/s, 0,25 s | Rampa de aceleración y reinicio tras pausas. |
| `max_linear` / `max_angular` | ver `supervisor_node` | Alinear con `velocity_smoother` y RPP. |

## Nav2 (`config/nav2_params.yaml`, marcados `TODO(verify)`)
- AMCL: `alpha1..5`, `laser_max_range` (el LD19 llega a 12 m), `min/max_particles`, `update_min_d/a`.
- RPP: `desired_linear_vel`, `lookahead_*`, `regulated_linear_scaling_*`, `max_angular_accel`.
- `velocity_smoother`: `max_accel` / `max_decel` según el Create 3 real.
- Costmaps: `robot_radius` (0,17 m provisional) o footprint real, inflación, `obstacle_max_range`, `raytrace_max_range`.
- Tolerancias de meta (`xy_goal_tolerance`, `yaw_goal_tolerance`) y progreso (`required_movement_radius`, `movement_time_allowance`).
- `planner_server`: `tolerance`, `allow_unknown`.

## Mapa y localización
- Crear el mapa con `mapping.launch.py` y guardarlo en `src/hrobot_bringup/maps/` (hoy vacío).
- Validar AMCL: pose inicial desde RViz, convergencia de partículas, relocalización tras empujones.
- `slam_toolbox_params.yaml`: resolución, rango, cierre de bucles en el entorno real.
