# create3_autonomous_explorer

Exploracion autonoma basada en fronteras (`explorer_node`) y guardado de mapas (`map_manager_node`).

## Guardar un mapa

Con el mapeo en marcha (slam_toolbox + Nav2 `map_saver`) y `map_manager_node` ejecutandose:

```bash
ros2 run create3_autonomous_explorer map_manager_node
ros2 service call /r2/save_exploration_map create3_bot_interfaces/srv/SaveExplorationMap "{map_name: sala1}"
```

- `map_name` se sanea: solo `[A-Za-z0-9_-]`, el resto (espacios, `/`, `\`, `.`, acentos...) pasa a `_`,
  maximo 64 caracteres. Si queda vacio o sin letras/numeros se usa `map_YYYYmmdd_HHMMSS`.
- Respuesta: `success`, `map_path` (ruta sin extension), `posegraph_path` y `message` (motivo de cualquier fallo).
- Si slam_toolbox no responde pero `map_saver` si, `success=true`, `posegraph_path` queda vacio y
  `message` avisa de que ese mapa no se podra ampliar.
- `ExplorerNode` llama al mismo servicio al terminar la exploracion (`save_map_on_finish`, por defecto `true`)
  y devuelve `map_path` en el resultado de la accion `explore_map`.

### Archivos generados (en `maps_dir`, por defecto `~/.create3_maps`)

| Archivo | Origen | Uso |
|---|---|---|
| `<nombre>.yaml` + `<nombre>.pgm` | `nav2_msgs/srv/SaveMap` (`map_saver/save_map`, `/r2/map_saver/save_map`) | Navegacion con `map_server`/AMCL |
| `<nombre>.posegraph` + `<nombre>.data` | `slam_toolbox/srv/SerializePoseGraph` (`slam_toolbox/serialize_map`, `/r2/slam_toolbox/serialize_map`) | Ampliar el mapa |

## Ampliar un mapa existente

slam_toolbox puede cargar el pose graph al arrancar y seguir mapeando sobre el:

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    map_file_name: /home/<usuario>/.create3_maps/sala1   # sin extension
    map_start_at_dock: true   # o map_start_pose: [x, y, yaw]
```

(Lo que se pasa a `map_file_name` es la ruta base, sin `.posegraph`.) El argumento de launch
`map:=<nombre>` que automatiza esto se implementa en T-06. Despues, lanzar `explorer_node` y guardar de
nuevo con el mismo u otro nombre.

## Nombres y namespace

Todos los topics, servicios y acciones (`map`, `frontiers`, `cmd_vel`, `navigate_to_pose`,
`explore_map`, `save_exploration_map`, `map_saver/save_map`, `slam_toolbox/serialize_map`) son **relativos**:
se resuelven bajo el namespace del nodo. Con el launch de `create3_bot_bringup` (namespace `r2` por defecto)
quedan `/r2/map`, `/r2/explore_map`, `/r2/save_exploration_map`, etc.; sin namespace, `/map`... Los
ejemplos de este README asumen `/r2`; sin namespace quita el prefijo.

## Parametros de `map_manager_node`

| Parametro | Defecto | Descripcion |
|---|---|---|
| `maps_dir` | `~/.create3_maps` | Directorio de salida (se expande `~` y se crea) |
| `map_saver_service` | `map_saver/save_map` | Servicio `nav2_msgs/srv/SaveMap` |
| `serialize_service` | `slam_toolbox/serialize_map` | Servicio `slam_toolbox/srv/SerializePoseGraph` |
| `service_timeout_sec` | `5.0` | Espera maxima (disponibilidad y respuesta) por cada servicio; se acota a 600 s |
| `map_topic` | `map` | Topico que lee `map_saver` |
| `image_format` | `pgm` | Formato de imagen |
| `map_mode` | `trinary` | Modo de `SaveMap` |
| `occupied_thresh` / `free_thresh` | `0.65` / `0.25` | Umbrales de `SaveMap` |
| `save_service_name` | `save_exploration_map` | Nombre del servicio ofrecido |

## Relacion entre timeouts

`map_manager_node` espera hasta `service_timeout_sec` por disponibilidad y otro tanto por respuesta
de cada servicio (`map_saver` y `serialize_map`), es decir, hasta `4 x service_timeout_sec` en el
peor caso. El parametro `save_timeout_sec` de `explorer_node` (defecto `30.0`) debe ser mayor que
ese valor; con el defecto `5.0` de `service_timeout_sec` el peor caso es 20 s. Si se aumenta
`service_timeout_sec`, aumente `save_timeout_sec` en consecuencia.

## Nota sobre parametros `double` en YAML

ROS 2 es estricto con los tipos: un parametro declarado como `double` debe escribirse con decimales
(`1.0`, no `1`), de lo contrario el nodo falla con `InvalidParameterTypeException`. Aplica a
`weight_*`, `blacklist_radius`, `control_rate_hz`, `service_timeout_sec`, `occupied_thresh`, etc.

## Tests

`PYTHONPATH=... colcon test --packages-select create3_autonomous_explorer` (GoogleTest: `test_frontier_detector`,
`test_goal_evaluator`, `test_map_naming`).
