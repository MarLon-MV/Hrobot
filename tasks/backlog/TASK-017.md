# TASK-017: Seguridad LiDAR — limitar velocidad ante tramos contiguos de rayos inválidos

- **Rama Git:** `feat/TASK-017`
- **Depende de:** TASK-010
- **Prioridad:** P0

## Contexto y objetivo de negocio
TASK-010 limita la velocidad cuando la **fracción** de rayos válidos en el cono frontal cae por debajo de `min_valid_ratio` (0.70). Una simulación de extremo a extremo (LD19 a 10 Hz, 450 puntos por vuelta) mostró su límite: un objeto sin retorno (negro, cristal, acero pulido) de ±10° al frente, rodeado de espacio libre, invalida solo ~29% del cono (±34°), supera el umbral y el robot sigue a **velocidad completa** (factor 1.0). Cualquier objeto sin retorno que ocupe menos del 30% del cono pasa desapercibido; a 1 m, 20° equivalen a unos 35 cm de ancho (una pierna, una puerta de cristal).

La diferencia entre un objeto sin retorno y los fallos aleatorios del sensor es la **contigüidad**: los fallos dispersos (≈10% de puntos sin eco) forman huecos de 1-3 rayos; un objeto forma un hueco continuo. Esta tarea detecta huecos contiguos y limita la velocidad, sin detener el robot.

## Alcance técnico
Modificar `src/hrobot_core/hrobot_core/domain/safety.py` (sin crear módulos nuevos; la API pública existente no cambia).

```python
@dataclass(frozen=True)
class SafetyConfig:
    ...  # campos existentes sin cambios
    max_invalid_run_rad: float = math.radians(8.0)   # hueco contiguo que activa la degradación, en (0, π]
```

Regla nueva en `speed_factor`:
- Se consideran los rayos del cono frontal (`±front_half_angle`, ángulo normalizado) **ordenados por ángulo normalizado creciente**, no por índice. Esto es esencial: en un scan con ángulos de 0 a 2π, el cono frontal queda partido entre los índices finales y los iniciales del arreglo, y un hueco centrado en el frente debe tratarse como un único tramo.
- Clasificación de rayos (la misma que ya existe): **inválido** = `NaN`, `-inf` o finito fuera de `[range_min, range_max]`; **válido** = obstáculo o `+inf` (libre). Un rayo válido (incluido `+inf`) interrumpe el tramo.
- Un **tramo** es una secuencia de rayos inválidos consecutivos en ese orden. Su anchura es `n_rayos × |angle_increment|`. Si `anchura >= max_invalid_run_rad - 1e-9` (tolerancia explícita para evitar errores de coma flotante en el límite) ⇒ hay degradación.
- Degradación = la misma que TASK-010: `factor = min(factor_actual, cfg.degraded_factor)`. **Solo baja la velocidad.** Se combina con la regla de la fracción (ambas aplican el mismo tope; no se acumulan).
- Las reglas de fallo seguro existentes no cambian (sin ningún rayo válido ⇒ 0.0, scan caducado ⇒ 0.0, etc.). Si `angle_increment == 0`, la regla de tramos no se aplica (las demás siguen vigentes).
- Los tramos se miden solo con los rayos dentro del cono; la parte de un hueco que quede fuera del cono no cuenta.
- Validación en `__post_init__`: `max_invalid_run_rad` finito y en `(0, π]`; si no, `ValueError`.
- Actualizar los docstrings de `SafetyConfig` y `speed_factor`. El valor 8° es una estimación inicial que debe calibrarse con scans reales (ver "Verificación humana").

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/safety.py`
- `tests/domain/test_safety.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_safety.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_safety.py --cov=hrobot_core.domain.safety --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Tests con un scan de 450 rayos (`angle_min=-π`, `angle_increment=2π/450`, ≈0.8°), todo `+inf`, config por defecto:
  - 10 rayos `NaN` contiguos al frente (anchura 8.0°) ⇒ **0.30**
  - 9 rayos `NaN` contiguos (7.2°) ⇒ **1.0**
  - banda `NaN` de ±10° (≈ 25 rayos) al frente ⇒ **0.30** (el caso que antes devolvía 1.0)
  - 10 rayos `NaN` contiguos más un obstáculo a 0.5 m (factor 0.231) en otro rayo ⇒ `min(0.231, 0.30) = 0.231`
- [ ] Test del cono partido (scan con `angle_min=0.0`, `angle_increment=2π/360`, todo `+inf`): `NaN` en los índices 355..359 y 0..4 (10 rayos, 10° contiguos al cruzar el índice 0) ⇒ **0.30**; `NaN` en los índices 355..359 y 5..9 (hueco real entre ellos con rayos válidos en 0..4) ⇒ **1.0** (dos tramos de 5°, no se suman)
- [ ] Tests de fallos dispersos: un rayo `NaN` de cada 10 (≈10% de fallos, sin tramos) ⇒ **1.0**; dos tramos separados de 5 rayos (4°) cada uno por rayos válidos ⇒ **1.0**
- [ ] Tests de clasificación: el tramo también se forma con `-inf`, con rayos finitos `< range_min` y con rayos finitos `> range_max`; un rayo `+inf` en medio de un hueco de 12 rayos lo parte en dos tramos que no alcanzan 8° ⇒ **1.0**
- [ ] Tests de combinación y no regresión: un hueco de 10 rayos con la fracción válida ≥ 0.70 degrada solo por contigüidad; el caso de TASK-010 (fracción < 0.70 sin tramos largos, p. ej. `NaN` repartidos uno de cada dos rayos) sigue dando **0.30**; todos los tests anteriores de `test_safety.py` siguen pasando **sin modificar sus aserciones**; si alguno debe cambiar, se explica en un comentario `# TASK-017` junto al test
- [ ] Tests de config: `max_invalid_run_rad` en `0`, `-0.1`, `NaN`, `inf`, `3.2` ⇒ `ValueError`; `math.pi` válido
- [ ] Test de `angle_increment == 0` y de `angle_increment` negativo: no lanza excepción y el resto de reglas siguen vigentes
- [ ] Test de propiedad (aleatorio con semilla fija, ≥ 500 scans): reemplazar un rayo libre (`+inf`) por `NaN` **nunca aumenta** el factor
- [ ] El módulo sigue sin importar `rclpy` ni mensajes ROS

## Verificación humana (fuera del DoD)
- Grabar scans reales del LD19 en el entorno de operación (pasillos, puertas y paredes de cristal, acero, superficies negras, personas con ropa oscura) y medir la distribución de longitudes de los tramos inválidos en situación normal. Con eso calibrar `max_invalid_run_rad`.
- **Efecto operativo a vigilar:** junto a paredes de cristal o equipos de acero pulido el robot puede quedar limitado a `degraded_factor` durante tramos largos del recorrido. Si es demasiado frecuente, hay que ajustar el umbral o complementar con otra fuente (sensores infrarrojos y de contacto del Create 3).
- Probar con el robot a baja velocidad frente a un objeto negro estrecho (≈ 35 cm de ancho a 1 m) y comprobar que reduce la velocidad.

## Fuera de alcance
Histéresis o filtrado temporal entre scans, parada completa por tramos largos, huecos que se extienden fuera del cono frontal, uso de los sensores del Create 3, cambios en `scan_builder.py` o en el supervisor.
