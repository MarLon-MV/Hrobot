# TASK-010: Seguridad LiDAR — limitar velocidad cuando hay demasiados rayos inválidos

- **Rama Git:** `feat/TASK-010`
- **Depende de:** TASK-004
- **Prioridad:** P0

## Contexto y objetivo de negocio
`speed_factor` (TASK-004) ignora los rayos inválidos (`NaN`, `-inf`, fuera de rango). Pero superficies negras, cristal, acero pulido o ropa oscura no devuelven retorno, y entonces un obstáculo real puede aparecer como una zona de rayos inválidos rodeada de rayos libres, con lo que el robot seguiría a velocidad completa. Esta tarea añade una degradación: si en el cono frontal la fracción de rayos válidos es baja, la velocidad se **limita** (no se detiene) a un tope configurable. El objetivo es que el robot siga operando con prudencia ante datos dudosos.

Casos reales a cubrir (detectados en la revisión de TASK-004, todos devuelven hoy 1.0):
- Zona sin retorno (`NaN`) de ±0.2 rad al frente rodeada de rayos libres.
- Cono frontal casi entero en `NaN` con un solo rayo libre.
- Puntos con distancia 0 al frente (el LD19 marca así los puntos sin medida).

## Alcance técnico
Modificar `src/hrobot_core/hrobot_core/domain/safety.py` (sin crear módulos nuevos):

```python
@dataclass(frozen=True)
class SafetyConfig:
    ...  # campos existentes sin cambios
    min_valid_ratio: float = 0.70   # fracción mínima de rayos válidos en el cono, en (0, 1]
    degraded_factor: float = 0.30   # tope de velocidad cuando hay pocos rayos válidos, en (0, 1)
```

Reglas:
- **Fracción válida** = `rayos_válidos / rayos_totales_en_el_cono`. Rayo válido = obstáculo o `+inf` (libre), según la clasificación existente de TASK-004. Rayos totales = todos los que caen en `±front_half_angle`, válidos o no.
- Si `fracción_válida >= min_valid_ratio` ⇒ comportamiento actual, sin cambios.
- Si `0 < fracción_válida < min_valid_ratio` ⇒ el resultado es `min(factor_actual, degraded_factor)`. Es decir, **solo se baja la velocidad**: si hay un obstáculo cercano que ya exige un factor menor, manda ese.
- Si no hay ningún rayo válido (fracción 0), cono vacío, scan caducado o geometría inválida ⇒ **0.0**, igual que en TASK-004 (no se modifica).
- Validación en `__post_init__`: `min_valid_ratio` finito en `(0, 1]` y `degraded_factor` finito en `(0, 1)`; si no, `ValueError`.
- Calcular la fracción con `rayos_válidos / rayos_totales >= min_valid_ratio` (sin igualdades frágiles entre floats; documentar en el docstring que el límite es inclusivo: justo en el umbral **no** se degrada).
- Actualizar el docstring de `speed_factor` y de `SafetyConfig` con la nueva regla.
- Las constantes `0.70` y `0.30` son valores iniciales a calibrar con scans reales (ver "Verificación humana"); deben quedar como valores por defecto del dataclass, no como números mágicos en la función.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/safety.py`
- `tests/domain/test_safety.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/domain/test_safety.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_safety.py --cov=hrobot_core.domain.safety --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Tests con un scan de 100 rayos dentro del cono (`angle_min=-0.5`, `angle_increment=0.01`) y config por defecto:
  - 30 `NaN` + 70 `+inf` (fracción 0.70 exacta) ⇒ **1.0** (el límite es inclusivo)
  - 31 `NaN` + 69 `+inf` ⇒ **0.30**
  - 29 `NaN` + 71 `+inf` ⇒ **1.0**
  - 99 `NaN` + 1 `+inf` ⇒ **0.30** (antes devolvía 1.0)
  - 100 `NaN` ⇒ **0.0** (sin cambios)
  - 40 distancias `0.0` (menores que `range_min`) + 60 `+inf` ⇒ **0.30**
- [ ] Tests de combinación: 40 `NaN` + obstáculo a 0.5 m ⇒ `min(factor_obstáculo, 0.30)` (manda el menor); 40 `NaN` + obstáculo a 0.35 m ⇒ 0.0; 10 `NaN` + obstáculo a 0.675 m ⇒ 0.5 (sin degradación)
- [ ] Tests de validación de `SafetyConfig`: `min_valid_ratio` en `0`, `-0.1`, `1.1`, `NaN`, `inf` ⇒ `ValueError`; `degraded_factor` en `0`, `1`, `1.5`, `NaN` ⇒ `ValueError`; `min_valid_ratio=1.0` y `degraded_factor=0.99` válidos
- [ ] Un test construye un scan realista de 450 rayos (`angle_min=-π`, `angle_increment=2π/450`) con una banda de ±0.2 rad de `NaN` al frente rodeada de `+inf` y verifica que el resultado es **0.30** y no 1.0
- [ ] Los tests existentes de TASK-004 siguen pasando. Si alguno depende de un scan con muchos rayos inválidos y ahora debe esperar otro valor, se ajusta su expectativa o su scan, y se explica el motivo en un comentario junto al test
- [ ] El módulo sigue sin importar `rclpy` ni mensajes ROS

## Verificación humana (fuera del DoD)
- Grabar scans reales del LD19 en el entorno de operación (pasillos, puertas de cristal, superficies oscuras, personas con ropa oscura) y comprobar qué fracción de rayos inválidos aparece en cada caso.
- Con esos datos, calibrar `min_valid_ratio` y `degraded_factor`. Los valores 0.70 y 0.30 son una primera estimación, no un valor validado.
- Probar con el robot a baja velocidad frente a un obstáculo oscuro antes de habilitar movimiento autónomo.

## Fuera de alcance
Histéresis o filtrado temporal entre scans, cambio de la clasificación de rayos de TASK-004, publicación en `/cmd_vel`, nodos ROS, lectura de scans grabados.
