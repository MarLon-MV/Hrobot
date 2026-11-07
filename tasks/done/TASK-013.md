# TASK-013: Constructor de scans de rejilla fija a partir de las tramas del LD19

- **Rama Git:** `feat/TASK-013`
- **Depende de:** TASK-006
- **Prioridad:** P1

## Contexto y objetivo de negocio
`slam_toolbox` descarta scans cuyo número de lecturas varía de una vuelta a otra, y hay reportes de que el driver de la comunidad del LD19 lo provoca (repite 0°/360° y emite longitud variable). El LD19 entrega ~450 puntos por vuelta, con distribución irregular. Este módulo convierte el flujo de `LidarPacket` (TASK-006) en scans **siempre de la misma longitud**, con una semántica de rangos coherente con REP-117 y con la política de seguridad (TASK-004/010). Es lógica pura, sin ROS.

## Contexto de convenciones (importante)
- El parser ya entrega ángulos en convención ROS (antihorario, `[0, 2π)`). El LD19 gira en sentido horario, así que **dentro de una vuelta los ángulos ROS van decreciendo** y al cerrar la vuelta saltan de ≈0 a ≈2π.
- La orientación de montaje se corrige en el TF; este módulo no aplica offsets.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/adapters/scan_builder.py   (sin imports de ROS)
@dataclass(frozen=True)
class ScanBuilderConfig:
    bins: int = 360              # rayos por scan; entero en [90, 3600]
    range_min: float = 0.02      # m, > 0
    range_max: float = 12.0      # m, > range_min  (alcance nominal del LD19: verificar con datasheet)
    max_points_per_revolution: int = 2000   # tope de seguridad para vueltas sin cierre

@dataclass(frozen=True)
class ScanFrame:
    ranges: tuple[float, ...]        # len == bins, ángulo creciente desde angle_min
    intensities: tuple[float, ...]   # len == bins; 0.0 donde no hay dato
    angle_min: float                 # 0.0
    angle_max: float                 # 2π - angle_increment
    angle_increment: float           # 2π / bins
    range_min: float
    range_max: float
    scan_time_s: float               # duración de la vuelta según timestamps del LD19
    valid_ratio: float               # fracción de bins con dato finito o +inf (no NaN)

class ScanBuilder:
    dropped_revolutions: int
    def __init__(self, cfg: ScanBuilderConfig = ...) -> None: ...
    def add_packet(self, packet: LidarPacket) -> list[ScanFrame]: ...   # 0 o 1 scans normalmente
    def reset(self) -> None: ...
```

### Reglas de asignación a rejilla
- `ScanBuilderConfig` inválida (bins fuera de rango o no entero, rangos no finitos o `range_min <= 0`, `range_max <= range_min`, `max_points_per_revolution <= 0`) ⇒ `ValueError`.
- Cada punto cae en el bin `round(angle_rad / angle_increment) % bins` (el bin `i` representa el ángulo `i * angle_increment`).
- Contribución de cada punto al bin:
  - `valid=False` (sin retorno) ⇒ **no aporta** (el bin queda NaN salvo que otro punto aporte).
  - `valid=True` y `distance_m < range_min` ⇒ no aporta (zona ciega).
  - `valid=True` y `distance_m > range_max` ⇒ aporta `+inf` (sin obstáculo dentro del alcance).
  - `valid=True` y dentro de `[range_min, range_max]` ⇒ aporta su distancia.
- Valor final del bin: si algún aporte finito ⇒ **el mínimo** (el obstáculo más cercano); si no, si hay aporte `+inf` ⇒ `+inf`; si no hay aportes ⇒ `NaN` (**nunca** se asume "libre" donde no hay dato; esto es coherente con TASK-010).
- `intensities[i]` = intensidad del punto elegido para el bin (0.0 si el bin es NaN o `+inf`).
- `valid_ratio` = bins no-NaN / bins.

### Cierre de vueltas
- Se detecta el cierre cuando el ángulo ROS de un punto **sube** más de π respecto al punto anterior (salto ≈0 → ≈2π). Los puntos posteriores al salto pertenecen a la vuelta siguiente (también dentro de un mismo paquete).
- **Solo se emiten vueltas completas.** Los puntos anteriores al primer cierre se descartan (vuelta parcial de arranque).
- `scan_time_s` = `((ts_último_paquete − ts_primer_paquete) mod 30000) / 1000` (el timestamp del LD19 se reinicia en 30000 ms).
- Si una vuelta acumula más de `max_points_per_revolution` puntos sin cierre, se descarta, `dropped_revolutions += 1` y se reinicia el acumulador.
- Sin dependencias de ROS: solo librería estándar y `ld19_parser`.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/scan_builder.py`
- `tests/adapters/test_scan_builder.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/adapters/test_scan_builder.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/adapters/test_scan_builder.py --cov=hrobot_core.adapters.scan_builder --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Test de longitud fija: con vueltas sintéticas de 440, 450 y 462 puntos, todos los `ScanFrame` emitidos tienen `len(ranges) == bins` y `len(intensities) == bins`
- [ ] Test anti-espejo de extremo a extremo: se generan tramas LD19 crudas (con CRC calculado en el test) que cubren 360° en sentido horario con todo a 3.0 m salvo un obstáculo a 1.0 m en el ángulo LD19 de 90° (lado derecho); se pasan por `Ld19Parser` y `ScanBuilder`; el scan resultante tiene `ranges[270]` ≈ 1.0 (±1 bin) y `ranges[90]` ≈ 3.0
- [ ] Test de semántica: punto inválido ⇒ NaN; punto `< range_min` ⇒ NaN; punto `> range_max` ⇒ `+inf`; dos puntos en el mismo bin (2.0 m y 1.5 m) ⇒ 1.5; bin vacío ⇒ NaN; `valid_ratio` calculado correctamente
- [ ] Test de vueltas: la vuelta parcial inicial no se emite; se emite exactamente un scan por cada cierre; el cierre dentro de un mismo paquete reparte los puntos correctamente entre las dos vueltas; `scan_time_s` correcto, incluido el salto del timestamp en 30000
- [ ] Test de `max_points_per_revolution`: un flujo sin cierre incrementa `dropped_revolutions` y no emite nada
- [ ] Tests de validación de `ScanBuilderConfig` (cada condición inválida ⇒ `ValueError`)
- [ ] Test de `reset()` y de que `angle_min == 0.0`, `angle_increment == 2π/bins`, `angle_max == 2π − angle_increment`
- [ ] El módulo no importa `rclpy`, mensajes ROS ni `serial`

## Verificación humana (fuera del DoD)
- Con el sensor real: comprobar que el scan no sale espejado en RViz (un obstáculo a la derecha del robot aparece a la derecha) y que `slam_toolbox` ya no descarta scans.
- Ajustar `bins` si la densidad real de puntos produce demasiados bins NaN (medir `valid_ratio` en un pasillo despejado) y confirmar el alcance máximo real del sensor.

## Fuera de alcance
Publicación ROS, lectura serie, offsets de montaje, interpolación de bins vacíos, filtrado temporal.
