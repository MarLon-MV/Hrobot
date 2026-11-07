# TASK-006: Adapter — parser de tramas del LiDAR LD19

- **Rama Git:** `feat/TASK-006`
- **Depende de:** TASK-002
- **Prioridad:** P1

## Contexto y objetivo de negocio
El LD19 entrega tramas binarias por UART (230400 baud). Un parser robusto, sin dependencia de hardware ni ROS, permite probar y validar la percepción de forma offline y aísla los fallos de comunicación.

## Alcance técnico
Fuente de verdad: *LD19 Development Manual* v2.3+ de LDROBOT (el Youyeetoo FHL-LD19 usa el mismo protocolo). Trama LD19 (47 bytes, little-endian): `0x54` header · `0x2C` ver_len (3 bits superiores = tipo 1, 5 bits inferiores = 12 puntos) · velocidad (u16, °/s) · ángulo inicial (u16, centésimas de grado) · 12 × {distancia u16 mm, intensidad u8} · ángulo final (u16, centésimas de grado) · timestamp (u16, ms) · CRC8 (u8).
```python
# src/hrobot_core/hrobot_core/adapters/ld19_parser.py
@dataclass(frozen=True)
class LidarPoint: angle_rad: float; distance_m: float; intensity: int
@dataclass(frozen=True)
class LidarPacket: speed_dps: int; timestamp_ms: int; points: tuple[LidarPoint, ...]

class Ld19Parser:
    def feed(self, data: bytes) -> list[LidarPacket]: ...   # acepta flujo arbitrario (fragmentado/concatenado)
```
Reglas:
- Resincroniza buscando `0x54 0x2C`; descarta bytes basura.
- Verifica CRC8 (polinomio del datasheet LDROBOT LD19, tabla de 256 entradas); tramas con CRC inválido se descartan y se cuentan en `parser.crc_errors`.
- Los ángulos de los 12 puntos se interpolan linealmente entre inicial y final, manejando el cruce de 360°→0.
- Distancia 0 ⇒ punto inválido (se omite o marca según se documente en el docstring; elegir una y probarla).
- El buffer interno está acotado (máx. 4096 bytes) para evitar crecimiento ilimitado.
- Si la tabla CRC o el layout no se pueden verificar con el datasheet, **detenerse con `BLOCKED:`** en lugar de inventar valores.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/ld19_parser.py`
- `tests/adapters/__init__.py`
- `tests/adapters/test_ld19_parser.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/adapters/test_ld19_parser.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/adapters/test_ld19_parser.py --cov=hrobot_core.adapters.ld19_parser --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Tests: trama válida construida en el test (con CRC calculado), trama dividida en 2 y 3 fragmentos, dos tramas concatenadas, basura previa, CRC corrupto, cruce de 360°, buffer acotado con 1 MB de basura
- [ ] El módulo no importa `rclpy`, `serial` ni mensajes ROS

## Fuera de alcance
Lectura del puerto serie, publicación de `LaserScan`.
