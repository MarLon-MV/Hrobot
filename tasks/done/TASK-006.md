# TASK-006: Adapter — parser de tramas del LiDAR LD19

- **Rama Git:** `feat/TASK-006`
- **Depende de:** TASK-002
- **Prioridad:** P1

## Contexto y objetivo de negocio
El LD19 (Youyeetoo FHL-LD19) entrega tramas binarias por UART. Un parser robusto, sin dependencia de hardware ni ROS, permite probar y validar la percepción de forma offline y aísla los fallos de comunicación. Un error aquí (CRC, ángulos, sentido de giro) contamina todo lo que venga después: SLAM, AMCL y la política de seguridad.

## Protocolo (verificado contra *LD19 Development Manual* v2.3, secciones 2, 3 y 4)
- UART 230400 baud, 8 bits, 1 stop, sin paridad, sin control de flujo. Comunicación unidireccional (el sensor emite sin recibir comandos).
- Trama de **47 bytes**, little-endian (LSB primero):

| Offset | Tamaño | Campo | Notas |
|---|---|---|---|
| 0 | 1 | header | fijo `0x54` |
| 1 | 1 | ver_len | fijo `0x2C` (3 bits altos = tipo 1; 5 bits bajos = 12 puntos) |
| 2 | 2 | speed | u16, grados/segundo |
| 4 | 2 | start_angle | u16, centésimas de grado |
| 6 | 36 | 12 × punto | cada punto: distancia u16 (mm) + intensidad u8 |
| 42 | 2 | end_angle | u16, centésimas de grado |
| 44 | 2 | timestamp | u16, ms; cuenta hasta 30000 y vuelve a empezar |
| 46 | 1 | crc8 | sobre los 46 bytes anteriores |

- **CRC-8:** polinomio `0x4D`, valor inicial `0x00`, sin reflexión de bits, sin XOR final. Algoritmo del fabricante: `crc = TABLE[(crc ^ byte) & 0xFF]` para cada byte. La tabla de 256 entradas del manual es exactamente la generada con ese polinomio (`TABLE[i]` = CRC de un solo byte `i`, procesando 8 bits MSB primero). Se genera en el módulo con una función; **no copiar 256 valores a mano**.
- **Ángulo de cada punto:** `step = (end_angle - start_angle) / (12 - 1)`; `angle_i = start_angle + step * i`, con `i` en `[0, 12)`. Si `end_angle < start_angle` (cruce por 360°), sumar 36000 a `end_angle` antes de calcular.
- **Sistema de coordenadas del LD19 (sección 4):** *levógiro*, 0° = frente del sensor, el ángulo **aumenta en sentido horario**. ROS (REP-103) es dextrógiro y el ángulo aumenta en sentido **antihorario**. Hay que convertir: `angle_ros = (2π − angle_ld19) mod 2π`. Si no se convierte, el scan queda espejado izquierda/derecha.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/adapters/ld19_parser.py
@dataclass(frozen=True)
class LidarPoint:
    angle_rad: float     # convención ROS (antihorario), normalizado a [0, 2π)
    distance_m: float    # metros; 0.0 si el punto es inválido
    intensity: int       # 0-255, tal cual
    valid: bool          # False si la distancia cruda es 0 (sin retorno)

@dataclass(frozen=True)
class LidarPacket:
    speed_dps: int
    timestamp_ms: int
    points: tuple[LidarPoint, ...]   # siempre 12, en el orden de la trama

class Ld19Parser:
    crc_errors: int                  # tramas descartadas por CRC
    @property
    def buffered_bytes(self) -> int: ...
    def feed(self, data: bytes) -> list[LidarPacket]: ...   # acepta flujo arbitrario
```
Reglas del parser:
- **Resincronización:** busca `0x54 0x2C`; descarta los bytes anteriores. Si hay `0x54` pero el siguiente byte no es `0x2C`, avanza 1 byte.
- **CRC inválido:** cuenta `crc_errors += 1` y descarta **solo el primer byte** (el supuesto header), no los 47, para no perder una trama real que empiece dentro de esos bytes.
- **Trama incompleta:** espera más datos (acepta fragmentos de cualquier tamaño, incluso de 1 byte).
- **Memoria acotada:** entre llamadas a `feed`, el buffer retiene como máximo 46 bytes. Entradas grandes se procesan en trozos de ≤ 4096 bytes. Una entrada de 1 MB de basura no debe hacer crecer el buffer.
- **Puntos inválidos:** distancia cruda 0 ⇒ `valid=False`, `distance_m=0.0`. Se **conservan** los 12 puntos para no perder la rejilla angular; quien consuma decide qué hacer (TASK-004/010 tratan los inválidos como tales).
- Sin dependencias de hardware: no importa `rclpy`, `serial` ni mensajes ROS. Solo librería estándar.

## Vector de prueba oficial (manual v2.3, sección 3.3)
Trama (47 bytes, hexadecimal):
```
54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50
```
Valores que el parser debe producir (calculados de los bytes crudos; **la tabla de ejemplo impresa en el manual tiene erratas en los puntos 2 y 12, no usarla**):
- `speed_dps = 2152`, `timestamp_ms = 6714` (`0x1A3A`), CRC esperado `0x50`.
- `start_angle = 32427` (324.27°), `end_angle = 33470` (334.70°).
- Distancias en mm: `[224, 220, 217, 213, 211, 208, 205, 202, 199, 197, 194, 192]`.
- Intensidades: `[228, 226, 229, 227, 228, 233, 228, 226, 233, 229, 229, 229]`.
- Ángulo LD19 del primer punto 324.27° ⇒ `angle_rad` ROS = `(2π − radians(324.27)) mod 2π` ≈ 0.6236 rad; del último, 334.70° ⇒ ≈ 0.4416 rad (tolerancia `abs=1e-3`).

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/adapters/ld19_parser.py`
- `tests/adapters/__init__.py`
- `tests/adapters/test_ld19_parser.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/adapters/test_ld19_parser.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/adapters/test_ld19_parser.py --cov=hrobot_core.adapters.ld19_parser --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check src tests`, `ruff format --check src tests` y `mypy --strict src/hrobot_core/hrobot_core` terminan con código 0
- [ ] Test del vector oficial: el parser devuelve exactamente los valores listados arriba (velocidad, timestamp, 12 distancias, 12 intensidades, ángulos ROS con `pytest.approx`)
- [ ] Test de la tabla CRC: la tabla generada empieza por `00 4D 9A D7 79 34 E3 AE` y termina en `5A 06 4B 9C D1 7F 32 E5 A8`; el CRC de los primeros 46 bytes del vector oficial es `0x50`
- [ ] Test de conversión de ángulos: un punto a 90° (LD19, horario) produce `3π/2` en convención ROS; 0° produce `0.0`; el resultado siempre está en `[0, 2π)`
- [ ] Test de cruce de 360°: `start_angle=35000`, `end_angle=500` produce ángulos monótonos que atraviesan 0° sin saltos (construyendo la trama en el test con su CRC calculado)
- [ ] Tests de flujo: trama dividida en 2 y en 3 fragmentos, entregada byte a byte, dos tramas concatenadas, basura antes y entre tramas, falso header (`0x54` seguido de otro byte), CRC corrupto (cuenta `crc_errors` y no emite), trama válida que empieza dentro de los bytes de una trama corrupta (se recupera)
- [ ] Test de puntos inválidos: distancia cruda 0 ⇒ `valid=False` y `distance_m == 0.0`, los otros puntos de la trama siguen válidos
- [ ] Test de memoria: tras `feed` de 1 MB de basura, `buffered_bytes <= 46`
- [ ] El módulo no importa `rclpy`, `serial` ni mensajes ROS: `grep -rE "import (rclpy|serial)" src/hrobot_core/hrobot_core/adapters/ld19_parser.py` sin coincidencias

## Verificación humana (fuera del DoD)
- Conectar el FHL-LD19 al adaptador USB-UART y comprobar que llegan tramas a 230400 8N1 y que el CRC pasa (con el sensor real, `crc_errors` debe mantenerse en 0 o casi).
- Con el robot y el nodo ROS futuro, comprobar en RViz que el scan **no está espejado**: un obstáculo a la izquierda del robot debe aparecer a la izquierda. La orientación de montaje del sensor se corrige en el TF (`laser_yaw`), no en el parser.
- Comprobar que el sensor alcanza ~10 Hz de rotación (campo `speed` ≈ 3600 °/s) tras unos segundos de arranque.

## Fuera de alcance
Lectura del puerto serie, publicación de `LaserScan`, relleno de rejilla angular fija, filtrado de intensidad, control PWM del motor.
