# Reglas de Arquitectura y Convenciones Técnicas — Robot AMR Hospitalario

## 1. Contexto del sistema

Robot móvil autónomo (AMR) para distribución intrahospitalaria de medicamentos.

- **Base:** iRobot Create 3 (control diferencial, odometría, potencia) vía ROS 2 nativo (`/odom`, `/cmd_vel`, `/tf`).
- **Cómputo:** Raspberry Pi 5 (Ubuntu 24.04 aarch64) ejecutando Nav2, sensores y lógica de misión.
- **Percepción:** LiDAR 2D **Youyeetoo FHL-LD19** (LD19 de LDROBOT; UART 230400 baud, 10 Hz, 12 puntos/trama; `/scan`, `sensor_msgs/LaserScan`, frame `laser`). Referencia: *LD19 Development Manual* (v2.3+). Variante confirmada: **FHL-LD19** (no P ni Plus). Confirmar su adaptador USB-UART.
- **Localización:** AMCL sobre mapa estático. **Mapeo:** `slam_toolbox` (o Cartographer) con loop closure.
- **Supervisión:** RViz2 (tf2, scan, costmaps, plan).
- **Dominio crítico:** transporte de medicamentos ⇒ la seguridad prevalece sobre la velocidad de desarrollo.

## 2. Stack y versiones de referencia

| Componente | Versión / Herramienta |
| --- | --- |
| SO | Ubuntu 24.04 LTS |
| ROS 2 | **Jazzy jalisco** |
| Lenguaje | Python 3.12 (`rclpy`); C++17 solo si una tarea lo justifica |
| Build | `colcon`; `ament_cmake` (interfaces) + `ament_python` (core, bringup) |
| Navegación | `nav2_bringup`, `nav2_amcl`, `slam_toolbox` |
| Tests | `pytest >= 8`; `launch_testing` para integración |
| Formato / Lint | `ruff format`, `ruff check` (`E,F,I,B,UP,S,SIM,C4`) |
| Tipos | `mypy --strict` |
| Git | >= 2.40 |

## 3. Estructura del repositorio

El repo es un workspace de colcon con tres paquetes de responsabilidad única:

```
src/
  hrobot_interfaces/         # (ament_cmake) Mensajes, servicios y acciones personalizadas. Solo definiciones .msg/.srv/.action
  hrobot_core/               # (ament_python) Código limpio
    hrobot_core/
      domain/                # Lógica pura: geometría, estados de misión, políticas de seguridad. SIN rclpy, SIN I/O
      services/              # Casos de uso; dependen de puertos (Protocol), nunca de rclpy
      adapters/              # rclpy: nodos, pub/sub, drivers (LD19), clientes de acciones Nav2
  hrobot_bringup/            # (ament_python) SOLO launch/, config/, maps/. Sin lógica de negocio
tests/                       # pytest de domain/services/adapters puros (espeja src/hrobot_core/hrobot_core)
```

Reglas de dependencia entre paquetes (sin ciclos):

- `hrobot_interfaces` no depende de ningún otro paquete propio.
- `hrobot_core` puede depender de `hrobot_interfaces`; **solo `adapters`** lo importa. `domain` y `services` usan tipos propios.
- `hrobot_bringup` depende (en ejecución) de `hrobot_core` y de paquetes externos (`nav2_bringup`, `slam_toolbox`); nadie depende de él.
- Cambiar un `.msg/.srv/.action` es un cambio de contrato: requiere tarea propia y actualizar sus consumidores en el mismo PR.

Reglas de capas dentro de `hrobot_core` (solo hacia adentro):

- `domain` no importa `rclpy`, mensajes ROS ni `hrobot_interfaces`, ni hace I/O. Se prueba **sin ROS instalado**.
- `services` depende de `domain` y de puertos (`typing.Protocol`).
- `adapters` es la única capa que habla con ROS/hardware. Los nodos son delgados: traducen mensajes ↔ tipos de dominio y delegan.

## 4. Convenciones ROS 2

- Nombres: paquetes `hrobot_interfaces`, `hrobot_core`, `hrobot_bringup`; nodos `snake_case`; tópicos y parámetros declarados en un único lugar (constantes en `adapters/names.py`).
- Frames REP-105: `map → odom → base_link → laser`. No publicar `odom→base_link` (lo publica Create 3).
- Todo parámetro configurable se declara (`declare_parameter`) con tipo y valor por defecto; sin números mágicos en nodos.
- QoS explícita: sensores (`/scan`) `SensorDataQoS`; comandos con `reliable`.
- Tiempo: usar `node.get_clock()`; nunca `time.sleep` en callbacks. Soportar `use_sim_time`.
- Create 3 y la RPi5 deben compartir `ROS_DOMAIN_ID` y el mismo RMW; configurarlo en launch/env, no en código.
- `/cmd_vel` solo lo publica **un** nodo final (mux/supervisor de seguridad) para garantizar el freno de emergencia.

## 4b. Supervisión con RViz2

- La RPi5 opera **headless**: RViz2 se ejecuta en un PC de desarrollo en la misma red, con el mismo `ROS_DOMAIN_ID` y RMW. Nunca se lanza RViz en el robot durante operación normal (consume CPU).
- La configuración vive en `hrobot_bringup/rviz/` y debe ser coherente con los tópicos y frames de `config/` (validado por tests).
- Vistas mínimas: tf2, `/scan`, `/map`, costmaps local/global, `/particle_cloud`, `/plan`, `/local_plan`; herramientas `SetInitialPose` y `SetGoal`.
- Los scans se visualizan con QoS `Best Effort` (coincidiendo con `SensorDataQoS`).
- RViz es solo observación y calibración: no es parte del lazo de seguridad.

## 5. Seguridad funcional (obligatoria)

1. Watchdog: si `/scan` o `/odom` envejece más de un umbral configurable ⇒ `cmd_vel = 0`.
2. Parada por obstáculo: distancia mínima en el cono frontal < umbral ⇒ reducir/detener (lógica en `domain`, testeada con casos borde).
3. Estado `EMERGENCY_STOP` enclavado: solo un reset explícito lo libera.
4. Toda lógica de seguridad es pura, determinista y con tests exhaustivos (límites, NaN/inf en LaserScan, rangos fuera de `range_min/max`).
5. Los agentes **no pueden verificar hardware real**: el DoD debe ser verificable en unit tests o simulación; las pruebas con robot físico las ejecuta un humano y se registran fuera del DoD.

## 6. Principios de diseño

SOLID, inyección de dependencias por constructor, sin estado global, modelos inmutables (`dataclass(frozen=True)`), excepciones específicas (nunca `except:` desnudo), `logging`/`node.get_logger()` en vez de `print`, `pathlib` para rutas, sin secretos en código.

## 7. Estilo y calidad (gates antes de revisión)

```bash
ruff format --check src tests
ruff check src tests
mypy --strict src/hrobot_core/hrobot_core
PYTHONPATH=src/hrobot_core pytest -q --cov=hrobot_core --cov-fail-under=85
```

Funciones <= 40 líneas, complejidad <= 10, tipos en toda firma pública, docstrings (Google) en API pública. Cada bugfix incluye un test que falla sin el arreglo. Los archivos YAML de configuración se validan con tests (frames, tópicos, rangos).

## 8. Política de ramas y commits

- `main` **protegida**: sin commits directos ni force-push; merge solo vía PR aprobado por un humano.
- Ramas transitorias `feat/TASK-XXX` desde `main`; integración con **squash-merge**; se eliminan tras el merge.
- Conventional Commits con scope: `feat(TASK-XXX): ...`, `fix:`, `test:`, `refactor:`, `docs:`, `chore:`.
- Un PR = una tarea.
