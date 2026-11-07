# TASK-015: Supervisor de seguridad — lógica pura de limitación de velocidad

- **Rama Git:** `feat/TASK-015`
- **Depende de:** TASK-004, TASK-005
- **Prioridad:** P0

## Contexto y objetivo de negocio
Nav2 genera órdenes de velocidad, pero según `architecture_rules.md` **solo un nodo final publica `/cmd_vel`**: el supervisor de seguridad. Este módulo es su núcleo, puro y sin ROS: recibe la orden de Nav2, el último scan y el estado de la misión, y decide la velocidad final. Cualquier duda (dato ausente, caducado, inválido) debe resultar en **velocidad cero**. Es la barrera que protege a personas y equipos en el hospital, por lo que se prueba de forma exhaustiva.

## Alcance técnico
```python
# src/hrobot_core/hrobot_core/services/safety_supervisor.py   (sin imports de ROS)
@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float      # m/s
    angular_z: float     # rad/s

@dataclass(frozen=True)
class SupervisorConfig:
    max_linear: float = 0.3          # m/s, tope duro, > 0
    max_angular: float = 1.0         # rad/s, tope duro, > 0
    cmd_timeout_s: float = 0.5       # edad máxima de la orden de Nav2
    mission_timeout_s: float = 2.0   # edad máxima del estado de misión
    require_mission_status: bool = True   # False solo para pruebas de banco
    max_rise_per_s: float = 1.0      # cuánto puede SUBIR el factor por segundo; bajar es instantáneo
    safety: SafetyConfig = field(default_factory=SafetyConfig)   # de domain/safety.py

class StopReason(Enum):
    NONE; EMERGENCY; MISSION_NOT_MOVING; NO_MISSION_STATUS; MISSION_STATUS_STALE
    NO_COMMAND; COMMAND_STALE; INVALID_COMMAND; NO_SCAN; SCAN_BLOCKED; RAMP_LIMITED

@dataclass(frozen=True)
class SupervisorOutput:
    command: VelocityCommand
    factor: float            # factor final aplicado, en [0, 1]
    reason: StopReason
    reverse_blocked: bool    # True si la orden pedía marcha atrás y se anuló

class SafetySupervisor:
    def __init__(self, cfg: SupervisorConfig = ...) -> None: ...
    def on_command(self, cmd: VelocityCommand, now_s: float) -> None: ...
    def on_scan(self, ranges: Sequence[float], angle_min: float, angle_increment: float,
                range_min: float, range_max: float, now_s: float) -> None: ...
    def on_mission_state(self, state: MissionState | None, now_s: float) -> None: ...
    def step(self, now_s: float) -> SupervisorOutput: ...
```
`now_s` es un reloj en segundos (el del nodo). Todo `now_s` no finito ⇒ `ValueError`. `SupervisorConfig` inválida (valores no finitos o `<= 0`) ⇒ `ValueError`.

### Reglas de `step`, en este orden (la primera que se cumple fija el resultado)
Cuando una regla 1–5 se cumple, la salida es `VelocityCommand(0.0, 0.0)`, `factor=0.0`, se pone el factor interno previo a 0 (la reanudación posterior parte de reposo) y se devuelve el `reason` indicado.
1. **Estado de misión** (solo si `require_mission_status`):
   - Nunca se recibió estado ⇒ `NO_MISSION_STATUS`.
   - Estado con edad `> mission_timeout_s` (o negativa) ⇒ `MISSION_STATUS_STALE`.
   - Estado `EMERGENCY_STOP` ⇒ `EMERGENCY`.
   - Estado distinto de `NAVIGATING_TO_DESTINATION` y `RETURNING_TO_BASE` ⇒ `MISSION_NOT_MOVING`.
   - `on_mission_state(None, ...)` equivale a "sin estado válido" ⇒ `NO_MISSION_STATUS`.
   - **Modo de banco (`require_mission_status=False`):** se relajan solo `NO_MISSION_STATUS`, `MISSION_STATUS_STALE` y `MISSION_NOT_MOVING`. La emergencia **nunca** se ignora: si el último estado recibido es `EMERGENCY_STOP` (sea cual sea su edad) ⇒ `EMERGENCY`, y permanece así hasta que llegue un estado posterior distinto de `EMERGENCY_STOP`.
2. **Orden de Nav2:** ninguna recibida ⇒ `NO_COMMAND`; edad `> cmd_timeout_s` o negativa ⇒ `COMMAND_STALE`.
3. **Orden inválida:** `linear_x` o `angular_z` no finitos (`NaN`, `±inf`) ⇒ `INVALID_COMMAND`.
4. **Scan:** ninguno recibido ⇒ `NO_SCAN`.
5. **Factor de seguridad:** se construye un `ScanData` con `age_s = now_s − instante_de_recepción` y se calcula `factor_seg = speed_factor(scan, cfg.safety)`. Si `factor_seg == 0.0` ⇒ `SCAN_BLOCKED`.
6. **Limitación de subida (rampa):** `dt = max(0, now_s − instante_del_paso_anterior)` (0 en el primer paso). Si `factor_seg <= factor_previo` ⇒ `factor = factor_seg` (**bajar es instantáneo**); si no ⇒ `factor = min(factor_seg, factor_previo + max_rise_per_s * dt)`. El factor previo inicial es 0.0, así que tras el arranque o tras cualquier parada el robot acelera de forma progresiva. Si `factor < factor_seg` el `reason` es `RAMP_LIMITED`; si no, `NONE`.
7. **Orden final:**
   - `linear_x`: si `cmd.linear_x < 0` ⇒ se anula (0.0) y `reverse_blocked=True` (**el robot no retrocede**); si no, `min(cmd.linear_x, max_linear)`.
   - `angular_z`: acotada a `±max_angular`.
   - Ambas se multiplican por `factor`. (Decisión de diseño conservadora: el giro también se atenúa; con `factor == 0` el robot no gira. Puede relajarse en una tarea futura.)
- Los valores de salida nunca son `NaN` ni exceden `max_linear` / `max_angular` en valor absoluto, **sea cual sea la entrada**.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/services/safety_supervisor.py`
- `tests/services/__init__.py`
- `tests/services/test_safety_supervisor.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/services/test_safety_supervisor.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/services/test_safety_supervisor.py --cov=hrobot_core.services.safety_supervisor --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Un test por cada `StopReason` demuestra que se produce en el caso descrito y que la salida es `(0.0, 0.0)` (salvo `NONE` y `RAMP_LIMITED`)
- [ ] Test de prioridad: con varias condiciones a la vez (p. ej. `EMERGENCY` + scan bloqueado + orden caducada) se devuelve el `reason` de la regla de menor número
- [ ] Test del flujo feliz: estado `NAVIGATING_TO_DESTINATION`, orden `(0.2, 0.5)`, scan libre: el primer paso devuelve 0 (rampa); con pasos posteriores cada 0.1 s el factor sube 0.1 por paso hasta 1.0 y la salida llega a `(0.2, 0.5)`
- [ ] Test de bajada instantánea: con factor 1.0, un obstáculo a `stop_distance` produce `(0.0, 0.0)` en **el mismo paso**; al retirarlo, el factor sube con la rampa (no salta a 1.0)
- [ ] Test de marcha atrás: `linear_x = -0.2` ⇒ `linear_x == 0.0`, `reverse_blocked=True`; el giro sigue permitido (atenuado por el factor)
- [ ] Tests de topes: `linear_x = 5.0` ⇒ `<= max_linear`; `angular_z = -9.0` ⇒ `>= -max_angular`
- [ ] Tests de datos inválidos: `NaN`/`inf` en la orden ⇒ `INVALID_COMMAND`; `now_s` `NaN` ⇒ `ValueError`; reloj que retrocede (`now_s` menor que el anterior) ⇒ `dt = 0` sin excepción y sin factor que suba; edad negativa de orden/estado/scan ⇒ tratada como caducada
- [ ] Tests del modo de banco (`require_mission_status=False`): sin estado de misión y con scan y orden válidos, el robot se mueve; con estado `IDLE` también se mueve; con estado `EMERGENCY_STOP` recibido (incluso caducado) la salida es `(0.0, 0.0)` con `reason == EMERGENCY`; tras recibir después un estado `NAVIGATING_TO_DESTINATION` vuelve a moverse (con rampa)
- [ ] Test de propiedad (aleatorio con semilla fija, ≥ 2000 combinaciones de estado de misión, edades, órdenes con `NaN`/`inf`/negativas/enormes, scans con `NaN`/`inf`/vacíos): la salida es finita, `0 <= linear_x <= max_linear`, `|angular_z| <= max_angular`, `0 <= factor <= 1`; y **siempre `(0.0, 0.0)` cuando falla cualquiera de las precondiciones de las reglas 1–5**
- [ ] Tests de validación de `SupervisorConfig` (cada campo inválido ⇒ `ValueError`)
- [ ] El módulo no importa `rclpy`, mensajes ROS ni `serial`

## Verificación humana (fuera del DoD)
- Revisión humana obligatoria del diff completo antes del merge: es la barrera de seguridad del robot.
- Con el robot elevado (ruedas sin tocar el suelo) y luego en un área despejada: confirmar parada inmediata ante un obstáculo, parada al cortar la orden de Nav2, parada con la misión en `EMERGENCY_STOP`, y aceleración progresiva tras cada parada.
- Decidir si el giro en el sitio debe permitirse con `factor == 0` (hoy está bloqueado).

## Fuera de alcance
Nodo ROS (TASK-016), lectura de la odometría, Collision Monitor de Nav2, límites de aceleración del Create 3, recuperación tras parada de emergencia (la gestiona la máquina de estados de misión).
