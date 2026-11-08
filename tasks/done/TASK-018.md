# TASK-018: Supervisor — reiniciar la rampa de velocidad tras una pérdida de supervisión

- **Rama Git:** `feat/TASK-018`
- **Depende de:** TASK-015
- **Prioridad:** P0

## Contexto y objetivo de negocio
`SafetySupervisor.step` limita cuánto puede **subir** el factor de velocidad: `factor <= factor_previo + max_rise_per_s × dt`, con `dt` = tiempo desde el paso anterior. La revisión de TASK-015 mostró que `dt` no tiene tope: tras un hueco largo sin pasos (el nodo se queda sin CPU, el reloj salta, una pausa del ejecutor), el siguiente paso con todo libre da **factor 1.0 de golpe**, sin rampa. Casos reproducidos:
- Robot parado y 10 s sin pasos, luego todo libre ⇒ velocidad completa en el primer paso.
- Emergencia, 5 s sin pasos, luego estado `NAVIGATING_TO_DESTINATION` ⇒ velocidad completa en el primer paso.

En operación normal el nodo da pasos a 20 Hz (`dt ≈ 0.05 s`) y esto no ocurre. Pero un hueco largo indica que el supervisor **dejó de supervisar**, y al volver el robot debe arrancar con suavidad, igual que en el primer paso tras el arranque.

## Alcance técnico
Modificar `src/hrobot_core/hrobot_core/services/safety_supervisor.py`. La API pública existente no cambia.

```python
@dataclass(frozen=True)
class SupervisorConfig:
    ...  # campos existentes sin cambios
    max_step_gap_s: float = 0.25   # hueco máximo entre pasos antes de considerar pérdida de supervisión; finito y > 0
```

Regla nueva en `step`:
- `gap = now_s − instante_del_paso_anterior` (solo si hay un paso anterior; el cálculo actual con reloj que retrocede se conserva: `gap` nunca es negativo y el instante de referencia sigue siendo el máximo visto).
- Si `gap > max_step_gap_s` (**estrictamente mayor**; un hueco exactamente igual **no** reinicia) ⇒ pérdida de supervisión: el factor previo interno pasa a `0.0` y el `dt` de ese paso se toma como `0.0`. Es decir, el paso se comporta como el **primer paso**: la salida es `(0.0, 0.0)` con `reason = RAMP_LIMITED` (si no hay otra causa de parada) y a partir del paso siguiente la rampa sube con normalidad.
- Se aplica antes de calcular la rampa y en todos los modos (incluido `require_mission_status=False`).
- Las reglas de parada 1–5 no cambian y siguen teniendo prioridad (si hay una causa de parada, el reinicio de la rampa ocurre igualmente, porque una parada ya deja el factor previo en 0).
- `SupervisorConfig` con `max_step_gap_s` no finito o `<= 0` ⇒ `ValueError`.
- Actualizar los docstrings de `SupervisorConfig` y `SafetySupervisor.step`, explicando que el valor debe ser **mayor que el periodo de paso** del nodo (con pasos a 20 Hz, 0.05 s; recomendado al menos el doble) y que un valor menor que el periodo haría que el robot nunca se mueva.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/services/safety_supervisor.py`
- `tests/services/test_safety_supervisor.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/services/test_safety_supervisor.py` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/services/test_safety_supervisor.py --cov=hrobot_core.services.safety_supervisor --cov-fail-under=98 -q` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Test del primer escenario reproducido: orden y scan libres y estado `NAVIGATING_TO_DESTINATION`; un paso en `t=0` con obstáculo (parado); luego un paso en `t=10` con todo libre ⇒ salida `(0.0, 0.0)`, `factor == 0.0`, `reason == RAMP_LIMITED`; y los pasos siguientes cada 0.1 s suben el factor 0.1 por paso hasta 1.0
- [ ] Test del segundo escenario: estado `EMERGENCY_STOP` en `t=0` (paso), luego en `t=5` llega `NAVIGATING_TO_DESTINATION` con orden y scan válidos ⇒ el primer paso da `(0.0, 0.0)` con `RAMP_LIMITED` (no velocidad completa), y después sube con rampa
- [ ] Test de hueco desde velocidad completa: tras llegar a factor 1.0 con pasos cada 0.1 s, un paso con hueco de 1.0 s (inputs frescos) ⇒ factor 0.0 (`RAMP_LIMITED`) y luego rampa
- [ ] Test del límite: hueco exactamente `0.25` s (con `max_step_gap_s=0.25`) **no** reinicia (el factor puede subir hasta `prev + max_rise_per_s × 0.25`); hueco de `0.2500001` s sí reinicia
- [ ] Test de funcionamiento normal: pasos a 20 Hz (cada 0.05 s) durante 3 s con todo libre desde reposo: no hay ningún reinicio espurio y el factor llega a 1.0 en torno a 1 s
- [ ] Test con reloj que retrocede y luego avanza (p. ej. `t = 1.0, 0.5, 1.1`): el hueco se mide contra el instante máximo visto (1.0), no genera reinicio y el factor nunca sube más de lo permitido
- [ ] Test del modo de banco (`require_mission_status=False`): el reinicio también se aplica
- [ ] Tests de validación de `SupervisorConfig`: `max_step_gap_s` en `0`, `-1.0`, `NaN`, `inf` ⇒ `ValueError`; `0.05` y `1.0` válidos
- [ ] Todos los tests anteriores de `test_safety_supervisor.py` siguen pasando. Si alguno usa pasos de tiempo mayores que `0.25` s esperando que la rampa avance, se corrige **su intervalo de tiempo o su configuración (`max_step_gap_s`)**, nunca se debilita la aserción, y se explica con un comentario `# TASK-018` junto al test
- [ ] El test de propiedad existente sigue cumpliendo que el factor nunca sube más de `max_rise_per_s × dt` entre pasos, y se amplía: tras cualquier hueco `> max_step_gap_s` el factor del primer paso es `0.0`
- [ ] El módulo sigue sin importar `rclpy`, mensajes ROS ni `serial`

## Verificación humana (fuera del DoD)
- En el nodo real (TASK-016), provocar una pausa del supervisor (por ejemplo, saturar la CPU brevemente o pausar el proceso con `SIGSTOP`/`SIGCONT`) con el robot **elevado** y comprobar que, al reanudar, las ruedas aceleran de forma progresiva y no de golpe.
- Confirmar que el periodo de paso real del nodo es claramente menor que `max_step_gap_s`.

## Fuera de alcance
El nodo ROS (TASK-016), cambios en la política de scan, el modo de giro con factor 0, limitación de la desaceleración.
