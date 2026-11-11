# 02 · Nodo de misión (bloqueado por decisiones de producto)

Hoy existen `domain/mission.py` (máquina de estados), el mensaje `MissionStatus` y la suscripción del supervisor, pero **no hay ningún nodo que cree misiones, mande objetivos a Nav2 ni publique el estado**. Sin él, con `require_mission_status:=true` el robot no se mueve.

## Decisiones que faltan (humano)
1. **Cómo se solicita una entrega y cómo se eligen los destinos.** ¿Interfaz (web/tablet/línea de comandos), servicio/acción ROS, lista fija de destinos (habitaciones) con sus poses en el mapa?
2. **Qué confirma la entrega.** ¿Botón físico, PIN/tarjeta del personal, pantalla, sensor de peso del cajón?
3. **Emergencia.** ¿Qué la dispara (botón físico, comando remoto, fallo de sensor)? Propuesta: el reset solo lo puede hacer un humano.
4. **Tiempo máximo de espera de la confirmación** y qué ocurre al agotarse (¿vuelve a base con la medicación?).
5. **Auditoría.** ¿Qué se registra (quién, qué, cuándo, resultado) y dónde (archivo, base de datos)? Importante en entorno hospitalario.

## Tareas que saldrán de ahí (a redactar con las respuestas)
- Servicio/acción de solicitud de misión y tabla de destinos (con poses y validación).
- `mission_node`: orquesta la máquina de estados, envía `NavigateToPose` a Nav2, publica `/hrobot/mission_status` con **RELIABLE + TRANSIENT_LOCAL**.
- Confirmación de entrega y temporizador de espera.
- Gestión de emergencia (enclavada; `RESET` solo humano).
- Registro de auditoría.
- `mission.launch.py` e inclusión en `navigation.launch.py`.
- Preservar `delivered` entre misiones/`RESET` para la auditoría (hoy se pierde al volver a `IDLE`).

## Fuera de alcance por ahora
Ascensores/puertas, multi-robot, integración con el sistema de información hospitalario.

## Qué ofrece ya el Create 3 (evitar reimplementarlo)
Topics/acciones bajo `/r2` (ver `Nodos_Topics.txt`). Decisiones de producto aún abiertas; esto solo acota las opciones.
- **Docking:** acciones `/r2/dock` y `/r2/undock`, estado en `/r2/dock_status`. Si "volver a base" implica la estación, usarlas en lugar de lógica propia.
- **Confirmación de entrega (decisión 2) y emergencia (decisión 3):** `/r2/interface_buttons` (botones físicos); servicio `/r2/e_stop` para detener el movimiento. Un botón podría disparar `EMERGENCY` o `DELIVERY_CONFIRMED`.
- **Aviso al operador:** `/r2/cmd_lightring` (anillo LED) y `/r2/cmd_audio` (sonidos) para indicar navegando / esperando confirmación / emergencia, sin display ni buzzer propios.
- **Batería:** `/r2/battery_state` para no iniciar misión con carga baja.
- **Estado del robot:** `/r2/stop_status`, `/r2/kidnap_status`, `/r2/slip_status`, `/r2/wheel_status` (levantado, deslizamiento) para diagnósticos y condiciones de fallo.
- **No usar para navegar:** `/r2/drive_distance`, `/r2/rotate_angle`, `/r2/navigate_to_position` competirían con Nav2 y se saltarían a `supervisor_node`.
