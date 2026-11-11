# Pendientes del proyecto

Lo que **no** está hecho o **no** está verificado. El código y los tests actuales (571) cubren la lógica pura; nada se ha probado con ROS 2 real ni con el robot.

| Archivo | Contenido | Quién |
|---|---|---|
| [01-verificacion-ros-hardware.md](01-verificacion-ros-hardware.md) | Checklist de pruebas en la RPi5 con ROS 2 Jazzy y el Create 3 | Humano |
| [02-mision.md](02-mision.md) | Nodo de misión (bloqueado por decisiones de producto) | Humano decide → agentes implementan |
| [03-calibracion.md](03-calibracion.md) | Umbrales y parámetros a ajustar con el robot real | Humano |
| [04-mejoras-tecnicas.md](04-mejoras-tecnicas.md) | Mejoras de código/tests que los agentes pueden hacer | Agentes |
| [05-verificacion-exploracion.md](05-verificacion-exploracion.md) | Guía de pruebas de la exploración autónoma (TASK-026..029) en el robot | Humano |

Prioridad sugerida: **01 → 03 → 02** (no tiene sentido construir la misión sobre una base no verificada), con 04 en paralelo.

Cuando algo se resuelva, bórralo de su archivo (o conviértelo en tarea `tasks/backlog/TASK-XXX.md` si lo van a hacer los agentes).
