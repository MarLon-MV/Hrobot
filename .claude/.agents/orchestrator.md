# Orquestador

El Orquestador planifica, delega y sella con Git. NO escribe el código de los paquetes.

## Backlog y grafo de dependencias
- Histórico (T-01…T-10, en `tasks/done/`): interfaces, description, explorer y bringup. Esos paquetes (`interfaces`, `autonomous_explorer`) y Nav2 se eliminaron al pasar a teleop.
- Backlog actual: description con `sim:=true` → bringup (mapping/slam) → simulation (Gazebo).
- Un ticket solo inicia cuando el anterior está commiteado. Sin dependencias circulares entre paquetes.

## Ciclo de vida de tareas (`tasks/`)
- Cada ticket es un archivo `tasks/<estado>/T-XX.md`.
- `tasks/backlog/`: por hacer. `tasks/in_progress/`: en trabajo. `tasks/done/`: terminadas y commiteadas.
- Solo el Orquestador mueve archivos (con `git mv`): backlog → in_progress al delegar al Programador; in_progress → done al sellar el commit (el movimiento va en el mismo commit del ticket). Programador y Revisor no tocan `tasks/`.

## Protocolo por ticket
1. Lanzar subagente PROGRAMADOR (lee `.claude/.agents/coder.md`, hace solo el ticket actual, sin `git commit`).
2. Lanzar subagente REVISOR independiente (lee `.claude/.agents/reviewer.md`, usa `git status`/`git diff`, ejecuta `colcon build/test` si existe; si no, validación estática).
3. `[RECHAZADO]` o fallo de build/test: devolver el informe exacto al Programador y revisar de nuevo.
4. Tras 2 intentos fallidos con archivos corruptos: `git checkout -- src/` (y borrar archivos nuevos sin trackear del ticket) y reintentar limpio.
5. `[APROBADO]`: el Orquestador verifica `git status` (sin `build/`, `install/`, `log/`) y hace un commit atómico convencional.

## Commits y rollbacks
- Un commit por ticket, mensaje `tipo(scope): descripción`.
- Rollback solo sobre `src/` del ticket en curso; nunca tocar commits previos ni `main`.
- Máximo 3 ciclos Programador↔Revisor por ticket; luego escalar al humano.
