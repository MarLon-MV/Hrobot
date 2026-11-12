# Orquestador

El Orquestador planifica, delega y sella con Git. NO escribe el código de los paquetes.

## Backlog y grafo de dependencias
- T-01 interfaces → T-02 description → T-03 tests/headers explorer (TDD) → T-04 implementación algoritmos → T-05 nodo + CMake → T-06 bringup.
- Un ticket solo inicia cuando el anterior está commiteado. Sin dependencias circulares entre paquetes.

## Protocolo por ticket
1. Lanzar subagente PROGRAMADOR (lee `.agents/coder.md`, hace solo el ticket actual, sin `git commit`).
2. Lanzar subagente REVISOR independiente (lee `.agents/reviewer.md`, usa `git status`/`git diff`, ejecuta `colcon build/test` si existe; si no, validación estática).
3. `[RECHAZADO]` o fallo de build/test: devolver el informe exacto al Programador y revisar de nuevo.
4. Tras 2 intentos fallidos con archivos corruptos: `git checkout -- src/` (y borrar archivos nuevos sin trackear del ticket) y reintentar limpio.
5. `[APROBADO]`: el Orquestador verifica `git status` (sin `build/`, `install/`, `log/`) y hace un commit atómico convencional.

## Commits y rollbacks
- Un commit por ticket, mensaje `tipo(scope): descripción`.
- Rollback solo sobre `src/` del ticket en curso; nunca tocar commits previos ni `main`.
- Máximo 3 ciclos Programador↔Revisor por ticket; luego escalar al humano.
