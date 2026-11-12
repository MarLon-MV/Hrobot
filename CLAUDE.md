# CLAUDE.md — Workspace ROS 2 Jazzy (C++17) para exploración autónoma

## Qué es este repo
Workspace ROS 2 Jazzy en C++17 para un robot iRobot Create 3 + Raspberry Pi 5 + LiDAR LD19.
Misión: explorar y mapear entornos desconocidos de forma autónoma (slam_toolbox + Nav2 + exploración por fronteras).

## Paquetes (`src/`)
| Paquete | Rol |
|---|---|
| `create3_bot_interfaces` | Acción `ExploreMap`, mensaje `FrontierInfo`, servicio `SaveExplorationMap`. Sin código. |
| `create3_bot_description` | URDF/xacro (`base_link`, `laser_frame`) y RViz. |
| `create3_autonomous_explorer` | Lib `${PROJECT_NAME}_lib` (algoritmos puros) + `explorer_node`. |
| `create3_bot_bringup` | Solo `config/` y `launch/`. |

Dependencias permitidas (sin ciclos): `interfaces` ← `explorer` ← `bringup`; `description` ← `bringup`.
Futuros paquetes (`mission_manager`, `web_bridge`) dependen de `interfaces`, nunca del explorer.

## Reglas globales
1. Los algoritmos puros (`types`, `frontier_detector`, `goal_evaluator`) NO incluyen headers de ROS.
2. `rclcpp` solo en `explorer_node` y `main.cpp`.
3. C++17, flags `-Wall -Wextra -Wpedantic`, cero warnings.
4. TDD con GoogleTest; ningún ticket se cierra con tests en rojo.
5. Los agentes no pueden probar hardware real.
6. Ante requerimientos ambiguos, preguntar; no inventar.

## Convenciones Git
- Rama de trabajo: `feat/autonomous-exploration`. Nunca commitear directo en `main`/`master`.
- Commits atómicos, Conventional Commits con scope: `feat(interfaces): ...`, `feat(description): ...`, `test(explorer): ...`, `feat(explorer): ...`, `feat(bringup): ...`, `chore(agents): ...`.
- Solo el Orquestador hace commit; Programador y Revisor no.
- Nunca commitear `build/`, `install/`, `log/`, `.cache/`. Verificar con `git status` antes de cada commit.
- Un humano aprueba el merge a `main`.

## Comandos
```bash
colcon build --packages-select <paquete>
colcon test --packages-select <paquete> && colcon test-result --verbose
```

## Tareas
Archivos `tasks/{backlog,in_progress,done}/T-XX.md`: backlog = por hacer, in_progress = en trabajo, done = terminadas. Solo el Orquestador las mueve (`git mv`).

## Flujo multi-agente
Ver `.claude/.agents/orchestrator.md`, `.claude/.agents/coder.md`, `.claude/.agents/reviewer.md`.
