# CLAUDE.md — Workspace ROS 2 Jazzy para mapeo teleoperado

## Qué es este repo
Workspace ROS 2 Jazzy para un robot iRobot Create 3 + Raspberry Pi 5 + LiDAR LD19.
Misión: mapear entornos con slam_toolbox moviendo el robot por **teleop** (sin exploración autónoma ni Nav2).
Antes de probar con el robot real se simula en **Gazebo Harmonic** (`ros_gz`).

## Paquetes (`src/`)
| Paquete | Rol |
|---|---|
| `create3_bot_description` | URDF/xacro (`base_link`, `laser_frame`; `sim:=true` añade ruedas y plugins de Gazebo) y RViz. |
| `create3_bot_bringup` | Solo `config/` y `launch/` del robot real (`mapping`, `hardware_base`, `slam`). |
| `create3_bot_simulation` | Gazebo: mundo, puente `ros_gz_bridge` y `sim.launch.py`. Solo PC de desarrollo; no se compila en la Pi. |

Dependencias permitidas (sin ciclos): `description` ← `bringup` ← `simulation`.

## Reglas globales
1. No hay código C++ ahora mismo; si vuelve a haberlo: C++17, `-Wall -Wextra -Wpedantic`, cero warnings y TDD con GoogleTest.
2. La simulación corre sin namespace; el robot real usa `robot_namespace` (por defecto `r2`).
3. Los agentes no pueden probar hardware real.
4. Ante requerimientos ambiguos, preguntar; no inventar.

## Convenciones Git
- Rama de trabajo: `feat/autonomous-exploration`. Nunca commitear directo en `main`/`master`.
- Commits atómicos, Conventional Commits con scope: `feat(description): ...`, `feat(bringup): ...`, `feat(simulation): ...`, `chore(agents): ...`.
- Solo el Orquestador hace commit; Programador y Revisor no.
- Nunca commitear `build/`, `install/`, `log/`, `.cache/`. Verificar con `git status` antes de cada commit.
- Un humano aprueba el merge a `main`.

## Comandos
```bash
colcon build --symlink-install --packages-select <paquete>
ros2 launch create3_bot_simulation sim.launch.py
```

## Tareas
Archivos `tasks/{backlog,in_progress,done}/T-XX.md`: backlog = por hacer, in_progress = en trabajo, done = terminadas. Solo el Orquestador las mueve (`git mv`).

## Flujo multi-agente
Ver `.claude/.agents/orchestrator.md`, `.claude/.agents/coder.md`, `.claude/.agents/reviewer.md`.
