# CLAUDE.md — Sistema multiagente autónomo basado en Git

## Qué es este repo
Robot AMR hospitalario (Create 3 + RPi5 + LiDAR LD19, Nav2/AMCL/slam_toolbox). Detalle en `.claude/architecture_rules.md`. La lógica de `domain` es pura (sin ROS); solo `adapters` importa `rclpy`. Los agentes no pueden probar hardware real.

Flujo `Manager → Coder ↔ Reviewer` orquestado por `scripts/orchestrator.py`. Las tareas son archivos Markdown que se mueven por `tasks/backlog → in_progress → done`; cada una se implementa en su rama `feat/TASK-XXX`. **Un humano aprueba siempre el merge a `main`.**

## Comandos rápidos
```bash
# Instalar herramientas de desarrollo (una vez)
python3 -m pip install -r requirements-dev.txt

# Tests / calidad (ROS 2 Jazzy + Python, ver .claude/architecture_rules.md)
PYTHONPATH=src/hrobot_core pytest -q
PYTHONPATH=src/hrobot_core pytest -q --cov=hrobot_core --cov-fail-under=85
ruff format --check src tests && ruff check src tests
mypy --strict src/hrobot_core/hrobot_core
colcon build --packages-select hrobot_interfaces hrobot_core hrobot_bringup   # solo si hay ROS 2 instalado

# Orquestador (requiere repo git en 'main', árbol limpio y al menos un commit)
python3 scripts/orchestrator.py --dry-run              # ver qué tarea tomaría
python3 scripts/orchestrator.py                        # siguiente tarea del backlog (backend: claude CLI)
python3 scripts/orchestrator.py --task TASK-002        # tarea específica
python3 scripts/orchestrator.py --backend stub         # probar el flujo sin llamar a agentes
python3 scripts/orchestrator.py -v                     # logs detallados
# Exit codes: 0 aprobada (esperando merge humano) · 1 error (rollback) · 2 HUMAN-INTERVENTION-NEEDED
```

## Mapa del repositorio
| Ruta | Contenido |
|---|---|
| `.claude/system_prompts/{manager,coder,reviewer}.md` | Prompts de cada rol |
| `.claude/architecture_rules.md` | Convenciones técnicas (fuente de verdad) |
| `.claude/settings.json` | Permisos de los agentes (allow/deny de comandos) |
| `tasks/backlog/` `in_progress/` `done/` | Ciclo de vida de tareas (`TASK-XXX.md`) |
| `tasks/backlog/TASK-001-template.md` | Plantilla; el orquestador la ignora al auto-seleccionar |
| `src/hrobot_interfaces/` (ament_cmake) | Mensajes, servicios y acciones personalizadas |
| `src/hrobot_core/` (ament_python) | `domain`, `services`, `adapters` (código limpio) |
| `src/hrobot_bringup/` (ament_python) | Solo `launch/`, `config/`, `rviz/`, `maps/` |
| `tests/` | Espeja `src/` |
| `scripts/orchestrator.py` | Bucle de orquestación |

## Reglas de navegación y trabajo
1. Antes de editar, lee la tarea activa en `tasks/in_progress/` y `.claude/architecture_rules.md`.
2. Modifica **solo** los archivos listados en "Archivos permitidos" de la tarea.
3. Nunca commitees ni hagas checkout/merge/push en `main`; trabaja en `feat/TASK-XXX`.
4. No muevas archivos en `tasks/` a mano: lo hace el orquestador (el Manager solo crea en `backlog/`).
5. Commits: Conventional Commits con scope, p. ej. `feat(TASK-001): add config validator`.
6. Ejecuta los comandos del DoD antes de dar una tarea por terminada.
7. Capas: `domain` no importa de `services`/`adapters`; I/O solo en `adapters`.
8. Máximo 3 iteraciones Coder↔Reviewer; luego se escala a un humano.
9. Ante requerimientos ambiguos, pregunta; no inventes.
