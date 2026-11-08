#!/usr/bin/env python3
"""Orquestador local multiagente (Coder <-> Reviewer) basado en Git.

Ciclo de vida por tarea:
  1. Toma la tarea de menor ID en tasks/backlog/ (o la indicada con --task).
  2. Crea y cambia a la rama feat/TASK-XXX desde la rama base.
  3. Mueve la tarea a tasks/in_progress/ (commit en la rama).
  4. Bucle Coder <-> Reviewer (máx. 3 iteraciones). En cada iteración:
       Coder -> scope guard -> gate del DoD (comandos ejecutados por el
       orquestador, no por el agente) -> Reviewer.
  5a. APROBADO  -> mueve a tasks/done/, genera resumen de PR y se detiene
                   pidiendo confirmación humana para el merge a main.
  5b. Excede N  -> deja la rama para inspección, escribe un archivo
                   HUMAN-INTERVENTION-NEEDED y sale con código 2.
  Ante cualquier error inesperado: rollback (reset --hard, clean, vuelve a la
  rama base y elimina la rama creada por esta ejecución).

Backends de agente (--backend):
  cli   Invoca `claude -p` con el system prompt de cada rol (por defecto).
  stub  Simula agentes sin red (para probar el orquestador; --stub-verdict).

Selección de tarea: respeta `**Depende de:**` (las dependencias deben estar en
tasks/done/ de la rama base, es decir, ya mergeadas por un humano) y omite
tareas cuya rama feat/TASK-XXX ya existe (pendientes de merge).

Permisos de los agentes: los define .claude/settings.json (allow/deny); aquí
solo se restringen las herramientas del Reviewer a lectura + verificación.

El orquestador NUNCA hace merge ni push a main.
"""
from __future__ import annotations

import argparse
import fnmatch
import glob
import logging
import os
import re
import shlex
import shutil
import subprocess
import sys
import textwrap
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Sequence

ROOT = Path(__file__).resolve().parent.parent
TASKS = ROOT / "tasks"
BACKLOG, IN_PROGRESS, DONE = TASKS / "backlog", TASKS / "in_progress", TASKS / "done"
PROMPTS = ROOT / ".claude" / "system_prompts"
TASK_ID_RE = re.compile(r"^(TASK-\d+)")
VERDICT_RE = re.compile(r"\[(APROBADO|RECHAZADO)\]")

EXIT_OK, EXIT_ERROR, EXIT_ESCALATED = 0, 1, 2

CODER_TOOLS = "Read,Edit,Write,Grep,Glob"  # Bash se rige por .claude/settings.json
REVIEWER_TOOLS = ("Read,Grep,Glob,Bash(git diff:*),Bash(git log:*),Bash(git status:*),"
                  "Bash(pytest:*),Bash(PYTHONPATH=src/hrobot_core pytest:*),Bash(ruff:*),Bash(mypy:*)")
REVIEWER_DENIED = "Edit,Write,NotebookEdit"
GATE_TOOLS = ("pytest", "ruff", "mypy")  # python3 solo con `-m py_compile`
GATE_TIMEOUT = 900

log = logging.getLogger("orchestrator")


class OrchestratorError(Exception):
    """Error controlado del orquestador."""


# --------------------------------------------------------------------------- #
# Git
# --------------------------------------------------------------------------- #
def git(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Ejecuta git en ROOT sin shell, con salida capturada."""
    cmd = ["git", *args]
    log.debug("$ %s", " ".join(cmd))
    try:
        proc = subprocess.run(
            cmd, cwd=ROOT, text=True, capture_output=True, check=False,
            env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
        )
    except FileNotFoundError as exc:
        raise OrchestratorError("git no está instalado o no está en PATH") from exc
    if check and proc.returncode != 0:
        raise OrchestratorError(
            f"git {' '.join(args)} falló ({proc.returncode}): {proc.stderr.strip() or proc.stdout.strip()}"
        )
    return proc


def current_branch() -> str:
    return git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()


def is_clean() -> bool:
    return git("status", "--porcelain").stdout.strip() == ""


def branch_exists(name: str) -> bool:
    return git("rev-parse", "--verify", "--quiet", f"refs/heads/{name}", check=False).returncode == 0


def is_tracked(path: Path) -> bool:
    rel = str(path.relative_to(ROOT))
    return git("ls-files", "--error-unmatch", rel, check=False).returncode == 0


def move_task(src: Path, dst_dir: Path) -> Path:
    """Mueve la tarea entre carpetas (git mv si está versionada)."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / src.name
    if dst.exists():
        raise OrchestratorError(f"Destino ya existe: {dst}")
    if is_tracked(src):
        git("mv", str(src.relative_to(ROOT)), str(dst.relative_to(ROOT)))
    else:
        shutil.move(str(src), str(dst))
        git("add", "-A", str(dst_dir.relative_to(ROOT)))
    return dst


def commit_all(message: str) -> bool:
    """Commit de todo lo pendiente. Devuelve False si no había cambios."""
    git("add", "-A")
    if is_clean():
        return False
    git("commit", "-m", message)
    return True


# --------------------------------------------------------------------------- #
# Tareas
# --------------------------------------------------------------------------- #
@dataclass
class Task:
    path: Path
    task_id: str
    title: str
    allowed: list[str] = field(default_factory=list)
    depends: list[str] = field(default_factory=list)
    gate: list[tuple[dict[str, str], list[str]]] = field(default_factory=list)

    @property
    def branch(self) -> str:
        return f"feat/{self.task_id}"

    @classmethod
    def load(cls, path: Path) -> "Task":
        m = TASK_ID_RE.match(path.name)
        if not m:
            raise OrchestratorError(f"Nombre de tarea inválido (esperado TASK-XXX*.md): {path.name}")
        text = path.read_text(encoding="utf-8")
        title_m = re.search(r"^#\s+(.+)$", text, re.MULTILINE)
        return cls(path=path, task_id=m.group(1), title=title_m.group(1).strip() if title_m else m.group(1),
                   allowed=parse_allowed_files(text), depends=parse_depends(text),
                   gate=extract_gate_commands(text))


def parse_allowed_files(text: str) -> list[str]:
    """Extrae rutas/globs entre backticks de la sección 'Archivos permitidos'."""
    sec = re.search(r"^##\s+Archivos permitidos[^\n]*\n(.*?)(?=^##\s|\Z)", text, re.MULTILINE | re.DOTALL)
    if not sec:
        return []
    return re.findall(r"^\s*[-*]\s+`([^`]+)`", sec.group(1), re.MULTILINE)


def parse_depends(text: str) -> list[str]:
    m = re.search(r"\*\*Depende de:\*\*\s*(.+)", text)
    return re.findall(r"TASK-\d+", m.group(1)) if m else []


def done_ids() -> set[str]:
    ids = set()
    for p in DONE.glob("TASK-*.md"):
        m = TASK_ID_RE.match(p.name)
        if m and not p.name.endswith(".PR.md"):
            ids.add(m.group(1))
    return ids


def extract_gate_commands(text: str) -> list[tuple[dict[str, str], list[str]]]:
    """Comandos verificables del DoD (líneas `- [ ]` con comandos entre backticks).

    Solo se ejecutan herramientas de la lista blanca (pytest, ruff, mypy y
    `python3 -m py_compile`), sin shell. Se omiten criterios marcados "Opcional".
    """
    sec = re.search(r"^##\s+Criterios de Aceptaci[^\n]*\n(.*?)(?=^##\s|\Z)", text, re.MULTILINE | re.DOTALL)
    if not sec:
        return []
    cmds: list[tuple[dict[str, str], list[str]]] = []
    for line in sec.group(1).splitlines():
        if not re.match(r"\s*-\s*\[.\]", line) or "opcional" in line.lower():
            continue
        for raw in re.findall(r"`([^`]+)`", line):
            try:
                argv = shlex.split(raw)
            except ValueError:
                continue
            env: dict[str, str] = {}
            while argv and re.match(r"^[A-Z_][A-Z0-9_]*=", argv[0]):
                k, v = argv.pop(0).split("=", 1)
                env[k] = v
            # Solo comandos completos: una herramienta citada a secas (`ruff`) no es un criterio.
            is_tool = len(argv) > 1 and argv[0] in GATE_TOOLS
            if argv[:1] == ["ruff"] and argv[1:2] not in (["check"], ["format"]):
                is_tool = False
            is_compile = len(argv) > 3 and argv[:3] == ["python3", "-m", "py_compile"]
            if not (is_tool or is_compile):
                continue
            expanded: list[str] = []
            for a in argv:
                expanded += sorted(glob.glob(a, root_dir=ROOT)) or [a] if "*" in a else [a]
            if (env, expanded) not in cmds:
                cmds.append((env, expanded))
    return cmds


def run_gate(task: Task) -> str | None:
    """Ejecuta el DoD verificable. Devuelve None si todo pasa, o el mensaje del primer fallo."""
    for env, argv in task.gate:
        label = " ".join([*(f"{k}={v}" for k, v in env.items()), *argv])
        log.info("gate $ %s", label)
        try:
            proc = subprocess.run(argv, cwd=ROOT, text=True, capture_output=True,
                                  env={**os.environ, **env}, timeout=GATE_TIMEOUT)
        except FileNotFoundError as exc:
            raise OrchestratorError(f"Herramienta del DoD no instalada: {argv[0]}") from exc
        except subprocess.TimeoutExpired:
            return f"Comando del DoD excedió {GATE_TIMEOUT}s: {label}"
        if proc.returncode != 0:
            tail = (proc.stdout + proc.stderr).strip()[-2000:]
            return f"Comando del DoD falló (exit {proc.returncode}): {label}\n{tail}"
    return None


def pick_next_task(explicit: str | None) -> Task:
    done = done_ids()
    if explicit:
        candidates = [p for p in BACKLOG.glob("*.md") if p.name.startswith(explicit)]
        if not candidates:
            raise OrchestratorError(f"No hay tarea '{explicit}' en {BACKLOG}")
        task = Task.load(sorted(candidates)[0])
        missing = [d for d in task.depends if d not in done]
        if missing:
            raise OrchestratorError(f"{task.task_id} depende de {missing}, aún no están en tasks/done/ de la rama base.")
        return task
    files = sorted(p for p in BACKLOG.glob("TASK-*.md") if "template" not in p.stem.lower())
    if not files:
        raise OrchestratorError("Backlog vacío (las plantillas *-template.md se ignoran).")
    for p in files:
        task = Task.load(p)
        missing = [d for d in task.depends if d not in done]
        if missing:
            log.info("Omitida %s: dependencias pendientes %s", task.task_id, missing)
        elif branch_exists(task.branch):
            log.info("Omitida %s: la rama %s ya existe (¿pendiente de merge humano?)", task.task_id, task.branch)
        else:
            return task
    raise OrchestratorError("No hay tareas elegibles (dependencias pendientes o ramas esperando merge).")


def in_scope(task: Task, path: str) -> bool:
    patterns = task.allowed + [f"tasks/**/{task.task_id}*", "tasks/**/.gitkeep"]
    return any(fnmatch.fnmatch(path, pat) for pat in patterns)


def out_of_scope_files(task: Task, base: str) -> list[str]:
    """Archivos cambiados respecto a base que no están en el alcance permitido."""
    changed = git("diff", "--name-only", f"{base}...HEAD").stdout.split()
    return [f for f in changed if not in_scope(task, f)]


def discard_out_of_scope_worktree(task: Task) -> list[str]:
    """Descarta cambios SIN commitear fuera del alcance (p. ej. un formateador global).

    Evita que la red de seguridad los commitee. Los ya commiteados por el propio
    agente los detecta después el scope guard.
    """
    raw = git("status", "--porcelain", "-z", "-uall").stdout.split("\0")
    discarded: list[str] = []
    i = 0
    while i < len(raw):
        entry = raw[i]
        i += 1
        if not entry:
            continue
        xy, path = entry[:2], entry[3:]
        if xy[0] in "RC":  # renombrado/copiado: la siguiente entrada es la ruta original
            i += 1
        if in_scope(task, path):
            continue
        if xy == "??":
            git("clean", "-fd", "--", path, check=False)
        else:
            git("checkout", "HEAD", "--", path, check=False)
        discarded.append(path)
    return discarded


# --------------------------------------------------------------------------- #
# Agentes
# --------------------------------------------------------------------------- #
AgentFn = Callable[[str, str], str]  # (system_prompt, user_prompt) -> salida


def cli_agent(cmd_template: Sequence[str], timeout: int, env: dict[str, str] | None = None) -> AgentFn:
    def run(system: str, prompt: str) -> str:
        cmd = [*cmd_template, "-p", prompt, "--append-system-prompt", system]
        try:
            proc = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=timeout, env=env)
        except FileNotFoundError as exc:
            raise OrchestratorError(f"Comando de agente no encontrado: {cmd_template[0]}") from exc
        except subprocess.TimeoutExpired as exc:
            raise OrchestratorError(f"Agente excedió el timeout de {timeout}s") from exc
        if proc.returncode != 0:
            detail = "\n".join(s for s in (proc.stderr.strip(), proc.stdout.strip()) if s)
            raise OrchestratorError(f"Agente falló ({proc.returncode}):\n{detail[-1500:]}")
        return proc.stdout
    return run


def stub_agents(verdict_after: int) -> tuple[AgentFn, AgentFn]:
    """Agentes simulados: el reviewer aprueba a partir de la iteración `verdict_after` (0 = nunca)."""
    state = {"review_calls": 0}

    def coder(_system: str, prompt: str) -> str:
        log.info("[stub] Coder recibió %d caracteres de prompt", len(prompt))
        return "stub: sin cambios"

    def reviewer(_system: str, _prompt: str) -> str:
        state["review_calls"] += 1
        if verdict_after and state["review_calls"] >= verdict_after:
            return "Veredicto: [APROBADO]\nstub: aprobado"
        return ("Veredicto: [RECHAZADO]\n1. **Archivo:** `src/x.py` · **Línea aprox.:** 1\n"
                "   **Fallo:** stub\n   **Criterio de corrección:** stub")

    return coder, reviewer


def parse_verdict(output: str) -> bool:
    """True si APROBADO. Falla cerrado: sin veredicto claro => rechazo."""
    first = next((ln for ln in output.splitlines() if ln.strip()), "")
    m = VERDICT_RE.search(first)
    return bool(m and m.group(1) == "APROBADO")


def read_prompt(name: str) -> str:
    path = PROMPTS / name
    if not path.is_file():
        raise OrchestratorError(f"Falta system prompt: {path}")
    return path.read_text(encoding="utf-8")


def coder_prompt(task: Task, in_progress_path: Path, feedback: str | None, iteration: int) -> str:
    base = (f"Implementa la tarea {task.task_id} descrita en {in_progress_path.relative_to(ROOT)}. "
            f"Estás en la rama {task.branch}. Lee también .claude/architecture_rules.md. "
            "Ejecuta los comandos del DoD y haz commits Conventional Commits.")
    if feedback:
        base += f"\n\nIteración {iteration}. Corrige EXACTAMENTE estos defectos del Reviewer:\n\n{feedback}"
    return base


def reviewer_prompt(task: Task, in_progress_path: Path, base: str) -> str:
    diff = git("diff", f"{base}...HEAD").stdout
    max_chars = 120_000
    if len(diff) > max_chars:
        diff = diff[:max_chars] + "\n... [diff truncado]"
    return (f"Revisa la tarea {task.task_id} (spec en {in_progress_path.relative_to(ROOT)}).\n"
            f"Diff contra {base}:\n\n```diff\n{diff}\n```")


# --------------------------------------------------------------------------- #
# Orquestación
# --------------------------------------------------------------------------- #
def write_pr_summary(task: Task, base: str, iterations: int, review: str) -> Path:
    log_txt = git("log", "--oneline", f"{base}..HEAD").stdout.strip()
    stat = git("diff", "--stat", f"{base}...HEAD").stdout.strip()
    out = DONE / f"{task.task_id}.PR.md"
    out.write_text(textwrap.dedent(f"""\
        # PR: {task.title}

        - **Rama:** `{task.branch}` → `{base}` (squash-merge)
        - **Iteraciones Coder↔Reviewer:** {iterations}

        ## Commits
        ```
        {log_txt}
        ```

        ## Cambios
        ```
        {stat}
        ```

        ## Veredicto del Reviewer
        {review.strip()}
        """), encoding="utf-8")
    return out


def escalate(task: Task, review: str, iterations: int) -> Path:
    marker = IN_PROGRESS / f"{task.task_id}.HUMAN-INTERVENTION-NEEDED.md"
    marker.write_text(
        f"# HUMAN-INTERVENTION-NEEDED: {task.task_id}\n\n"
        f"El loop Coder↔Reviewer no convergió tras {iterations} iteraciones.\n"
        f"Rama con el trabajo parcial: `{task.branch}`\n\n## Último veredicto del Reviewer\n\n{review}\n",
        encoding="utf-8",
    )
    return marker


def rollback(base: str, branch: str | None, created: bool) -> None:
    """Deja el repo limpio en la rama base y borra la rama creada por esta ejecución."""
    log.warning("Rollback: restaurando estado limpio en '%s'", base)
    git("reset", "--hard", check=False)
    git("clean", "-fd", check=False)
    git("checkout", base, check=False)
    if created and branch and branch_exists(branch):
        git("branch", "-D", branch, check=False)


def run_task(task: Task, base: str, coder: AgentFn, reviewer: AgentFn, max_iter: int,
             gate_enabled: bool = True) -> int:
    coder_sys, reviewer_sys = read_prompt("coder.md"), read_prompt("reviewer.md")

    if branch_exists(task.branch):
        raise OrchestratorError(f"La rama {task.branch} ya existe; revísala o elimínala manualmente.")
    git("checkout", "-b", task.branch, base)
    created = True
    try:
        in_progress_path = move_task(task.path, IN_PROGRESS)
        commit_all(f"chore({task.task_id}): move task to in_progress")

        feedback: str | None = None
        review = ""
        for i in range(1, max_iter + 1):
            log.info("=== %s · iteración %d/%d · Coder ===", task.task_id, i, max_iter)
            coder_out = coder(coder_sys, coder_prompt(task, in_progress_path, feedback, i)) or ""
            tail = "\n".join(coder_out.strip().splitlines()[-12:])
            if tail:
                log.info("Respuesta del Coder (final):\n%s", tail)
            blocked = [ln.strip() for ln in coder_out.splitlines() if ln.strip().startswith("BLOCKED:")]
            if blocked:
                rollback(base, task.branch, created)
                created = False
                print(f"\n⛔ {task.task_id} BLOQUEADA por el Coder (no es un fallo de código):")
                for ln in blocked:
                    print(f"   {ln}")
                print("   Resuelve el bloqueo (spec o prerrequisito) y relanza; la rama se eliminó y la tarea sigue en backlog.")
                return EXIT_ESCALATED
            discarded = discard_out_of_scope_worktree(task)
            if discarded:
                log.warning("Descartados cambios fuera de alcance sin commitear: %s", discarded)
            commit_all(f"chore({task.task_id}): coder changes (iteration {i})")  # red de seguridad
            note = (f"Se descartaron automáticamente cambios tuyos fuera de alcance: {', '.join(discarded)}. "
                    "No ejecutes formateadores/linters con --fix sobre todo el repo; solo sobre tus archivos.\n\n"
                    if discarded else "")

            stray = out_of_scope_files(task, base)
            if stray:
                feedback = (note + "Commiteaste archivos fuera del alcance permitido: "
                            + ", ".join(stray) + ". Revierte esos cambios.")
                review = f"Veredicto: [RECHAZADO]\n(scope guard) {feedback}"
                log.warning("Scope guard: %s", stray)
                continue

            if gate_enabled:
                failure = run_gate(task)
                if failure:
                    feedback = f"{note}El gate del DoD falló. Corrige:\n{failure}"
                    review = f"Veredicto: [RECHAZADO]\n(gate) {failure}"
                    log.warning("Gate del DoD falló; se omite al Reviewer en esta iteración.")
                    continue

            log.info("=== %s · iteración %d/%d · Reviewer ===", task.task_id, i, max_iter)
            review = reviewer(reviewer_sys, reviewer_prompt(task, in_progress_path, base))
            if parse_verdict(review):
                done_path = move_task(in_progress_path, DONE)
                task.path = done_path
                pr = write_pr_summary(task, base, i, review)
                commit_all(f"chore({task.task_id}): mark task done")
                print(f"\n✅ {task.task_id} APROBADA en {i} iteración(es).")
                print(f"   Rama: {task.branch}   Resumen PR: {pr.relative_to(ROOT)}")
                print(f"   ⏸  Detenido: requiere confirmación HUMANA para el merge a '{base}'.")
                print(f"   Sugerido: git checkout {base} && git merge --squash {task.branch}")
                git("checkout", base)
                return EXIT_OK
            feedback = note + review
            log.info("Rechazado. Feedback enviado al Coder.")

        marker = escalate(task, review, max_iter)
        commit_all(f"chore({task.task_id}): HUMAN-INTERVENTION-NEEDED")
        git("checkout", base)
        print(f"\n🛑 HUMAN-INTERVENTION-NEEDED: {task.task_id} no convergió en {max_iter} iteraciones.")
        print(f"   Rama conservada: {task.branch}   Detalle: {marker.relative_to(ROOT)} (en esa rama)")
        return EXIT_ESCALATED
    except BaseException:
        rollback(base, task.branch, created)
        raise


def build_agents(args: argparse.Namespace) -> tuple[AgentFn, AgentFn]:
    if args.backend == "stub":
        return stub_agents(args.stub_approve_after)
    cmd = args.agent_cmd.split()
    coder_cmd = [*cmd, "--permission-mode", "acceptEdits", "--allowedTools", CODER_TOOLS]
    reviewer_cmd = [*cmd, "--allowedTools", REVIEWER_TOOLS, "--disallowedTools", REVIEWER_DENIED]
    env = dict(os.environ)
    if args.no_api_key:
        env.pop("ANTHROPIC_API_KEY", None)
    elif env.get("ANTHROPIC_API_KEY"):
        log.warning("ANTHROPIC_API_KEY está definida y tiene prioridad sobre tu sesión de claude.ai "
                    "(si usas suscripción: `unset ANTHROPIC_API_KEY` o pasa --no-api-key).")
    return cli_agent(coder_cmd, args.timeout, env), cli_agent(reviewer_cmd, args.timeout, env)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", help="Prefijo de la tarea (p. ej. TASK-002). Por defecto, la de menor ID.")
    p.add_argument("--base", default="main", help="Rama base (default: main)")
    p.add_argument("--max-iterations", type=int, default=3, help="Máx. iteraciones Coder↔Reviewer (default: 3)")
    p.add_argument("--backend", choices=["cli", "stub"], default="cli")
    p.add_argument("--agent-cmd", default=os.environ.get("AGENT_CMD", "claude"), help="Binario del agente (default: claude)")
    p.add_argument("--timeout", type=int, default=1800, help="Timeout por llamada de agente en segundos")
    p.add_argument("--stub-approve-after", type=int, default=2, help="[stub] aprobar en la iteración N (0 = nunca)")
    p.add_argument("--no-api-key", action="store_true",
                   help="Ignorar ANTHROPIC_API_KEY al llamar a claude (usar la sesión de claude.ai)")
    p.add_argument("--no-gate", action="store_true", help="No ejecutar el gate del DoD antes del Reviewer")
    p.add_argument("--dry-run", action="store_true", help="Solo muestra qué tarea se ejecutaría")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s %(message)s")
    max_iter = min(args.max_iterations, 3) if args.max_iterations > 0 else 3  # tope duro: 3
    started_on: str | None = None
    try:
        if git("rev-parse", "--is-inside-work-tree", check=False).stdout.strip() != "true":
            raise OrchestratorError("No es un repositorio Git. Ejecuta `git init -b main` y haz un commit inicial.")
        if git("rev-parse", "--verify", "--quiet", "HEAD", check=False).returncode != 0:
            raise OrchestratorError("El repositorio no tiene commits. Haz un commit inicial en main.")
        started_on = current_branch()
        if started_on != args.base:
            raise OrchestratorError(f"Debes iniciar desde '{args.base}' (rama actual: '{started_on}').")
        if not is_clean():
            raise OrchestratorError("Working tree sucio. Commitea o descarta cambios antes de orquestar.")

        task = pick_next_task(args.task)
        log.info("Tarea seleccionada: %s — %s (permitidos: %s; gate: %d comandos)",
                 task.task_id, task.title, task.allowed or "¡ninguno!", len(task.gate))
        if not task.allowed:
            raise OrchestratorError(f"{task.path.name} no define 'Archivos permitidos'; el scope guard no puede operar.")
        if args.dry_run:
            print(f"[dry-run] Ejecutaría {task.task_id} en {task.branch}")
            for env, argv in task.gate:
                print("  gate:", " ".join([*(f"{k}={v}" for k, v in env.items()), *argv]))
            return EXIT_OK

        coder, reviewer = build_agents(args)
        return run_task(task, args.base, coder, reviewer, max_iter, gate_enabled=not args.no_gate)
    except OrchestratorError as exc:
        log.error("%s", exc)
        return EXIT_ERROR
    except KeyboardInterrupt:
        log.error("Interrumpido por el usuario (rollback aplicado).")
        return 130
    except Exception:  # noqa: BLE001 - último recurso, ya hubo rollback en run_task
        log.exception("Error inesperado")
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
