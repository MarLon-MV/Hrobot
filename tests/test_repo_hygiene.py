"""Red de seguridad sobre el árbol completo: sintaxis, conflictos de merge y entry points."""

from __future__ import annotations

import ast
from collections.abc import Iterator
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

EXCLUDED_DIRS = frozenset(
    {".git", ".venv", "__pycache__", ".mypy_cache", ".ruff_cache", "build", "install", "log"}
)
TEXT_SUFFIXES = frozenset(
    {".py", ".md", ".yaml", ".yml", ".xml", ".cfg", ".toml", ".rviz", ".msg", ".txt", ".json"}
)
CONFLICT_START = "<<<<<<< "
CONFLICT_END = ">>>>>>> "
CONFLICT_SEPARATOR = "======="


def iter_files(root: Path, suffixes: frozenset[str]) -> Iterator[Path]:
    """Recorre ``root`` omitiendo directorios excluidos y devuelve archivos con sufijo válido."""
    for path in sorted(root.rglob("*")):
        rel_parts = path.relative_to(root).parts
        if any(part in EXCLUDED_DIRS for part in rel_parts[:-1]):
            continue
        if path.is_file() and path.suffix in suffixes:
            yield path


def find_syntax_errors(root: Path) -> list[str]:
    """Devuelve un mensaje por cada ``.py`` que no se puede parsear."""
    errors: list[str] = []
    for path in iter_files(root, frozenset({".py"})):
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, ValueError, UnicodeDecodeError) as exc:
            errors.append(f"{path.relative_to(root)}: {exc}")
    return errors


def find_conflict_markers(root: Path) -> list[str]:
    """Devuelve ``archivo:línea`` de cada marcador de conflicto de merge."""
    hits: list[str] = []
    for path in iter_files(root, TEXT_SUFFIXES):
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            hits.append(f"{path.relative_to(root)}: no es UTF-8")
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            if (
                line.startswith(CONFLICT_START)
                or line.startswith(CONFLICT_END)
                or line == CONFLICT_SEPARATOR
            ):
                hits.append(f"{path.relative_to(root)}:{number}")
    return hits


def _console_scripts(setup_py: Path) -> list[str]:
    """Lee (sin ejecutar) las entradas ``console_scripts`` de un ``setup.py``."""
    tree = ast.parse(setup_py.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for keyword in node.keywords:
            if keyword.arg != "entry_points":
                continue
            entry_points = ast.literal_eval(keyword.value)
            return list(entry_points.get("console_scripts", []))
    return []


def _top_level_functions(module_file: Path) -> set[str]:
    tree = ast.parse(module_file.read_text(encoding="utf-8"))
    return {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef)
    }


def _resolve_module(package_dir: Path, module: str) -> Path | None:
    base = package_dir.joinpath(*module.split("."))
    for candidate in (base.with_suffix(".py"), base / "__init__.py"):
        if candidate.is_file():
            return candidate
    return None


def check_entry_points(root: Path) -> list[str]:
    """Valida cada ``console_scripts`` de los ``setup.py`` bajo ``root``."""
    errors: list[str] = []
    for setup_py in iter_files(root, frozenset({".py"})):
        if setup_py.name != "setup.py":
            continue
        rel = setup_py.relative_to(root)
        for entry in _console_scripts(setup_py):
            target = entry.split("=", 1)[1].strip() if "=" in entry else ""
            module, sep, func = target.partition(":")
            if not (module and sep and func):
                errors.append(f"{rel}: entrada mal formada '{entry}'")
                continue
            module_file = _resolve_module(setup_py.parent, module)
            if module_file is None:
                errors.append(f"{rel}: módulo inexistente '{module}' ({entry})")
            elif func not in _top_level_functions(module_file):
                errors.append(f"{rel}: función '{func}' no existe en '{module}' ({entry})")
    return errors


def check_colcon_packages(root: Path) -> list[str]:
    """Verifica los archivos mínimos de cada paquete colcon bajo ``root/src``."""
    errors: list[str] = []
    src = root / "src"
    if not src.is_dir():
        return [f"{src}: no existe"]
    for pkg in sorted(p for p in src.iterdir() if p.is_dir()):
        if not (pkg / "package.xml").is_file():
            errors.append(f"{pkg.name}: falta package.xml")
        if (pkg / "setup.py").is_file():
            for required in ("setup.cfg", f"resource/{pkg.name}"):
                if not (pkg / required).is_file():
                    errors.append(f"{pkg.name}: falta {required}")
    return errors


# --- Tests sobre el repo real ---------------------------------------------------------------


def test_all_python_files_parse() -> None:
    assert find_syntax_errors(REPO_ROOT) == []


def test_no_conflict_markers() -> None:
    assert find_conflict_markers(REPO_ROOT) == []


def test_console_script_entry_points_resolve() -> None:
    assert check_entry_points(REPO_ROOT) == []


def test_colcon_packages_are_complete() -> None:
    assert check_colcon_packages(REPO_ROOT) == []


# --- Tests negativos ------------------------------------------------------------------------


def _write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


@pytest.mark.parametrize(
    "line", ["<<<<<<< HEAD", ">>>>>>> feat/TASK-016", "======="], ids=["start", "end", "sep"]
)
def test_conflict_marker_detected(tmp_path: Path, line: str) -> None:
    _write(tmp_path / "a" / "cfg.yaml", f"ok: 1\n{line}\n")
    assert find_conflict_markers(tmp_path) == [str(Path("a/cfg.yaml")) + ":2"]


def test_conflict_marker_ignores_lookalikes_and_excluded_dirs(tmp_path: Path) -> None:
    _write(tmp_path / "doc.md", "====== título\n=======x\n  =======\n")
    _write(tmp_path / ".git" / "x.txt", "=======\n")
    _write(tmp_path / "build" / "y.py", "=======\n")
    assert find_conflict_markers(tmp_path) == []


def test_invalid_syntax_detected(tmp_path: Path) -> None:
    _write(tmp_path / "ok.py", "x = 1\n")
    _write(tmp_path / "bad.py", "def f(:\n")
    errors = find_syntax_errors(tmp_path)
    assert len(errors) == 1
    assert errors[0].startswith("bad.py")


_SETUP = """from setuptools import setup
setup(name="pkg", entry_points={"console_scripts": ["%s"]})
"""


def test_entry_point_ok(tmp_path: Path) -> None:
    _write(tmp_path / "pkg" / "setup.py", _SETUP % "n = pkg.mod:main")
    _write(tmp_path / "pkg" / "pkg" / "mod.py", "def main() -> None:\n    pass\n")
    assert check_entry_points(tmp_path) == []


def test_entry_point_missing_module_detected(tmp_path: Path) -> None:
    _write(tmp_path / "pkg" / "setup.py", _SETUP % "n = pkg.nope:main")
    errors = check_entry_points(tmp_path)
    assert len(errors) == 1
    assert "módulo inexistente" in errors[0]


def test_entry_point_missing_function_detected(tmp_path: Path) -> None:
    _write(tmp_path / "pkg" / "setup.py", _SETUP % "n = pkg.mod:main")
    _write(tmp_path / "pkg" / "pkg" / "mod.py", "class Other:\n    def main(self) -> None: ...\n")
    errors = check_entry_points(tmp_path)
    assert len(errors) == 1
    assert "función 'main'" in errors[0]


def test_entry_point_malformed_detected(tmp_path: Path) -> None:
    _write(tmp_path / "pkg" / "setup.py", _SETUP % "n = pkg.mod")
    assert "mal formada" in check_entry_points(tmp_path)[0]


def test_colcon_package_missing_files_detected(tmp_path: Path) -> None:
    _write(tmp_path / "src" / "p1" / "setup.py", "")
    _write(tmp_path / "src" / "p2" / "package.xml", "<package/>")
    errors = check_colcon_packages(tmp_path)
    assert "p1: falta package.xml" in errors
    assert "p1: falta setup.cfg" in errors
    assert "p1: falta resource/p1" in errors
    assert errors
    assert all(not e.startswith("p2") for e in errors)
