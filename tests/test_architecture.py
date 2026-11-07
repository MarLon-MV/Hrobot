"""Tests estáticos (ast) que hacen cumplir capas y la regla de /cmd_vel."""

from __future__ import annotations

import ast
from collections.abc import Callable
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "src" / "hrobot_core" / "hrobot_core"
ROS_MODULES = frozenset(
    {
        "rclpy",
        "sensor_msgs",
        "geometry_msgs",
        "std_msgs",
        "nav2_msgs",
        "hrobot_interfaces",
    }
)
SERIAL_MODULE = "serial"
FORBIDDEN_PURE = ROS_MODULES | {SERIAL_MODULE}
SUPERVISOR = "supervisor_node.py"
CMD_VEL_LITERAL = "/cmd_vel"
CMD_VEL_NAME = "CMD_VEL_TOPIC"


# ------------------------------------------------------------------ detectores
def imported_roots(source: str) -> set[str]:
    """Módulos raíz importados por ``source``."""
    roots: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            roots.add(node.module.split(".")[0])
    return roots


def _is_cmd_vel_arg(arg: ast.expr) -> bool:
    if isinstance(arg, ast.Constant):
        return arg.value == CMD_VEL_LITERAL
    if isinstance(arg, ast.Name):
        return arg.id == CMD_VEL_NAME
    if isinstance(arg, ast.Attribute):
        return arg.attr == CMD_VEL_NAME
    return False


def publishes_cmd_vel(source: str) -> bool:
    """True si crea un publisher cuyo tópico es CMD_VEL_TOPIC o el literal ``/cmd_vel``."""
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == "create_publisher"):
            continue
        args = [*node.args, *(kw.value for kw in node.keywords if kw.arg == "topic")]
        if any(_is_cmd_vel_arg(a) for a in args):
            return True
    return False


def uses_sleep_or_print(source: str) -> bool:
    """True si llama a ``time.sleep``/``sleep`` o a ``print``."""
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Name) and func.id in {"print", "sleep"}:
            return True
        if isinstance(func, ast.Attribute) and func.attr == "sleep":
            return True
    return False


def _files(subdir: str) -> list[Path]:
    return sorted((PKG / subdir).rglob("*.py"))


def _violations(files: list[Path], check: Callable[[str], bool]) -> list[str]:
    return [str(p.relative_to(PKG)) for p in files if check(p.read_text(encoding="utf-8"))]


# ------------------------------------------------------------------ código actual
def test_package_exists() -> None:
    assert _files("domain") and _files("adapters")


@pytest.mark.parametrize("layer", ["domain", "services"])
def test_pure_layers_do_not_import_ros(layer: str) -> None:
    bad = _violations(_files(layer), lambda s: bool(imported_roots(s) & FORBIDDEN_PURE))
    assert bad == []


def test_only_node_files_import_ros() -> None:
    files = [p for p in _files("adapters") if not p.name.endswith("_node.py")]
    assert _violations(files, lambda s: bool(imported_roots(s) & ROS_MODULES)) == []


def test_only_node_files_import_serial() -> None:
    files = [p for p in _files("adapters") if not p.name.endswith("_node.py")]
    assert _violations(files, lambda s: SERIAL_MODULE in imported_roots(s)) == []


def test_only_supervisor_publishes_cmd_vel() -> None:
    files = [p for p in PKG.rglob("*.py") if p.name != SUPERVISOR]
    assert _violations(files, publishes_cmd_vel) == []


def test_no_sleep_or_print_in_package() -> None:
    assert _violations(sorted(PKG.rglob("*.py")), uses_sleep_or_print) == []


# ------------------------------------------------------------------ fuentes sintéticas
@pytest.mark.parametrize(
    "source",
    [
        "import rclpy",
        "from rclpy.node import Node",
        "import serial",
        "from sensor_msgs.msg import LaserScan",
        "from geometry_msgs.msg import Twist",
        "import std_msgs.msg",
        "from nav2_msgs.action import NavigateToPose",
        "from hrobot_interfaces.msg import MissionStatus",
    ],
)
def test_import_detector_flags_violations(source: str) -> None:
    assert imported_roots(source) & FORBIDDEN_PURE


def test_import_detector_ignores_clean_sources() -> None:
    roots = imported_roots("import math\nfrom . import x\nfrom hrobot_core.domain import y")
    assert roots == {"math", "hrobot_core"}
    assert not roots & FORBIDDEN_PURE


def test_serial_only_rule_detects_synthetic_violation() -> None:
    assert SERIAL_MODULE in imported_roots("from serial import Serial")
    assert SERIAL_MODULE not in imported_roots("import math")


@pytest.mark.parametrize(
    "source",
    [
        "pub = node.create_publisher(Twist, '/cmd_vel', 10)",
        "pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)",
        "pub = self.create_publisher(Twist, names.CMD_VEL_TOPIC, 10)",
        "pub = self.create_publisher(Twist, topic='/cmd_vel', qos=10)",
    ],
)
def test_cmd_vel_detector_flags_violations(source: str) -> None:
    assert publishes_cmd_vel(source)


@pytest.mark.parametrize(
    "source",
    [
        "pub = self.create_publisher(Twist, NAV_CMD_VEL_TOPIC, 10)",
        "pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)",
        "sub = self.create_subscription(Twist, CMD_VEL_TOPIC, cb, 10)",
        "x = 1",
    ],
)
def test_cmd_vel_detector_ignores_clean_sources(source: str) -> None:
    assert not publishes_cmd_vel(source)


@pytest.mark.parametrize(
    "source",
    ["import time\ntime.sleep(1)", "from time import sleep\nsleep(1)", "print('x')"],
)
def test_sleep_print_detector_flags_violations(source: str) -> None:
    assert uses_sleep_or_print(source)


def test_sleep_print_detector_ignores_clean_sources() -> None:
    assert not uses_sleep_or_print("import logging\nlogging.getLogger().info('x')")
    assert not uses_sleep_or_print("# print('x')\ns = 'time.sleep(1)'")


def test_violation_scan_reports_offending_file(tmp_path: Path) -> None:
    bad = tmp_path / "bad.py"
    bad.write_text("import rclpy\n", encoding="utf-8")
    good = tmp_path / "good.py"
    good.write_text("import math\n", encoding="utf-8")
    found = [p.name for p in (bad, good) if imported_roots(p.read_text()) & FORBIDDEN_PURE]
    assert found == ["bad.py"]
