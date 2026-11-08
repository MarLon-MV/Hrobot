"""Análisis estático (ast) de los launch files de hrobot_bringup. No requiere ROS.

Que estos tests pasen NO demuestra que los launch arranquen en un robot real.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src" / "hrobot_bringup"
CORE = ROOT / "src" / "hrobot_core"
LAUNCH_DIR = BRINGUP / "launch"
LAUNCH_NAMES = ("sensors", "safety", "mapping", "rviz")
LAUNCH_FILES = [LAUNCH_DIR / f"{n}.launch.py" for n in LAUNCH_NAMES]
FORBIDDEN_FRAGMENTS = ("/home/", "/Users/", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION")


def _tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _strings(tree: ast.AST) -> list[str]:
    return [
        n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
    ]


def _declared_args(tree: ast.AST) -> dict[str, str | None]:
    """Mapa nombre -> default_value de cada DeclareLaunchArgument."""
    out: dict[str, str | None] = {}
    for n in ast.walk(tree):
        is_decl = (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Name)
            and n.func.id == "DeclareLaunchArgument"
        )
        if not is_decl or not n.args or not isinstance(n.args[0], ast.Constant):
            continue
        default = None
        for kw in n.keywords:
            if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                default = kw.value.value
        out[str(n.args[0].value)] = default
    return out


def _console_scripts() -> set[str]:
    tree = _tree(CORE / "setup.py")
    scripts: set[str] = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.keyword) and n.arg == "entry_points":
            for s in _strings(n.value):
                if "=" in s:
                    scripts.add(s.split("=")[0].strip())
    return scripts


def _node_executables(tree: ast.AST) -> list[str]:
    exes: list[str] = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "Node":
            for kw in n.keywords:
                if kw.arg == "executable" and isinstance(kw.value, ast.Constant):
                    exes.append(str(kw.value.value))
    return exes


def _ld19_default_params() -> set[str]:
    tree = _tree(CORE / "hrobot_core" / "adapters" / "ld19_node.py")
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef) and fn.name == "_declare_parameters":
            for n in ast.walk(fn):
                if isinstance(n, ast.Dict):
                    return {str(k.value) for k in n.keys if isinstance(k, ast.Constant)}
    raise AssertionError("defaults de ld19_node no encontrados")


def _ld19_passed_params() -> set[str]:
    tree = _tree(LAUNCH_DIR / "sensors.launch.py")
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "Node":
            kws = {kw.arg: kw.value for kw in n.keywords}
            exe = kws.get("executable")
            if isinstance(exe, ast.Constant) and exe.value == "ld19_node":
                params = kws["parameters"]
                assert isinstance(params, ast.List)
                keys: set[str] = set()
                for d in params.elts:
                    assert isinstance(d, ast.Dict)
                    keys |= {str(k.value) for k in d.keys if isinstance(k, ast.Constant)}
                return keys
    raise AssertionError("Node ld19_node no encontrado")


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=LAUNCH_NAMES)
def test_defines_generate_launch_description(path: Path) -> None:
    names = {n.name for n in _tree(path).body if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in names


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=LAUNCH_NAMES)
def test_declares_use_sim_time_false(path: Path) -> None:
    assert _declared_args(_tree(path)).get("use_sim_time") == "false"


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=LAUNCH_NAMES)
def test_no_absolute_paths_or_env_fixing(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    for frag in FORBIDDEN_FRAGMENTS:
        assert frag not in text, f"{path.name} contiene {frag}"


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=LAUNCH_NAMES)
def test_compiles(path: Path) -> None:
    compile(path.read_text(encoding="utf-8"), str(path), "exec")


def test_sensors_args() -> None:
    args = _declared_args(_tree(LAUNCH_DIR / "sensors.launch.py"))
    assert args["lidar_port"] == "/dev/ttyUSB0"
    for name in ("laser_x", "laser_y", "laser_z", "laser_yaw"):
        assert args[name] == "0.0"


def test_safety_require_mission_status_default_true_and_documented() -> None:
    path = LAUNCH_DIR / "safety.launch.py"
    assert _declared_args(_tree(path))["require_mission_status"] == "true"
    text = path.read_text(encoding="utf-8")
    assert "banco" in text


def test_safety_declares_create3_namespace_and_passes_cmd_vel_topic() -> None:
    path = LAUNCH_DIR / "safety.launch.py"
    assert _declared_args(_tree(path))["create3_namespace"] == "/r2"
    text = path.read_text(encoding="utf-8")
    assert 'namespaced(namespace, "/cmd_vel")' in text
    assert '"cmd_vel_topic"' in text
    assert "hrobot_core.adapters.names" in text


def test_executables_exist_as_console_scripts() -> None:
    scripts = _console_scripts()
    assert {"ld19_node", "supervisor_node"} <= scripts
    assert "ld19_node" in _node_executables(_tree(LAUNCH_DIR / "sensors.launch.py"))
    assert "supervisor_node" in _node_executables(_tree(LAUNCH_DIR / "safety.launch.py"))
    for exe, launch in (("ld19_node", "sensors"), ("supervisor_node", "safety")):
        assert exe in scripts, f"{exe} ({launch}) no es console_script"


def test_ld19_params_exist_in_node_defaults() -> None:
    passed = _ld19_passed_params() - {"use_sim_time"}  # use_sim_time lo declara rclpy
    assert {"port", "baudrate", "frame_id", "scan_topic"} <= passed
    assert passed <= _ld19_default_params()


def test_mapping_includes_sensors_safety_and_slam() -> None:
    strings = _strings(_tree(LAUNCH_DIR / "mapping.launch.py"))
    assert "sensors.launch.py" in strings
    assert "safety.launch.py" in strings
    assert "online_async_launch.py" in strings
    assert "slam_toolbox_params.yaml" in strings


def test_mapping_does_not_reference_navigation() -> None:
    assert "navigation" not in (LAUNCH_DIR / "mapping.launch.py").read_text(encoding="utf-8")


def test_referenced_config_files_exist() -> None:
    for launch, rel in (
        ("mapping", BRINGUP / "config" / "slam_toolbox_params.yaml"),
        ("rviz", BRINGUP / "rviz" / "hrobot_nav.rviz"),
    ):
        assert rel.is_file()
        assert rel.name in _strings(_tree(LAUNCH_DIR / f"{launch}.launch.py"))


def test_rviz_launch_uses_package_share() -> None:
    text = (LAUNCH_DIR / "rviz.launch.py").read_text(encoding="utf-8")
    assert "get_package_share_directory" in text
    assert "rviz2" in _node_executables(_tree(LAUNCH_DIR / "rviz.launch.py"))


def _setup_install_dirs() -> set[str]:
    """Directorios instalados en share/hrobot_bringup, extraídos de setup.py con ast."""
    tree = _tree(BRINGUP / "setup.py")
    resource_dirs: tuple[str, ...] = ()
    for n in ast.walk(tree):
        if (
            isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "RESOURCE_DIRS" for t in n.targets)
            and isinstance(n.value, ast.Tuple)
        ):
            resource_dirs = tuple(str(e.value) for e in n.value.elts if isinstance(e, ast.Constant))
    assert resource_dirs, "RESOURCE_DIRS no encontrado"
    # Verifica que data_files los use con prefijo share/<paquete>/ y glob.
    setup_text = (BRINGUP / "setup.py").read_text(encoding="utf-8")
    assert "RESOURCE_DIRS" in setup_text.split("data_files", 1)[1]
    assert "glob" in setup_text
    assert ".gitkeep" not in setup_text
    return set(resource_dirs)


def test_setup_installs_required_dirs() -> None:
    assert {"launch", "config", "rviz", "maps"} <= _setup_install_dirs()
