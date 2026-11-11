"""Analisis estatico (ast) de exploration.launch.py y del modo use_slam de navigation."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
LAUNCH_DIR = ROOT / "src" / "hrobot_bringup" / "launch"
EXPLORATION = LAUNCH_DIR / "exploration.launch.py"
NAVIGATION = LAUNCH_DIR / "navigation.launch.py"
SRC = EXPLORATION.read_text(encoding="utf-8")
TREE = ast.parse(SRC)
NAV_SRC = NAVIGATION.read_text(encoding="utf-8")
NAV_TREE = ast.parse(NAV_SRC)


def _strings(tree: ast.AST) -> list[str]:
    return [
        n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
    ]


def _node_kwargs(tree: ast.AST) -> list[dict[str, ast.expr]]:
    return [
        {str(k.arg): k.value for k in n.keywords}
        for n in ast.walk(tree)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "Node"
    ]


def _executables() -> list[str]:
    return [
        str(kw["executable"].value)
        for kw in _node_kwargs(TREE)
        if isinstance(kw.get("executable"), ast.Constant)
    ]


def test_launches_exploration_node_and_map_saver() -> None:
    exes = _executables()
    assert "exploration_node" in exes
    assert "map_saver_server" in exes
    assert "lifecycle_manager" in exes


def test_no_amcl_nor_static_map_server() -> None:
    strings = _strings(TREE)
    for banned in ("amcl", "nav2_amcl", "map_server", "nav2_amcl"):
        assert banned not in strings
    assert "map_server" not in _executables()


def test_only_supervisor_publishes_cmd_vel() -> None:
    strings = _strings(TREE)
    assert not [s for s in strings if s.strip("/").endswith("cmd_vel") and s.strip("/")]
    assert "supervisor_node" not in _executables()  # lo lanza safety.launch.py via mapping
    assert "velocity_smoother" not in SRC


def test_includes_mapping_and_navigation_in_slam_mode() -> None:
    strings = _strings(TREE)
    assert "mapping.launch.py" in strings
    assert "navigation.launch.py" in strings
    assert "use_slam" in strings
    assert "true" in strings


def test_reuses_existing_arguments() -> None:
    declared = {
        str(n.args[0].value)
        for n in ast.walk(TREE)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "DeclareLaunchArgument"
        and isinstance(n.args[0], ast.Constant)
    }
    assert {"use_sim_time", "create3_namespace", "nav_cmd_vel_topic", "map_save_path"} <= declared


def test_tf_remappings_for_every_node() -> None:
    kws = _node_kwargs(TREE)
    assert len(kws) == 3
    for kw in kws:
        remaps = ast.get_source_segment(SRC, kw["remappings"]) or ""
        assert "_tf_remappings()" in remaps
    assert "tf_static" in _strings(TREE)


def test_map_saved_under_bringup_maps() -> None:
    assert "maps" in _strings(TREE)
    assert "exploration_params.yaml" in _strings(TREE)


def test_no_business_logic_imports() -> None:
    imports = [
        n.module
        for n in ast.walk(TREE)
        if isinstance(n, ast.ImportFrom) and n.module and n.module.startswith("hrobot_core")
    ]
    assert imports == ["hrobot_core.adapters.names"]


@pytest.mark.parametrize("frag", ["/home/", "/Users/", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"])
def test_no_forbidden_fragments(frag: str) -> None:
    assert frag not in SRC


def test_navigation_use_slam_defaults_to_false() -> None:
    for n in ast.walk(NAV_TREE):
        if (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Name)
            and n.func.id == "DeclareLaunchArgument"
            and isinstance(n.args[0], ast.Constant)
            and n.args[0].value == "use_slam"
        ):
            assert ast.literal_eval(n.keywords[0].value) == "false"
            return
    raise AssertionError("use_slam no declarado en navigation.launch.py")


def test_navigation_slam_mode_skips_localization_and_includes() -> None:
    # amcl, map_server, su lifecycle y sensors/safety llevan condition=UnlessCondition(use_slam)
    assert "UnlessCondition" in NAV_SRC
    assert NAV_SRC.count("condition=not_slam") == 4
    assert 'UnlessCondition(LaunchConfiguration("use_slam"))' in NAV_SRC


def test_navigation_slam_mode_keeps_safety_remaps_and_odom() -> None:
    assert '("cmd_vel", LaunchConfiguration("nav_cmd_vel_topic"))' in NAV_SRC
    assert "_tf_remappings()" in NAV_SRC
    assert "odom," in NAV_SRC  # odom_topic de bt_navigator
    assert "velocity_smoother" not in NAV_SRC
    assert "validate_nav_cmd_vel_topic" in NAV_SRC
