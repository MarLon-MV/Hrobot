"""Valida exploration_params.yaml contra los parametros declarados en exploration_node."""

from __future__ import annotations

import ast
import math
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / "src" / "hrobot_bringup" / "config" / "exploration_params.yaml"
NODE = ROOT / "src" / "hrobot_core" / "hrobot_core" / "adapters" / "exploration_node.py"
INT_PARAMS = (
    "min_frontier_size",
    "max_consecutive_failures",
    "no_frontier_confirmations",
    "kidnap_clear_cycles",
)
POSITIVE_FLOATS = (
    "blacklist_radius",
    "min_goal_distance",
    "exploration_period_s",
    "map_save_timeout_s",
    "undock_timeout_s",
    "status_timeout_s",
)
NEW_PARAMS = {
    "create3_namespace",
    "undock_timeout_s",
    "status_timeout_s",
    "kidnap_clear_cycles",
    "min_battery_pct",
}


def _yaml() -> dict[str, Any]:
    data = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _node_params() -> dict[str, Any]:
    return _yaml()["exploration_node"]["ros__parameters"]


def _declared_names() -> set[str]:
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    for n in tree.body:
        target = None
        if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name):
            target = n.target.id
        if target == "_PARAM_DEFAULTS" and isinstance(n.value, ast.Dict):  # type: ignore[union-attr]
            return {str(k.value) for k in n.value.keys if isinstance(k, ast.Constant)}
    raise AssertionError("_PARAM_DEFAULTS no encontrado")


def test_names_match_node_declarations() -> None:
    assert set(_node_params()) == _declared_names()


def test_int_params_are_positive_ints() -> None:
    for name in INT_PARAMS:
        v = _node_params()[name]
        assert isinstance(v, int) and not isinstance(v, bool) and v >= 1, name


def test_float_params_are_finite_positive() -> None:
    for name in POSITIVE_FLOATS:
        v = _node_params()[name]
        assert isinstance(v, float) and math.isfinite(v) and v > 0, name


def test_map_save_name_is_non_empty_str() -> None:
    v = _node_params()["map_save_name"]
    assert isinstance(v, str) and v.strip()


def test_safety_params_declared_in_yaml_and_node() -> None:
    assert set(_node_params()) >= NEW_PARAMS
    assert _declared_names() >= NEW_PARAMS


def test_create3_namespace_is_valid_str() -> None:
    v = _node_params()["create3_namespace"]
    assert isinstance(v, str) and v.startswith("/") and not any(c.isspace() for c in v)


def test_min_battery_pct_in_range() -> None:
    v = _node_params()["min_battery_pct"]
    assert isinstance(v, float) and math.isfinite(v) and 0.0 <= v <= 100.0


def test_map_saver_thresholds_in_range() -> None:
    p = _yaml()["map_saver"]["ros__parameters"]
    assert 0.0 <= p["free_thresh_default"] < p["occupied_thresh_default"] <= 1.0
    assert p["save_map_timeout"] > 0


def test_map_saver_lifecycle_manages_map_saver() -> None:
    p = _yaml()["lifecycle_manager_map_saver"]["ros__parameters"]
    assert p["node_names"] == ["map_saver"]
    assert p["autostart"] is True


def test_has_todo_verify_markers() -> None:
    assert "TODO(verify)" in PARAMS.read_text(encoding="utf-8")


def test_no_velocity_smoother() -> None:
    assert "velocity_smoother" not in PARAMS.read_text(encoding="utf-8")
