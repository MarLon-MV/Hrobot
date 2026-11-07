"""Valida la coherencia de nav2_params.yaml y slam_toolbox_params.yaml.

Que estos tests pasen NO demuestra que Nav2 arranque con los archivos.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[2] / "src" / "hrobot_bringup" / "config"

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
SCAN_TOPIC = "/scan"
MAX_LINEAR_VEL = 0.3
MAX_ANGULAR_VEL = 1.0
RADIUS_MIN = 0.10
RADIUS_MAX = 0.30
MAX_LASER_RANGE_LD19 = 12.0

Config = dict[str, Any]


def _load(name: str) -> Config:
    data = yaml.safe_load((CONFIG_DIR / name).read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _get(cfg: Any, *path: str) -> Any:
    """Navega por claves anidadas; devuelve None si falta alguna."""
    for key in path:
        if not isinstance(cfg, dict) or key not in cfg:
            return None
        cfg = cfg[key]
    return cfg


def _params(nav2: Config, slam: Config) -> dict[str, Any]:
    """Extrae los valores relevantes (None si faltan)."""
    lc = ("local_costmap", "local_costmap", "ros__parameters")
    gc = ("global_costmap", "global_costmap", "ros__parameters")
    fp = ("controller_server", "ros__parameters", "FollowPath")
    sl = ("slam_toolbox", "ros__parameters")
    return {
        "frames_map": {
            "amcl.global_frame_id": _get(nav2, "amcl", "ros__parameters", "global_frame_id"),
            "bt_navigator.global_frame": _get(
                nav2, "bt_navigator", "ros__parameters", "global_frame"
            ),
            "global_costmap.global_frame": _get(nav2, *gc, "global_frame"),
            "slam_toolbox.map_frame": _get(slam, *sl, "map_frame"),
        },
        "frames_odom": {
            "amcl.odom_frame_id": _get(nav2, "amcl", "ros__parameters", "odom_frame_id"),
            "local_costmap.global_frame": _get(nav2, *lc, "global_frame"),
            "slam_toolbox.odom_frame": _get(slam, *sl, "odom_frame"),
        },
        "frames_base": {
            "amcl.base_frame_id": _get(nav2, "amcl", "ros__parameters", "base_frame_id"),
            "bt_navigator.robot_base_frame": _get(
                nav2, "bt_navigator", "ros__parameters", "robot_base_frame"
            ),
            "local_costmap.robot_base_frame": _get(nav2, *lc, "robot_base_frame"),
            "global_costmap.robot_base_frame": _get(nav2, *gc, "robot_base_frame"),
            "slam_toolbox.base_frame": _get(slam, *sl, "base_frame"),
        },
        "scan_topics": {
            "amcl.scan_topic": _get(nav2, "amcl", "ros__parameters", "scan_topic"),
            "local_costmap.obstacle_layer.scan.topic": _get(
                nav2, *lc, "obstacle_layer", "scan", "topic"
            ),
            "global_costmap.obstacle_layer.scan.topic": _get(
                nav2, *gc, "obstacle_layer", "scan", "topic"
            ),
            "slam_toolbox.scan_topic": _get(slam, *sl, "scan_topic"),
        },
        "fp": {k: _get(nav2, *fp, k) for k in ("min_vel_x", "max_vel_x", "max_vel_theta")},
    }


def _is_num(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool)


def _check_equal(group: dict[str, Any], expected: str, label: str) -> list[str]:
    return [
        f"{label}: {key} = {val!r}, se esperaba {expected!r}"
        for key, val in group.items()
        if val != expected
    ]


def _check_velocity(nav2: Config, fp: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    max_x, max_th, min_x = fp["max_vel_x"], fp["max_vel_theta"], fp["min_vel_x"]
    if not _is_num(max_x) or not 0.0 < max_x <= MAX_LINEAR_VEL:
        errors.append(f"max_vel_x = {max_x!r} fuera de (0, {MAX_LINEAR_VEL}]")
    if not _is_num(max_th) or not 0.0 < max_th <= MAX_ANGULAR_VEL:
        errors.append(f"max_vel_theta = {max_th!r} fuera de (0, {MAX_ANGULAR_VEL}]")
    if not _is_num(min_x) or min_x != 0.0:
        errors.append(f"min_vel_x = {min_x!r}, se esperaba 0.0 (sin marcha atrás)")
    smoother = _get(nav2, "velocity_smoother", "ros__parameters", "max_velocity")
    if not (isinstance(smoother, list) and smoother and _is_num(smoother[0])):
        errors.append(f"velocity_smoother.max_velocity inválido: {smoother!r}")
    elif not 0.0 < smoother[0] <= MAX_LINEAR_VEL:
        errors.append(
            f"velocity_smoother.max_velocity[0] = {smoother[0]!r} excede {MAX_LINEAR_VEL}"
        )
    return errors


def _check_costmaps(nav2: Config, slam: Config) -> list[str]:
    errors: list[str] = []
    slam_res = _get(slam, "slam_toolbox", "ros__parameters", "resolution")
    for name in ("local_costmap", "global_costmap"):
        cm = _get(nav2, name, name, "ros__parameters")
        radius = _get(cm, "robot_radius")
        infl = _get(cm, "inflation_layer", "inflation_radius")
        if not _is_num(radius) or not RADIUS_MIN < radius < RADIUS_MAX:
            errors.append(f"{name}.robot_radius = {radius!r} fuera de ({RADIUS_MIN}, {RADIUS_MAX})")
        elif not _is_num(infl) or infl <= radius:
            errors.append(f"{name}.inflation_radius = {infl!r} debe ser > robot_radius {radius!r}")
        res = _get(cm, "resolution")
        if res != slam_res:
            errors.append(f"{name}.resolution = {res!r} != slam_toolbox.resolution {slam_res!r}")
    return errors


def _check_misc(nav2: Config, slam: Config) -> list[str]:
    errors: list[str] = []
    sl = _get(slam, "slam_toolbox", "ros__parameters")
    if _get(sl, "do_loop_closing") is not True:
        errors.append("slam_toolbox.do_loop_closing debe ser true")
    max_range = _get(sl, "max_laser_range")
    if not _is_num(max_range) or not 0.0 < max_range <= MAX_LASER_RANGE_LD19:
        errors.append(f"slam_toolbox.max_laser_range = {max_range!r} fuera de (0, 12.0]")
    for label, cfg in (("nav2", nav2), ("slam", slam)):
        errors.extend(f"{label}: use_sim_time true en {p}" for p in _sim_time_true(cfg))
    return errors


def _sim_time_true(node: Any, path: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(node, dict):
        for key, val in node.items():
            sub = f"{path}.{key}" if path else str(key)
            if key == "use_sim_time" and val is not False:
                found.append(sub)
            found.extend(_sim_time_true(val, sub))
    return found


def validate(nav2: Config, slam: Config) -> list[str]:
    """Devuelve la lista de incoherencias entre ambas configuraciones (vacía si OK)."""
    for label, cfg, key in (("nav2", nav2, "amcl"), ("slam", slam, "slam_toolbox")):
        if _get(cfg, key, "ros__parameters") is None:
            return [f"{label}: falta {key}.ros__parameters"]
    p = _params(nav2, slam)
    errors: list[str] = []
    errors += _check_equal(p["frames_map"], FRAME_MAP, "frame map")
    errors += _check_equal(p["frames_odom"], FRAME_ODOM, "frame odom")
    errors += _check_equal(p["frames_base"], FRAME_BASE, "frame base")
    errors += _check_equal(p["scan_topics"], SCAN_TOPIC, "scan topic")
    errors += _check_velocity(nav2, p["fp"])
    errors += _check_costmaps(nav2, slam)
    errors += _check_misc(nav2, slam)
    return errors


@pytest.fixture(name="nav2")
def nav2_fixture() -> Config:
    return _load("nav2_params.yaml")


@pytest.fixture(name="slam")
def slam_fixture() -> Config:
    return _load("slam_toolbox_params.yaml")


LC = ("local_costmap", "local_costmap", "ros__parameters")
GC = ("global_costmap", "global_costmap", "ros__parameters")


def test_real_files_have_ros_parameters(nav2: Config, slam: Config) -> None:
    for node in ("amcl", "bt_navigator", "controller_server", "planner_server", "map_server"):
        assert _get(nav2, node, "ros__parameters") is not None, node
    assert _get(nav2, *LC) is not None
    assert _get(nav2, *GC) is not None
    assert _get(nav2, "velocity_smoother", "ros__parameters") is not None
    assert _get(slam, "slam_toolbox", "ros__parameters") is not None


def test_map_server_has_empty_yaml_filename(nav2: Config) -> None:
    assert _get(nav2, "map_server", "ros__parameters", "yaml_filename") == ""


def test_real_files_are_coherent(nav2: Config, slam: Config) -> None:
    assert validate(nav2, slam) == []


def test_frames_map_roles(nav2: Config, slam: Config) -> None:
    for key, val in _params(nav2, slam)["frames_map"].items():
        assert val == "map", key


def test_frames_odom_roles(nav2: Config, slam: Config) -> None:
    for key, val in _params(nav2, slam)["frames_odom"].items():
        assert val == "odom", key
    assert _get(nav2, *LC, "global_frame") == "odom"


def test_frames_base_roles(nav2: Config, slam: Config) -> None:
    for key, val in _params(nav2, slam)["frames_base"].items():
        assert val == "base_link", key


def test_scan_topic_everywhere(nav2: Config, slam: Config) -> None:
    for key, val in _params(nav2, slam)["scan_topics"].items():
        assert val == "/scan", key


def test_robot_radius_in_range(nav2: Config) -> None:
    for path in (LC, GC):
        assert 0.10 < _get(nav2, *path, "robot_radius") < 0.30


def test_inflation_greater_than_radius(nav2: Config) -> None:
    for path in (LC, GC):
        radius = _get(nav2, *path, "robot_radius")
        assert _get(nav2, *path, "inflation_layer", "inflation_radius") > radius


def test_velocity_limits(nav2: Config) -> None:
    fp = _get(nav2, "controller_server", "ros__parameters", "FollowPath")
    assert fp["plugin"] == "dwb_core::DWBLocalPlanner"
    assert fp["max_vel_x"] <= 0.3
    assert fp["max_vel_theta"] <= 1.0
    assert fp["min_vel_x"] == 0.0


def test_velocity_smoother_cap(nav2: Config) -> None:
    assert _get(nav2, "velocity_smoother", "ros__parameters", "max_velocity")[0] <= 0.3


def test_loop_closing_enabled(slam: Config) -> None:
    assert _get(slam, "slam_toolbox", "ros__parameters", "do_loop_closing") is True


def test_resolution_matches(nav2: Config, slam: Config) -> None:
    res = _get(slam, "slam_toolbox", "ros__parameters", "resolution")
    assert _get(nav2, *LC, "resolution") == res
    assert _get(nav2, *GC, "resolution") == res


def test_no_sim_time_true(nav2: Config, slam: Config) -> None:
    assert _sim_time_true(nav2) == []
    assert _sim_time_true(slam) == []


def _validate_mutated(
    nav2: Config, slam: Config, which: str, path: tuple[str, ...], value: Any
) -> list[str]:
    n, s = copy.deepcopy(nav2), copy.deepcopy(slam)
    node = n if which == "nav2" else s
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    return validate(n, s)


def test_neg_inconsistent_base_frame(nav2: Config, slam: Config) -> None:
    errs = _validate_mutated(
        nav2,
        slam,
        "nav2",
        ("bt_navigator", "ros__parameters", "robot_base_frame"),
        "base_footprint",
    )
    assert any("base_footprint" in e for e in errs)


def test_neg_scan_topic_differs(nav2: Config, slam: Config) -> None:
    errs = _validate_mutated(
        nav2, slam, "slam", ("slam_toolbox", "ros__parameters", "scan_topic"), "scan"
    )
    assert any("scan topic" in e for e in errs)


def test_neg_max_vel_x_too_high(nav2: Config, slam: Config) -> None:
    path = ("controller_server", "ros__parameters", "FollowPath", "max_vel_x")
    assert any("max_vel_x" in e for e in _validate_mutated(nav2, slam, "nav2", path, 0.5))


def test_neg_min_vel_x_negative(nav2: Config, slam: Config) -> None:
    path = ("controller_server", "ros__parameters", "FollowPath", "min_vel_x")
    assert any("min_vel_x" in e for e in _validate_mutated(nav2, slam, "nav2", path, -0.1))


def test_neg_local_costmap_in_map_frame(nav2: Config, slam: Config) -> None:
    errs = _validate_mutated(nav2, slam, "nav2", (*LC, "global_frame"), "map")
    assert any("local_costmap.global_frame" in e for e in errs)


def test_neg_inflation_below_radius(nav2: Config, slam: Config) -> None:
    errs = _validate_mutated(nav2, slam, "nav2", (*LC, "inflation_layer", "inflation_radius"), 0.1)
    assert any("inflation_radius" in e for e in errs)


def test_neg_missing_section_is_reported_not_raised(slam: Config) -> None:
    assert validate({}, slam) == ["nav2: falta amcl.ros__parameters"]


def test_neg_sim_time_true(nav2: Config, slam: Config) -> None:
    errs = _validate_mutated(nav2, slam, "nav2", ("amcl", "ros__parameters", "use_sim_time"), True)
    assert any("use_sim_time" in e for e in errs)
