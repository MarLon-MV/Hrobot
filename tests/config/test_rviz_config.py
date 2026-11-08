"""Valida la coherencia de hrobot_nav.rviz con nav2_params.yaml y slam_toolbox_params.yaml.

Que estos tests pasen NO demuestra que RViz2 cargue el archivo correctamente.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest
import yaml

BRINGUP_DIR = Path(__file__).resolve().parents[2] / "src" / "hrobot_bringup"
RVIZ_FILE = BRINGUP_DIR / "rviz" / "hrobot_nav.rviz"
CONFIG_DIR = BRINGUP_DIR / "config"

FRAME_MAP = "map"
PARTICLE_TOPIC = "/particle_cloud"
PLAN_TOPIC = "/plan"
LOCAL_PLAN_TOPIC = "/local_plan"
LOCAL_COSTMAP_TOPIC = "/local_costmap/costmap"
GLOBAL_COSTMAP_TOPIC = "/global_costmap/costmap"
BEST_EFFORT = "Best Effort"

PLUGIN_PREFIX = "rviz_default_plugins/"
CLASS_TF = PLUGIN_PREFIX + "TF"
CLASS_SCAN = PLUGIN_PREFIX + "LaserScan"
CLASS_MAP = PLUGIN_PREFIX + "Map"
CLASS_POSE_ARRAY = PLUGIN_PREFIX + "PoseArray"
CLASS_PATH = PLUGIN_PREFIX + "Path"
CLASS_INITIAL_POSE = PLUGIN_PREFIX + "SetInitialPose"
CLASS_GOAL = PLUGIN_PREFIX + "SetGoal"
CLASS_TOP_DOWN = PLUGIN_PREFIX + "TopDownOrtho"

Config = dict[str, Any]


def _load(path: Path) -> Config:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(data, dict), f"{path.name} no parsea como mapa YAML"
    return data


def _get(cfg: Any, *path: str) -> Any:
    """Navega por claves anidadas; devuelve None si falta alguna."""
    for key in path:
        if not isinstance(cfg, dict) or key not in cfg:
            return None
        cfg = cfg[key]
    return cfg


def _required(cfg: Config, *path: str) -> Any:
    value = _get(cfg, *path)
    assert value is not None, f"falta la clave requerida: {'.'.join(path)}"
    return value


def _displays(rviz: Config) -> list[Config]:
    displays = _get(rviz, "Visualization Manager", "Displays")
    return [d for d in displays or [] if isinstance(d, dict)]


def _tools(rviz: Config) -> list[Config]:
    tools = _get(rviz, "Visualization Manager", "Tools")
    return [t for t in tools or [] if isinstance(t, dict)]


def _by_class(rviz: Config, cls: str) -> list[Config]:
    return [d for d in _displays(rviz) if d.get("Class") == cls]


def _topic(display: Config) -> Any:
    return _get(display, "Topic", "Value")


def validate(rviz: Config, nav2: Config, slam: Config) -> list[str]:
    """Devuelve la lista de incoherencias entre RViz y las configs de Nav2/SLAM."""
    errors: list[str] = []
    errors += _check_frame(rviz, nav2)
    errors += _check_scan(rviz, nav2, slam)
    errors += _check_displays(rviz)
    return errors


def _check_frame(rviz: Config, nav2: Config) -> list[str]:
    fixed = _get(rviz, "Visualization Manager", "Global Options", "Fixed Frame")
    amcl = _get(nav2, "amcl", "ros__parameters", "global_frame_id")
    costmap = _get(nav2, "global_costmap", "global_costmap", "ros__parameters", "global_frame")
    errors = []
    if fixed != FRAME_MAP:
        errors.append(f"Fixed Frame debe ser {FRAME_MAP!r}, es {fixed!r}")
    if fixed != amcl:
        errors.append(f"Fixed Frame {fixed!r} != amcl.global_frame_id {amcl!r}")
    if fixed != costmap:
        errors.append(f"Fixed Frame {fixed!r} != global_costmap.global_frame {costmap!r}")
    return errors


def _check_scan(rviz: Config, nav2: Config, slam: Config) -> list[str]:
    amcl_scan = _get(nav2, "amcl", "ros__parameters", "scan_topic")
    slam_scan = _get(slam, "slam_toolbox", "ros__parameters", "scan_topic")
    errors = []
    if amcl_scan != slam_scan:
        errors.append(f"scan_topic amcl {amcl_scan!r} != slam_toolbox {slam_scan!r}")
    scans = _by_class(rviz, CLASS_SCAN)
    if not scans:
        return [*errors, "falta display LaserScan"]
    for scan in scans:
        if _topic(scan) != amcl_scan:
            errors.append(f"LaserScan en {_topic(scan)!r} != scan_topic {amcl_scan!r}")
        if _get(scan, "Topic", "Reliability Policy") != BEST_EFFORT:
            errors.append(f"LaserScan debe usar QoS {BEST_EFFORT!r}")
    return errors


def _check_displays(rviz: Config) -> list[str]:
    errors = []
    if not _by_class(rviz, CLASS_TF):
        errors.append("falta display TF")
    expected = {
        CLASS_POSE_ARRAY: [PARTICLE_TOPIC],
        CLASS_PATH: [PLAN_TOPIC, LOCAL_PLAN_TOPIC],
        CLASS_MAP: [LOCAL_COSTMAP_TOPIC, GLOBAL_COSTMAP_TOPIC],
    }
    for cls, topics in expected.items():
        found = {_topic(d) for d in _by_class(rviz, cls)}
        errors += [f"falta {cls} en {t}" for t in topics if t not in found]
    tool_classes = {t.get("Class") for t in _tools(rviz)}
    for cls in (CLASS_INITIAL_POSE, CLASS_GOAL):
        if cls not in tool_classes:
            errors.append(f"falta tool {cls}")
    return errors


@pytest.fixture(scope="module")
def rviz() -> Config:
    return _load(RVIZ_FILE)


@pytest.fixture(scope="module")
def nav2() -> Config:
    return _load(CONFIG_DIR / "nav2_params.yaml")


@pytest.fixture(scope="module")
def slam() -> Config:
    return _load(CONFIG_DIR / "slam_toolbox_params.yaml")


def test_rviz_parses_as_yaml(rviz: Config) -> None:
    assert "Visualization Manager" in rviz


def test_fixed_frame_matches_nav2(rviz: Config, nav2: Config) -> None:
    fixed = _required(rviz, "Visualization Manager", "Global Options", "Fixed Frame")
    assert fixed == FRAME_MAP
    assert fixed == _required(nav2, "amcl", "ros__parameters", "global_frame_id")
    assert fixed == _required(
        nav2, "global_costmap", "global_costmap", "ros__parameters", "global_frame"
    )
    # El local_costmap trabaja en odom y es correcto: no se compara con Fixed Frame.
    assert (
        _required(nav2, "local_costmap", "local_costmap", "ros__parameters", "global_frame")
        != fixed
    )


def test_scan_display_matches_configs(rviz: Config, nav2: Config, slam: Config) -> None:
    amcl_scan = _required(nav2, "amcl", "ros__parameters", "scan_topic")
    slam_scan = _required(slam, "slam_toolbox", "ros__parameters", "scan_topic")
    scans = _by_class(rviz, CLASS_SCAN)
    assert len(scans) == 1
    assert _topic(scans[0]) == amcl_scan == slam_scan
    assert _get(scans[0], "Topic", "Reliability Policy") == BEST_EFFORT


def test_required_displays_present(rviz: Config) -> None:
    assert len(_by_class(rviz, CLASS_TF)) == 1
    assert [_topic(d) for d in _by_class(rviz, CLASS_POSE_ARRAY)] == [PARTICLE_TOPIC]
    path_topics = {_topic(d) for d in _by_class(rviz, CLASS_PATH)}
    assert {PLAN_TOPIC, LOCAL_PLAN_TOPIC} <= path_topics


def test_costmap_displays_by_topic(rviz: Config) -> None:
    topics = [_topic(d) for d in _by_class(rviz, CLASS_MAP)]
    assert LOCAL_COSTMAP_TOPIC in topics
    assert GLOBAL_COSTMAP_TOPIC in topics
    assert "/map" in topics


def test_tools_present(rviz: Config) -> None:
    tools = {t.get("Class"): t for t in _tools(rviz)}
    assert _get(tools[CLASS_INITIAL_POSE], "Topic", "Value") == "/initialpose"
    assert _get(tools[CLASS_GOAL], "Topic", "Value") == "/goal_pose"


def test_view_is_top_down_on_robot(rviz: Config) -> None:
    view = _required(rviz, "Visualization Manager", "Views", "Current")
    assert view["Class"] == CLASS_TOP_DOWN
    assert view["Target Frame"] == "base_link"


def test_validator_accepts_real_configs(rviz: Config, nav2: Config, slam: Config) -> None:
    assert validate(rviz, nav2, slam) == []


def test_validator_detects_scan_topic_mismatch(rviz: Config, nav2: Config, slam: Config) -> None:
    bad = copy.deepcopy(rviz)
    _by_class(bad, CLASS_SCAN)[0]["Topic"]["Value"] = "/other_scan"
    assert any("LaserScan" in e for e in validate(bad, nav2, slam))


def test_validator_detects_wrong_fixed_frame(rviz: Config, nav2: Config, slam: Config) -> None:
    bad = copy.deepcopy(rviz)
    bad["Visualization Manager"]["Global Options"]["Fixed Frame"] = "odom"
    assert any("Fixed Frame" in e for e in validate(bad, nav2, slam))


def test_validator_detects_missing_tool_and_qos(rviz: Config, nav2: Config, slam: Config) -> None:
    bad = copy.deepcopy(rviz)
    bad["Visualization Manager"]["Tools"] = [t for t in _tools(bad) if t["Class"] != CLASS_GOAL]
    _by_class(bad, CLASS_SCAN)[0]["Topic"]["Reliability Policy"] = "Reliable"
    errors = validate(bad, nav2, slam)
    assert any("SetGoal" in e for e in errors)
    assert any("QoS" in e for e in errors)


def test_validator_detects_nav2_slam_scan_mismatch(
    rviz: Config, nav2: Config, slam: Config
) -> None:
    bad = copy.deepcopy(slam)
    bad["slam_toolbox"]["ros__parameters"]["scan_topic"] = "/other"
    assert any("slam_toolbox" in e for e in validate(rviz, nav2, bad))
