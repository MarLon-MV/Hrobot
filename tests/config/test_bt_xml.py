"""Valida el árbol de comportamiento sin recuperaciones (bt_navigate_no_recovery.xml)."""

from __future__ import annotations

import xml.etree.ElementTree as ET  # noqa: S405 - XML propio del repo, no entrada externa
from pathlib import Path

import pytest

BT_PATH = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "hrobot_bringup"
    / "config"
    / "bt_navigate_no_recovery.xml"
)
FORBIDDEN = ("BackUp", "Spin", "Wait", "RecoveryNode", "ClearEntireCostmap")
REQUIRED = ("ComputePathToPose", "FollowPath", "PipelineSequence", "RateController")


def _tags(root: ET.Element) -> set[str]:
    return {el.tag for el in root.iter()}


@pytest.fixture(name="root")
def root_fixture() -> ET.Element:
    return ET.parse(BT_PATH).getroot()  # noqa: S314 - XML propio del repo


def test_xml_parses_with_main_tree(root: ET.Element) -> None:
    assert root.tag == "root"
    assert root.get("main_tree_to_execute") == "MainTree"
    assert root.find("BehaviorTree[@ID='MainTree']") is not None


@pytest.mark.parametrize("tag", FORBIDDEN)
def test_no_forbidden_nodes(root: ET.Element, tag: str) -> None:
    assert tag not in _tags(root)


@pytest.mark.parametrize("tag", REQUIRED)
def test_required_nodes(root: ET.Element, tag: str) -> None:
    assert tag in _tags(root)


def test_detects_forbidden_node_in_mutated_tree() -> None:
    bad = ET.fromstring("<root><BehaviorTree><BackUp/></BehaviorTree></root>")  # noqa: S314
    assert "BackUp" in _tags(bad)
