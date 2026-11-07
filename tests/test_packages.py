"""Verifica el scaffold de los paquetes y la configuración de herramientas."""

import importlib
import re
import tomllib
import xml.etree.ElementTree as ET  # noqa: S405  # solo parsea package.xml del propio repo
from pathlib import Path

import pytest

import hrobot_core

ROOT = Path(__file__).resolve().parents[1]

BUILD_TYPES = {
    "hrobot_interfaces": "ament_cmake",
    "hrobot_core": "ament_python",
    "hrobot_bringup": "ament_python",
}


def _dev_dependencies() -> list[str]:
    with (ROOT / "pyproject.toml").open("rb") as fh:
        data = tomllib.load(fh)
    dev: list[str] = data["project"]["optional-dependencies"]["dev"]
    return dev


def test_version() -> None:
    assert hrobot_core.__version__ == "0.1.0"


@pytest.mark.parametrize("layer", ["domain", "services", "adapters"])
def test_layers_importable_without_ros(layer: str) -> None:
    assert importlib.import_module(f"hrobot_core.{layer}") is not None


@pytest.mark.parametrize(("package", "build_type"), BUILD_TYPES.items())
def test_package_xml_build_type(package: str, build_type: str) -> None:
    path = ROOT / "src" / package / "package.xml"
    root = ET.parse(path).getroot()  # noqa: S314  # archivo del propio repo
    assert root.findtext("name") == package
    assert root.findtext("export/build_type") == build_type


def test_pyproject_dev_dependencies() -> None:
    names = {re.split(r"[<>=!~\s]", dep, maxsplit=1)[0].lower() for dep in _dev_dependencies()}
    assert {"pytest", "ruff", "mypy", "pyyaml"} <= names


def test_pyproject_matches_requirements_dev() -> None:
    lines = (ROOT / "requirements-dev.txt").read_text().splitlines()
    reqs = [ln.strip() for ln in lines if ln.strip() and not ln.startswith("#")]
    assert sorted(_dev_dependencies()) == sorted(reqs)
