"""Fixtures compartidos."""

from collections.abc import Iterator

import pytest
from ros_fakes import FakeRos, purge_node_modules


@pytest.fixture
def ros_fakes(monkeypatch: pytest.MonkeyPatch) -> Iterator[FakeRos]:
    """Instala fakes de rclpy/mensajes/serial y purga módulos ``*_node`` antes y después."""
    purge_node_modules()
    fake = FakeRos()
    fake.install(monkeypatch.setitem)
    try:
        yield fake
    finally:
        purge_node_modules()
