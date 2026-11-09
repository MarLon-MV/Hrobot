"""Valores exactos de las constantes de nombres."""

import pytest
from hrobot_core.adapters import names


def test_exact_values() -> None:
    assert names.SCAN_TOPIC == "/scan"
    assert names.ODOM_TOPIC == "/odom"
    assert names.CMD_VEL_TOPIC == "/cmd_vel"
    assert names.NAV_CMD_VEL_TOPIC == "/cmd_vel_nav"
    assert names.MISSION_STATUS_TOPIC == "/hrobot/mission_status"
    assert names.FRAME_MAP == "map"
    assert names.FRAME_ODOM == "odom"
    assert names.FRAME_BASE == "base_link"
    assert names.FRAME_LASER == "laser"
    assert names.LD19_DEFAULT_PORT == "/dev/ttyUSB0"
    assert names.LD19_BAUDRATE == 230400


@pytest.mark.parametrize(
    ("namespace", "topic", "expected"),
    [
        ("/r2", "/cmd_vel", "/r2/cmd_vel"),
        ("r2", "/cmd_vel", "/r2/cmd_vel"),
        ("/r2/", "cmd_vel", "/r2/cmd_vel"),
        ("", "/cmd_vel", "/cmd_vel"),
        ("/", "/odom", "/odom"),
    ],
)
def test_namespaced(namespace: str, topic: str, expected: str) -> None:
    assert names.namespaced(namespace, topic) == expected


@pytest.mark.parametrize(
    ("namespace", "topic"), [("/r 2", "/cmd_vel"), (" ", "/odom"), ("/r2", "/a b")]
)
def test_namespaced_rejects_spaces(namespace: str, topic: str) -> None:
    with pytest.raises(ValueError):
        names.namespaced(namespace, topic)


@pytest.mark.parametrize(
    ("namespace", "topic", "expected"),
    [
        ("/r2", names.TF_TOPIC, "/r2/tf"),
        ("/r2", names.TF_STATIC_TOPIC, "/r2/tf_static"),
        ("/", names.TF_TOPIC, "/tf"),
        ("", names.TF_STATIC_TOPIC, "/tf_static"),
    ],
)
def test_tf_topics_namespaced(namespace: str, topic: str, expected: str) -> None:
    assert names.namespaced(namespace, topic) == expected


def test_tf_topic_values() -> None:
    assert names.TF_TOPIC == "/tf"
    assert names.TF_STATIC_TOPIC == "/tf_static"


def test_default_namespace() -> None:
    assert names.DEFAULT_CREATE3_NAMESPACE == "/r2"


def test_cmd_vel_topics_differ() -> None:
    assert names.CMD_VEL_TOPIC != names.NAV_CMD_VEL_TOPIC
