"""Valores exactos de las constantes de nombres."""

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


def test_cmd_vel_topics_differ() -> None:
    assert names.CMD_VEL_TOPIC != names.NAV_CMD_VEL_TOPIC
