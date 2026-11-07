"""Pruebas de los fakes de ROS/pyserial."""

import importlib
import sys
from pathlib import Path

import pytest
from ros_fakes import FakeRos, SerialException, is_node_module

MODULE_NAME = "hrobot_core.adapters.dummy_node"


def _make_node(ros: FakeRos):  # noqa: ANN202 - fake dinámico
    import rclpy.node

    return rclpy.node.Node("n")


def test_parameter_default_and_override(ros_fakes: FakeRos) -> None:
    ros_fakes.set_params({"rate": 20})
    node = _make_node(ros_fakes)
    assert node.declare_parameter("rate", 10).value == 20
    assert node.declare_parameter("other", 1.5).value == 1.5
    assert node.get_parameter("other").value == 1.5
    with pytest.raises(KeyError):
        node.get_parameter("undeclared")


def test_publisher_records_messages(ros_fakes: FakeRos) -> None:
    from geometry_msgs.msg import Twist

    node = _make_node(ros_fakes)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    msg = Twist()
    msg.linear.x = 0.3
    pub.publish(msg)
    assert pub.published == [msg]
    assert ros_fakes.published("/cmd_vel") == [msg]
    assert ros_fakes.published("/other") == []


def test_subscription_receives_deliver(ros_fakes: FakeRos) -> None:
    from sensor_msgs.msg import LaserScan

    node = _make_node(ros_fakes)
    got: list[object] = []
    node.create_subscription(LaserScan, "/scan", got.append, 10)
    scan = LaserScan()
    assert ros_fakes.deliver("/scan", scan) == 1
    assert ros_fakes.deliver("/nothing", scan) == 0
    assert got == [scan]


def test_timer_fires_per_fake_clock_and_cancel(ros_fakes: FakeRos) -> None:
    node = _make_node(ros_fakes)
    calls: list[int] = []
    timer = node.create_timer(0.5, lambda: calls.append(1))
    assert ros_fakes.fire_timers() == 0
    ros_fakes.advance(0.4)
    assert ros_fakes.fire_timers() == 0
    ros_fakes.advance(0.1)
    assert ros_fakes.fire_timers() == 1
    assert ros_fakes.fire_timers() == 0
    ros_fakes.advance(0.5)
    import rclpy

    rclpy.spin(node)
    assert len(calls) == 2
    timer.cancel()
    ros_fakes.advance(5)
    assert ros_fakes.fire_timers() == 0


def test_logger_records_levels(ros_fakes: FakeRos) -> None:
    node = _make_node(ros_fakes)
    log = node.get_logger()
    log.info("a")
    log.warning("b")
    log.error("c")
    assert log.records == [("info", "a"), ("warning", "b"), ("error", "c")]
    assert ros_fakes.logs("warning") == [("warning", "b")]
    assert len(ros_fakes.logs()) == 3


def test_clock_and_rclpy_lifecycle(ros_fakes: FakeRos) -> None:
    import rclpy

    assert not rclpy.ok()
    rclpy.init(args=None)
    assert rclpy.ok()
    node = _make_node(ros_fakes)
    ros_fakes.advance(2.5)
    now = node.get_clock().now()
    assert now.nanoseconds == 2_500_000_000
    stamp = now.to_msg()
    assert (stamp.sec, stamp.nanosec) == (2, 500_000_000)
    node.destroy_node()
    assert node.destroyed
    rclpy.shutdown()
    assert not rclpy.ok()


def test_qos_and_messages(ros_fakes: FakeRos) -> None:
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    from hrobot_interfaces.msg import MissionStatus

    qos = QoSProfile(
        depth=3,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    assert qos.depth == 3
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
    status = MissionStatus()
    assert (status.mission_id, status.state, status.delivered) == ("", "", False)


def test_serial_queue_read(ros_fakes: FakeRos) -> None:
    import serial

    ros_fakes.serial.data.extend(b"abcde")
    port = serial.Serial("/dev/ttyUSB0", 230400, timeout=0)
    assert port.is_open
    assert port.in_waiting == 5
    assert port.read(3) == b"abc"
    assert port.read(10) == b"de"
    assert port.read(1) == b""
    port.close()
    assert not port.is_open


def test_serial_injected_failures(ros_fakes: FakeRos) -> None:
    import serial

    assert serial.SerialException is SerialException
    ros_fakes.serial.open_error = serial.SerialException("no device")
    with pytest.raises(serial.SerialException):
        serial.Serial("/dev/ttyUSB0", 230400, timeout=0)
    ros_fakes.serial.open_error = None
    port = serial.Serial("/dev/ttyUSB0", 230400, timeout=0)
    ros_fakes.serial.read_error = serial.SerialException("unplugged")
    with pytest.raises(serial.SerialException):
        port.read(1)


def test_fixture_purges_node_modules_part1(ros_fakes: FakeRos, tmp_path: Path) -> None:
    """Importa un módulo ``*_node`` ficticio; el test siguiente comprueba que no sobrevive."""
    pkg = importlib.import_module("hrobot_core.adapters")
    src = tmp_path / "dummy_node.py"
    src.write_text("import rclpy\nVALUE = 1\n")
    pkg.__path__.append(str(tmp_path))
    try:
        mod = importlib.import_module(MODULE_NAME)
    finally:
        pkg.__path__.remove(str(tmp_path))
    assert mod.VALUE == 1
    assert MODULE_NAME in sys.modules


def test_fixture_purges_node_modules_part2(ros_fakes: FakeRos) -> None:
    assert MODULE_NAME not in sys.modules
    assert not [n for n in sys.modules if is_node_module(n)]
