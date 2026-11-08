import ast
import importlib
import math
import struct
from pathlib import Path
from typing import Any

import pytest
from hrobot_core.adapters.ld19_parser import crc8
from hrobot_core.adapters.names import CMD_VEL_TOPIC
from ros_fakes import FakeRos, SerialException

NODE_FILE = (
    Path(__file__).resolve().parents[2] / "src/hrobot_core/hrobot_core/adapters/ld19_node.py"
)
CENTIDEG_PER_REV = 36000
READ_PERIOD_S = 0.01
RECONNECT_S = 2.0
NS_PER_S = 1_000_000_000
TS_STEP_MS = 3
WRAP_OFFSET_CENTIDEG = 300


def frame_bytes(start: int, end: int, ts: int, dist_mm: int = 1500) -> bytes:
    body = bytearray([0x54, 0x2C]) + struct.pack("<HH", 3600, start % CENTIDEG_PER_REV)
    for _ in range(12):
        body += struct.pack("<HB", dist_mm, 50)
    body += struct.pack("<HH", end % CENTIDEG_PER_REV, ts)
    return bytes(body) + bytes([crc8(bytes(body))])


def revolutions(frames_per_rev: list[int]) -> bytes:
    """Vueltas completas con distinto número de tramas (y por tanto de puntos)."""
    out = bytearray()
    tick = 0
    for n in frames_per_rev:
        span = CENTIDEG_PER_REV // n
        for k in range(n):
            start = k * span + WRAP_OFFSET_CENTIDEG  # el cierre de vuelta cae a mitad de trama
            out += frame_bytes(start, start + span - 100, tick * TS_STEP_MS)
            tick += 1
    return bytes(out)


def make_node(ros: FakeRos, **params: Any) -> Any:
    ros.set_params(params)
    module = importlib.import_module("hrobot_core.adapters.ld19_node")
    return module.Ld19Node()


def pump(ros: FakeRos, seconds: float = READ_PERIOD_S) -> None:
    ros.advance(seconds)
    ros.fire_timers()
    while ros.serial.data:  # cada lectura devuelve como máximo 4096 bytes
        ros.advance(READ_PERIOD_S)
        ros.fire_timers()


def scans(ros: FakeRos) -> list[Any]:
    return ros.published("/scan")


def test_publishes_fixed_length_scans(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes)
    ros_fakes.serial.data += revolutions([30, 31, 33, 36, 30])
    pump(ros_fakes)
    out = scans(ros_fakes)
    assert len(out) == 4
    for msg in out:
        assert len(msg.ranges) == 360
        assert len(msg.intensities) == 360
        assert msg.header.frame_id == "laser"
        assert msg.angle_min == 0.0
        assert msg.angle_increment == pytest.approx(2 * math.pi / 360)
        assert msg.angle_max == pytest.approx(2 * math.pi - 2 * math.pi / 360)
        assert msg.range_max == 12.0
        assert msg.time_increment == pytest.approx(msg.scan_time / 360)


def test_stamp_is_revolution_start(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes)
    ros_fakes.advance(5.0)
    ros_fakes.serial.data += revolutions([30, 30])
    pump(ros_fakes)
    (msg,) = scans(ros_fakes)
    now_ns = ros_fakes.now_ns
    stamp_ns = msg.header.stamp.sec * NS_PER_S + msg.header.stamp.nanosec
    assert msg.scan_time > 0.0
    assert stamp_ns == now_ns - round(msg.scan_time * NS_PER_S)


def test_publisher_topic_and_qos(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes, scan_topic="/my_scan", frame_id="lidar", bins=720)
    ros_fakes.serial.data += revolutions([30, 30])
    pump(ros_fakes)
    qos_mod = importlib.import_module("rclpy.qos")
    (pub,) = ros_fakes.publishers
    assert pub.topic == "/my_scan"
    assert pub.qos is qos_mod.qos_profile_sensor_data
    (msg,) = pub.published
    assert len(msg.ranges) == 720
    assert msg.header.frame_id == "lidar"


def test_message_types_are_strict(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes, range_max=12)  # entero inyectado: debe salir float
    ros_fakes.serial.data += revolutions([30, 32, 30, 31])
    pump(ros_fakes)
    out = scans(ros_fakes)
    assert out
    for msg in out:
        for name in (
            "angle_min",
            "angle_max",
            "angle_increment",
            "time_increment",
            "scan_time",
            "range_min",
            "range_max",
        ):
            assert type(getattr(msg, name)) is float, name
        for seq in (msg.ranges, msg.intensities):
            assert type(seq) is list
            assert all(type(v) is float for v in seq)
        assert type(msg.header.frame_id) is str
        assert type(msg.header.stamp.sec) is int
        assert type(msg.header.stamp.nanosec) is int


def test_open_failure_retries(ros_fakes: FakeRos) -> None:
    ros_fakes.serial.open_error = SerialException("no device")
    make_node(ros_fakes)
    assert len(ros_fakes.logs("error")) == 1
    assert ros_fakes.serial.instances == []
    pump(ros_fakes, RECONNECT_S)  # reintento fallido: un error más
    assert len(ros_fakes.logs("error")) == 2
    ros_fakes.serial.open_error = None
    ros_fakes.serial.data += revolutions([30, 30])
    pump(ros_fakes, RECONNECT_S)
    assert len(ros_fakes.serial.instances) == 1
    assert len(ros_fakes.logs("error")) == 2
    pump(ros_fakes)
    assert len(scans(ros_fakes)) == 1


def test_open_failure_oserror_does_not_raise(ros_fakes: FakeRos) -> None:
    ros_fakes.serial.open_error = OSError("busy")
    make_node(ros_fakes)
    assert ros_fakes.logs("error")


def test_read_failure_resets_state(ros_fakes: FakeRos) -> None:
    node = make_node(ros_fakes)
    stream = revolutions([30, 30])
    ros_fakes.serial.data += stream[:20]  # trama partida antes del fallo
    pump(ros_fakes)
    assert node._parser.buffered_bytes == 20
    ros_fakes.serial.read_error = SerialException("unplugged")
    pump(ros_fakes)
    first = ros_fakes.serial.instances[0]
    assert first.is_open is False
    assert node._parser.buffered_bytes == 0
    assert any("read failed" in m for _, m in ros_fakes.logs("error"))
    ros_fakes.serial.read_error = None
    ros_fakes.serial.data.clear()
    ros_fakes.serial.data += revolutions([30, 30])
    pump(ros_fakes, RECONNECT_S)  # reabre
    assert len(ros_fakes.serial.instances) == 2
    pump(ros_fakes)
    (msg,) = scans(ros_fakes)
    assert len(msg.ranges) == 360
    assert node._parser.crc_errors == 0


def test_read_failure_oserror_and_close_error(ros_fakes: FakeRos) -> None:
    node = make_node(ros_fakes)
    ros_fakes.serial.read_error = OSError("io")
    port = ros_fakes.serial.instances[0]

    def bad_close() -> None:
        raise OSError("close failed")

    port.close = bad_close
    pump(ros_fakes)
    assert node._serial is None
    assert any("error closing" in m for _, m in ros_fakes.logs("warning"))


def test_no_data_warning_once(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes)
    pump(ros_fakes, 3 * RECONNECT_S - 0.5)
    assert not ros_fakes.logs("warning")
    pump(ros_fakes, 1.0)
    assert len(ros_fakes.logs("warning")) == 1
    pump(ros_fakes, 1.0)
    pump(ros_fakes, 1.0)
    assert len(ros_fakes.logs("warning")) == 1


def test_no_data_warning_rearms_after_data(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes)
    pump(ros_fakes, 7.0)
    assert len(ros_fakes.logs("warning")) == 1
    ros_fakes.serial.data += b"\x00"
    pump(ros_fakes)
    pump(ros_fakes, 7.0)
    assert len(ros_fakes.logs("warning")) == 2


def test_stats_log_reports_crc_errors(ros_fakes: FakeRos) -> None:
    make_node(ros_fakes)
    bad = bytearray(frame_bytes(0, 1000, 0))
    bad[-1] ^= 0xFF
    ros_fakes.serial.data += bytes(bad) + revolutions([30, 30])
    pump(ros_fakes)
    pump(ros_fakes, 10.0)
    stats = [m for _, m in ros_fakes.logs("info") if "stats" in m]
    assert len(stats) == 1
    assert "crc_errors=1" in stats[0]
    assert "dropped_revolutions=0" in stats[0]
    assert "scans=1" in stats[0]
    pump(ros_fakes, 10.0)
    stats = [m for _, m in ros_fakes.logs("info") if "stats" in m]
    assert "scans=0" in stats[-1]


def test_invalid_bins_raises(ros_fakes: FakeRos) -> None:
    with pytest.raises(ValueError, match="bins"):
        make_node(ros_fakes, bins=10)


def test_main_lifecycle(ros_fakes: FakeRos) -> None:
    module = importlib.import_module("hrobot_core.adapters.ld19_node")
    module.main([])
    (node,) = ros_fakes.nodes
    assert node.destroyed is True
    assert ros_fakes.serial.instances[0].is_open is False
    assert ros_fakes.initialized is False


def test_main_invalid_param_shuts_down(ros_fakes: FakeRos) -> None:
    ros_fakes.set_params({"bins": 10})
    module = importlib.import_module("hrobot_core.adapters.ld19_node")
    with pytest.raises(ValueError, match="bins"):
        module.main([])
    assert ros_fakes.initialized is False


def test_static_no_sleep_print_or_cmd_vel() -> None:
    tree = ast.parse(NODE_FILE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            assert not (isinstance(func, ast.Name) and func.id in {"print", "sleep"})
            assert not (isinstance(func, ast.Attribute) and func.attr == "sleep")
            if isinstance(func, ast.Attribute) and func.attr == "create_publisher":
                for arg in ast.walk(node):
                    if isinstance(arg, ast.Constant):
                        assert arg.value != CMD_VEL_TOPIC
                    assert not (isinstance(arg, ast.Name) and arg.id == "CMD_VEL_TOPIC")
        if isinstance(node, ast.ImportFrom | ast.Import):
            names = [a.name for a in node.names]
            assert "sleep" not in names
