import math
import struct
from pathlib import Path

import pytest
from hrobot_core.adapters import ld19_parser as m
from hrobot_core.adapters.ld19_parser import (
    CRC_TABLE,
    Ld19Parser,
    crc8,
    ld19_to_ros_angle,
)

OFFICIAL = bytes.fromhex(
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 "
    "E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50"
)
TWO_PI = 2 * math.pi


def build(start: int, end: int, dists: list[int] | None = None, speed: int = 3600) -> bytes:
    dists = dists if dists is not None else [100 + i for i in range(12)]
    body = bytearray([0x54, 0x2C]) + struct.pack("<HH", speed, start)
    for d in dists:
        body += struct.pack("<HB", d, 50)
    body += struct.pack("<HH", end, 1234)
    return bytes(body) + bytes([crc8(bytes(body))])


def test_official_vector() -> None:
    assert len(OFFICIAL) == 47
    (pkt,) = Ld19Parser().feed(OFFICIAL)
    assert pkt.speed_dps == 2152
    assert pkt.timestamp_ms == 6714
    assert [round(p.distance_m * 1000) for p in pkt.points] == [
        224, 220, 217, 213, 211, 208, 205, 202, 199, 197, 194, 192,
    ]  # fmt: skip
    assert [p.intensity for p in pkt.points] == [
        228, 226, 229, 227, 228, 233, 228, 226, 233, 229, 229, 229,
    ]  # fmt: skip
    assert len(pkt.points) == 12
    assert all(p.valid for p in pkt.points)
    assert pkt.points[0].angle_rad == pytest.approx(0.6236, abs=1e-3)
    assert pkt.points[-1].angle_rad == pytest.approx(0.4416, abs=1e-3)


def test_crc_table() -> None:
    assert len(CRC_TABLE) == 256
    assert bytes(CRC_TABLE[:8]) == bytes.fromhex("004D9AD77934E3AE")
    assert bytes(CRC_TABLE[-9:]) == bytes.fromhex("5A064B9CD17F32E5A8")
    assert crc8(OFFICIAL[:-1]) == 0x50


def test_angle_conversion() -> None:
    assert ld19_to_ros_angle(9000) == pytest.approx(3 * math.pi / 2)
    assert ld19_to_ros_angle(0) == 0.0
    for c in (0, 1, 17999, 18000, 35999, 36000, 36001):
        assert 0.0 <= ld19_to_ros_angle(c) < TWO_PI
    assert ld19_to_ros_angle(0.0001) < TWO_PI


def test_wraparound_360() -> None:
    (pkt,) = Ld19Parser().feed(build(35000, 500))
    ld = [(2 * math.pi - p.angle_rad) % TWO_PI for p in pkt.points]
    unwrapped = [a if a > math.pi else a + TWO_PI for a in ld]
    diffs = [b - a for a, b in zip(unwrapped, unwrapped[1:], strict=False)]
    expected = math.radians(1500 / 11 / 100)
    assert all(d == pytest.approx(expected, abs=1e-6) for d in diffs)


def test_split_two_and_three() -> None:
    p = Ld19Parser()
    assert p.feed(OFFICIAL[:20]) == []
    assert len(p.feed(OFFICIAL[20:])) == 1
    p = Ld19Parser()
    assert p.feed(OFFICIAL[:10]) == []
    assert p.feed(OFFICIAL[10:30]) == []
    assert len(p.feed(OFFICIAL[30:])) == 1


def test_byte_by_byte() -> None:
    p = Ld19Parser()
    out = []
    for b in OFFICIAL:
        out += p.feed(bytes([b]))
    assert len(out) == 1
    assert p.buffered_bytes == 0


def test_two_frames_and_garbage() -> None:
    other = build(0, 1100)
    p = Ld19Parser()
    out = p.feed(b"\x01\x02" + OFFICIAL + b"\xff\x00\x54" + other)
    assert len(out) == 2
    assert out[1].speed_dps == 3600


def test_false_header() -> None:
    out = Ld19Parser().feed(b"\x54\x99\x54" + OFFICIAL)
    assert len(out) == 1


def test_split_header_across_feeds() -> None:
    p = Ld19Parser()
    assert p.feed(b"\x00\x54") == []
    assert p.buffered_bytes == 1
    assert len(p.feed(OFFICIAL[1:])) == 1


def test_corrupt_crc() -> None:
    bad = bytearray(OFFICIAL)
    bad[10] ^= 0xFF
    p = Ld19Parser()
    assert p.feed(bytes(bad)) == []
    assert p.crc_errors == 1


def test_valid_frame_inside_corrupt_frame() -> None:
    bad = bytearray(OFFICIAL)
    bad[20] ^= 0xFF
    p = Ld19Parser()
    out = p.feed(bytes(bad[:10]) + OFFICIAL)
    assert len(out) == 1
    assert p.crc_errors == 1


def test_invalid_points() -> None:
    dists = [100] * 12
    dists[3] = 0
    (pkt,) = Ld19Parser().feed(build(1000, 2000, dists))
    assert len(pkt.points) == 12
    assert not pkt.points[3].valid
    assert pkt.points[3].distance_m == 0.0
    assert all(p.valid for i, p in enumerate(pkt.points) if i != 3)


def test_memory_bounded() -> None:
    p = Ld19Parser()
    assert p.feed(b"\xaa" * 1_000_000) == []
    assert p.buffered_bytes <= 46
    p.feed(b"\x54" * 1_000_000)
    assert p.buffered_bytes <= 46
    p.feed(b"\x54\x2c" * 500_000)
    assert p.buffered_bytes <= 46
    assert len(p.feed(OFFICIAL)) == 1


def test_no_forbidden_imports() -> None:
    src = Path(m.__file__).read_text(encoding="utf-8")
    assert "import rclpy" not in src
    assert "import serial" not in src
