import math
import struct
from typing import Any

import pytest
from hrobot_core.adapters.ld19_parser import Ld19Parser, LidarPacket, LidarPoint, crc8
from hrobot_core.adapters.scan_builder import ScanBuilder, ScanBuilderConfig, ScanFrame

TWO_PI = 2 * math.pi
INC = TWO_PI / 360


def pt(angle: float, dist: float = 3.0, valid: bool = True, inten: int = 50) -> LidarPoint:
    return LidarPoint(angle_rad=angle, distance_m=dist, intensity=inten, valid=valid)


def pkt(points: list[LidarPoint], ts: int = 0) -> LidarPacket:
    return LidarPacket(speed_dps=3600, timestamp_ms=ts, points=tuple(points))


def revolution(n: int, dist: float = 3.0) -> list[LidarPoint]:
    """Una vuelta horaria en ROS: ángulos decrecientes desde ~2π hacia ~0."""
    return [pt(TWO_PI * (1 - (i + 0.5) / n), dist) for i in range(n)]


def feed_points(sb: ScanBuilder, pts: list[LidarPoint], per: int = 12) -> list[ScanFrame]:
    frames: list[ScanFrame] = []
    for k in range(0, len(pts), per):
        frames += sb.add_packet(pkt(pts[k : k + per], 2 * (k // per)))
    return frames


@pytest.mark.parametrize("n", [440, 450, 462])
def test_fixed_length(n: int) -> None:
    frames = feed_points(ScanBuilder(), revolution(n) * 5)
    assert len(frames) == 3
    for f in frames:
        assert len(f.ranges) == 360
        assert len(f.intensities) == 360


def test_geometry_fields() -> None:
    sb = ScanBuilder(ScanBuilderConfig(bins=720))
    f = feed_points(sb, revolution(450) * 3)[0]
    inc = TWO_PI / 720
    assert len(f.ranges) == 720
    assert f.angle_min == 0.0
    assert f.angle_increment == pytest.approx(inc)
    assert f.angle_max == pytest.approx(TWO_PI - inc)
    assert f.range_min == 0.02
    assert f.range_max == 12.0


def test_semantics() -> None:
    rev = [
        pt(105 * INC, 1.0, valid=False),  # inválido -> NaN
        pt(104 * INC, 0.01),  # < range_min -> NaN
        pt(103 * INC, 20.0),  # > range_max -> +inf
        pt(102 * INC, 2.0, inten=10),
        pt(102 * INC, 1.5, inten=77),  # mismo bin -> mínimo
        pt(101 * INC, math.inf),  # no finito -> NaN
        pt(100 * INC, 5.0),
    ]
    sb = ScanBuilder()
    frames = feed_points(sb, [pt(0.1), pt(6.0), *rev, pt(6.0), pt(6.0)])
    assert len(frames) == 1
    f = frames[0]
    assert math.isnan(f.ranges[105])
    assert math.isnan(f.ranges[104])
    assert f.ranges[103] == math.inf
    assert f.ranges[102] == 1.5
    assert f.intensities[102] == 77.0
    assert f.intensities[103] == 0.0
    assert math.isnan(f.ranges[101])
    assert f.ranges[100] == 5.0
    assert math.isnan(f.ranges[7])  # bin vacío
    assert f.intensities[7] == 0.0
    assert f.valid_ratio == pytest.approx(4 / 360)  # incluye el pt(6.0) inicial


def test_finite_beats_inf_in_same_bin() -> None:
    rev = [pt(50 * INC, 30.0), pt(50 * INC, 4.0), pt(50 * INC, 40.0)]
    f = feed_points(ScanBuilder(), [pt(1.0), pt(6.0), *rev, pt(6.0), pt(6.0)])[0]
    assert f.ranges[50] == 4.0


def test_nan_angle_skipped() -> None:
    pts = revolution(100) * 3
    pts.insert(10, pt(math.nan))
    assert len(feed_points(ScanBuilder(), pts)) == 1


def test_partial_initial_not_emitted_and_one_per_closure() -> None:
    sb = ScanBuilder()
    assert feed_points(sb, revolution(450)[200:]) == []  # sin cierre aún
    # Tres vueltas completas -> cierres: partial|v1 (no emite), v1|v2, v2|v3, v3|v4
    frames = feed_points(sb, revolution(450) * 3 + [pt(6.0)])
    assert len(frames) == 3


def test_closure_inside_packet_splits_points() -> None:
    sb = ScanBuilder()
    sb.add_packet(pkt([pt(0.2), pt(0.1)], ts=10))
    assert sb.add_packet(pkt([pt(6.2), pt(100 * INC, 2.0)], ts=20)) == []  # 1er cierre
    out = sb.add_packet(pkt([pt(50 * INC, 1.0), pt(6.2)], ts=30))  # 2º cierre en medio
    assert len(out) == 1
    f = out[0]
    assert f.ranges[100] == 2.0
    assert f.ranges[50] == 1.0
    # 3 puntos de la vuelta: 6.2 (bin ~355), 100*INC, 50*INC; el 6.2 final es de la siguiente
    assert f.valid_ratio == pytest.approx(3 / 360)
    assert f.scan_time_s == pytest.approx(0.010)


def test_closure_at_packet_boundary() -> None:
    sb = ScanBuilder()
    sb.add_packet(pkt([pt(0.2)], ts=1))
    sb.add_packet(pkt([pt(6.2)], ts=5))
    out = sb.add_packet(pkt([pt(0.2)], ts=9))
    out += sb.add_packet(pkt([pt(6.2)], ts=13))
    assert len(out) == 1
    assert out[0].scan_time_s == pytest.approx(0.004)  # 5 -> 9 ms


def test_scan_time_timestamp_wrap() -> None:
    sb = ScanBuilder()
    sb.add_packet(pkt([pt(0.2)], ts=29950))
    sb.add_packet(pkt([pt(6.2), pt(3.0)], ts=29980))
    sb.add_packet(pkt([pt(0.5)], ts=40))  # tras el reinicio del timestamp
    out = sb.add_packet(pkt([pt(6.2)], ts=60))
    assert len(out) == 1
    assert out[0].scan_time_s == pytest.approx(0.060)  # (40 - 29980) % 30000 = 60


def test_max_points_drops_revolution() -> None:
    sb = ScanBuilder(ScanBuilderConfig(max_points_per_revolution=50))
    pts = [pt(5.0 - 0.001 * i) for i in range(110)]  # sin cierre
    assert feed_points(sb, pts) == []
    assert sb.dropped_revolutions == 2
    # Tras el descarte exige sincronizar con un cierre antes de emitir.
    frames = feed_points(sb, revolution(40) * 4)
    assert len(frames) == 2
    assert sb.dropped_revolutions == 2


def test_reset() -> None:
    sb = ScanBuilder()
    assert len(feed_points(sb, revolution(100) * 4)) == 2
    sb.reset()
    # Tras reset no hay vuelta sincronizada: el primer cierre no emite.
    assert sb.add_packet(pkt([pt(0.1), pt(6.0)])) == []


@pytest.mark.parametrize(
    "kwargs",
    [
        {"bins": 89},
        {"bins": 3601},
        {"bins": 360.0},
        {"bins": True},
        {"range_min": 0.0},
        {"range_min": -1.0},
        {"range_min": math.nan},
        {"range_max": math.inf},
        {"range_max": math.nan},
        {"range_min": 1.0, "range_max": 1.0},
        {"range_min": 2.0, "range_max": 1.0},
        {"max_points_per_revolution": 0},
        {"max_points_per_revolution": -5},
        {"max_points_per_revolution": True},
    ],
)
def test_invalid_config(kwargs: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        ScanBuilderConfig(**kwargs)


def test_valid_config_bounds() -> None:
    assert ScanBuilderConfig(bins=90).bins == 90
    assert ScanBuilderConfig(bins=3600).bins == 3600


def build_raw(start_cd: int, end_cd: int, dists: list[int], ts: int) -> bytes:
    body = bytearray([0x54, 0x2C]) + struct.pack("<HH", 3600, start_cd)
    for d in dists:
        body += struct.pack("<HB", d, 100)
    body += struct.pack("<HH", end_cd, ts)
    return bytes(body) + bytes([crc8(bytes(body))])


def test_end_to_end_anti_mirror() -> None:
    """Obstáculo a 1 m en LD19 90° (derecha) => bin 270 en convención ROS."""
    n_frames = 30  # 30 tramas x 12 puntos = 360 puntos (1 por grado) por vuelta
    raw = bytearray()
    for rev in range(3):
        for k in range(n_frames):
            start = k * 1200
            dists = []
            for i in range(12):
                deg = (start + i * 100) / 100.0
                dists.append(1000 if deg == 90.0 else 3000)
            raw += build_raw(start, (start + 1100) % 36000, dists, (rev * n_frames + k) * 3)
    packets = Ld19Parser().feed(bytes(raw))
    assert len(packets) == 90
    sb = ScanBuilder()
    frames: list[ScanFrame] = []
    for p in packets:
        frames += sb.add_packet(p)
    assert frames
    f = frames[0]
    near = [i for i, r in enumerate(f.ranges) if r < 2.0]
    assert near
    assert any(abs(i - 270) <= 1 for i in near)
    assert f.ranges[90] == pytest.approx(3.0)
