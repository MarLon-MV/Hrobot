"""Parser de tramas binarias del LiDAR LD19 (sin hardware ni ROS).

Protocolo: LD19 Development Manual v2.3. Trama de 47 bytes, little-endian.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

HEADER = 0x54
VER_LEN = 0x2C
FRAME_SIZE = 47
POINTS_PER_FRAME = 12
CRC_POLY = 0x4D
MAX_BUFFER_BYTES = FRAME_SIZE - 1
MAX_CHUNK_BYTES = 4096
ANGLE_UNITS_PER_REV = 36000
MM_PER_M = 1000.0
_POINTS_OFFSET = 6
_POINT_SIZE = 3
_TWO_PI = 2.0 * math.pi


def _build_crc_table(poly: int) -> tuple[int, ...]:
    """Genera la tabla CRC-8 (MSB primero, sin reflexión)."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


CRC_TABLE = _build_crc_table(CRC_POLY)


def crc8(data: bytes) -> int:
    """Calcula el CRC-8 del LD19 (init 0x00, sin XOR final)."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


@dataclass(frozen=True)
class LidarPoint:
    """Punto del LiDAR en convención ROS (antihorario)."""

    angle_rad: float
    distance_m: float
    intensity: int
    valid: bool


@dataclass(frozen=True)
class LidarPacket:
    """Trama decodificada con 12 puntos."""

    speed_dps: int
    timestamp_ms: int
    points: tuple[LidarPoint, ...]


def ld19_to_ros_angle(angle_centideg: float) -> float:
    """Convierte ángulo LD19 (horario, centésimas de grado) a ROS [0, 2π)."""
    angle_ld = math.radians(angle_centideg / 100.0)
    result = (_TWO_PI - angle_ld) % _TWO_PI
    # Evita que el redondeo float devuelva exactamente 2π.
    return 0.0 if result >= _TWO_PI else result


def _decode_frame(frame: bytes) -> LidarPacket:
    speed, start = struct.unpack_from("<HH", frame, 2)
    end, timestamp = struct.unpack_from("<HH", frame, 42)
    if end < start:
        end += ANGLE_UNITS_PER_REV
    step = (end - start) / (POINTS_PER_FRAME - 1)
    points = []
    for i in range(POINTS_PER_FRAME):
        dist_mm, intensity = struct.unpack_from("<HB", frame, _POINTS_OFFSET + i * _POINT_SIZE)
        valid = dist_mm != 0
        points.append(
            LidarPoint(
                angle_rad=ld19_to_ros_angle((start + step * i) % ANGLE_UNITS_PER_REV),
                distance_m=dist_mm / MM_PER_M if valid else 0.0,
                intensity=intensity,
                valid=valid,
            )
        )
    return LidarPacket(speed_dps=speed, timestamp_ms=timestamp, points=tuple(points))


class Ld19Parser:
    """Parser incremental de flujo de bytes del LD19."""

    def __init__(self) -> None:
        self.crc_errors = 0
        self._buf = bytearray()

    @property
    def buffered_bytes(self) -> int:
        """Bytes retenidos entre llamadas a ``feed``."""
        return len(self._buf)

    def feed(self, data: bytes) -> list[LidarPacket]:
        """Alimenta bytes arbitrarios y devuelve las tramas válidas completadas."""
        packets: list[LidarPacket] = []
        for pos in range(0, len(data), MAX_CHUNK_BYTES):
            self._buf.extend(data[pos : pos + MAX_CHUNK_BYTES])
            self._drain(packets)
        return packets

    def _drain(self, packets: list[LidarPacket]) -> None:
        buf = self._buf
        while True:
            idx = buf.find(bytes((HEADER, VER_LEN)))
            if idx < 0:
                # Conserva un posible 0x54 final por si el 0x2C llega después.
                keep = 1 if buf and buf[-1] == HEADER else 0
                del buf[: len(buf) - keep]
                return
            del buf[:idx]
            if len(buf) < FRAME_SIZE:
                return
            frame = bytes(buf[:FRAME_SIZE])
            if crc8(frame[:-1]) == frame[-1]:
                packets.append(_decode_frame(frame))
                del buf[:FRAME_SIZE]
            else:
                self.crc_errors += 1
                del buf[:1]
