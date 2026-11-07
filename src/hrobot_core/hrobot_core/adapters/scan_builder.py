"""Constructor de scans de rejilla fija a partir de paquetes del LD19 (sin ROS).

Convierte el flujo de ``LidarPacket`` en ``ScanFrame`` de longitud constante,
con semántica de rangos coherente con REP-117 y fallo seguro: un bin sin dato
es ``NaN`` (nunca se asume "libre").
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hrobot_core.adapters.ld19_parser import LidarPacket, LidarPoint

MIN_BINS = 90
MAX_BINS = 3600
TIMESTAMP_WRAP_MS = 30000
MS_PER_S = 1000.0
REVOLUTION_JUMP_RAD = math.pi
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class ScanBuilderConfig:
    """Parámetros del constructor de scans."""

    bins: int = 360
    range_min: float = 0.02
    range_max: float = 12.0  # alcance nominal del LD19: verificar con datasheet
    max_points_per_revolution: int = 2000

    def __post_init__(self) -> None:
        """Valida la configuración.

        Raises:
            ValueError: si algún parámetro es inválido.
        """
        if isinstance(self.bins, bool) or not isinstance(self.bins, int):
            raise ValueError("bins must be an integer")
        if not MIN_BINS <= self.bins <= MAX_BINS:
            raise ValueError(f"bins must be in [{MIN_BINS}, {MAX_BINS}]")
        if not (math.isfinite(self.range_min) and math.isfinite(self.range_max)):
            raise ValueError("range_min and range_max must be finite")
        if self.range_min <= 0.0:
            raise ValueError("range_min must be > 0")
        if self.range_max <= self.range_min:
            raise ValueError("range_max must be > range_min")
        if isinstance(self.max_points_per_revolution, bool) or self.max_points_per_revolution <= 0:
            raise ValueError("max_points_per_revolution must be > 0")


@dataclass(frozen=True)
class ScanFrame:
    """Scan de una vuelta con rejilla fija (ángulo creciente desde 0)."""

    ranges: tuple[float, ...]
    intensities: tuple[float, ...]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    scan_time_s: float
    valid_ratio: float


class ScanBuilder:
    """Acumula paquetes y emite un ``ScanFrame`` por vuelta completa."""

    def __init__(self, cfg: ScanBuilderConfig | None = None) -> None:
        self._cfg = cfg if cfg is not None else ScanBuilderConfig()
        self._increment = _TWO_PI / self._cfg.bins
        self.dropped_revolutions = 0
        self._points: list[LidarPoint] = []
        self._prev_angle: float | None = None
        self._synced = False
        self._first_ts = 0
        self._last_ts = 0

    def reset(self) -> None:
        """Descarta el estado acumulado (no reinicia ``dropped_revolutions``)."""
        self._points = []
        self._prev_angle = None
        self._synced = False
        self._first_ts = 0
        self._last_ts = 0

    def add_packet(self, packet: LidarPacket) -> list[ScanFrame]:
        """Procesa un paquete y devuelve los scans completados (normalmente 0 o 1)."""
        frames: list[ScanFrame] = []
        for point in packet.points:
            if not math.isfinite(point.angle_rad):
                continue  # ángulo inválido: no se puede ubicar ni detectar cierre
            prev = self._prev_angle
            self._prev_angle = point.angle_rad
            if prev is not None and point.angle_rad - prev > REVOLUTION_JUMP_RAD:
                if self._synced:
                    frames.append(self._build())
                self._points = []
                self._synced = True
                self._first_ts = packet.timestamp_ms
            if not self._points:
                self._first_ts = packet.timestamp_ms
            self._points.append(point)
            self._last_ts = packet.timestamp_ms
            if len(self._points) > self._cfg.max_points_per_revolution:
                self.dropped_revolutions += 1
                self._points = []
                self._synced = False  # lo que sigue sería una vuelta parcial
        return frames

    def _build(self) -> ScanFrame:
        cfg = self._cfg
        bins = cfg.bins
        best: list[float | None] = [None] * bins  # mínimo finito, +inf o sin dato
        inten = [0.0] * bins
        for p in self._points:
            contrib = self._contribution(p)
            if contrib is None:
                continue
            idx = round(p.angle_rad / self._increment) % bins
            cur = best[idx]
            if cur is None or contrib < cur:
                best[idx] = contrib
                inten[idx] = float(p.intensity) if math.isfinite(contrib) else 0.0
        ranges = tuple(math.nan if v is None else v for v in best)
        valid = sum(1 for v in best if v is not None)
        scan_time = ((self._last_ts - self._first_ts) % TIMESTAMP_WRAP_MS) / MS_PER_S
        return ScanFrame(
            ranges=ranges,
            intensities=tuple(inten),
            angle_min=0.0,
            angle_max=_TWO_PI - self._increment,
            angle_increment=self._increment,
            range_min=cfg.range_min,
            range_max=cfg.range_max,
            scan_time_s=scan_time,
            valid_ratio=valid / bins,
        )

    def _contribution(self, p: LidarPoint) -> float | None:
        """Aporte del punto a su bin: distancia, +inf o ``None`` (sin aporte)."""
        if not p.valid or not math.isfinite(p.distance_m):
            return None
        if p.distance_m < self._cfg.range_min:
            return None
        if p.distance_m > self._cfg.range_max:
            return math.inf
        return p.distance_m
