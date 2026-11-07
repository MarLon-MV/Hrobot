"""Política de seguridad por LiDAR (pura, sin ROS).

Convierte un scan 2D en un factor de velocidad en [0, 1] (0 = parar, 1 = libre).
Unidades: metros, radianes, segundos. Convención de rangos: REP-117.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from hrobot_core.domain.geometry import normalize_angle

STOP_FACTOR = 0.0
FREE_FACTOR = 1.0


@dataclass(frozen=True)
class SafetyConfig:
    """Parámetros de la política de seguridad.

    Attributes:
        stop_distance: Distancia (m) por debajo de la cual el factor es 0.
        slow_distance: Distancia (m) a partir de la cual el factor es 1.
        front_half_angle: Semiángulo (rad) del cono frontal evaluado.
        stale_timeout: Edad máxima (s) admitida del scan.

    Raises:
        ValueError: Si algún valor no es finito o <= 0, o si
            ``stop_distance >= slow_distance``.
    """

    stop_distance: float = 0.35
    slow_distance: float = 1.00
    front_half_angle: float = 0.6
    stale_timeout: float = 0.5

    def __post_init__(self) -> None:
        values = {
            "stop_distance": self.stop_distance,
            "slow_distance": self.slow_distance,
            "front_half_angle": self.front_half_angle,
            "stale_timeout": self.stale_timeout,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0, got {value!r}")
        if self.stop_distance >= self.slow_distance:
            raise ValueError("stop_distance must be < slow_distance")


@dataclass(frozen=True)
class ScanData:
    """Scan 2D en tipos propios (equivalente a LaserScan)."""

    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    age_s: float


def _is_fresh(scan: ScanData, cfg: SafetyConfig) -> bool:
    return math.isfinite(scan.age_s) and 0.0 <= scan.age_s <= cfg.stale_timeout


def _geometry_valid(scan: ScanData) -> bool:
    return (
        math.isfinite(scan.angle_min)
        and math.isfinite(scan.angle_increment)
        and math.isfinite(scan.range_min)
        and math.isfinite(scan.range_max)
        and scan.range_min <= scan.range_max
    )


def _linear_factor(distance: float, cfg: SafetyConfig) -> float:
    span = cfg.slow_distance - cfg.stop_distance
    return min(FREE_FACTOR, max(STOP_FACTOR, (distance - cfg.stop_distance) / span))


def speed_factor(scan: ScanData, cfg: SafetyConfig) -> float:
    """Calcula el factor de velocidad en [0.0, 1.0] a partir del scan.

    Solo se evalúan los rayos dentro de ``±cfg.front_half_angle`` (ángulo
    normalizado respecto al frente). Clasificación de cada rayo (REP-117):

    * Obstáculo: finito en ``[range_min, range_max]``; aporta su distancia.
    * Libre: ``+inf``; rayo válido que no limita la velocidad.
    * Inválido (ignorado): ``NaN``, ``-inf`` o finito fuera de
      ``[range_min, range_max]``. Un rayo finito ``< range_min`` cae en la zona
      ciega del sensor y se trata como inválido, no como obstáculo.

    La distancia efectiva es el mínimo de los obstáculos; entre
    ``stop_distance`` y ``slow_distance`` el factor crece linealmente de 0 a 1.
    Fallo seguro (0.0): scan caducado o con edad/geometría no finita, y cono
    frontal sin ningún rayo válido (scan vacío incluido). Solo rayos libres ⇒ 1.0.

    Args:
        scan: Datos del scan.
        cfg: Configuración de seguridad.

    Returns:
        Factor de velocidad en [0.0, 1.0].
    """
    if not _is_fresh(scan, cfg) or not _geometry_valid(scan):
        return STOP_FACTOR
    nearest = math.inf
    valid_rays = 0
    for index, distance in enumerate(scan.ranges):
        raw_angle = scan.angle_min + index * scan.angle_increment
        if not math.isfinite(raw_angle):
            # Desbordamiento numérico: geometría inutilizable ⇒ fallo seguro.
            return STOP_FACTOR
        angle = normalize_angle(raw_angle)
        if abs(angle) > cfg.front_half_angle:
            continue
        if distance == math.inf:
            valid_rays += 1
        elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            valid_rays += 1
            nearest = min(nearest, distance)
    if valid_rays == 0:
        return STOP_FACTOR
    if nearest == math.inf:
        return FREE_FACTOR
    return _linear_factor(nearest, cfg)
