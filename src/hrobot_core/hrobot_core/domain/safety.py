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
_RUN_TOLERANCE_RAD = 1e-9  # tolerancia de coma flotante en el límite del tramo


@dataclass(frozen=True)
class SafetyConfig:
    """Parámetros de la política de seguridad.

    Attributes:
        stop_distance: Distancia (m) por debajo de la cual el factor es 0.
        slow_distance: Distancia (m) a partir de la cual el factor es 1.
        front_half_angle: Semiángulo (rad) del cono frontal evaluado.
        stale_timeout: Edad máxima (s) admitida del scan.
        min_valid_ratio: Fracción mínima de rayos válidos (obstáculo o libre)
            sobre el total de rayos del cono, en ``(0, 1]``. El límite es
            inclusivo: justo en el umbral **no** se degrada. Valor inicial a
            calibrar con scans reales.
        degraded_factor: Tope de velocidad en ``(0, 1)`` cuando la fracción
            válida es positiva pero menor que ``min_valid_ratio``. Valor
            inicial a calibrar con scans reales.
        max_invalid_run_rad: Anchura (rad) de un tramo contiguo de rayos
            inválidos (``n_rayos × |angle_increment|``) a partir de la cual se
            degrada la velocidad, en ``(0, π]``. El límite es inclusivo (con
            tolerancia 1e-9). Estimación inicial de 8°, a calibrar con scans
            reales.

    Raises:
        ValueError: Si algún valor de distancia/ángulo/tiempo no es finito o
            <= 0, si ``stop_distance >= slow_distance``, si
            ``min_valid_ratio`` no es finito en ``(0, 1]`` o si
            ``degraded_factor`` no es finito en ``(0, 1)`` o si
            ``max_invalid_run_rad`` no es finito en ``(0, π]``.
    """

    stop_distance: float = 0.35
    slow_distance: float = 1.00
    front_half_angle: float = 0.6
    stale_timeout: float = 0.5
    min_valid_ratio: float = 0.70
    degraded_factor: float = 0.30
    max_invalid_run_rad: float = math.radians(8.0)

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
        ratio = self.min_valid_ratio
        if not math.isfinite(ratio) or not 0.0 < ratio <= 1.0:
            raise ValueError(f"min_valid_ratio must be in (0, 1], got {ratio!r}")
        degraded = self.degraded_factor
        if not math.isfinite(degraded) or not 0.0 < degraded < 1.0:
            raise ValueError(f"degraded_factor must be in (0, 1), got {degraded!r}")
        run = self.max_invalid_run_rad
        if not math.isfinite(run) or not 0.0 < run <= math.pi:
            raise ValueError(f"max_invalid_run_rad must be in (0, pi], got {run!r}")


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


def _has_long_invalid_run(
    cone: list[tuple[float, bool]], angle_increment: float, cfg: SafetyConfig
) -> bool:
    """Indica si algún tramo contiguo de rayos inválidos alcanza el umbral.

    ``cone`` contiene ``(ángulo normalizado, válido)``; se ordena por ángulo para
    que un hueco que cruza el índice 0 del arreglo sea un único tramo.
    """
    step = abs(angle_increment)
    if step == 0.0:
        return False
    run = 0
    for _, valid in sorted(cone, key=lambda item: item[0]):
        run = 0 if valid else run + 1
        if run * step >= cfg.max_invalid_run_rad - _RUN_TOLERANCE_RAD:
            return True
    return False


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

    Degradación por datos dudosos: la fracción válida es
    ``rayos_válidos / rayos_totales_en_el_cono`` (los inválidos cuentan en el
    total). Si ``fracción >= cfg.min_valid_ratio`` (límite inclusivo) no hay
    cambios; si ``0 < fracción < cfg.min_valid_ratio`` el resultado es
    ``min(factor, cfg.degraded_factor)``: solo se reduce la velocidad.

    Degradación por contigüidad: los rayos del cono se ordenan por ángulo
    normalizado (no por índice) y un tramo es una secuencia de rayos inválidos
    consecutivos; cualquier rayo válido (incluido ``+inf``) lo interrumpe. Si su
    anchura ``n × |angle_increment|`` es ``>= cfg.max_invalid_run_rad`` (con
    tolerancia 1e-9) se aplica el mismo tope ``cfg.degraded_factor`` (sin
    acumularse con la regla de la fracción). Con ``angle_increment == 0`` esta
    regla no se aplica. Solo cuentan los rayos dentro del cono.

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
    cone_rays = 0
    cone: list[tuple[float, bool]] = []
    for index, distance in enumerate(scan.ranges):
        raw_angle = scan.angle_min + index * scan.angle_increment
        if not math.isfinite(raw_angle):
            # Desbordamiento numérico: geometría inutilizable ⇒ fallo seguro.
            return STOP_FACTOR
        angle = normalize_angle(raw_angle)
        if abs(angle) > cfg.front_half_angle:
            continue
        cone_rays += 1
        is_valid = distance == math.inf or (
            math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
        )
        cone.append((angle, is_valid))
        if is_valid:
            valid_rays += 1
            nearest = min(nearest, distance)
    if valid_rays == 0:
        return STOP_FACTOR
    factor = FREE_FACTOR if nearest == math.inf else _linear_factor(nearest, cfg)
    if valid_rays / cone_rays < cfg.min_valid_ratio or _has_long_invalid_run(
        cone, scan.angle_increment, cfg
    ):
        return min(factor, cfg.degraded_factor)
    return factor
