"""Geometría 2D pura (sin ROS): poses, ángulos y cuaterniones.

Unidades: metros y radianes (REP-103).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Pose2D:
    """Pose plana: posición (m) y orientación yaw (rad)."""

    x: float
    y: float
    yaw: float


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")


def _require_tolerance(name: str, value: float) -> None:
    _require_finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value!r}")


def _validate_pose(name: str, pose: Pose2D) -> None:
    _require_finite(name + ".x", pose.x)
    _require_finite(name + ".y", pose.y)
    _require_finite(name + ".yaw", pose.yaw)


def normalize_angle(a: float) -> float:
    """Normaliza un ángulo al intervalo (-π, π].

    Raises:
        ValueError: si `a` es NaN o infinito.
    """
    _require_finite("a", a)
    r = math.remainder(a, _TWO_PI)  # en [-π, π]
    return math.pi if r <= -math.pi else r


def angle_diff(target: float, current: float) -> float:
    """Diferencia angular más corta con signo (target - current) en (-π, π].

    Raises:
        ValueError: si algún argumento no es finito.
    """
    _require_finite("target", target)
    _require_finite("current", current)
    return normalize_angle(target - current)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convierte yaw (rotación en Z) a cuaternión (x, y, z, w).

    Raises:
        ValueError: si `yaw` no es finito.
    """
    _require_finite("yaw", yaw)
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extrae el yaw de un cuaternión (x, y, z, w), resultado en (-π, π].

    Raises:
        ValueError: si algún componente no es finito o la norma es cero.
    """
    for name, value in (("x", x), ("y", y), ("z", z), ("w", w)):
        _require_finite(name, value)
    if x * x + y * y + z * z + w * w == 0.0:
        raise ValueError("quaternion must have non-zero norm")
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return normalize_angle(yaw)


def distance(a: Pose2D, b: Pose2D) -> float:
    """Distancia euclídea (m) entre las posiciones de dos poses.

    Raises:
        ValueError: si alguna coordenada no es finita.
    """
    _validate_pose("a", a)
    _validate_pose("b", b)
    return math.hypot(a.x - b.x, a.y - b.y)


def reached(current: Pose2D, goal: Pose2D, xy_tol: float, yaw_tol: float) -> bool:
    """Indica si `current` está dentro de las tolerancias respecto a `goal`.

    Raises:
        ValueError: si hay valores no finitos o tolerancias negativas.
    """
    _require_tolerance("xy_tol", xy_tol)
    _require_tolerance("yaw_tol", yaw_tol)
    return distance(current, goal) <= xy_tol and abs(angle_diff(goal.yaw, current.yaw)) <= yaw_tol
