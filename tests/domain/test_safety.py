"""Tests de la política de seguridad por LiDAR."""

from __future__ import annotations

import math

import pytest
from hrobot_core.domain.safety import SafetyConfig, ScanData, speed_factor

CFG = SafetyConfig()
INF = math.inf
NAN = math.nan


def make_scan(
    ranges: list[float],
    *,
    angle_min: float = -0.1,
    inc: float = 0.1,
    range_min: float = 0.05,
    range_max: float = 12.0,
    age_s: float = 0.0,
) -> ScanData:
    return ScanData(ranges, angle_min, inc, range_min, range_max, age_s)


def test_at_stop_distance_is_zero() -> None:
    assert speed_factor(make_scan([0.35, 0.35, 0.35]), CFG) == 0.0


def test_below_stop_distance_is_zero() -> None:
    assert speed_factor(make_scan([0.1, 2.0, 2.0]), CFG) == 0.0


def test_at_slow_distance_is_one() -> None:
    assert speed_factor(make_scan([1.0, 1.0, 1.0]), CFG) == 1.0


def test_beyond_slow_distance_is_one() -> None:
    assert speed_factor(make_scan([5.0, 5.0, 5.0]), CFG) == 1.0


def test_midpoint_is_half() -> None:
    mid = (CFG.stop_distance + CFG.slow_distance) / 2
    assert speed_factor(make_scan([mid, 3.0, 3.0]), CFG) == pytest.approx(0.5)


def test_minimum_ray_wins() -> None:
    assert speed_factor(make_scan([3.0, 0.35, 3.0]), CFG) == 0.0


def test_obstacle_outside_cone_ignored() -> None:
    scan = make_scan([0.1, 3.0, 3.0], angle_min=-1.0, inc=0.5)  # -1.0, -0.5, 0.0
    assert speed_factor(scan, CFG) == 1.0


def test_cone_boundary_inclusive() -> None:
    scan = make_scan([0.1], angle_min=0.6, inc=0.1)
    assert speed_factor(scan, CFG) == 0.0


def test_wrapped_angles_are_normalized() -> None:
    scan = make_scan([0.1], angle_min=2 * math.pi, inc=0.1)  # frente tras normalizar
    assert speed_factor(scan, CFG) == 0.0


def test_empty_scan_is_zero() -> None:
    assert speed_factor(make_scan([]), CFG) == 0.0


def test_empty_cone_is_zero() -> None:
    scan = make_scan([3.0, 3.0], angle_min=2.0, inc=0.1)
    assert speed_factor(scan, CFG) == 0.0


def test_all_nan_is_zero() -> None:
    assert speed_factor(make_scan([NAN, NAN, NAN]), CFG) == 0.0


def test_all_posinf_is_one() -> None:
    assert speed_factor(make_scan([INF, INF, INF]), CFG) == 1.0


def test_nan_and_posinf_is_one() -> None:
    # TASK-010: con [NAN, INF, NAN] la fracción válida es 1/3 < 0.70 y se
    # degrada; se usan 3 rayos +inf y 1 NaN (0.75) para el caso sin degradación.
    assert speed_factor(make_scan([NAN, INF, INF, INF]), CFG) == 1.0


def test_nan_and_obstacle_uses_obstacle() -> None:
    # TASK-010: [NAN, mid, NAN] (fracción 1/3) ahora se degrada; se usa 1 NaN
    # de 4 rayos (0.75) para comprobar que el obstáculo manda sin degradación.
    mid = (CFG.stop_distance + CFG.slow_distance) / 2
    assert speed_factor(make_scan([NAN, mid, mid, mid]), CFG) == pytest.approx(0.5)


def test_neginf_only_is_zero() -> None:
    assert speed_factor(make_scan([-INF, -INF, -INF]), CFG) == 0.0


def test_finite_below_range_min_ignored_as_invalid() -> None:
    assert speed_factor(make_scan([0.01, 0.01, 0.01]), CFG) == 0.0
    # TASK-010: 1 de 3 rayos válidos (< 0.70) ⇒ degradado a 0.30 (antes 1.0).
    assert speed_factor(make_scan([0.01, INF, 0.01]), CFG) == pytest.approx(0.30)
    assert speed_factor(make_scan([0.01, INF, INF, INF]), CFG) == 1.0


def test_finite_above_range_max_ignored() -> None:
    assert speed_factor(make_scan([20.0, 20.0, 20.0]), CFG) == 0.0
    # TASK-010: 1 de 3 válidos (< 0.70) ⇒ min(0.5, 0.30); con 3 de 4 no degrada.
    assert speed_factor(make_scan([20.0, 0.675, 20.0]), CFG) == pytest.approx(0.30)
    assert speed_factor(make_scan([20.0, 0.675, 0.675, 0.675]), CFG) == pytest.approx(0.5)


def test_range_bounds_inclusive() -> None:
    assert speed_factor(make_scan([0.05, 3.0, 3.0]), CFG) == 0.0
    assert speed_factor(make_scan([12.0, 12.0, 12.0]), CFG) == 1.0


def test_stale_scan_is_zero() -> None:
    assert speed_factor(make_scan([5.0, 5.0, 5.0], age_s=0.51), CFG) == 0.0


def test_scan_at_timeout_is_fresh() -> None:
    assert speed_factor(make_scan([5.0, 5.0, 5.0], age_s=0.5), CFG) == 1.0


@pytest.mark.parametrize("age", [NAN, INF, -0.1])
def test_invalid_age_is_zero(age: float) -> None:
    assert speed_factor(make_scan([5.0, 5.0, 5.0], age_s=age), CFG) == 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"angle_min": NAN},
        {"inc": INF},
        {"range_min": NAN},
        {"range_max": INF},
        {"range_min": 5.0, "range_max": 1.0},
    ],
)
def test_invalid_geometry_is_zero(kwargs: dict[str, float]) -> None:
    assert speed_factor(make_scan([5.0, 5.0, 5.0], **kwargs), CFG) == 0.0


def test_angle_overflow_is_zero_and_does_not_raise() -> None:
    scan = make_scan([5.0, 5.0, 5.0, 5.0], angle_min=0.0, inc=1e308)
    assert speed_factor(scan, CFG) == 0.0


def test_zero_angle_increment_all_rays_at_same_angle() -> None:
    scan = make_scan([5.0, 0.2], angle_min=0.0, inc=0.0)
    assert speed_factor(scan, CFG) == 0.0


def test_zero_angle_increment_outside_cone_is_zero() -> None:
    scan = make_scan([5.0, 5.0], angle_min=math.pi / 2, inc=0.0)
    assert speed_factor(scan, CFG) == 0.0


def test_negative_angle_increment_is_supported() -> None:
    scan = make_scan([5.0, 0.2, 5.0], angle_min=0.1, inc=-0.1)
    assert speed_factor(scan, CFG) == 0.0


def test_negative_range_min_negative_distance_is_stop() -> None:
    scan = make_scan([-1.0, 5.0], range_min=-2.0)
    assert speed_factor(scan, CFG) == 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"stop_distance": 1.0, "slow_distance": 1.0},
        {"stop_distance": 1.5, "slow_distance": 1.0},
        {"stop_distance": 0.0},
        {"stop_distance": -0.1},
        {"slow_distance": -1.0},
        {"front_half_angle": 0.0},
        {"stale_timeout": 0.0},
        {"stale_timeout": NAN},
        {"slow_distance": INF},
        {"min_valid_ratio": 0.0},
        {"min_valid_ratio": -0.1},
        {"min_valid_ratio": 1.1},
        {"min_valid_ratio": NAN},
        {"min_valid_ratio": INF},
        {"degraded_factor": 0.0},
        {"degraded_factor": 1.0},
        {"degraded_factor": 1.5},
        {"degraded_factor": NAN},
    ],
)
def test_invalid_config_raises(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)


def test_boundary_config_values_are_valid() -> None:
    cfg = SafetyConfig(min_valid_ratio=1.0, degraded_factor=0.99)
    assert cfg.min_valid_ratio == 1.0
    assert cfg.degraded_factor == 0.99


def hundred(prefix: list[float], rest: list[float]) -> ScanData:
    """100 rayos dentro del cono: angle_min=-0.5, inc=0.01."""
    ranges = prefix + rest
    assert len(ranges) == 100
    return make_scan(ranges, angle_min=-0.5, inc=0.01)


@pytest.mark.parametrize(
    ("n_nan", "expected"),
    [(29, 1.0), (30, 1.0), (31, 0.30), (99, 0.30), (100, 0.0)],
)
def test_valid_ratio_thresholds(n_nan: int, expected: float) -> None:
    scan = hundred([NAN] * n_nan, [INF] * (100 - n_nan))
    assert speed_factor(scan, CFG) == pytest.approx(expected)


def test_zero_distances_below_range_min_degrade() -> None:
    scan = hundred([0.0] * 40, [INF] * 60)
    assert speed_factor(scan, CFG) == pytest.approx(0.30)


def test_degraded_with_close_obstacle_takes_minimum() -> None:
    # 0.5 m exige (0.5-0.35)/0.65 ≈ 0.2308 < 0.30: manda el obstáculo.
    scan = hundred([NAN] * 40, [0.5] + [INF] * 59)
    expected = (0.5 - CFG.stop_distance) / (CFG.slow_distance - CFG.stop_distance)
    assert speed_factor(scan, CFG) == pytest.approx(expected)


def test_degraded_with_far_obstacle_uses_cap() -> None:
    # 0.9 m exige ≈ 0.846 > 0.30: manda el tope degradado.
    scan = hundred([NAN] * 40, [0.9] + [INF] * 59)
    assert speed_factor(scan, CFG) == pytest.approx(0.30)


def test_degraded_with_obstacle_in_stop_zone_is_zero() -> None:
    scan = hundred([NAN] * 40, [0.35] + [INF] * 59)
    assert speed_factor(scan, CFG) == 0.0


def test_few_nan_with_obstacle_is_not_degraded() -> None:
    scan = hundred([NAN] * 10, [0.675] + [INF] * 89)
    assert speed_factor(scan, CFG) == pytest.approx(0.5)


def test_realistic_450_ray_scan_with_nan_band_is_degraded() -> None:
    inc = 2 * math.pi / 450
    ranges = [NAN if abs(-math.pi + i * inc) <= 0.2 else INF for i in range(450)]
    scan = make_scan(ranges, angle_min=-math.pi, inc=inc)
    assert speed_factor(scan, CFG) == pytest.approx(0.30)
