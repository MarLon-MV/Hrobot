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
    assert speed_factor(make_scan([NAN, INF, NAN]), CFG) == 1.0


def test_nan_and_obstacle_uses_obstacle() -> None:
    mid = (CFG.stop_distance + CFG.slow_distance) / 2
    assert speed_factor(make_scan([NAN, mid, NAN]), CFG) == pytest.approx(0.5)


def test_neginf_only_is_zero() -> None:
    assert speed_factor(make_scan([-INF, -INF, -INF]), CFG) == 0.0


def test_finite_below_range_min_ignored_as_invalid() -> None:
    assert speed_factor(make_scan([0.01, 0.01, 0.01]), CFG) == 0.0
    assert speed_factor(make_scan([0.01, INF, 0.01]), CFG) == 1.0


def test_finite_above_range_max_ignored() -> None:
    assert speed_factor(make_scan([20.0, 20.0, 20.0]), CFG) == 0.0
    assert speed_factor(make_scan([20.0, 0.675, 20.0]), CFG) == pytest.approx(0.5)


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
    ],
)
def test_invalid_config_raises(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)
