"""Tests de la política de seguridad por LiDAR."""

from __future__ import annotations

import math
import random

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
    # TASK-017: 29 y 30 NaN contiguos (0.29/0.30 rad > 8°) ahora degradan por
    # contigüidad; el límite de la fracción se conserva en
    # test_valid_ratio_thresholds_scattered con NaN dispersos.
    [(29, 0.30), (30, 0.30), (31, 0.30), (99, 0.30), (100, 0.0)],
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


# --- TASK-017: tramos contiguos de rayos inválidos ---------------------------

N450 = 450
INC450 = 2 * math.pi / N450
FRONT = N450 // 2  # índice del rayo a 0 rad
OBSTACLE_05_FACTOR = (0.5 - CFG.stop_distance) / (CFG.slow_distance - CFG.stop_distance)
INC360 = 2 * math.pi / 360


def scan450(invalid: dict[int, float] | None = None) -> ScanData:
    """450 rayos desde -π, todo +inf salvo los índices dados."""
    ranges = [INF] * N450
    for index, value in (invalid or {}).items():
        ranges[index] = value
    return make_scan(ranges, angle_min=-math.pi, inc=INC450)


def front_run(n: int, value: float = NAN) -> dict[int, float]:
    start = FRONT - n // 2
    return dict.fromkeys(range(start, start + n), value)


def scan360(invalid_idx: list[int]) -> ScanData:
    ranges = [INF] * 360
    for i in invalid_idx:
        ranges[i] = NAN
    return make_scan(ranges, angle_min=0.0, inc=INC360)


def test_run_of_10_nan_degrades() -> None:
    assert speed_factor(scan450(front_run(10)), CFG) == pytest.approx(0.30)


def test_run_of_9_nan_is_free() -> None:
    assert speed_factor(scan450(front_run(9)), CFG) == 1.0


def test_nan_band_ten_degrees_degrades() -> None:
    limit = math.radians(10)
    band = {i: NAN for i in range(N450) if abs(-math.pi + i * INC450) <= limit}
    assert 24 <= len(band) <= 26
    assert speed_factor(scan450(band), CFG) == pytest.approx(0.30)


def test_run_plus_close_obstacle_takes_minimum() -> None:
    invalid = {**front_run(10), FRONT + 20: 0.5}
    assert speed_factor(scan450(invalid), CFG) == pytest.approx(OBSTACLE_05_FACTOR)


def test_split_cone_single_run_across_index_zero() -> None:
    scan = scan360([*range(355, 360), *range(5)])
    assert speed_factor(scan, CFG) == pytest.approx(0.30)


def test_split_cone_two_separate_runs_not_summed() -> None:
    scan = scan360([*range(355, 360), *range(5, 10)])
    assert speed_factor(scan, CFG) == 1.0


def test_scattered_failures_do_not_degrade() -> None:
    invalid = dict.fromkeys(range(0, N450, 10), NAN)
    assert speed_factor(scan450(invalid), CFG) == 1.0


def test_two_runs_of_five_separated_by_valid_rays_are_free() -> None:
    idx = [*range(FRONT - 12, FRONT - 7), *range(FRONT + 5, FRONT + 10)]
    assert speed_factor(scan450(dict.fromkeys(idx, NAN)), CFG) == 1.0


@pytest.mark.parametrize("bad", [-INF, NAN, 0.01, 20.0])  # range 0.05..12.0
def test_run_formed_by_any_invalid_class(bad: float) -> None:
    assert speed_factor(scan450(front_run(10, bad)), CFG) == pytest.approx(0.30)


def test_posinf_in_the_middle_splits_run() -> None:
    invalid = front_run(12)
    del invalid[FRONT]  # +inf en medio: dos tramos < 8°
    assert speed_factor(scan450(invalid), CFG) == 1.0


def test_ratio_rule_still_applies_with_alternating_nan() -> None:
    invalid = dict.fromkeys(range(0, N450, 2), NAN)
    assert speed_factor(scan450(invalid), CFG) == pytest.approx(0.30)


@pytest.mark.parametrize(("n_nan", "expected"), [(29, 1.0), (30, 1.0), (31, 0.30)])
def test_valid_ratio_thresholds_scattered(n_nan: int, expected: float) -> None:
    # Grupos de 3 NaN (0.03 rad) separados cada 4 rayos: sin tramos largos.
    nan_idx = {i for i in range(100) if i % 4 != 3}
    nan_idx = set(sorted(nan_idx)[:n_nan])
    ranges = [NAN if i in nan_idx else INF for i in range(100)]
    scan = make_scan(ranges, angle_min=-0.5, inc=0.01)
    assert speed_factor(scan, CFG) == pytest.approx(expected)


@pytest.mark.parametrize("value", [0.0, -0.1, NAN, INF, 3.2])
def test_invalid_max_invalid_run_raises(value: float) -> None:
    with pytest.raises(ValueError):
        SafetyConfig(max_invalid_run_rad=value)


def test_max_invalid_run_pi_is_valid() -> None:
    assert SafetyConfig(max_invalid_run_rad=math.pi).max_invalid_run_rad == math.pi


@pytest.mark.parametrize("inc", [0.0, -0.1])
def test_zero_and_negative_increment_other_rules_hold(inc: float) -> None:
    near = make_scan([0.1, NAN, NAN], angle_min=0.0, inc=inc)
    assert speed_factor(near, CFG) == 0.0
    blind = make_scan([NAN, NAN, NAN], angle_min=0.0, inc=inc)
    assert speed_factor(blind, CFG) == 0.0


def test_zero_increment_ratio_rule_still_degrades() -> None:
    scan = make_scan([INF] + [NAN] * 9, angle_min=0.0, inc=0.0)
    assert speed_factor(scan, CFG) == pytest.approx(0.30)


def test_zero_increment_run_rule_not_applied() -> None:
    # 1 de 4 inválidos (fracción 0.75 ≥ 0.70) y sin anchura medible ⇒ libre.
    scan = make_scan([INF, INF, INF, NAN], angle_min=0.0, inc=0.0)
    assert speed_factor(scan, CFG) == 1.0


def test_negative_increment_run_measured_by_abs_width() -> None:
    ranges = [INF] * N450
    for i in range(FRONT - 5, FRONT + 5):
        ranges[i] = NAN
    scan = make_scan(ranges, angle_min=math.pi, inc=-INC450)
    assert speed_factor(scan, CFG) == pytest.approx(0.30)


def test_property_replacing_free_ray_with_nan_never_increases_factor() -> None:
    rng = random.Random(1234)  # noqa: S311 - aleatoriedad de test con semilla fija
    checked = 0
    while checked < 600:
        n = rng.randint(20, 120)
        ranges = [
            INF if rng.random() < 0.6 else rng.choice([NAN, 0.2, 0.8, 2.0, 30.0, -INF])
            for _ in range(n)
        ]
        free = [i for i, r in enumerate(ranges) if r == INF]
        if not free:
            continue
        a_min = -rng.uniform(0.1, 3.0)
        inc = rng.uniform(0.005, 0.06)
        before = speed_factor(make_scan(ranges, angle_min=a_min, inc=inc), CFG)
        mutated = list(ranges)
        mutated[rng.choice(free)] = NAN
        after = speed_factor(make_scan(mutated, angle_min=a_min, inc=inc), CFG)
        assert after <= before + 1e-12
        checked += 1
