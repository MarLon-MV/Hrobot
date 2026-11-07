"""Tests de hrobot_core.domain.geometry."""

import math

import pytest
from hrobot_core.domain.geometry import (
    Pose2D,
    angle_diff,
    distance,
    normalize_angle,
    quaternion_to_yaw,
    reached,
    yaw_to_quaternion,
)

BAD = [math.nan, math.inf, -math.inf]


def test_normalize_boundaries() -> None:
    assert normalize_angle(math.pi) == math.pi
    assert normalize_angle(-math.pi) == math.pi
    assert normalize_angle(0.0) == 0.0


def test_normalize_wraps() -> None:
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(2 * math.pi + 0.5) == pytest.approx(0.5)
    assert normalize_angle(-2 * math.pi - 0.5) == pytest.approx(-0.5)
    assert -math.pi < normalize_angle(-3 * math.pi) <= math.pi


@pytest.mark.parametrize("bad", BAD)
def test_normalize_rejects_non_finite(bad: float) -> None:
    with pytest.raises(ValueError):
        normalize_angle(bad)


def test_angle_diff_shortest_path() -> None:
    assert angle_diff(-3.1, 3.1) == pytest.approx(2 * math.pi - 6.2)
    assert angle_diff(-3.1, 3.1) == pytest.approx(0.083, abs=1e-3)
    assert angle_diff(3.1, -3.1) == pytest.approx(-0.083, abs=1e-3)
    assert angle_diff(1.0, 0.5) == pytest.approx(0.5)


@pytest.mark.parametrize("bad", BAD)
def test_angle_diff_rejects_non_finite(bad: float) -> None:
    with pytest.raises(ValueError):
        angle_diff(bad, 0.0)
    with pytest.raises(ValueError):
        angle_diff(0.0, bad)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -0.5, math.pi / 2, -math.pi / 2, 3.0, -3.0])
def test_yaw_quaternion_roundtrip(yaw: float) -> None:
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    assert (qx, qy) == (0.0, 0.0)
    assert qx**2 + qy**2 + qz**2 + qw**2 == pytest.approx(1.0)
    assert quaternion_to_yaw(qx, qy, qz, qw) == pytest.approx(yaw)


def test_yaw_quaternion_roundtrip_pi() -> None:
    assert quaternion_to_yaw(*yaw_to_quaternion(math.pi)) == pytest.approx(math.pi)


def test_quaternion_known_value() -> None:
    _, _, z, w = yaw_to_quaternion(math.pi / 2)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize("bad", BAD)
def test_quaternion_rejects_non_finite(bad: float) -> None:
    with pytest.raises(ValueError):
        yaw_to_quaternion(bad)
    for args in (
        (bad, 0.0, 0.0, 1.0),
        (0.0, bad, 0.0, 1.0),
        (0.0, 0.0, bad, 1.0),
        (0.0, 0.0, 0.0, bad),
    ):
        with pytest.raises(ValueError):
            quaternion_to_yaw(*args)


def test_quaternion_zero_norm_rejected() -> None:
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)


def test_distance() -> None:
    assert distance(Pose2D(0, 0, 0), Pose2D(3, 4, 1)) == pytest.approx(5.0)
    assert distance(Pose2D(1, 1, 0), Pose2D(1, 1, 0)) == 0.0


@pytest.mark.parametrize("bad", BAD)
def test_distance_rejects_non_finite(bad: float) -> None:
    with pytest.raises(ValueError):
        distance(Pose2D(bad, 0, 0), Pose2D(0, 0, 0))
    with pytest.raises(ValueError):
        distance(Pose2D(0, 0, 0), Pose2D(0, bad, 0))


def test_reached_true_and_boundary() -> None:
    goal = Pose2D(1.0, 1.0, 0.0)
    assert reached(Pose2D(1.0, 1.0, 0.0), goal, 0.1, 0.1)
    assert reached(Pose2D(1.1, 1.0, 0.0), goal, 0.1 + 1e-12, 0.1)
    assert reached(Pose2D(1.0, 1.0, 0.1), goal, 0.1, 0.1)


def test_reached_false_by_position_or_yaw() -> None:
    goal = Pose2D(0.0, 0.0, 0.0)
    assert not reached(Pose2D(0.5, 0.0, 0.0), goal, 0.1, 1.0)
    assert not reached(Pose2D(0.0, 0.0, 0.5), goal, 1.0, 0.1)


def test_reached_yaw_wraparound() -> None:
    goal = Pose2D(0.0, 0.0, 3.1)
    assert reached(Pose2D(0.0, 0.0, -3.1), goal, 0.1, 0.1)


def test_reached_zero_tolerance_ok() -> None:
    p = Pose2D(1.0, 2.0, 0.3)
    assert reached(p, p, 0.0, 0.0)


@pytest.mark.parametrize("bad", [-0.1, *BAD])
def test_reached_rejects_bad_tolerances(bad: float) -> None:
    p = Pose2D(0, 0, 0)
    with pytest.raises(ValueError):
        reached(p, p, bad, 0.1)
    with pytest.raises(ValueError):
        reached(p, p, 0.1, bad)


def test_reached_rejects_non_finite_pose() -> None:
    with pytest.raises(ValueError):
        reached(Pose2D(0, 0, math.nan), Pose2D(0, 0, 0), 0.1, 0.1)


def test_pose_is_frozen() -> None:
    p = Pose2D(0, 0, 0)
    with pytest.raises(AttributeError):
        p.x = 1.0  # type: ignore[misc]  # frozen dataclass: verifica inmutabilidad
