"""Tests de la máquina de estados de misión."""

from __future__ import annotations

import itertools

import pytest
from hrobot_core.domain.geometry import Pose2D
from hrobot_core.domain.mission import (
    InvalidTransition,
    Mission,
    MissionEvent,
    MissionState,
    transition,
)

S = MissionState
E = MissionEvent

DEST = Pose2D(5.0, 1.0, 0.0)
BASE = Pose2D(0.0, 0.0, 0.0)

VALID: dict[tuple[S, E], S] = {
    (S.IDLE, E.START): S.NAVIGATING_TO_DESTINATION,
    (S.NAVIGATING_TO_DESTINATION, E.GOAL_REACHED): S.AWAITING_CONFIRMATION,
    (S.AWAITING_CONFIRMATION, E.DELIVERY_CONFIRMED): S.RETURNING_TO_BASE,
    (S.AWAITING_CONFIRMATION, E.CONFIRMATION_TIMEOUT): S.RETURNING_TO_BASE,
    (S.RETURNING_TO_BASE, E.GOAL_REACHED): S.IDLE,
    (S.EMERGENCY_STOP, E.RESET): S.IDLE,
}
for _s in S:
    VALID[(_s, E.EMERGENCY)] = S.EMERGENCY_STOP
    if _s not in (S.IDLE, S.FAILED):
        VALID[(_s, E.NAV_FAILED)] = S.FAILED


def make(state: S = S.IDLE, **kw: object) -> Mission:
    return Mission("M1", DEST, BASE, state=state, **kw)  # type: ignore[arg-type]  # kwargs de test


@pytest.mark.parametrize(("state", "event"), list(itertools.product(S, E)))
def test_all_state_event_combinations(state: S, event: E) -> None:
    m = make(state)
    expected = VALID.get((state, event))
    if expected is None:
        with pytest.raises(InvalidTransition) as exc:
            transition(m, event)
        assert state.name in str(exc.value)
        assert event.name in str(exc.value)
    else:
        assert transition(m, event).state is expected


def test_happy_path_delivered() -> None:
    m = make()
    for ev in (E.START, E.GOAL_REACHED, E.DELIVERY_CONFIRMED, E.GOAL_REACHED):
        m = transition(m, ev)
    assert m.state is S.IDLE
    assert m.delivered is True


def test_confirmation_timeout_not_delivered() -> None:
    m = make()
    for ev in (E.START, E.GOAL_REACHED, E.CONFIRMATION_TIMEOUT):
        m = transition(m, ev)
    assert m.state is S.RETURNING_TO_BASE
    assert m.delivered is False
    assert transition(m, E.GOAL_REACHED).delivered is False


def test_start_resets_delivered() -> None:
    m = transition(make(delivered=True), E.START)
    assert m.delivered is False


def test_emergency_saves_resume_state() -> None:
    m = transition(make(S.AWAITING_CONFIRMATION), E.EMERGENCY)
    assert m.state is S.EMERGENCY_STOP
    assert m.resume_state is S.AWAITING_CONFIRMATION


def test_repeated_emergency_keeps_original_resume_state() -> None:
    m = transition(make(S.RETURNING_TO_BASE), E.EMERGENCY)
    assert transition(m, E.EMERGENCY) == m


@pytest.mark.parametrize("event", [E.GOAL_REACHED, E.START])
def test_emergency_stop_rejects_event(event: E) -> None:
    with pytest.raises(InvalidTransition):
        transition(make(S.EMERGENCY_STOP), event)


def test_reset_goes_idle_and_never_resumes() -> None:
    m = transition(make(S.NAVIGATING_TO_DESTINATION), E.EMERGENCY)
    m = transition(m, E.RESET)
    assert m.state is S.IDLE
    assert m.resume_state is None
    assert m.delivered is False


def test_immutability() -> None:
    m = make()
    m2 = transition(m, E.START)
    assert m.state is S.IDLE
    assert m2 is not m
    with pytest.raises(AttributeError):
        m.state = S.FAILED  # type: ignore[misc]  # verifica que el dataclass es frozen
