"""Tests de ``ExplorationManager`` con puertos falsos."""

from __future__ import annotations

import logging
import math
from functools import partial
from typing import Any

import pytest
from hrobot_core.domain.exploration import ExplorationConfig, GridMap, Point2D
from hrobot_core.services.exploration_manager import (
    REASON_LOW_BATTERY,
    REASON_MAP_SAVED,
    REASON_SAVE_FAILED,
    REASON_STATUS_TIMEOUT,
    REASON_STOPPED,
    REASON_TOO_MANY_FAILURES,
    REASON_UNDOCK_FAILED,
    REASON_UNDOCK_TIMEOUT,
    ExplorationManager,
    ExplorationState,
    InvalidTransitionError,
    ManagerConfig,
    RobotStatus,
)

EXPLORATION = ExplorationConfig(min_frontier_size=1)
ROBOT = Point2D(3.5, 1.5)
OK = RobotStatus(docked=False, kidnapped=False, battery_pct=80.0)
DOCKED = RobotStatus(docked=True, kidnapped=False, battery_pct=80.0)
UNKNOWN_DOCK = RobotStatus(docked=None, kidnapped=False, battery_pct=80.0)


class FakeNavigator:
    def __init__(self) -> None:
        self.goals: list[Point2D] = []
        self.cancels = 0

    def send_goal(self, goal: Point2D) -> None:
        self.goals.append(goal)

    def cancel(self) -> None:
        self.cancels += 1


class FakeDock:
    def __init__(self) -> None:
        self.undocks = 0
        self.cancels = 0

    def undock(self) -> None:
        self.undocks += 1

    def cancel(self) -> None:
        self.cancels += 1


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class FakeSaver:
    def __init__(self, ok: bool = True) -> None:
        self.ok = ok
        self.names: list[str] = []

    def save(self, name: str) -> bool:
        self.names.append(name)
        return self.ok


def _grid(unknown_cols: tuple[int, ...] = (0, 10), width: int = 11) -> GridMap:
    """Pasillo libre (fila 1) con celdas desconocidas sobre las columnas indicadas."""
    top = [100] * width
    for col in unknown_cols:
        top[col] = -1
    rows = [top, [0] * width, [100] * width]
    return GridMap(
        width=width,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(v for r in rows for v in r),
    )


def _closed() -> GridMap:
    return _grid(unknown_cols=())


def _make(ok: bool = True, **kwargs: Any) -> tuple[ExplorationManager, FakeNavigator, FakeSaver]:
    nav, saver = FakeNavigator(), FakeSaver(ok)
    cfg = ManagerConfig(exploration=EXPLORATION, **kwargs)
    return ExplorationManager(nav, saver, FakeDock(), cfg), nav, saver


def _make_full(
    **kwargs: Any,
) -> tuple[ExplorationManager, FakeNavigator, FakeSaver, FakeDock, FakeClock]:
    nav, saver, dock, clock = FakeNavigator(), FakeSaver(), FakeDock(), FakeClock()
    cfg = ManagerConfig(exploration=EXPLORATION, **kwargs)
    return ExplorationManager(nav, saver, dock, cfg, clock), nav, saver, dock, clock


def test_initial_state_is_idle_and_step_is_noop() -> None:
    mgr, nav, _ = _make()
    assert mgr.state is ExplorationState.IDLE
    assert mgr.reason == ""
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals == []


def test_happy_path_to_done_saves_map() -> None:
    mgr, nav, saver = _make(map_save_name="hospital")
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    assert len(nav.goals) == 1
    mgr.on_goal_result(True)
    for _ in range(2):
        mgr.step(_closed(), ROBOT, OK)
        assert mgr.state is ExplorationState.EXPLORING
        assert saver.names == []
    mgr.step(_closed(), ROBOT, OK)
    assert mgr.state is ExplorationState.DONE
    assert mgr.reason == REASON_MAP_SAVED
    assert saver.names == ["hospital"]


def test_failed_goal_is_blacklisted_and_next_frontier_is_chosen() -> None:
    mgr, nav, _ = _make()
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(False)
    mgr.step(_grid(), ROBOT, OK)
    assert len(nav.goals) == 2
    assert nav.goals[0] != nav.goals[1]
    assert nav.goals[0].x < ROBOT.x < nav.goals[1].x


def test_max_consecutive_failures_leads_to_failed() -> None:
    mgr, nav, _ = _make(max_consecutive_failures=2)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_TOO_MANY_FAILURES
    mgr.step(_grid(), ROBOT, OK)
    assert len(nav.goals) == 2


def test_success_resets_failure_count() -> None:
    mgr, _, _ = _make(max_consecutive_failures=2)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(False)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(True)
    mgr.step(_grid((5,)), ROBOT, OK)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING


def test_frontier_cycle_resets_no_frontier_count() -> None:
    mgr, nav, saver = _make()
    mgr.start(OK)
    mgr.step(_closed(), ROBOT, OK)
    mgr.step(_closed(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, OK)  # reaparece una frontera: reinicia la cuenta
    mgr.on_goal_result(True)
    mgr.step(_closed(), ROBOT, OK)
    mgr.step(_closed(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1


def test_save_failure_leads_to_failed() -> None:
    mgr, _, saver = _make(ok=False, no_frontier_confirmations=1)
    mgr.start(OK)
    mgr.step(_closed(), ROBOT, OK)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_SAVE_FAILED
    assert saver.names == ["map"]


def test_does_not_resend_goal_while_active() -> None:
    mgr, nav, _ = _make()
    mgr.start(OK)
    for _ in range(5):
        mgr.step(_grid(), ROBOT, OK)
    assert len(nav.goals) == 1


def test_no_frontier_confirmation_not_counted_while_goal_active() -> None:
    mgr, nav, saver = _make(no_frontier_confirmations=1)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_closed(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1


@pytest.mark.parametrize("target", ["idle", "exploring", "done", "failed"])
def test_stop_cancels_from_any_state(target: str) -> None:
    mgr, nav, _ = _make(ok=target != "failed", no_frontier_confirmations=1)
    if target != "idle":
        mgr.start(OK)
    if target == "exploring":
        mgr.step(_grid(), ROBOT, OK)
    elif target in {"done", "failed"}:
        mgr.step(_closed(), ROBOT, OK)
    mgr.stop()
    assert nav.cancels == 1
    assert mgr.state is ExplorationState.IDLE
    assert mgr.reason == REASON_STOPPED


def test_result_after_stop_is_ignored() -> None:
    mgr, nav, _ = _make()
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.stop()
    mgr.on_goal_result(False)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals[0] == nav.goals[1]  # la lista negra no se contaminó


def test_start_invalid_while_exploring() -> None:
    mgr, _, _ = _make()
    mgr.start(OK)
    with pytest.raises(InvalidTransitionError):
        mgr.start(OK)
    assert mgr.state is ExplorationState.EXPLORING


@pytest.mark.parametrize("state", ["undocking", "paused"])
def test_start_invalid_while_undocking_or_paused(state: str) -> None:
    mgr, _, _, _, _ = _make_full()
    if state == "undocking":
        mgr.start(RobotStatus(docked=True, kidnapped=False))
    else:
        mgr.start(OK)
        mgr.step(_grid(), ROBOT, RobotStatus(docked=False, kidnapped=True))
    assert mgr.state.value == state.upper()
    with pytest.raises(InvalidTransitionError):
        mgr.start(OK)


@pytest.mark.parametrize("final_ok", [True, False])
def test_start_after_done_or_failed_resets_run(final_ok: bool) -> None:
    mgr, nav, _ = _make(ok=final_ok, no_frontier_confirmations=1)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_goal_result(False)  # lista negra con la primera frontera
    mgr.step(_closed(), ROBOT, OK)
    assert mgr.state in {ExplorationState.DONE, ExplorationState.FAILED}
    mgr.start(OK)
    assert mgr.state is ExplorationState.EXPLORING
    assert mgr.reason == ""
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals[0] == nav.goals[-1]  # lista negra reiniciada


def test_result_without_active_goal_is_ignored() -> None:
    mgr, _, _ = _make(max_consecutive_failures=1)
    mgr.start(OK)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_robot_pose_sends_nothing(bad: float, caplog: pytest.LogCaptureFixture) -> None:
    mgr, nav, saver = _make(no_frontier_confirmations=1)
    mgr.start(OK)
    with caplog.at_level(logging.WARNING):
        mgr.step(_closed(), Point2D(bad, 0.0), OK)
        mgr.step(_grid(), Point2D(0.0, bad), OK)
    assert nav.goals == []
    assert saver.names == []
    assert mgr.state is ExplorationState.EXPLORING
    assert "not finite" in caplog.text


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_consecutive_failures": 0},
        {"no_frontier_confirmations": 0},
        {"map_save_name": "  "},
        {"undock_timeout_s": 0.0},
        {"undock_timeout_s": math.nan},
        {"status_timeout_s": -1.0},
        {"status_timeout_s": math.inf},
        {"kidnap_clear_cycles": 0},
        {"min_battery_pct": -1.0},
        {"min_battery_pct": 101.0},
        {"min_battery_pct": math.nan},
    ],
)
def test_invalid_manager_config_rejected(kwargs: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        ManagerConfig(**kwargs)


def test_default_config() -> None:
    nav, saver = FakeNavigator(), FakeSaver()
    mgr = ExplorationManager(nav, saver, FakeDock())
    mgr.start(OK)
    assert mgr.state is ExplorationState.EXPLORING


# --------------------------------------------------------------------------- dock
def test_docked_start_undocks_and_sends_no_goal_until_undocked() -> None:
    mgr, nav, _, dock, _ = _make_full()
    mgr.start(DOCKED)
    assert mgr.state is ExplorationState.UNDOCKING
    assert dock.undocks == 1
    for _ in range(3):
        mgr.step(_grid(), ROBOT, DOCKED)
    assert nav.goals == []
    assert dock.undocks == 1  # no repite el undock
    mgr.on_undock_result(True)
    mgr.step(_grid(), ROBOT, DOCKED)
    assert mgr.state is ExplorationState.UNDOCKING  # aún se exige docked == False
    mgr.step(_grid(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING
    mgr.step(_grid(), ROBOT, OK)
    assert len(nav.goals) == 1


def test_undock_failure_leads_to_failed() -> None:
    mgr, nav, _, _, _ = _make_full()
    mgr.start(DOCKED)
    mgr.on_undock_result(False)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_UNDOCK_FAILED
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals == []


def test_undock_timeout_leads_to_failed() -> None:
    mgr, _, _, dock, clock = _make_full(undock_timeout_s=20.0)
    mgr.start(DOCKED)
    clock.now = 19.9
    mgr.step(_grid(), ROBOT, DOCKED)
    assert mgr.state is ExplorationState.UNDOCKING
    assert dock.cancels == 0
    clock.now = 20.0
    mgr.step(_grid(), ROBOT, DOCKED)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_UNDOCK_TIMEOUT
    assert dock.cancels == 1  # el undock pendiente se cancela


def test_stop_during_undocking_cancels_the_undock() -> None:
    mgr, _, _, dock, _ = _make_full()
    mgr.start(DOCKED)
    mgr.stop()
    assert mgr.state is ExplorationState.IDLE
    assert dock.cancels == 1
    mgr.on_undock_result(False)  # resultado tardío: no altera el estado
    assert mgr.state is ExplorationState.IDLE


def test_undock_timeout_also_with_unknown_dock_after_request() -> None:
    mgr, _, _, _, clock = _make_full(undock_timeout_s=5.0, status_timeout_s=1.0)
    mgr.start(DOCKED)
    clock.now = 4.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.state is ExplorationState.UNDOCKING  # status_timeout no aplica tras pedir undock
    clock.now = 5.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.reason == REASON_UNDOCK_TIMEOUT


@pytest.mark.parametrize("status", [None, UNKNOWN_DOCK])
def test_unknown_dock_waits_then_fails_after_status_timeout(status: RobotStatus | None) -> None:
    mgr, nav, _, dock, clock = _make_full(status_timeout_s=10.0)
    mgr.start(status)
    assert mgr.state is ExplorationState.UNDOCKING
    clock.now = 9.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.state is ExplorationState.UNDOCKING
    clock.now = 10.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_STATUS_TIMEOUT
    assert nav.goals == []
    assert dock.undocks == 0


def test_unknown_dock_then_docked_requests_undock() -> None:
    mgr, _, _, dock, clock = _make_full()
    mgr.start(None)
    clock.now = 1.0
    mgr.step(_grid(), ROBOT, DOCKED)
    assert dock.undocks == 1
    assert mgr.state is ExplorationState.UNDOCKING


def test_unknown_dock_then_undocked_explores() -> None:
    mgr, _, _, dock, _ = _make_full()
    mgr.start(None)
    mgr.step(_grid(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING
    assert dock.undocks == 0


def test_undock_result_ignored_outside_undocking() -> None:
    mgr, _, _, _, _ = _make_full()
    mgr.on_undock_result(False)  # IDLE
    assert mgr.state is ExplorationState.IDLE
    mgr.start(OK)
    mgr.on_undock_result(False)  # EXPLORING
    assert mgr.state is ExplorationState.EXPLORING
    mgr.stop()
    mgr.start(None)
    mgr.on_undock_result(False)  # UNDOCKING sin undock solicitado
    assert mgr.state is ExplorationState.UNDOCKING


@pytest.mark.parametrize("dock", [True, None])
def test_no_goals_while_exploring_if_dock_not_confirmed_false(dock: bool | None) -> None:
    mgr, nav, _, _, _ = _make_full()
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, RobotStatus(docked=dock, kidnapped=False, battery_pct=80.0))
    assert nav.goals == []
    assert mgr.state is ExplorationState.EXPLORING


# --------------------------------------------------------------------------- secuestro
def _kidnapped(value: bool | None) -> RobotStatus:
    return RobotStatus(docked=False, kidnapped=value, battery_pct=80.0)


@pytest.mark.parametrize("value", [True, None])
def test_kidnap_cancels_and_pauses_without_counting_failure(value: bool | None) -> None:
    mgr, nav, _ = _make(max_consecutive_failures=1)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, _kidnapped(value))
    assert mgr.state is ExplorationState.PAUSED
    assert nav.cancels == 1
    mgr.on_goal_result(False)  # resultado tardío de la cancelación: ignorado
    assert mgr.state is ExplorationState.PAUSED
    for _ in range(3):
        mgr.step(_grid(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals[0] == nav.goals[1]  # sin lista negra


def test_resumes_only_after_clear_cycles_and_dirty_cycle_resets_count() -> None:
    mgr, nav, _ = _make(kidnap_clear_cycles=3)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, _kidnapped(True))
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, _kidnapped(True))  # reinicia
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, _kidnapped(None))  # desconocido también reinicia
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, OK)
    assert mgr.state is ExplorationState.PAUSED
    assert nav.goals == []
    mgr.step(_grid(), ROBOT, OK)
    assert mgr.state is ExplorationState.EXPLORING


def test_hazard_ignored_while_paused() -> None:
    mgr, _, _ = _make()
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, _kidnapped(True))
    mgr.on_hazard(Point2D(1.0, 1.0))
    assert mgr.state is ExplorationState.PAUSED


# --------------------------------------------------------------------------- batería
def _battery(pct: float | None) -> RobotStatus:
    return RobotStatus(docked=False, kidnapped=False, battery_pct=pct)


def test_low_battery_cancels_saves_partial_map_and_finishes_done() -> None:
    mgr, nav, saver = _make(map_save_name="parcial")
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.step(_grid(), ROBOT, _battery(19.9))
    assert mgr.state is ExplorationState.DONE
    assert mgr.reason == REASON_LOW_BATTERY
    assert saver.names == ["parcial"]
    assert nav.cancels == 1


def test_battery_exactly_at_threshold_does_not_trigger() -> None:
    mgr, nav, saver = _make(min_battery_pct=20.0)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, _battery(20.0))
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1


def test_low_battery_with_save_failure_is_failed() -> None:
    mgr, _, _ = _make(ok=False)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, _battery(5.0))
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_SAVE_FAILED


@pytest.mark.parametrize("bad", [None, math.nan, math.inf, -math.inf, -5.0, 150.0])
def test_unknown_battery_warns_once_and_does_not_abort(
    bad: float | None, caplog: pytest.LogCaptureFixture
) -> None:
    mgr, nav, saver = _make()
    mgr.start(OK)
    with caplog.at_level(logging.WARNING):
        mgr.step(_grid(), ROBOT, _battery(bad))
        mgr.step(_grid(), ROBOT, _battery(bad))
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1
    assert caplog.text.count("battery level unknown") == 1
    mgr.step(_grid(), ROBOT, _battery(10.0))  # vuelve el dato y está bajo
    assert mgr.reason == REASON_LOW_BATTERY


# --------------------------------------------------------------------------- peligros
def test_hazard_cancels_blacklists_point_and_counts_failure() -> None:
    mgr, nav, _ = _make(max_consecutive_failures=2)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    first = nav.goals[0]
    mgr.on_hazard(first)
    assert nav.cancels == 1
    assert mgr.state is ExplorationState.EXPLORING
    mgr.on_goal_result(False)  # resultado de la cancelación: no cuenta doble
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals[1] != first  # el punto del peligro quedó en lista negra
    mgr.on_hazard(nav.goals[1])
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_TOO_MANY_FAILURES


def test_hazard_without_pose_does_not_crash_and_blacklists_active_goal() -> None:
    mgr, nav, _ = _make(max_consecutive_failures=2)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    mgr.on_hazard(None)
    assert nav.cancels == 1
    mgr.step(_grid(), ROBOT, OK)
    assert nav.goals[0] != nav.goals[1]
    mgr.on_hazard(Point2D(math.nan, 0.0))  # pose no finita: se trata como sin pose
    assert mgr.state is ExplorationState.FAILED


def test_hazard_without_active_goal_or_pose_only_counts_failure() -> None:
    mgr, _, _ = _make(max_consecutive_failures=2)
    mgr.start(OK)
    mgr.on_hazard(None)
    assert mgr.state is ExplorationState.EXPLORING
    mgr.on_hazard(None)
    assert mgr.state is ExplorationState.FAILED


def test_hazard_ignored_when_not_exploring() -> None:
    mgr, nav, _ = _make()
    mgr.on_hazard(Point2D(1.0, 1.0))
    assert nav.cancels == 0
    assert mgr.state is ExplorationState.IDLE


def test_stop_during_undocking_returns_to_idle() -> None:
    mgr, nav, _, _, _ = _make_full()
    mgr.start(DOCKED)
    mgr.stop()
    assert mgr.state is ExplorationState.IDLE
    assert nav.cancels == 1


# --------------------------------------------------------------------------- concurrencia
@pytest.mark.parametrize("hazard_first", [True, False])
def test_hazard_and_goal_failure_on_same_goal_count_exactly_one(hazard_first: bool) -> None:
    mgr, nav, _ = _make()
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, OK)
    goal = nav.goals[0]
    if hazard_first:
        mgr.on_hazard(goal)
        mgr.on_goal_result(False)
    else:
        mgr.on_goal_result(False)
        mgr.on_hazard(goal)
    assert mgr._failures == 1


def test_hazard_and_goal_result_from_threads_count_one_failure() -> None:
    import threading

    for _ in range(50):
        mgr, nav, _ = _make()
        mgr.start(OK)
        mgr.step(_grid(), ROBOT, OK)
        goal = nav.goals[0]
        barrier = threading.Barrier(2)

        def run(fn: Any, barrier: threading.Barrier = barrier) -> None:
            barrier.wait()
            fn()

        threads = [
            threading.Thread(target=run, args=(partial(mgr.on_hazard, goal),)),
            threading.Thread(target=run, args=(partial(mgr.on_goal_result, False),)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert mgr._failures == 1


# --------------------------------------------------------------------------- dock en EXPLORING
def test_exploring_docked_beyond_timeout_returns_to_undocking() -> None:
    mgr, nav, _, dock, clock = _make_full(status_timeout_s=5.0)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, DOCKED)
    assert mgr.state is ExplorationState.EXPLORING
    clock.now = 5.0
    mgr.step(_grid(), ROBOT, DOCKED)
    assert mgr.state is ExplorationState.UNDOCKING
    assert dock.undocks == 1
    assert nav.goals == []


def test_exploring_unknown_dock_beyond_timeout_fails() -> None:
    mgr, _, _, _, clock = _make_full(status_timeout_s=5.0)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    clock.now = 5.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_STATUS_TIMEOUT


def test_exploring_dock_wait_resets_when_undocked() -> None:
    mgr, _, _, _, clock = _make_full(status_timeout_s=5.0)
    mgr.start(OK)
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    clock.now = 4.0
    mgr.step(_grid(), ROBOT, OK)  # confirma no acoplado: reinicia la espera
    mgr.on_goal_result(True)
    clock.now = 6.0
    mgr.step(_grid(), ROBOT, UNKNOWN_DOCK)
    assert mgr.state is ExplorationState.EXPLORING
