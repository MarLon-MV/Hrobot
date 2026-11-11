"""Tests de ``ExplorationManager`` con puertos falsos."""

from __future__ import annotations

import logging
import math

import pytest
from hrobot_core.domain.exploration import ExplorationConfig, GridMap, Point2D
from hrobot_core.services.exploration_manager import (
    REASON_MAP_SAVED,
    REASON_SAVE_FAILED,
    REASON_STOPPED,
    REASON_TOO_MANY_FAILURES,
    ExplorationManager,
    ExplorationState,
    InvalidTransitionError,
    ManagerConfig,
)

EXPLORATION = ExplorationConfig(min_frontier_size=1)
ROBOT = Point2D(3.5, 1.5)


class FakeNavigator:
    def __init__(self) -> None:
        self.goals: list[Point2D] = []
        self.cancels = 0

    def send_goal(self, goal: Point2D) -> None:
        self.goals.append(goal)

    def cancel(self) -> None:
        self.cancels += 1


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


def _make(ok: bool = True, **kwargs: int) -> tuple[ExplorationManager, FakeNavigator, FakeSaver]:
    nav, saver = FakeNavigator(), FakeSaver(ok)
    cfg = ManagerConfig(exploration=EXPLORATION, **kwargs)
    return ExplorationManager(nav, saver, cfg), nav, saver


def test_initial_state_is_idle_and_step_is_noop() -> None:
    mgr, nav, _ = _make()
    assert mgr.state is ExplorationState.IDLE
    assert mgr.reason == ""
    mgr.step(_grid(), ROBOT)
    assert nav.goals == []


def test_happy_path_to_done_saves_map() -> None:
    mgr, nav, saver = _make(map_save_name="hospital")
    mgr.start()
    mgr.step(_grid(), ROBOT)
    assert len(nav.goals) == 1
    mgr.on_goal_result(True)
    for _ in range(2):
        mgr.step(_closed(), ROBOT)
        assert mgr.state is ExplorationState.EXPLORING
        assert saver.names == []
    mgr.step(_closed(), ROBOT)
    assert mgr.state is ExplorationState.DONE
    assert mgr.reason == REASON_MAP_SAVED
    assert saver.names == ["hospital"]


def test_failed_goal_is_blacklisted_and_next_frontier_is_chosen() -> None:
    mgr, nav, _ = _make()
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(False)
    mgr.step(_grid(), ROBOT)
    assert len(nav.goals) == 2
    assert nav.goals[0] != nav.goals[1]
    assert nav.goals[0].x < ROBOT.x < nav.goals[1].x


def test_max_consecutive_failures_leads_to_failed() -> None:
    mgr, nav, _ = _make(max_consecutive_failures=2)
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_TOO_MANY_FAILURES
    mgr.step(_grid(), ROBOT)
    assert len(nav.goals) == 2


def test_success_resets_failure_count() -> None:
    mgr, _, _ = _make(max_consecutive_failures=2)
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(False)
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(True)
    mgr.step(_grid((5,)), ROBOT)
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING


def test_frontier_cycle_resets_no_frontier_count() -> None:
    mgr, nav, saver = _make()
    mgr.start()
    mgr.step(_closed(), ROBOT)
    mgr.step(_closed(), ROBOT)
    mgr.step(_grid(), ROBOT)  # reaparece una frontera: reinicia la cuenta
    mgr.on_goal_result(True)
    mgr.step(_closed(), ROBOT)
    mgr.step(_closed(), ROBOT)
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1


def test_save_failure_leads_to_failed() -> None:
    mgr, _, saver = _make(ok=False, no_frontier_confirmations=1)
    mgr.start()
    mgr.step(_closed(), ROBOT)
    assert mgr.state is ExplorationState.FAILED
    assert mgr.reason == REASON_SAVE_FAILED
    assert saver.names == ["map"]


def test_does_not_resend_goal_while_active() -> None:
    mgr, nav, _ = _make()
    mgr.start()
    for _ in range(5):
        mgr.step(_grid(), ROBOT)
    assert len(nav.goals) == 1


def test_no_frontier_confirmation_not_counted_while_goal_active() -> None:
    mgr, nav, saver = _make(no_frontier_confirmations=1)
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.step(_closed(), ROBOT)
    assert mgr.state is ExplorationState.EXPLORING
    assert saver.names == []
    assert len(nav.goals) == 1


@pytest.mark.parametrize("target", ["idle", "exploring", "done", "failed"])
def test_stop_cancels_from_any_state(target: str) -> None:
    mgr, nav, _ = _make(ok=target != "failed", no_frontier_confirmations=1)
    if target != "idle":
        mgr.start()
    if target == "exploring":
        mgr.step(_grid(), ROBOT)
    elif target in {"done", "failed"}:
        mgr.step(_closed(), ROBOT)
    mgr.stop()
    assert nav.cancels == 1
    assert mgr.state is ExplorationState.IDLE
    assert mgr.reason == REASON_STOPPED


def test_result_after_stop_is_ignored() -> None:
    mgr, nav, _ = _make()
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.stop()
    mgr.on_goal_result(False)
    mgr.start()
    mgr.step(_grid(), ROBOT)
    assert nav.goals[0] == nav.goals[1]  # la lista negra no se contaminó


def test_start_invalid_while_exploring() -> None:
    mgr, _, _ = _make()
    mgr.start()
    with pytest.raises(InvalidTransitionError):
        mgr.start()
    assert mgr.state is ExplorationState.EXPLORING


@pytest.mark.parametrize("final_ok", [True, False])
def test_start_after_done_or_failed_resets_run(final_ok: bool) -> None:
    mgr, nav, _ = _make(ok=final_ok, no_frontier_confirmations=1)
    mgr.start()
    mgr.step(_grid(), ROBOT)
    mgr.on_goal_result(False)  # lista negra con la primera frontera
    mgr.step(_closed(), ROBOT)
    assert mgr.state in {ExplorationState.DONE, ExplorationState.FAILED}
    mgr.start()
    assert mgr.state is ExplorationState.EXPLORING
    assert mgr.reason == ""
    mgr.step(_grid(), ROBOT)
    assert nav.goals[0] == nav.goals[-1]  # lista negra reiniciada


def test_result_without_active_goal_is_ignored() -> None:
    mgr, _, _ = _make(max_consecutive_failures=1)
    mgr.start()
    mgr.on_goal_result(False)
    assert mgr.state is ExplorationState.EXPLORING


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_robot_pose_sends_nothing(bad: float, caplog: pytest.LogCaptureFixture) -> None:
    mgr, nav, saver = _make(no_frontier_confirmations=1)
    mgr.start()
    with caplog.at_level(logging.WARNING):
        mgr.step(_closed(), Point2D(bad, 0.0))
        mgr.step(_grid(), Point2D(0.0, bad))
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
    ],
)
def test_invalid_manager_config_rejected(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        ManagerConfig(**kwargs)  # type: ignore[arg-type]  # kwargs heterogéneos a propósito


def test_default_config() -> None:
    nav, saver = FakeNavigator(), FakeSaver()
    mgr = ExplorationManager(nav, saver)
    mgr.start()
    assert mgr.state is ExplorationState.EXPLORING
