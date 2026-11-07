"""Tests del supervisor de seguridad."""

from __future__ import annotations

import math
import random

import pytest
from hrobot_core.domain.mission import MissionState
from hrobot_core.services.safety_supervisor import (
    SafetySupervisor,
    StopReason,
    SupervisorConfig,
    SupervisorOutput,
    VelocityCommand,
)

NAV = MissionState.NAVIGATING_TO_DESTINATION
N_RAYS = 360
STEP_S = 0.1
INC = 2 * math.pi / N_RAYS
CMD = VelocityCommand(0.2, 0.5)


def feed_scan(sup: SafetySupervisor, now_s: float, distance: float = 2.0) -> None:
    sup.on_scan([distance] * N_RAYS, -math.pi, INC, 0.05, 12.0, now_s)


def make(
    now_s: float = 0.0,
    cmd: VelocityCommand | None = CMD,
    state: MissionState | None = NAV,
    scan: bool = True,
    cfg: SupervisorConfig | None = None,
) -> SafetySupervisor:
    sup = SafetySupervisor(cfg)
    if state is not None:
        sup.on_mission_state(state, now_s)
    if cmd is not None:
        sup.on_command(cmd, now_s)
    if scan:
        feed_scan(sup, now_s)
    return sup


def refresh(sup: SafetySupervisor, now_s: float, cmd: VelocityCommand = CMD) -> None:
    sup.on_mission_state(NAV, now_s)
    sup.on_command(cmd, now_s)
    feed_scan(sup, now_s)


def assert_stopped(out: SupervisorOutput, reason: StopReason) -> None:
    assert out.reason is reason
    assert out.command == VelocityCommand(0.0, 0.0)
    assert out.factor == 0.0
    assert out.reverse_blocked is False


def ramp_up(sup: SafetySupervisor, cmd: VelocityCommand = CMD) -> SupervisorOutput:
    out = sup.step(0.0)
    for i in range(1, 13):
        refresh(sup, i * STEP_S, cmd)
        out = sup.step(i * STEP_S)
    return out


# --- un test por StopReason ---------------------------------------------------


def test_reason_no_mission_status() -> None:
    assert_stopped(make(state=None).step(0.0), StopReason.NO_MISSION_STATUS)


def test_reason_none_state_equals_no_status() -> None:
    sup = make()
    sup.on_mission_state(None, 0.0)
    assert_stopped(sup.step(0.0), StopReason.NO_MISSION_STATUS)


def test_reason_mission_status_stale() -> None:
    sup = make()
    sup.on_command(CMD, 2.5)
    feed_scan(sup, 2.5)
    assert_stopped(sup.step(2.5), StopReason.MISSION_STATUS_STALE)


def test_reason_emergency() -> None:
    sup = make(state=MissionState.EMERGENCY_STOP)
    assert_stopped(sup.step(0.0), StopReason.EMERGENCY)


@pytest.mark.parametrize(
    "state",
    [MissionState.IDLE, MissionState.AWAITING_CONFIRMATION, MissionState.FAILED],
)
def test_reason_mission_not_moving(state: MissionState) -> None:
    assert_stopped(make(state=state).step(0.0), StopReason.MISSION_NOT_MOVING)


def test_reason_no_command() -> None:
    assert_stopped(make(cmd=None).step(0.0), StopReason.NO_COMMAND)


def test_reason_command_stale() -> None:
    sup = make()
    sup.on_mission_state(NAV, 1.0)
    feed_scan(sup, 1.0)
    assert_stopped(sup.step(1.0), StopReason.COMMAND_STALE)


@pytest.mark.parametrize(
    "cmd",
    [
        VelocityCommand(math.nan, 0.0),
        VelocityCommand(0.1, math.inf),
        VelocityCommand(-math.inf, 0.0),
        VelocityCommand(0.1, math.nan),
    ],
)
def test_reason_invalid_command(cmd: VelocityCommand) -> None:
    assert_stopped(make(cmd=cmd).step(0.0), StopReason.INVALID_COMMAND)


def test_reason_no_scan() -> None:
    assert_stopped(make(scan=False).step(0.0), StopReason.NO_SCAN)


def test_reason_scan_blocked_by_obstacle() -> None:
    sup = make()
    feed_scan(sup, 0.0, 0.35)
    assert_stopped(sup.step(0.0), StopReason.SCAN_BLOCKED)


def test_reason_scan_blocked_by_empty_scan() -> None:
    sup = make()
    sup.on_scan([], 0.0, 0.01, 0.05, 12.0, 0.0)
    assert_stopped(sup.step(0.0), StopReason.SCAN_BLOCKED)


def test_reason_scan_blocked_by_stale_scan() -> None:
    sup = make()
    sup.on_mission_state(NAV, 1.0)
    sup.on_command(CMD, 1.0)
    assert_stopped(sup.step(1.0), StopReason.SCAN_BLOCKED)


def test_reason_none_and_ramp_limited() -> None:
    first = make().step(0.0)
    assert first.reason is StopReason.RAMP_LIMITED
    assert first.command == VelocityCommand(0.0, 0.0)
    assert ramp_up(make()).reason is StopReason.NONE


# --- prioridad ----------------------------------------------------------------


def test_priority_emergency_over_everything() -> None:
    sup = SafetySupervisor()
    sup.on_mission_state(MissionState.EMERGENCY_STOP, 0.0)
    sup.on_command(VelocityCommand(math.nan, 0.0), 0.0)
    sup.on_scan([0.1] * 10, 0.0, 0.01, 0.05, 12.0, 0.0)
    assert_stopped(sup.step(0.4), StopReason.EMERGENCY)


def test_priority_emergency_over_stale_command_and_blocked_scan() -> None:
    sup = make(state=MissionState.EMERGENCY_STOP)
    sup.on_mission_state(MissionState.EMERGENCY_STOP, 1.0)  # misión vigente
    feed_scan(sup, 1.0, 0.35)  # scan bloqueado; orden de t=0 caducada (1.0 > 0.5)
    assert_stopped(sup.step(1.0), StopReason.EMERGENCY)


def test_priority_not_moving_over_stale_command() -> None:
    sup = make(state=MissionState.IDLE)
    sup.on_mission_state(MissionState.IDLE, 1.0)
    feed_scan(sup, 1.0)
    assert_stopped(sup.step(1.0), StopReason.MISSION_NOT_MOVING)


def test_priority_not_moving_over_invalid_command_and_blocked_scan() -> None:
    sup = make(state=MissionState.IDLE, cmd=VelocityCommand(math.nan, 0.0))
    feed_scan(sup, 0.0, 0.35)
    assert_stopped(sup.step(0.0), StopReason.MISSION_NOT_MOVING)


def test_priority_invalid_command_over_blocked_scan() -> None:
    sup = make(cmd=VelocityCommand(math.nan, 0.0))
    feed_scan(sup, 0.0, 0.35)
    assert_stopped(sup.step(0.0), StopReason.INVALID_COMMAND)


def test_priority_stale_mission_over_emergency() -> None:
    sup = make(state=MissionState.EMERGENCY_STOP)
    assert_stopped(sup.step(5.0), StopReason.MISSION_STATUS_STALE)


def test_priority_stale_command_over_invalid_and_no_scan() -> None:
    sup = make(cmd=VelocityCommand(math.nan, 0.0), scan=False)
    sup.on_mission_state(NAV, 1.0)
    assert_stopped(sup.step(1.0), StopReason.COMMAND_STALE)


def test_priority_invalid_command_over_no_scan() -> None:
    sup = make(cmd=VelocityCommand(math.nan, 0.0), scan=False)
    assert_stopped(sup.step(0.0), StopReason.INVALID_COMMAND)


def test_priority_no_command_over_no_scan() -> None:
    assert_stopped(make(cmd=None, scan=False).step(0.0), StopReason.NO_COMMAND)


# --- flujo feliz, rampa y bajada -----------------------------------------------


def test_happy_path_ramp() -> None:
    sup = make()
    first = sup.step(0.0)
    assert first.factor == 0.0
    assert first.command == VelocityCommand(0.0, 0.0)
    prev = 0.0
    for i in range(1, 11):
        refresh(sup, i * STEP_S)
        out = sup.step(i * STEP_S)
        assert out.factor == pytest.approx(i * 0.1)
        assert out.factor >= prev
        prev = out.factor
    refresh(sup, 1.1)
    out = sup.step(1.1)
    assert out.factor == pytest.approx(1.0)
    assert out.command.linear_x == pytest.approx(0.2)
    assert out.command.angular_z == pytest.approx(0.5)
    assert out.reason is StopReason.NONE


def test_instant_drop_and_ramped_recovery() -> None:
    sup = make()
    ramp_up(sup)
    t = 1.3
    refresh(sup, t)
    feed_scan(sup, t, 0.35)
    assert_stopped(sup.step(t), StopReason.SCAN_BLOCKED)
    refresh(sup, t + STEP_S)
    out = sup.step(t + STEP_S)
    assert out.factor == pytest.approx(0.1)
    assert out.reason is StopReason.RAMP_LIMITED


def test_partial_drop_is_instant() -> None:
    sup = make()
    ramp_up(sup)
    t = 1.3
    refresh(sup, t)
    feed_scan(sup, t, 0.675)
    out = sup.step(t)
    assert out.factor == pytest.approx(0.5)
    assert out.reason is StopReason.NONE


# --- orden final ---------------------------------------------------------------


def test_reverse_is_blocked_but_turn_allowed() -> None:
    cmd = VelocityCommand(-0.2, 0.5)
    out = ramp_up(make(cmd=cmd), cmd)
    assert out.command.linear_x == 0.0
    assert out.reverse_blocked is True
    assert out.command.angular_z == pytest.approx(0.5)


def test_reverse_flagged_while_ramping() -> None:
    out = make(cmd=VelocityCommand(-0.2, 0.5)).step(0.0)
    assert out.reverse_blocked is True
    assert out.command == VelocityCommand(0.0, 0.0)


def test_caps_linear() -> None:
    cmd = VelocityCommand(5.0, 0.0)
    out = ramp_up(make(cmd=cmd), cmd)
    assert out.command.linear_x == pytest.approx(0.3)


@pytest.mark.parametrize(("raw", "expected"), [(-9.0, -1.0), (9.0, 1.0)])
def test_caps_angular(raw: float, expected: float) -> None:
    cmd = VelocityCommand(0.1, raw)
    out = ramp_up(make(cmd=cmd), cmd)
    assert out.command.angular_z == pytest.approx(expected)


def test_huge_values_stay_bounded() -> None:
    cmd = VelocityCommand(1e308, -1e308)
    out = ramp_up(make(cmd=cmd), cmd)
    assert out.command.linear_x == pytest.approx(0.3)
    assert out.command.angular_z == pytest.approx(-1.0)


# --- datos inválidos y reloj ---------------------------------------------------


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_now_raises(bad: float) -> None:
    sup = SafetySupervisor()
    with pytest.raises(ValueError, match="now_s"):
        sup.step(bad)
    with pytest.raises(ValueError, match="now_s"):
        sup.on_command(CMD, bad)
    with pytest.raises(ValueError, match="now_s"):
        sup.on_scan([1.0], 0.0, 0.1, 0.05, 12.0, bad)
    with pytest.raises(ValueError, match="now_s"):
        sup.on_mission_state(NAV, bad)


def test_clock_going_backwards_no_exception_and_no_rise() -> None:
    sup = make(now_s=10.0)
    sup.step(10.0)
    refresh(sup, 10.5)
    before = sup.step(10.5)
    assert before.factor == pytest.approx(0.5)
    # Datos frescos (edad 0) con reloj retrocedido: dt = 0, el factor no sube.
    refresh(sup, 10.2)
    back = sup.step(10.2)
    assert back.reason is StopReason.RAMP_LIMITED
    assert back.factor == before.factor
    # dt se mide respecto al máximo now_s visto (10.5), no al último (10.2).
    refresh(sup, 10.6)
    assert sup.step(10.6).factor == pytest.approx(0.6)


def test_negative_mission_age_is_stale() -> None:
    assert_stopped(make(now_s=5.0).step(4.0), StopReason.MISSION_STATUS_STALE)


def test_negative_command_age_is_stale() -> None:
    bench = make(now_s=5.0, cfg=SupervisorConfig(require_mission_status=False))
    assert_stopped(bench.step(4.0), StopReason.COMMAND_STALE)


def test_negative_scan_age_is_stale() -> None:
    sup = SafetySupervisor()
    feed_scan(sup, 5.0)
    sup.on_mission_state(NAV, 4.9)
    sup.on_command(CMD, 4.9)
    assert_stopped(sup.step(4.9), StopReason.SCAN_BLOCKED)


def test_stop_resets_ramp() -> None:
    sup = make()
    ramp_up(sup)
    assert_stopped(sup.step(5.0), StopReason.MISSION_STATUS_STALE)
    refresh(sup, 5.1)
    assert sup.step(5.1).factor == pytest.approx(0.1)


def test_scan_is_copied() -> None:
    sup = make(scan=False)
    ranges = [2.0] * N_RAYS
    sup.on_scan(ranges, -math.pi, INC, 0.05, 12.0, 0.0)
    ranges[:] = [0.1] * N_RAYS
    assert sup.step(0.0).reason is StopReason.RAMP_LIMITED


# --- modo de banco ---------------------------------------------------------------

BENCH = SupervisorConfig(require_mission_status=False)


def test_bench_without_state_moves() -> None:
    out = ramp_up(make(state=None, cfg=BENCH))
    assert out.command.linear_x == pytest.approx(0.2)


def test_bench_idle_moves() -> None:
    sup = make(state=MissionState.IDLE, cfg=BENCH)
    out = sup.step(0.0)
    for i in range(1, 13):
        sup.on_mission_state(MissionState.IDLE, i * STEP_S)
        sup.on_command(CMD, i * STEP_S)
        feed_scan(sup, i * STEP_S)
        out = sup.step(i * STEP_S)
    assert out.command.linear_x == pytest.approx(0.2)


def test_bench_emergency_persists_until_other_state() -> None:
    sup = make(state=MissionState.EMERGENCY_STOP, cfg=BENCH)
    assert_stopped(sup.step(0.0), StopReason.EMERGENCY)
    sup.on_command(CMD, 100.0)
    feed_scan(sup, 100.0)
    assert_stopped(sup.step(100.0), StopReason.EMERGENCY)
    sup.on_mission_state(None, 100.0)
    assert_stopped(sup.step(100.0), StopReason.EMERGENCY)
    sup.on_mission_state(NAV, 100.1)
    sup.on_command(CMD, 100.1)
    feed_scan(sup, 100.1)
    out = sup.step(100.1)
    assert out.reason is StopReason.RAMP_LIMITED
    assert out.factor == pytest.approx(0.1)  # rampa desde reposo, no salta a 1.0
    sup.on_command(CMD, 100.2)
    feed_scan(sup, 100.2)
    assert sup.step(100.2).factor == pytest.approx(0.2)


def test_bench_still_requires_command() -> None:
    assert_stopped(make(state=None, cmd=None, cfg=BENCH).step(0.0), StopReason.NO_COMMAND)


# --- configuración -------------------------------------------------------------------

_FIELDS = ["max_linear", "max_angular", "cmd_timeout_s", "mission_timeout_s", "max_rise_per_s"]


@pytest.mark.parametrize("name", _FIELDS)
@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_config_invalid_values(name: str, bad: float) -> None:
    kwargs: dict[str, float] = {name: bad}
    with pytest.raises(ValueError, match=name):
        SupervisorConfig(**kwargs)  # type: ignore[arg-type]  # kwargs dinámicos de prueba


def test_config_defaults() -> None:
    cfg = SupervisorConfig()
    assert cfg.max_linear == 0.3
    assert cfg.require_mission_status is True


# --- propiedad -----------------------------------------------------------------------

_STATES: list[MissionState | None] = [*MissionState, None]
_SPECIAL = [math.nan, math.inf, -math.inf, 0.0, -0.3, 0.2, 1e300, -1e300, 5.0, -9.0]
_AGES = [-1.0, 0.0, 0.1, 0.49, 0.5, 0.51, 1.9, 2.0, 2.1, 50.0]
_RANGE_POOL = [math.nan, math.inf, -math.inf, 0.0, 0.01, 0.35, 0.6, 1.5, 20.0, 3.0]
_MOVING = (NAV, MissionState.RETURNING_TO_BASE)
NOW = 100.0


def _rand_value(rng: random.Random) -> float:
    return rng.choice(_SPECIAL) if rng.random() < 0.6 else rng.uniform(-3.0, 3.0)


def _populate(sup: SafetySupervisor, rng: random.Random) -> None:
    if rng.random() < 0.8:
        sup.on_mission_state(rng.choice(_STATES), NOW - rng.choice(_AGES))
    if rng.random() < 0.85:
        cmd = VelocityCommand(_rand_value(rng), _rand_value(rng))
        sup.on_command(cmd, NOW - rng.choice(_AGES))
    if rng.random() < 0.85:
        ranges = [rng.choice(_RANGE_POOL) for _ in range(rng.choice([0, 1, 5, 90, 360]))]
        sup.on_scan(
            ranges,
            rng.choice([-math.pi, 0.0, math.nan]),
            rng.choice([0.0175, 0.1, 0.0, math.inf]),
            0.05,
            12.0,
            NOW - rng.choice(_AGES),
        )


def _must_stop(sup: SafetySupervisor, cfg: SupervisorConfig) -> bool:
    """Oráculo independiente de las reglas 1-4, leyendo el estado interno."""
    state, cmd = sup._mission, sup._cmd  # noqa: SLF001 - oráculo de prueba
    if cfg.require_mission_status:
        age = NOW - sup._mission_t  # noqa: SLF001
        if state not in _MOVING or not 0.0 <= age <= cfg.mission_timeout_s:
            return True
    elif sup._emergency_latched:  # noqa: SLF001
        return True
    if cmd is None or sup._scan is None:  # noqa: SLF001
        return True
    cmd_age = NOW - sup._cmd_t  # noqa: SLF001
    if not 0.0 <= cmd_age <= cfg.cmd_timeout_s:
        return True
    return not (math.isfinite(cmd.linear_x) and math.isfinite(cmd.angular_z))


def test_property_safe_outputs_and_preconditions() -> None:
    rng = random.Random(1234)  # noqa: S311 - semilla fija, no criptográfico
    stops = 0
    for _ in range(3000):
        cfg = SupervisorConfig(require_mission_status=rng.random() > 0.3)
        sup = SafetySupervisor(cfg)
        _populate(sup, rng)
        if rng.random() < 0.3:
            sup.step(NOW - 0.1)
        out = sup.step(NOW)

        assert math.isfinite(out.command.linear_x)
        assert math.isfinite(out.command.angular_z)
        assert 0.0 <= out.command.linear_x <= cfg.max_linear
        assert abs(out.command.angular_z) <= cfg.max_angular
        assert 0.0 <= out.factor <= 1.0
        if _must_stop(sup, cfg):
            stops += 1
            assert out.command == VelocityCommand(0.0, 0.0)
            assert out.factor == 0.0
    assert 0 < stops < 3000
