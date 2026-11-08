"""Tests de ``supervisor_node`` con fakes de rclpy (sin ROS real)."""

from __future__ import annotations

import ast
import importlib
import math
from pathlib import Path
from typing import Any

import pytest
from hrobot_core.adapters.names import (
    CMD_VEL_TOPIC,
    MISSION_STATUS_TOPIC,
    NAV_CMD_VEL_TOPIC,
    SCAN_TOPIC,
)
from ros_fakes import FakeRos, LaserScan, MissionStatus, Twist

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/hrobot_core/hrobot_core/adapters/supervisor_node.py"
SETUP = ROOT / "src/hrobot_core/setup.py"
TICK = 0.05


def _mod() -> Any:
    return importlib.import_module("hrobot_core.adapters.supervisor_node")


def _node(ros: FakeRos, **params: Any) -> Any:
    ros.set_params(params)
    return _mod().SupervisorNode()


def _scan(dist: float = 5.0) -> LaserScan:
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = math.radians(1.0)
    msg.range_min = 0.05
    msg.range_max = 12.0
    msg.ranges = [dist] * 360
    return msg


def _twist(x: float, z: float) -> Twist:
    msg = Twist()
    msg.linear.x = x
    msg.angular.z = z
    return msg


def _mission(state: str) -> MissionStatus:
    msg = MissionStatus()
    msg.state = state
    return msg


def _feed(ros: FakeRos, state: str = "NAVIGATING_TO_DESTINATION", dist: float = 5.0) -> None:
    ros.deliver(MISSION_STATUS_TOPIC, _mission(state))
    ros.deliver(NAV_CMD_VEL_TOPIC, _twist(0.2, 0.5))
    ros.deliver(SCAN_TOPIC, _scan(dist))


def _tick(ros: FakeRos, n: int = 1, refresh: bool = False, **kw: Any) -> None:
    for _ in range(n):
        ros.advance(TICK)
        if refresh:
            _feed(ros, **kw)
        ros.fire_timers()


def _last(ros: FakeRos) -> Any:
    return ros.published(CMD_VEL_TOPIC)[-1]


def _reason_logs(ros: FakeRos) -> list[str]:
    return [m for _, m in ros.logs() if "supervisor reason" in m]


def test_no_input_publishes_zeros_each_tick(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 10)
    out = ros_fakes.published(CMD_VEL_TOPIC)
    assert len(out) == 10
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in out)


def test_happy_path_ramps_up(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    speeds = []
    for _ in range(40):
        _tick(ros_fakes, refresh=True)
        speeds.append(_last(ros_fakes).linear.x)
    assert speeds[0] == 0.0
    assert any(0.0 < s < 0.2 for s in speeds)
    assert speeds == sorted(speeds)
    assert _last(ros_fakes).linear.x == pytest.approx(0.2)
    assert _last(ros_fakes).angular.z == pytest.approx(0.5)


def test_obstacle_at_stop_distance_stops(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 40, refresh=True)
    assert _last(ros_fakes).linear.x > 0.0
    _tick(ros_fakes, refresh=True, dist=0.35)
    assert _last(ros_fakes).linear.x == 0.0
    assert _last(ros_fakes).angular.z == 0.0


def test_stale_nav_command_stops(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 40, refresh=True)
    assert _last(ros_fakes).linear.x > 0.0
    ros_fakes.advance(0.6)
    ros_fakes.deliver(MISSION_STATUS_TOPIC, _mission("NAVIGATING_TO_DESTINATION"))
    ros_fakes.deliver(SCAN_TOPIC, _scan())
    ros_fakes.fire_timers()
    assert _last(ros_fakes).linear.x == 0.0


def test_emergency_stop_then_resume_with_ramp(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 40, refresh=True)
    _tick(ros_fakes, 5, refresh=True, state="EMERGENCY_STOP")
    assert all(m.linear.x == 0.0 for m in ros_fakes.published(CMD_VEL_TOPIC)[-5:])
    _tick(ros_fakes, 1, refresh=True)
    assert 0.0 < _last(ros_fakes).linear.x < 0.05
    _tick(ros_fakes, 3, refresh=True)
    assert 0.03 < _last(ros_fakes).linear.x < 0.2


def test_unknown_mission_state_warns_and_stops(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 10, refresh=True, state="BOGUS")
    assert _last(ros_fakes).linear.x == 0.0
    assert any("BOGUS" in m for _, m in ros_fakes.logs("warning"))


def test_reverse_is_blocked_and_logged_once(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    for _ in range(10):
        ros_fakes.advance(TICK)
        ros_fakes.deliver(MISSION_STATUS_TOPIC, _mission("RETURNING_TO_BASE"))
        ros_fakes.deliver(NAV_CMD_VEL_TOPIC, _twist(-0.2, 0.5))
        ros_fakes.deliver(SCAN_TOPIC, _scan())
        ros_fakes.fire_timers()
        assert _last(ros_fakes).linear.x == 0.0
    assert len([1 for _, m in ros_fakes.logs("warning") if "reverse_blocked" in m]) == 1


@pytest.mark.parametrize(
    "params",
    [
        {"max_linear": -1.0},
        {"publish_rate_hz": 0.0},
        {"publish_rate_hz": 500.0},
        {"publish_rate_hz": float("nan")},
        {"max_linear": "fast"},
        {"max_linear": True},
        {"require_mission_status": 1},
        {"stop_distance": 2.0},
    ],
)
def test_invalid_parameters_raise(ros_fakes: FakeRos, params: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        _node(ros_fakes, **params)


def test_cross_constraint_gap_vs_rate(ros_fakes: FakeRos) -> None:
    with pytest.raises(ValueError):
        _node(ros_fakes, publish_rate_hz=20.0, max_step_gap_s=0.05)
    _node(ros_fakes, publish_rate_hz=20.0, max_step_gap_s=0.1)
    _node(ros_fakes, publish_rate_hz=20.0, max_step_gap_s=0.25)


def test_pause_restarts_ramp(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 40, refresh=True)
    assert _last(ros_fakes).linear.x == pytest.approx(0.2)
    ros_fakes.advance(5.0)
    _feed(ros_fakes)
    ros_fakes.fire_timers()
    assert _last(ros_fakes).linear.x == 0.0
    _tick(ros_fakes, 2, refresh=True)
    assert 0.0 < _last(ros_fakes).linear.x < 0.2


def test_new_params_declared_and_respected(ros_fakes: FakeRos) -> None:
    node = _node(ros_fakes, max_invalid_run_rad=0.5, max_step_gap_s=0.5)
    assert node.get_parameter("max_invalid_run_rad").value == 0.5
    assert node.get_parameter("max_step_gap_s").value == 0.5
    assert node._supervisor._cfg.safety.max_invalid_run_rad == 0.5
    assert node._supervisor._cfg.max_step_gap_s == 0.5


def test_default_params_match_configs(ros_fakes: FakeRos) -> None:
    node = _node(ros_fakes)
    assert node.get_parameter("max_invalid_run_rad").value == pytest.approx(math.radians(8.0))
    assert node.get_parameter("max_step_gap_s").value == 0.25
    assert node.get_parameter("publish_rate_hz").value == 20.0


def test_overridden_max_linear_respected(ros_fakes: FakeRos) -> None:
    _node(ros_fakes, max_linear=0.1)
    _tick(ros_fakes, 40, refresh=True)
    assert _last(ros_fakes).linear.x == pytest.approx(0.1)


def test_main_publishes_zeros_in_finally(
    ros_fakes: FakeRos, monkeypatch: pytest.MonkeyPatch
) -> None:
    import rclpy

    def boom(node: Any) -> None:
        raise RuntimeError("spin failed")

    monkeypatch.setattr(rclpy, "spin", boom)
    with pytest.raises(RuntimeError):
        _mod().main([])
    assert len(ros_fakes.published(CMD_VEL_TOPIC)) == 1
    assert _last(ros_fakes).linear.x == 0.0
    assert ros_fakes.nodes[0].destroyed
    assert not ros_fakes.initialized


def test_main_with_invalid_params_shuts_down(ros_fakes: FakeRos) -> None:
    ros_fakes.set_params({"max_linear": -1.0})
    with pytest.raises(ValueError):
        _mod().main([])
    assert not ros_fakes.initialized
    assert ros_fakes.published(CMD_VEL_TOPIC) == []


def test_main_skips_shutdown_if_not_ok(ros_fakes: FakeRos, monkeypatch: pytest.MonkeyPatch) -> None:
    import rclpy

    calls: list[int] = []
    monkeypatch.setattr(rclpy, "ok", lambda: False)
    monkeypatch.setattr(rclpy, "shutdown", lambda: calls.append(1))
    monkeypatch.setattr(rclpy, "spin", lambda node: None)
    _mod().main([])
    assert calls == []


def test_reason_logged_only_on_change(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    _tick(ros_fakes, 20)
    assert len(_reason_logs(ros_fakes)) == 1
    assert ros_fakes.logs("warning")
    _tick(ros_fakes, 40, refresh=True)
    reasons = _reason_logs(ros_fakes)
    assert reasons[-1].endswith("NONE")
    assert ("info", reasons[-1]) in ros_fakes.logs("info")
    before = len(reasons)
    _tick(ros_fakes, 10, refresh=True)
    assert len(_reason_logs(ros_fakes)) == before


def test_published_twist_fields_are_exact_floats(ros_fakes: FakeRos) -> None:
    _node(ros_fakes, max_linear=1)
    _tick(ros_fakes, 3)
    _tick(ros_fakes, 40, refresh=True)
    for m in ros_fakes.published(CMD_VEL_TOPIC):
        for v in (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z):
            assert type(v) is float


def test_publisher_and_qos(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    pubs = [p for p in ros_fakes.publishers if p.topic == CMD_VEL_TOPIC]
    assert len(pubs) == 1
    assert (pubs[0].qos.reliability, pubs[0].qos.depth) == ("RELIABLE", 1)
    subs = {s.topic: s for s in ros_fakes.subscriptions}
    assert set(subs) == {NAV_CMD_VEL_TOPIC, SCAN_TOPIC, MISSION_STATUS_TOPIC}
    nav = subs[NAV_CMD_VEL_TOPIC].qos
    assert (nav.reliability, nav.depth, nav.durability) == ("RELIABLE", 1, "VOLATILE")
    assert subs[SCAN_TOPIC].qos is importlib.import_module("rclpy.qos").qos_profile_sensor_data
    mis = subs[MISSION_STATUS_TOPIC].qos
    assert (mis.reliability, mis.depth, mis.durability) == ("RELIABLE", 1, "TRANSIENT_LOCAL")


def test_namespaced_cmd_vel_topic(ros_fakes: FakeRos) -> None:
    _node(ros_fakes, cmd_vel_topic="/r2/cmd_vel")
    topics = [p.topic for p in ros_fakes.publishers]
    assert topics == ["/r2/cmd_vel"]
    assert "/cmd_vel" not in topics


def test_default_cmd_vel_topic(ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    assert [p.topic for p in ros_fakes.publishers] == [CMD_VEL_TOPIC]


@pytest.mark.parametrize("topic", ["", "/", "cmd_vel", "r2/cmd_vel", "/r2/cmd vel", 5, None])
def test_invalid_cmd_vel_topic_raises(ros_fakes: FakeRos, topic: Any) -> None:
    with pytest.raises(ValueError):
        _node(ros_fakes, cmd_vel_topic=topic)
    assert ros_fakes.publishers == []


def test_timer_period_matches_rate(ros_fakes: FakeRos) -> None:
    _node(ros_fakes, publish_rate_hz=10.0)
    assert ros_fakes.timers[0].period_ns == 100_000_000


def test_static_source_rules() -> None:
    tree = ast.parse(SRC.read_text(encoding="utf-8"))
    pubs = 0
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            assert not (isinstance(f, ast.Name) and f.id == "print")
            assert not (isinstance(f, ast.Attribute) and f.attr == "sleep")
            if isinstance(f, ast.Attribute) and f.attr == "create_publisher":
                assert isinstance(n.args[1], ast.Name)
                assert n.args[1].id == "cmd_vel_topic"  # validado desde el parametro
                pubs += 1
    assert pubs == 1


def test_entry_point_registered() -> None:
    text = SETUP.read_text(encoding="utf-8")
    assert "supervisor_node = hrobot_core.adapters.supervisor_node:main" in text
