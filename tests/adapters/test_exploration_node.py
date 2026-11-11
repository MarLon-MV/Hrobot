"""Tests de ``exploration_node`` con fakes de rclpy/Nav2/tf2 (sin ROS real).

Los fakes simulan *nuestro uso* de la API, no la API real de Jazzy: requieren verificación
humana con ROS 2 instalado.
"""

from __future__ import annotations

import ast
import importlib
import sys
import types
from collections.abc import Callable, Iterator
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from hrobot_core.adapters.names import (
    EXPLORATION_STATUS_TOPIC,
    MAP_SAVER_SERVICE,
    MAP_TOPIC,
    NAVIGATE_TO_POSE_ACTION,
)
from ros_fakes import FakeRos

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/hrobot_core/hrobot_core/adapters/exploration_node.py"
STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6
PERIOD_S = 2.0


# --------------------------------------------------------------------------- fakes extra
class FakeFuture:
    def __init__(self) -> None:
        self.done = False
        self._value: Any = None
        self._error: Exception | None = None
        self._callbacks: list[Callable[[FakeFuture], None]] = []
        self.cancelled = False

    def add_done_callback(self, cb: Callable[[FakeFuture], None]) -> None:
        if self.done:
            cb(self)
        else:
            self._callbacks.append(cb)

    def resolve(self, value: Any = None, error: Exception | None = None) -> None:
        self.done = True
        self._value, self._error = value, error
        for cb in self._callbacks:
            cb(self)

    def result(self) -> Any:
        if self._error is not None:
            raise self._error
        return self._value

    def cancel(self) -> None:
        self.cancelled = True


class FakeHandle:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancels = 0

    def get_result_async(self) -> FakeFuture:
        return self.result_future

    def cancel_goal_async(self) -> None:
        self.cancels += 1


class FakeActionClient:
    instances: list[FakeActionClient] = []

    def __init__(self, node: Any, action_type: Any, name: str) -> None:
        self.name = name
        self.ready = True
        self.goals: list[Any] = []
        self.futures: list[FakeFuture] = []
        FakeActionClient.instances.append(self)

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(self, goal: Any) -> FakeFuture:
        self.goals.append(goal)
        fut = FakeFuture()
        self.futures.append(fut)
        return fut


class FakeServiceClient:
    def __init__(self, srv_type: Any, name: str, group: Any) -> None:
        self.name = name
        self.ready = True
        self.requests: list[Any] = []
        self.futures: list[FakeFuture] = []
        self.auto: tuple[Any, Exception | None] | None = (SimpleNamespace(result=True), None)

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, req: Any) -> FakeFuture:
        self.requests.append(req)
        fut = FakeFuture()
        self.futures.append(fut)
        if self.auto is not None:
            fut.resolve(*self.auto)
        return fut


class FakeTransformException(Exception):
    pass


class TfState:
    def __init__(self) -> None:
        self.pose: tuple[float, float] | None = (3.5, 1.5)
        self.lookups: list[tuple[str, str]] = []


class Ext:
    """Todo lo que el nodo necesita además de lo que ya traen los fakes base."""

    def __init__(self, ros: FakeRos) -> None:
        self.ros = ros
        self.tf = TfState()
        self.clients: list[FakeServiceClient] = []
        self.services: dict[str, Callable[[Any, Any], Any]] = {}
        self.executors: list[Any] = []
        FakeActionClient.instances = []

    @property
    def action(self) -> FakeActionClient:
        return FakeActionClient.instances[0]

    @property
    def saver(self) -> FakeServiceClient:
        return self.clients[0]

    def call(self, name: str) -> Any:
        return self.services[name](None, SimpleNamespace(success=None, message=None))


@pytest.fixture
def ext(ros_fakes: FakeRos, monkeypatch: pytest.MonkeyPatch) -> Iterator[Ext]:
    ex = Ext(ros_fakes)
    tf = ex.tf

    base_node = ros_fakes.modules["rclpy.node"].Node

    class ExtNode(base_node):  # type: ignore[misc, valid-type]
        def has_parameter(self, name: str) -> bool:
            return name in self._declared

        def create_client(self, srv_type: Any, name: str, callback_group: Any = None) -> Any:
            client = FakeServiceClient(srv_type, name, callback_group)
            ex.clients.append(client)
            return client

        def create_service(self, srv_type: Any, name: str, callback: Any) -> None:
            ex.services[name] = callback

    class Buffer:
        def lookup_transform(self, target: str, source: str, when: Any) -> Any:
            tf.lookups.append((target, source))
            if tf.pose is None:
                raise FakeTransformException("no tf")
            t = SimpleNamespace(x=tf.pose[0], y=tf.pose[1])
            return SimpleNamespace(transform=SimpleNamespace(translation=t))

    class Executor:
        def __init__(self) -> None:
            self.nodes: list[Any] = []
            self.spun = False
            ex.executors.append(self)

        def add_node(self, node: Any) -> None:
            self.nodes.append(node)

        def spin(self) -> None:
            self.spun = True

    class PoseStamped:
        def __init__(self) -> None:
            self.header = SimpleNamespace(frame_id="", stamp=None)
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )

    class Goal:
        pose: Any = None

    class SaveRequest:
        pass

    class TriggerResponse:
        pass

    class String:
        data = ""

    def mod(name: str, **attrs: Any) -> types.ModuleType:
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    monkeypatch.setattr(ros_fakes.modules["rclpy.node"], "Node", ExtNode)
    monkeypatch.setattr(ros_fakes.modules["geometry_msgs.msg"], "PoseStamped", PoseStamped, False)
    rclpy_mod = ros_fakes.modules["rclpy"]
    time_mod = mod("rclpy.time", Time=lambda: None)
    action_mod = mod("rclpy.action", ActionClient=FakeActionClient)
    group_mod = mod("rclpy.callback_groups", ReentrantCallbackGroup=object)
    exec_mod = mod("rclpy.executors", MultiThreadedExecutor=Executor)
    for sub, m in (
        ("time", time_mod),
        ("action", action_mod),
        ("callback_groups", group_mod),
        ("executors", exec_mod),
    ):
        monkeypatch.setattr(rclpy_mod, sub, m, False)
    mod("action_msgs.msg", GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=STATUS_SUCCEEDED))
    mod("action_msgs")
    mod("nav_msgs.msg", OccupancyGrid=object)
    mod("nav_msgs")
    mod("std_msgs.msg", String=String)
    mod("std_msgs")
    mod("std_srvs.srv", Trigger=SimpleNamespace(Response=TriggerResponse))
    mod("std_srvs")
    mod(
        "tf2_ros",
        Buffer=Buffer,
        TransformListener=lambda b, n: None,
        TransformException=FakeTransformException,
    )
    mod("nav2_msgs.action", NavigateToPose=SimpleNamespace(Goal=Goal))
    mod("nav2_msgs.srv", SaveMap=SimpleNamespace(Request=SaveRequest))
    mod("nav2_msgs")
    yield ex


def _mod() -> Any:
    return importlib.import_module("hrobot_core.adapters.exploration_node")


def _node(ros: FakeRos, **params: Any) -> Any:
    ros.set_params({"min_frontier_size": 1, **params})
    return _mod().ExplorationNode()


def _map_msg(unknown_cols: tuple[int, ...] = (0, 10), width: int = 11, yaw_z: float = 0.0) -> Any:
    top = [100] * width
    for col in unknown_cols:
        top[col] = -1
    data = [*top, *([0] * width), *([100] * width)]
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=yaw_z, w=1.0 if yaw_z == 0.0 else 0.0),
    )
    info = SimpleNamespace(width=width, height=3, resolution=1.0, origin=origin)
    return SimpleNamespace(info=info, data=data)


def _cycle(ros: FakeRos) -> None:
    ros.advance(PERIOD_S)
    ros.fire_timers()


def _status(ros: FakeRos) -> str:
    return str(ros.published(EXPLORATION_STATUS_TOPIC)[-1].data)


# --------------------------------------------------------------------------- conversión
def test_occupancy_to_grid_converts_fields(ext: Ext) -> None:
    grid = _mod().occupancy_to_grid(_map_msg())
    assert (grid.width, grid.height, grid.resolution) == (11, 3, 1.0)
    assert (grid.origin_x, grid.origin_y) == (0.0, 0.0)
    assert grid.data[0] == -1
    assert len(grid.data) == 33


@pytest.mark.parametrize("mutate", ["rotated", "bad_size", "bad_res"])
def test_occupancy_to_grid_rejects_invalid(ext: Ext, mutate: str) -> None:
    msg = _map_msg(yaw_z=1.0 if mutate == "rotated" else 0.0)
    if mutate == "bad_size":
        msg.data = msg.data[:-1]
    if mutate == "bad_res":
        msg.info.resolution = float("nan")
    with pytest.raises(ValueError):
        _mod().occupancy_to_grid(msg)


# --------------------------------------------------------------------------- parámetros
def test_parameters_declared_with_defaults(ext: Ext, ros_fakes: FakeRos) -> None:
    ros_fakes.set_params({})
    node = _mod().ExplorationNode()
    declared = node._declared
    assert declared == {
        "min_frontier_size": 8,
        "blacklist_radius": 0.5,
        "min_goal_distance": 0.4,
        "exploration_period_s": 2.0,
        "max_consecutive_failures": 5,
        "no_frontier_confirmations": 3,
        "map_save_name": "map",
        "map_save_timeout_s": 10.0,
        "use_sim_time": False,
    }
    assert isinstance(declared["min_frontier_size"], int)
    assert isinstance(declared["exploration_period_s"], float)


@pytest.mark.parametrize(
    "bad",
    [
        {"max_consecutive_failures": 2.5},
        {"no_frontier_confirmations": True},
        {"min_frontier_size": 0},
        {"exploration_period_s": 0.0},
        {"exploration_period_s": float("nan")},
        {"blacklist_radius": -1.0},
        {"map_save_timeout_s": True},
        {"map_save_name": 3},
        {"map_save_name": "  "},
    ],
)
def test_invalid_parameters_fail_at_startup(
    ext: Ext, ros_fakes: FakeRos, bad: dict[str, Any]
) -> None:
    with pytest.raises(ValueError):
        _node(ros_fakes, **bad)


def test_node_wiring(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    sub = next(s for s in ros_fakes.subscriptions if s.topic == MAP_TOPIC)
    assert sub.qos.durability == "TRANSIENT_LOCAL"
    assert sub.qos.reliability == "RELIABLE"
    assert ext.action.name == NAVIGATE_TO_POSE_ACTION
    assert ext.saver.name == MAP_SAVER_SERVICE
    assert set(ext.services) == {"~/start", "~/stop"}
    assert [p.topic for p in ros_fakes.publishers] == [EXPLORATION_STATUS_TOPIC]
    assert ros_fakes.timers[0].period_ns == 2_000_000_000


def test_only_publisher_is_status_and_no_sleep_or_print() -> None:
    tree = ast.parse(SRC.read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    publishers = [
        c for c in calls if isinstance(c.func, ast.Attribute) and c.func.attr == "create_publisher"
    ]
    assert len(publishers) == 1
    assert ast.unparse(publishers[0].args[1]) == "EXPLORATION_STATUS_TOPIC"
    names = {ast.unparse(c.func) for c in calls}
    assert not names & {"print", "sleep", "time.sleep"}


# --------------------------------------------------------------------------- flujo
def test_idle_until_started_then_sends_goal_to_nav2(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    _cycle(ros_fakes)
    assert ext.action.goals == []
    assert _status(ros_fakes) == "IDLE"
    res = ext.call("~/start")
    assert res.success is True
    _cycle(ros_fakes)
    assert len(ext.action.goals) == 1
    pose = ext.action.goals[0].pose
    assert pose.header.frame_id == "map"
    assert pose.pose.orientation.w == 1.0
    assert pose.pose.position.x < 3.5
    assert _status(ros_fakes) == "EXPLORING"
    _cycle(ros_fakes)
    assert len(ext.action.goals) == 1  # objetivo en curso: no reenvía
    assert ext.tf.lookups[0] == ("map", "base_link")


def test_full_run_to_done_saves_map(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes, map_save_name="hospital")
    ext.call("~/start")
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    _cycle(ros_fakes)
    ext.action.futures[0].resolve(FakeHandle())
    handle = ext.action.futures[0].result()
    handle.result_future.resolve(SimpleNamespace(status=STATUS_SUCCEEDED))
    ros_fakes.deliver(MAP_TOPIC, _map_msg(unknown_cols=()))
    for _ in range(3):
        _cycle(ros_fakes)
    assert ext.saver.requests[0].map_url == "hospital"
    assert ext.saver.requests[0].map_topic == MAP_TOPIC
    assert _status(ros_fakes) == "DONE: map saved"


def test_aborted_goal_is_blacklisted_and_next_one_sent(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ext.call("~/start")
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    _cycle(ros_fakes)
    handle = FakeHandle()
    ext.action.futures[0].resolve(handle)
    handle.result_future.resolve(SimpleNamespace(status=STATUS_ABORTED))
    _cycle(ros_fakes)
    first, second = (g.pose.pose.position.x for g in ext.action.goals)
    assert first < 3.5 < second


def test_stop_cancels_goal_and_ignores_late_result(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ext.call("~/start")
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    _cycle(ros_fakes)
    handle = FakeHandle()
    ext.action.futures[0].resolve(handle)
    res = ext.call("~/stop")
    assert res.success is True
    assert handle.cancels == 1
    handle.result_future.resolve(SimpleNamespace(status=STATUS_ABORTED))
    _cycle(ros_fakes)
    assert _status(ros_fakes) == "IDLE: stopped"


def test_start_twice_reports_failure(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    assert ext.call("~/start").success is True
    second = ext.call("~/start")
    assert second.success is False
    assert "EXPLORING" in second.message


# --------------------------------------------------------------------------- degradación
def test_missing_map_warns_once_and_does_nothing(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ext.call("~/start")
    for _ in range(4):
        _cycle(ros_fakes)
    assert ext.action.goals == []
    assert len(ros_fakes.logs("warning")) == 1
    assert ext.tf.lookups == []


def test_missing_tf_warns_once_then_recovers(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ext.call("~/start")
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    ext.tf.pose = None
    for _ in range(3):
        _cycle(ros_fakes)
    assert ext.action.goals == []
    assert len(ros_fakes.logs("warning")) == 1
    ext.tf.pose = (3.5, 1.5)
    _cycle(ros_fakes)
    assert len(ext.action.goals) == 1
    ext.tf.pose = None
    ext.action.goals.clear()
    handle = FakeHandle()
    ext.action.futures[0].resolve(handle)
    handle.result_future.resolve(SimpleNamespace(status=STATUS_SUCCEEDED))
    _cycle(ros_fakes)
    assert len(ros_fakes.logs("warning")) == 2  # volvió a aparecer tras resolverse


def test_invalid_map_is_rejected_with_single_warning(ext: Ext, ros_fakes: FakeRos) -> None:
    _node(ros_fakes)
    ext.call("~/start")
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    bad = _map_msg(yaw_z=1.0)
    ros_fakes.deliver(MAP_TOPIC, bad)
    ros_fakes.deliver(MAP_TOPIC, bad)
    _cycle(ros_fakes)
    assert ext.action.goals == []
    warnings = [m for _, m in ros_fakes.logs("warning") if "invalid /map" in m]
    assert len(warnings) == 1
    ros_fakes.deliver(MAP_TOPIC, _map_msg())
    ros_fakes.deliver(MAP_TOPIC, bad)
    assert len([m for _, m in ros_fakes.logs("warning") if "invalid /map" in m]) == 2


# --------------------------------------------------------------------------- Nav2Navigator
def _navigator(ext: Ext) -> tuple[Any, list[bool], Any]:
    results: list[bool] = []
    node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warning=lambda _m: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")),
    )
    client = FakeActionClient(node, None, "x")
    nav = _mod().Nav2Navigator(node, client, results.append, "map")
    return nav, results, client


def _point(x: float = 1.0, y: float = 2.0) -> Any:
    from hrobot_core.domain.exploration import Point2D

    return Point2D(x, y)


def test_navigator_server_not_ready_reports_failure(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    client.ready = False
    nav.send_goal(_point())
    assert results == [False]
    assert client.goals == []


def test_navigator_rejected_goal_reports_failure(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.send_goal(_point())
    client.futures[0].resolve(FakeHandle(accepted=False))
    assert results == [False]


def test_navigator_send_error_reports_failure(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.send_goal(_point())
    client.futures[0].resolve(error=RuntimeError("boom"))
    assert results == [False]


def test_navigator_result_error_reports_failure(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.send_goal(_point())
    handle = FakeHandle()
    client.futures[0].resolve(handle)
    handle.result_future.resolve(error=RuntimeError("boom"))
    assert results == [False]


def test_navigator_cancel_before_response_cancels_late_handle(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.send_goal(_point())
    nav.cancel()
    late = FakeHandle()
    client.futures[0].resolve(late)
    assert late.cancels == 1
    assert results == []


def test_navigator_stale_results_are_ignored(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.send_goal(_point())
    old = FakeHandle()
    client.futures[0].resolve(old)
    nav.send_goal(_point(5.0, 5.0))  # reemplaza al anterior
    old.result_future.resolve(SimpleNamespace(status=STATUS_SUCCEEDED))
    assert results == []
    # una respuesta tardía rechazada/errónea de un objetivo obsoleto tampoco informa
    nav.cancel()
    client.futures[1].resolve(FakeHandle(accepted=False))
    assert results == []


def test_navigator_stale_error_ignored_and_cancel_without_goal(ext: Ext) -> None:
    nav, results, client = _navigator(ext)
    nav.cancel()  # sin objetivo: no falla
    nav.send_goal(_point())
    nav.cancel()
    client.futures[0].resolve(error=RuntimeError("late"))
    assert results == []


# --------------------------------------------------------------------------- Nav2MapSaver
def _saver(ext: Ext, timeout_s: float = 0.5) -> tuple[Any, FakeServiceClient]:
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=lambda _m: None))
    client = FakeServiceClient(None, "x", None)
    return _mod().Nav2MapSaver(node, client, timeout_s), client


def test_saver_success_and_request_fields(ext: Ext) -> None:
    saver, client = _saver(ext)
    assert saver.save("m") is True
    req = client.requests[0]
    assert (req.map_url, req.map_topic) == ("m", MAP_TOPIC)
    assert req.map_mode == "trinary"
    assert req.free_thresh == pytest.approx(0.25)
    assert req.occupied_thresh == pytest.approx(0.65)


def test_saver_negative_response(ext: Ext) -> None:
    saver, client = _saver(ext)
    client.auto = (SimpleNamespace(result=False), None)
    assert saver.save("m") is False


def test_saver_service_not_ready(ext: Ext) -> None:
    saver, client = _saver(ext)
    client.ready = False
    assert saver.save("m") is False
    assert client.requests == []


def test_saver_timeout(ext: Ext) -> None:
    saver, client = _saver(ext, timeout_s=0.01)
    client.auto = None
    assert saver.save("m") is False
    assert client.futures[0].cancelled


def test_saver_future_error(ext: Ext) -> None:
    saver, client = _saver(ext)
    client.auto = (None, RuntimeError("boom"))
    assert saver.save("m") is False


# --------------------------------------------------------------------------- main
def test_main_spins_multithreaded_and_cleans_up(ext: Ext, ros_fakes: FakeRos) -> None:
    mod = _mod()
    ros_fakes.set_params({"min_frontier_size": 1})
    mod.main([])
    assert ext.executors[0].spun is True
    assert ros_fakes.nodes[0].destroyed is True
    assert ros_fakes.initialized is False


def test_main_cleans_up_when_node_creation_fails(ext: Ext, ros_fakes: FakeRos) -> None:
    ros_fakes.set_params({"exploration_period_s": -1.0})
    with pytest.raises(ValueError):
        _mod().main([])
    assert ros_fakes.initialized is False
