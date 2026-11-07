"""Fakes mínimos de rclpy, mensajes ROS y pyserial para probar nodos sin ROS instalado.

Simulan *nuestro uso* de la API, no la API real de ROS 2 Jazzy: un nodo puede pasar sus
tests con estos fakes y fallar en ROS real. Requieren verificación humana con ROS 2.
"""

from __future__ import annotations

import sys
import types
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

NS_PER_S = 1_000_000_000
NODE_MODULE_PREFIX = "hrobot_core.adapters."
NODE_MODULE_SUFFIX = "_node"


def is_node_module(name: str) -> bool:
    """True si ``name`` es un módulo ``hrobot_core.adapters.*_node``."""
    return name.startswith(NODE_MODULE_PREFIX) and name.endswith(NODE_MODULE_SUFFIX)


# --------------------------------------------------------------------------- mensajes
class Time:
    """Equivalente mínimo de builtin_interfaces/Time."""

    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self) -> None:
        self.stamp = Time()
        self.frame_id = ""


class Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist:
    def __init__(self) -> None:
        self.linear = Vector3()
        self.angular = Vector3()


class LaserScan:
    def __init__(self) -> None:
        self.header = Header()
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges: list[float] = []
        self.intensities: list[float] = []


class MissionStatus:
    def __init__(self) -> None:
        self.mission_id = ""
        self.state = ""
        self.delivered = False


# --------------------------------------------------------------------------- QoS
class ReliabilityPolicy:
    RELIABLE = "RELIABLE"
    BEST_EFFORT = "BEST_EFFORT"


class DurabilityPolicy:
    VOLATILE = "VOLATILE"
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"


class QoSProfile:
    def __init__(
        self,
        depth: int = 10,
        reliability: str = ReliabilityPolicy.RELIABLE,
        durability: str = DurabilityPolicy.VOLATILE,
    ) -> None:
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


# --------------------------------------------------------------------------- nodo
class FakeParameter:
    def __init__(self, value: Any) -> None:
        self.value = value


class FakeLogger:
    def __init__(self, sink: list[tuple[str, str]]) -> None:
        self.records = sink

    def _log(self, level: str, msg: str) -> None:
        self.records.append((level, msg))

    def debug(self, msg: str) -> None:
        self._log("debug", msg)

    def info(self, msg: str) -> None:
        self._log("info", msg)

    def warning(self, msg: str) -> None:
        self._log("warning", msg)

    def error(self, msg: str) -> None:
        self._log("error", msg)

    def fatal(self, msg: str) -> None:
        self._log("fatal", msg)


class FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def to_msg(self) -> Time:
        return Time(self.nanoseconds // NS_PER_S, self.nanoseconds % NS_PER_S)


class FakeClock:
    def __init__(self, ros: FakeRos) -> None:
        self._ros = ros

    def now(self) -> FakeTime:
        return FakeTime(self._ros.now_ns)


class FakePublisher:
    def __init__(self, msg_type: Any, topic: str, qos: Any) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class FakeSubscription:
    def __init__(
        self, msg_type: Any, topic: str, callback: Callable[[Any], None], qos: Any
    ) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class FakeTimer:
    def __init__(self, ros: FakeRos, period_s: float, callback: Callable[[], None]) -> None:
        self.period_ns = round(period_s * NS_PER_S)
        self.callback = callback
        self.next_fire_ns = ros.now_ns + self.period_ns
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


# --------------------------------------------------------------------------- serial
class SerialException(OSError):  # noqa: N818 - nombre fijado por pyserial
    """Equivalente de ``serial.SerialException``."""


@dataclass
class SerialControl:
    """Estado inyectable desde el test para ``serial.Serial``."""

    data: bytearray = field(default_factory=bytearray)
    open_error: Exception | None = None
    read_error: Exception | None = None
    instances: list[Any] = field(default_factory=list)


# --------------------------------------------------------------------------- FakeRos
class FakeRos:
    """Estado compartido de los fakes y helpers para el test."""

    def __init__(self) -> None:
        self.now_ns = 0
        self.params: dict[str, Any] = {}
        self.nodes: list[Any] = []
        self.timers: list[FakeTimer] = []
        self.publishers: list[FakePublisher] = []
        self.subscriptions: list[FakeSubscription] = []
        self.log_records: list[tuple[str, str]] = []
        self.serial = SerialControl()
        self.initialized = False
        self.modules: dict[str, types.ModuleType] = {}
        self._build_modules()

    # ---- helpers para tests
    def set_params(self, params: dict[str, Any]) -> None:
        """Sobreescribe valores por defecto de parámetros (antes de crear el nodo)."""
        self.params.update(params)

    def advance(self, seconds: float) -> None:
        """Avanza el reloj falso."""
        self.now_ns += round(seconds * NS_PER_S)

    def fire_timers(self) -> int:
        """Ejecuta una vez cada timer vencido y no cancelado; devuelve cuántos se ejecutaron."""
        fired = 0
        for timer in list(self.timers):
            if not timer.cancelled and self.now_ns >= timer.next_fire_ns:
                timer.next_fire_ns = self.now_ns + timer.period_ns
                timer.callback()
                fired += 1
        return fired

    def deliver(self, topic: str, msg: Any) -> int:
        """Entrega ``msg`` a las suscripciones de ``topic``; devuelve cuántas lo recibieron."""
        subs = [s for s in self.subscriptions if s.topic == topic]
        for sub in subs:
            sub.callback(msg)
        return len(subs)

    def published(self, topic: str) -> list[Any]:
        """Mensajes publicados en ``topic`` (todos los publishers, en orden de creación)."""
        return [m for p in self.publishers if p.topic == topic for m in p.published]

    def logs(self, level: str | None = None) -> list[tuple[str, str]]:
        """Registros ``(nivel, mensaje)``, opcionalmente filtrados por nivel."""
        return [r for r in self.log_records if level is None or r[0] == level]

    def install(self, setitem: Callable[[dict[str, types.ModuleType], str, Any], None]) -> None:
        """Instala los módulos falsos en ``sys.modules`` mediante ``setitem``."""
        for name, module in self.modules.items():
            setitem(sys.modules, name, module)

    # ---- construcción de módulos falsos
    def _mod(self, name: str, **attrs: Any) -> types.ModuleType:
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        self.modules[name] = module
        return module

    def _build_modules(self) -> None:
        self._build_rclpy()
        self._build_messages()
        self._build_serial()

    def _build_rclpy(self) -> None:
        ros = self

        class Node:
            def __init__(self, node_name: str) -> None:
                self.node_name = node_name
                self._declared: dict[str, Any] = {}
                self._logger = FakeLogger(ros.log_records)
                self.destroyed = False
                ros.nodes.append(self)

            def declare_parameter(self, name: str, default: Any = None) -> FakeParameter:
                self._declared[name] = ros.params.get(name, default)
                return FakeParameter(self._declared[name])

            def get_parameter(self, name: str) -> FakeParameter:
                if name not in self._declared:
                    raise KeyError(f"parameter not declared: {name}")
                return FakeParameter(self._declared[name])

            def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> FakePublisher:
                pub = FakePublisher(msg_type, topic, qos)
                ros.publishers.append(pub)
                return pub

            def create_subscription(
                self, msg_type: Any, topic: str, callback: Callable[[Any], None], qos: Any
            ) -> FakeSubscription:
                sub = FakeSubscription(msg_type, topic, callback, qos)
                ros.subscriptions.append(sub)
                return sub

            def create_timer(self, period_s: float, callback: Callable[[], None]) -> FakeTimer:
                timer = FakeTimer(ros, period_s, callback)
                ros.timers.append(timer)
                return timer

            def get_logger(self) -> FakeLogger:
                return self._logger

            def get_clock(self) -> FakeClock:
                return FakeClock(ros)

            def destroy_node(self) -> None:
                self.destroyed = True

        def init(args: Any = None) -> None:
            ros.initialized = True

        def shutdown() -> None:
            ros.initialized = False

        def ok() -> bool:
            return ros.initialized

        def spin(node: Any) -> None:
            ros.fire_timers()

        node_mod = self._mod("rclpy.node", Node=Node)
        qos_mod = self._mod(
            "rclpy.qos",
            qos_profile_sensor_data=QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            QoSProfile=QoSProfile,
            ReliabilityPolicy=ReliabilityPolicy,
            DurabilityPolicy=DurabilityPolicy,
        )
        self._mod(
            "rclpy", init=init, shutdown=shutdown, ok=ok, spin=spin, node=node_mod, qos=qos_mod
        )

    def _build_messages(self) -> None:
        specs: dict[str, dict[str, Any]] = {
            "sensor_msgs": {"LaserScan": LaserScan},
            "geometry_msgs": {"Twist": Twist},
            "hrobot_interfaces": {"MissionStatus": MissionStatus},
        }
        for pkg, classes in specs.items():
            msg_mod = self._mod(f"{pkg}.msg", **classes)
            self._mod(pkg, msg=msg_mod)

    def _build_serial(self) -> None:
        control = self.serial

        class Serial:
            def __init__(self, port: str, baudrate: int = 9600, timeout: float | None = 0) -> None:
                if control.open_error is not None:
                    raise control.open_error
                self.port = port
                self.baudrate = baudrate
                self.timeout = timeout
                self.is_open = True
                control.instances.append(self)

            @property
            def in_waiting(self) -> int:
                return len(control.data)

            def read(self, n: int = 1) -> bytes:
                if control.read_error is not None:
                    raise control.read_error
                chunk = bytes(control.data[:n])
                del control.data[:n]
                return chunk

            def close(self) -> None:
                self.is_open = False

        self._mod("serial", Serial=Serial, SerialException=SerialException)


def purge_node_modules() -> None:
    """Elimina de ``sys.modules`` todo ``hrobot_core.adapters.*_node``."""
    for name in [n for n in sys.modules if is_node_module(n)]:
        del sys.modules[name]
