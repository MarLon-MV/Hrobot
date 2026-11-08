"""Nodo del supervisor de seguridad: único publicador de ``/cmd_vel``.

Capa delgada sobre ``SafetySupervisor``: traduce mensajes ROS a tipos de dominio y
publica a ritmo fijo (incluso ceros) la velocidad ya limitada.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from hrobot_core.adapters.names import (
    CMD_VEL_TOPIC,
    MISSION_STATUS_TOPIC,
    NAV_CMD_VEL_TOPIC,
    SCAN_TOPIC,
)
from hrobot_core.domain.mission import MissionState
from hrobot_core.domain.safety import SafetyConfig
from hrobot_core.services.safety_supervisor import (
    SafetySupervisor,
    StopReason,
    SupervisorConfig,
    VelocityCommand,
)
from hrobot_interfaces.msg import MissionStatus

NODE_NAME = "supervisor_node"
QOS_DEPTH = 1
NS_PER_S = 1e9
MAX_PUBLISH_RATE_HZ = 100.0
STEP_PERIODS_MIN_GAP = 2.0
_QUIET_REASONS = frozenset({StopReason.NONE, StopReason.RAMP_LIMITED})

_SUP_DEFAULTS = SupervisorConfig()
_SAFETY_DEFAULTS = SafetyConfig()
_DEFAULT_RATE_HZ = 20.0

_FLOAT_PARAMS: dict[str, float] = {
    "max_linear": _SUP_DEFAULTS.max_linear,
    "max_angular": _SUP_DEFAULTS.max_angular,
    "cmd_timeout_s": _SUP_DEFAULTS.cmd_timeout_s,
    "mission_timeout_s": _SUP_DEFAULTS.mission_timeout_s,
    "max_rise_per_s": _SUP_DEFAULTS.max_rise_per_s,
    "max_step_gap_s": _SUP_DEFAULTS.max_step_gap_s,
    "stop_distance": _SAFETY_DEFAULTS.stop_distance,
    "slow_distance": _SAFETY_DEFAULTS.slow_distance,
    "front_half_angle": _SAFETY_DEFAULTS.front_half_angle,
    "stale_timeout": _SAFETY_DEFAULTS.stale_timeout,
    "min_valid_ratio": _SAFETY_DEFAULTS.min_valid_ratio,
    "degraded_factor": _SAFETY_DEFAULTS.degraded_factor,
    "max_invalid_run_rad": _SAFETY_DEFAULTS.max_invalid_run_rad,
    "publish_rate_hz": _DEFAULT_RATE_HZ,
}
_SAFETY_KEYS = (
    "stop_distance",
    "slow_distance",
    "front_half_angle",
    "stale_timeout",
    "min_valid_ratio",
    "degraded_factor",
    "max_invalid_run_rad",
)
_SUPERVISOR_KEYS = (
    "max_linear",
    "max_angular",
    "cmd_timeout_s",
    "mission_timeout_s",
    "max_rise_per_s",
    "max_step_gap_s",
)
_REQUIRE_MISSION_PARAM = "require_mission_status"


def _as_float(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"parameter {name} must be a number, got {value!r}")
    return float(value)


def build_config(values: dict[str, Any]) -> tuple[SupervisorConfig, float]:
    """Construye la configuración del supervisor y la frecuencia de publicación.

    Args:
        values: Valores de parámetros por nombre.

    Returns:
        ``(SupervisorConfig, publish_rate_hz)``.

    Raises:
        ValueError: Si algún parámetro es inválido o incumple la restricción cruzada.
    """
    nums = {k: _as_float(k, values[k]) for k in _FLOAT_PARAMS}
    require = values[_REQUIRE_MISSION_PARAM]
    if not isinstance(require, bool):
        raise ValueError(f"parameter {_REQUIRE_MISSION_PARAM} must be bool, got {require!r}")
    rate = nums["publish_rate_hz"]
    if not math.isfinite(rate) or not 0.0 < rate <= MAX_PUBLISH_RATE_HZ:
        raise ValueError(f"publish_rate_hz must be in (0, {MAX_PUBLISH_RATE_HZ}], got {rate!r}")
    cfg = SupervisorConfig(
        require_mission_status=require,
        safety=SafetyConfig(**{k: nums[k] for k in _SAFETY_KEYS}),
        **{k: nums[k] for k in _SUPERVISOR_KEYS},
    )
    if cfg.max_step_gap_s < STEP_PERIODS_MIN_GAP / rate:
        raise ValueError(
            f"max_step_gap_s ({cfg.max_step_gap_s}) must be >= {STEP_PERIODS_MIN_GAP} / "
            f"publish_rate_hz ({STEP_PERIODS_MIN_GAP / rate})"
        )
    return cfg, rate


def _make_twist(linear_x: float, angular_z: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(angular_z)
    return msg


class SupervisorNode(Node):
    """Nodo ROS 2 que limita y publica ``/cmd_vel`` a ritmo fijo."""

    def __init__(self) -> None:
        """Declara parámetros, valida y crea suscripciones, publisher y timer.

        Raises:
            ValueError: Si algún parámetro es inválido (el nodo falla al arrancar).
        """
        super().__init__(NODE_NAME)
        defaults: dict[str, Any] = {**_FLOAT_PARAMS, _REQUIRE_MISSION_PARAM: True}
        values = {
            name: self.declare_parameter(name, default).value for name, default in defaults.items()
        }
        cfg, rate = build_config(values)
        self._supervisor = SafetySupervisor(cfg)
        self._last_reason: StopReason | None = None
        self._last_reverse = False

        reliable = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.RELIABLE)
        mission_qos = QoSProfile(
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Twist, CMD_VEL_TOPIC, reliable)
        self.create_subscription(Twist, NAV_CMD_VEL_TOPIC, self._on_nav_cmd, reliable)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(MissionStatus, MISSION_STATUS_TOPIC, self._on_mission, mission_qos)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _now_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) / NS_PER_S

    def _on_nav_cmd(self, msg: Twist) -> None:
        cmd = VelocityCommand(msg.linear.x, msg.angular.z)
        self._supervisor.on_command(cmd, self._now_s())

    def _on_scan(self, msg: LaserScan) -> None:
        self._supervisor.on_scan(
            list(msg.ranges),
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            self._now_s(),
        )

    def _on_mission(self, msg: MissionStatus) -> None:
        state: MissionState | None
        try:
            state = MissionState[msg.state]
        except KeyError:
            self.get_logger().warning(f"unknown mission state {msg.state!r}; treated as none")
            state = None
        self._supervisor.on_mission_state(state, self._now_s())

    def _on_timer(self) -> None:
        out = self._supervisor.step(self._now_s())
        self._log_changes(out.reason, out.reverse_blocked)
        self._pub.publish(_make_twist(out.command.linear_x, out.command.angular_z))

    def _log_changes(self, reason: StopReason, reverse_blocked: bool) -> None:
        if reason is not self._last_reason:
            self._last_reason = reason
            text = f"supervisor reason: {reason.name}"
            if reason in _QUIET_REASONS:
                self.get_logger().info(text)
            else:
                self.get_logger().warning(text)
        if reverse_blocked and not self._last_reverse:
            self.get_logger().warning("reverse_blocked: negative linear.x command ignored")
        self._last_reverse = reverse_blocked

    def publish_stop(self) -> None:
        """Publica una orden de velocidad cero."""
        self._pub.publish(_make_twist(0.0, 0.0))


def main(args: list[str] | None = None) -> None:
    """Punto de entrada: arranca el nodo y publica ceros al terminar."""
    rclpy.init(args=args)
    node: SupervisorNode | None = None
    try:
        node = SupervisorNode()
        rclpy.spin(node)
    finally:
        try:
            if node is not None:
                node.publish_stop()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
