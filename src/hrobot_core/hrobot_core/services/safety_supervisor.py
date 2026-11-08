"""Supervisor de seguridad: lógica pura de limitación de velocidad (sin ROS).

Recibe la orden de Nav2, el último scan y el estado de la misión y decide la
velocidad final. Cualquier duda (dato ausente, caducado o inválido) resulta en
velocidad cero (fallo seguro). Unidades: m, rad, s.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum, auto

from hrobot_core.domain.mission import MissionState
from hrobot_core.domain.safety import SafetyConfig, ScanData, speed_factor

_MOVING_STATES = frozenset({MissionState.NAVIGATING_TO_DESTINATION, MissionState.RETURNING_TO_BASE})


@dataclass(frozen=True)
class VelocityCommand:
    """Orden de velocidad (m/s, rad/s)."""

    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class SupervisorConfig:
    """Parámetros del supervisor.

    Attributes:
        max_linear: Tope duro de velocidad lineal (m/s), > 0.
        max_angular: Tope duro de velocidad angular (rad/s), > 0.
        cmd_timeout_s: Edad máxima (s) de la orden de Nav2, > 0.
        mission_timeout_s: Edad máxima (s) del estado de misión, > 0.
        require_mission_status: False solo para pruebas de banco; la
            emergencia nunca se ignora.
        max_rise_per_s: Cuánto puede subir el factor por segundo (bajar es
            instantáneo), > 0.
        max_step_gap_s: Hueco máximo (s) entre dos pasos consecutivos antes de
            considerar que se perdió la supervisión; finito y > 0. Un hueco
            estrictamente mayor reinicia la rampa (el paso se comporta como el
            primero: factor 0.0). Debe ser **mayor que el periodo de paso** del
            nodo (con pasos a 20 Hz, 0.05 s; se recomienda al menos el doble):
            un valor menor que el periodo haría que el robot nunca se mueva.
        safety: Configuración de la política de seguridad por LiDAR.

    Raises:
        ValueError: Si algún valor numérico no es finito o es <= 0.
    """

    max_linear: float = 0.3
    max_angular: float = 1.0
    cmd_timeout_s: float = 0.5
    mission_timeout_s: float = 2.0
    require_mission_status: bool = True
    max_rise_per_s: float = 1.0
    max_step_gap_s: float = 0.25
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def __post_init__(self) -> None:
        values = {
            "max_linear": self.max_linear,
            "max_angular": self.max_angular,
            "cmd_timeout_s": self.cmd_timeout_s,
            "mission_timeout_s": self.mission_timeout_s,
            "max_rise_per_s": self.max_rise_per_s,
            "max_step_gap_s": self.max_step_gap_s,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0, got {value!r}")


class StopReason(Enum):
    """Motivo del resultado del supervisor."""

    NONE = auto()
    EMERGENCY = auto()
    MISSION_NOT_MOVING = auto()
    NO_MISSION_STATUS = auto()
    MISSION_STATUS_STALE = auto()
    NO_COMMAND = auto()
    COMMAND_STALE = auto()
    INVALID_COMMAND = auto()
    NO_SCAN = auto()
    SCAN_BLOCKED = auto()
    RAMP_LIMITED = auto()


@dataclass(frozen=True)
class SupervisorOutput:
    """Resultado de un paso del supervisor."""

    command: VelocityCommand
    factor: float
    reason: StopReason
    reverse_blocked: bool


@dataclass(frozen=True)
class _ScanSnapshot:
    ranges: tuple[float, ...]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


_ZERO = VelocityCommand(0.0, 0.0)


def _require_finite(now_s: float) -> None:
    if not math.isfinite(now_s):
        raise ValueError(f"now_s must be finite, got {now_s!r}")


def _is_stale(age_s: float, timeout_s: float) -> bool:
    return age_s < 0.0 or age_s > timeout_s


class SafetySupervisor:
    """Decide la velocidad final a partir de orden, scan y estado de misión."""

    def __init__(self, cfg: SupervisorConfig | None = None) -> None:
        """Crea el supervisor.

        Args:
            cfg: Configuración; por defecto ``SupervisorConfig()``.
        """
        self._cfg = cfg if cfg is not None else SupervisorConfig()
        self._cmd: VelocityCommand | None = None
        self._cmd_t = 0.0
        self._scan: _ScanSnapshot | None = None
        self._scan_t = 0.0
        self._mission: MissionState | None = None
        self._mission_t = 0.0
        self._emergency_latched = False
        self._prev_factor = 0.0
        self._prev_step_t: float | None = None

    def on_command(self, cmd: VelocityCommand, now_s: float) -> None:
        """Registra la última orden de Nav2."""
        _require_finite(now_s)
        self._cmd = cmd
        self._cmd_t = now_s

    def on_scan(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        now_s: float,
    ) -> None:
        """Registra el último scan (se copia)."""
        _require_finite(now_s)
        self._scan = _ScanSnapshot(tuple(ranges), angle_min, angle_increment, range_min, range_max)
        self._scan_t = now_s

    def on_mission_state(self, state: MissionState | None, now_s: float) -> None:
        """Registra el estado de misión; ``None`` equivale a "sin estado válido"."""
        _require_finite(now_s)
        self._mission = state
        self._mission_t = now_s
        if state is MissionState.EMERGENCY_STOP:
            self._emergency_latched = True
        elif state is not None:
            self._emergency_latched = False

    def step(self, now_s: float) -> SupervisorOutput:
        """Calcula la salida para el instante ``now_s``.

        Si el hueco desde el paso anterior supera ``max_step_gap_s`` (estrictamente)
        se considera pérdida de supervisión: la rampa se reinicia y este paso se
        comporta como el primero (``dt = 0``, factor previo 0.0). Por eso
        ``max_step_gap_s`` debe ser mayor que el periodo de paso del nodo.
        """
        _require_finite(now_s)
        prev_t = self._prev_step_t
        dt = 0.0 if prev_t is None else max(0.0, now_s - prev_t)
        self._prev_step_t = now_s if prev_t is None else max(prev_t, now_s)
        if dt > self._cfg.max_step_gap_s:
            self._prev_factor = 0.0
            dt = 0.0

        reason = self._stop_reason(now_s)
        if reason is not StopReason.NONE or self._cmd is None or self._scan is None:
            return self._stop(reason)
        scan = self._scan
        factor_seg = speed_factor(
            ScanData(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                now_s - self._scan_t,
            ),
            self._cfg.safety,
        )
        if not factor_seg > 0.0:
            return self._stop(StopReason.SCAN_BLOCKED)
        return self._moving(self._cmd, factor_seg, dt)

    def _stop(self, reason: StopReason) -> SupervisorOutput:
        self._prev_factor = 0.0
        return SupervisorOutput(_ZERO, 0.0, reason, False)

    def _mission_reason(self, now_s: float) -> StopReason:
        cfg = self._cfg
        if not cfg.require_mission_status:
            if self._emergency_latched:
                return StopReason.EMERGENCY
            return StopReason.NONE
        if self._mission is None:
            return StopReason.NO_MISSION_STATUS
        if _is_stale(now_s - self._mission_t, cfg.mission_timeout_s):
            return StopReason.MISSION_STATUS_STALE
        if self._mission is MissionState.EMERGENCY_STOP:
            return StopReason.EMERGENCY
        if self._mission not in _MOVING_STATES:
            return StopReason.MISSION_NOT_MOVING
        return StopReason.NONE

    def _stop_reason(self, now_s: float) -> StopReason:
        """Evalúa las reglas 1-4 (la primera que se cumple gana)."""
        reason = self._mission_reason(now_s)
        if reason is not StopReason.NONE:
            return reason
        if self._cmd is None:
            return StopReason.NO_COMMAND
        if _is_stale(now_s - self._cmd_t, self._cfg.cmd_timeout_s):
            return StopReason.COMMAND_STALE
        if not (math.isfinite(self._cmd.linear_x) and math.isfinite(self._cmd.angular_z)):
            return StopReason.INVALID_COMMAND
        if self._scan is None:
            return StopReason.NO_SCAN
        return StopReason.NONE

    def _moving(self, cmd: VelocityCommand, factor_seg: float, dt: float) -> SupervisorOutput:
        cfg = self._cfg
        prev = self._prev_factor
        if factor_seg <= prev:
            factor = factor_seg
        else:
            factor = min(factor_seg, prev + cfg.max_rise_per_s * dt)
        factor = min(1.0, max(0.0, factor))
        self._prev_factor = factor
        reverse = cmd.linear_x < 0.0
        linear = 0.0 if reverse else min(cmd.linear_x, cfg.max_linear)
        angular = max(-cfg.max_angular, min(cfg.max_angular, cmd.angular_z))
        reason = StopReason.RAMP_LIMITED if factor < factor_seg else StopReason.NONE
        # "+ 0.0" normaliza -0.0 a 0.0.
        out = VelocityCommand(linear * factor + 0.0, angular * factor + 0.0)
        return SupervisorOutput(out, factor, reason, reverse)
