"""Gestor de exploración autónoma por fronteras (sin ROS).

Máquina de estados ``IDLE -> UNDOCKING -> EXPLORING <-> PAUSED -> DONE | FAILED``. Elige la
siguiente frontera con la lógica pura de ``domain.exploration`` y delega el movimiento, el
undock y el guardado del mapa en puertos (``Protocol``) que implementa la capa ``adapters``.
Condiciones de seguridad del Create 3 (política conservadora ante datos desconocidos): no se
explora en la base, se pausa si el robot es levantado, se termina con batería baja y los
peligros (bump/cliff/stuck) cancelan el objetivo y cuentan como fallo.
"""

from __future__ import annotations

import functools
import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol, TypeVar, cast

from hrobot_core.domain.exploration import (
    ExplorationConfig,
    GridMap,
    Point2D,
    find_frontiers,
    select_frontier,
)

_LOG = logging.getLogger(__name__)

DEFAULT_MAX_CONSECUTIVE_FAILURES = 5
DEFAULT_NO_FRONTIER_CONFIRMATIONS = 3
DEFAULT_MAP_SAVE_NAME = "map"
DEFAULT_UNDOCK_TIMEOUT_S = 30.0
DEFAULT_STATUS_TIMEOUT_S = 10.0
DEFAULT_KIDNAP_CLEAR_CYCLES = 3
DEFAULT_MIN_BATTERY_PCT = 20.0
MAX_BATTERY_PCT = 100.0

REASON_NONE = ""
REASON_STOPPED = "stopped"
REASON_MAP_SAVED = "map saved"
REASON_SAVE_FAILED = "map save failed"
REASON_TOO_MANY_FAILURES = "too many consecutive goal failures"
REASON_LOW_BATTERY = "LOW_BATTERY"
REASON_UNDOCK_FAILED = "undock failed"
REASON_UNDOCK_TIMEOUT = "undock timeout"
REASON_STATUS_TIMEOUT = "robot status timeout"


class ExplorationState(Enum):
    """Estado del gestor de exploración."""

    IDLE = "IDLE"
    UNDOCKING = "UNDOCKING"
    EXPLORING = "EXPLORING"
    PAUSED = "PAUSED"
    DONE = "DONE"
    FAILED = "FAILED"


_ACTIVE_STATES = frozenset(
    {ExplorationState.UNDOCKING, ExplorationState.EXPLORING, ExplorationState.PAUSED}
)


class InvalidTransitionError(RuntimeError):
    """Transición de estado no permitida (p. ej. ``start()`` en ``EXPLORING``)."""


class NavigatorPort(Protocol):
    """Puerto de navegación hacia un objetivo en el frame ``map``."""

    def send_goal(self, goal: Point2D) -> None:
        """Envía un objetivo (metros, frame ``map``)."""

    def cancel(self) -> None:
        """Cancela el objetivo activo, si lo hay."""


class DockPort(Protocol):
    """Puerto de la base de carga (undock del Create 3)."""

    def undock(self) -> None:
        """Solicita el undock; el resultado se informa con ``on_undock_result``."""

    def cancel(self) -> None:
        """Cancela el undock en curso y descarta su resultado pendiente, si lo hay."""


class MapSaverPort(Protocol):
    """Puerto de guardado del mapa."""

    def save(self, name: str) -> bool:
        """Guarda el mapa con ``name``; devuelve ``True`` si tuvo éxito."""


@dataclass(frozen=True)
class RobotStatus:
    """Estado agregado del Create 3; ``None`` = dato desconocido (caducado o sin recibir)."""

    docked: bool | None = None
    kidnapped: bool | None = None
    battery_pct: float | None = None


@dataclass(frozen=True)
class ManagerConfig:
    """Parámetros del gestor."""

    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)
    max_consecutive_failures: int = DEFAULT_MAX_CONSECUTIVE_FAILURES
    no_frontier_confirmations: int = DEFAULT_NO_FRONTIER_CONFIRMATIONS
    map_save_name: str = DEFAULT_MAP_SAVE_NAME
    undock_timeout_s: float = DEFAULT_UNDOCK_TIMEOUT_S
    status_timeout_s: float = DEFAULT_STATUS_TIMEOUT_S
    kidnap_clear_cycles: int = DEFAULT_KIDNAP_CLEAR_CYCLES
    min_battery_pct: float = DEFAULT_MIN_BATTERY_PCT

    def __post_init__(self) -> None:
        if self.max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures debe ser >= 1")
        if self.no_frontier_confirmations < 1:
            raise ValueError("no_frontier_confirmations debe ser >= 1")
        if not self.map_save_name.strip():
            raise ValueError("map_save_name no puede estar vacío")
        for name in ("undock_timeout_s", "status_timeout_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} debe ser finito y > 0")
        if self.kidnap_clear_cycles < 1:
            raise ValueError("kidnap_clear_cycles debe ser >= 1")
        if not math.isfinite(self.min_battery_pct) or not 0 <= self.min_battery_pct <= 100:
            raise ValueError("min_battery_pct debe estar en [0, 100]")


_F = TypeVar("_F", bound=Callable[..., Any])


def _synchronized(method: _F) -> _F:
    """Serializa la llamada con el lock del gestor (callbacks de ROS en hilos distintos)."""

    @functools.wraps(method)
    def wrapper(self: ExplorationManager, *args: Any, **kwargs: Any) -> Any:
        with self._lock:
            return method(self, *args, **kwargs)

    return cast(_F, wrapper)


class ExplorationManager:
    """Orquesta la exploración: una frontera activa a la vez."""

    def __init__(
        self,
        navigator: NavigatorPort,
        saver: MapSaverPort,
        dock: DockPort,
        config: ManagerConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Crea el gestor en ``IDLE``.

        Args:
            navigator: puerto de navegación.
            saver: puerto de guardado de mapa.
            dock: puerto de undock.
            config: parámetros; por defecto ``ManagerConfig()``.
            clock: reloj en segundos para los timeouts.
        """
        self._navigator = navigator
        self._saver = saver
        self._dock = dock
        self._config = config if config is not None else ManagerConfig()
        self._clock = clock
        self._lock = threading.RLock()
        self._state = ExplorationState.IDLE
        self._reason = REASON_NONE
        self._reset_run()

    @property
    def state(self) -> ExplorationState:
        """Estado actual."""
        return self._state

    @property
    def reason(self) -> str:
        """Motivo corto del último fin (``DONE``/``FAILED``/parada); vacío si no aplica."""
        return self._reason

    def _reset_run(self) -> None:
        self._blacklist: list[Point2D] = []
        self._active_goal: Point2D | None = None
        self._failures = 0
        self._no_frontier_cycles = 0
        self._clear_cycles = 0
        self._undock_requested = False
        self._phase_start = 0.0
        self._battery_unknown = False
        self._dock_wait_start: float | None = None
        self._goal_just_failed = False  # el fallo del último objetivo ya se contó

    @_synchronized
    def start(self, status: RobotStatus | None = None) -> None:
        """Inicia la exploración y reinicia lista negra y contadores.

        Con ``docked is True`` solicita el undock; con ``docked is None`` (o sin ``status``)
        espera el estado del robot en ``UNDOCKING`` sin explorar.

        Args:
            status: último estado conocido del robot.

        Raises:
            InvalidTransitionError: si el estado es ``UNDOCKING``, ``EXPLORING`` o ``PAUSED``.
        """
        if self._state in _ACTIVE_STATES:
            raise InvalidTransitionError(f"start() no es válido en {self._state.value}")
        self._reset_run()
        self._reason = REASON_NONE
        self._phase_start = self._clock()
        docked = status.docked if status is not None else None
        if docked is False:
            self._state = ExplorationState.EXPLORING
        else:
            self._state = ExplorationState.UNDOCKING
            if docked is True:
                self._request_undock()
        _LOG.info("exploration started (%s)", self._state.value)

    @_synchronized
    def stop(self) -> None:
        """Cancela el objetivo y vuelve a ``IDLE`` desde cualquier estado."""
        self._navigator.cancel()
        self._dock.cancel()  # un undock en curso no debe seguir moviendo el robot tras la parada
        self._active_goal = None
        self._state = ExplorationState.IDLE
        self._reason = REASON_STOPPED
        _LOG.info("exploration stopped")

    @_synchronized
    def on_goal_result(self, success: bool) -> None:
        """Informa el resultado del objetivo activo (``False`` = fallo o cancelación).

        Se ignora si no hay exploración en curso ni objetivo activo.
        """
        goal = self._active_goal
        if self._state is not ExplorationState.EXPLORING or goal is None:
            return
        self._active_goal = None
        if success:
            self._failures = 0
            return
        self._goal_just_failed = True
        self._register_failure(goal)

    @_synchronized
    def on_undock_result(self, success: bool) -> None:
        """Informa el resultado del undock; un fallo termina en ``FAILED``.

        Un éxito no basta: se exige ``docked == False`` para empezar a explorar.
        """
        if self._state is not ExplorationState.UNDOCKING or not self._undock_requested:
            return
        if not success:
            self._fail(REASON_UNDOCK_FAILED)

    @_synchronized
    def on_hazard(self, point: Point2D | None) -> None:
        """Peligro (bump/cliff/stuck) en ``point`` (pose aproximada del contacto).

        Cancela el objetivo, añade el punto a la lista negra y suma un fallo consecutivo.
        Sin pose (``None`` o no finita) se pone en lista negra el objetivo activo.
        Solo actúa en ``EXPLORING``.
        """
        if self._state is not ExplorationState.EXPLORING:
            return
        goal, self._active_goal = self._active_goal, None
        self._navigator.cancel()
        if goal is None and self._goal_just_failed:
            # el resultado del objetivo ya contó este fallo: no contarlo doble
            self._goal_just_failed = False
            if point is not None and math.isfinite(point.x) and math.isfinite(point.y):
                self._blacklist.append(point)
            return
        if point is not None and math.isfinite(point.x) and math.isfinite(point.y):
            blocked: Point2D | None = point
        else:
            _LOG.warning("hazard without a valid pose; blacklisting the active goal")
            blocked = goal
        self._register_failure(blocked)

    def _register_failure(self, blocked: Point2D | None) -> None:
        if blocked is not None:
            self._blacklist.append(blocked)
        self._failures += 1
        _LOG.warning("goal failure or hazard (%d)", self._failures)
        if self._failures >= self._config.max_consecutive_failures:
            self._fail(REASON_TOO_MANY_FAILURES)

    def _fail(self, reason: str) -> None:
        self._state = ExplorationState.FAILED
        self._reason = reason

    def _request_undock(self) -> None:
        self._undock_requested = True
        self._phase_start = self._clock()
        self._dock.undock()

    @_synchronized
    def step(self, grid: GridMap, robot: Point2D, status: RobotStatus) -> None:
        """Un ciclo de exploración; no hace nada en ``IDLE``/``DONE``/``FAILED``.

        Fallo seguro: con pose no finita, o con dock/secuestro desconocidos, no se envía
        ningún objetivo.
        """
        if self._state is ExplorationState.UNDOCKING:
            self._step_undocking(status)
        elif self._state is ExplorationState.PAUSED:
            self._step_paused(status)
        elif self._state is ExplorationState.EXPLORING:
            self._step_exploring(grid, robot, status)

    def _step_undocking(self, status: RobotStatus) -> None:
        elapsed = self._clock() - self._phase_start
        if status.docked is False:
            self._state = ExplorationState.EXPLORING
        elif status.docked is True and not self._undock_requested:
            self._request_undock()
        elif not self._undock_requested and elapsed >= self._config.status_timeout_s:
            self._fail(REASON_STATUS_TIMEOUT)
        elif self._undock_requested and elapsed >= self._config.undock_timeout_s:
            self._dock.cancel()
            self._fail(REASON_UNDOCK_TIMEOUT)

    def _step_paused(self, status: RobotStatus) -> None:
        if status.kidnapped is not False:  # None = desconocido: conservador
            self._clear_cycles = 0
            return
        self._clear_cycles += 1
        if self._clear_cycles >= self._config.kidnap_clear_cycles:
            self._state = ExplorationState.EXPLORING
            self._no_frontier_cycles = 0
            self._dock_wait_start = None
            _LOG.info("robot released; exploration resumed")

    def _battery_low(self, pct: float | None) -> bool:
        if pct is None or not math.isfinite(pct) or not 0.0 <= pct <= MAX_BATTERY_PCT:
            if not self._battery_unknown:
                _LOG.warning("battery level unknown (%r); continuing", pct)
            self._battery_unknown = True
            return False
        self._battery_unknown = False
        return pct < self._config.min_battery_pct

    def _step_exploring(self, grid: GridMap, robot: Point2D, status: RobotStatus) -> None:
        if status.kidnapped is not False:  # True o None (desconocido): pausar
            self._navigator.cancel()
            self._active_goal = None
            self._clear_cycles = 0
            self._state = ExplorationState.PAUSED
            _LOG.warning("robot kidnapped or status unknown; exploration paused")
            return
        if self._battery_low(status.battery_pct):
            self._navigator.cancel()
            self._active_goal = None
            self._finish(REASON_LOW_BATTERY)
            return
        if self._active_goal is not None:
            return
        if status.docked is not False:
            self._step_dock_blocked(status)
            return
        self._dock_wait_start = None
        if not (math.isfinite(robot.x) and math.isfinite(robot.y)):
            _LOG.warning("robot pose is not finite; skipping cycle")
            return
        self._explore(grid, robot)

    def _step_dock_blocked(self, status: RobotStatus) -> None:
        """Con ``docked`` True/desconocido en ``EXPLORING``: espera acotada y luego transiciona."""
        now = self._clock()
        if self._dock_wait_start is None:
            self._dock_wait_start = now
        _LOG.warning("robot docked or dock status unknown; not sending goals")
        if now - self._dock_wait_start < self._config.status_timeout_s:
            return
        self._dock_wait_start = None
        if status.docked is True:
            self._state = ExplorationState.UNDOCKING
            self._undock_requested = False
            self._request_undock()
        else:
            self._fail(REASON_STATUS_TIMEOUT)

    def _explore(self, grid: GridMap, robot: Point2D) -> None:
        cfg = self._config.exploration
        frontier = select_frontier(find_frontiers(grid, cfg), robot, self._blacklist, cfg)
        if frontier is None:
            self._no_frontier_cycles += 1
            if self._no_frontier_cycles >= self._config.no_frontier_confirmations:
                self._finish(REASON_MAP_SAVED)
            return
        self._no_frontier_cycles = 0
        self._active_goal = frontier.centroid
        self._goal_just_failed = False
        self._navigator.send_goal(frontier.centroid)

    def _finish(self, done_reason: str) -> None:
        if self._saver.save(self._config.map_save_name):
            self._state = ExplorationState.DONE
            self._reason = done_reason
        else:
            self._fail(REASON_SAVE_FAILED)
        _LOG.info("exploration finished: %s (%s)", self._state.value, self._reason)
