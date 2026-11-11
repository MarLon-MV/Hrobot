"""Gestor de exploración autónoma por fronteras (sin ROS).

Máquina de estados ``IDLE -> EXPLORING -> DONE | FAILED``. Elige la siguiente frontera con
la lógica pura de ``domain.exploration`` y delega el movimiento y el guardado del mapa en
puertos (``Protocol``) que implementa la capa ``adapters``.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol

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

REASON_NONE = ""
REASON_STOPPED = "stopped"
REASON_MAP_SAVED = "map saved"
REASON_SAVE_FAILED = "map save failed"
REASON_TOO_MANY_FAILURES = "too many consecutive goal failures"


class ExplorationState(Enum):
    """Estado del gestor de exploración."""

    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    DONE = "DONE"
    FAILED = "FAILED"


class InvalidTransitionError(RuntimeError):
    """Transición de estado no permitida (p. ej. ``start()`` en ``EXPLORING``)."""


class NavigatorPort(Protocol):
    """Puerto de navegación hacia un objetivo en el frame ``map``."""

    def send_goal(self, goal: Point2D) -> None:
        """Envía un objetivo (metros, frame ``map``)."""

    def cancel(self) -> None:
        """Cancela el objetivo activo, si lo hay."""


class MapSaverPort(Protocol):
    """Puerto de guardado del mapa."""

    def save(self, name: str) -> bool:
        """Guarda el mapa con ``name``; devuelve ``True`` si tuvo éxito."""


@dataclass(frozen=True)
class ManagerConfig:
    """Parámetros del gestor."""

    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)
    max_consecutive_failures: int = DEFAULT_MAX_CONSECUTIVE_FAILURES
    no_frontier_confirmations: int = DEFAULT_NO_FRONTIER_CONFIRMATIONS
    map_save_name: str = DEFAULT_MAP_SAVE_NAME

    def __post_init__(self) -> None:
        if self.max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures debe ser >= 1")
        if self.no_frontier_confirmations < 1:
            raise ValueError("no_frontier_confirmations debe ser >= 1")
        if not self.map_save_name.strip():
            raise ValueError("map_save_name no puede estar vacío")


class ExplorationManager:
    """Orquesta la exploración: una frontera activa a la vez."""

    def __init__(
        self,
        navigator: NavigatorPort,
        saver: MapSaverPort,
        config: ManagerConfig | None = None,
    ) -> None:
        """Crea el gestor en ``IDLE``.

        Args:
            navigator: puerto de navegación.
            saver: puerto de guardado de mapa.
            config: parámetros; por defecto ``ManagerConfig()``.
        """
        self._navigator = navigator
        self._saver = saver
        self._config = config if config is not None else ManagerConfig()
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

    def start(self) -> None:
        """Inicia la exploración y reinicia lista negra y contadores.

        Raises:
            InvalidTransitionError: si el estado es ``EXPLORING``.
        """
        if self._state is ExplorationState.EXPLORING:
            raise InvalidTransitionError("start() no es válido en EXPLORING")
        self._reset_run()
        self._reason = REASON_NONE
        self._state = ExplorationState.EXPLORING
        _LOG.info("exploration started")

    def stop(self) -> None:
        """Cancela el objetivo y vuelve a ``IDLE`` desde cualquier estado."""
        self._navigator.cancel()
        self._active_goal = None
        self._state = ExplorationState.IDLE
        self._reason = REASON_STOPPED
        _LOG.info("exploration stopped")

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
        self._blacklist.append(goal)
        self._failures += 1
        _LOG.warning("goal (%.2f, %.2f) failed (%d)", goal.x, goal.y, self._failures)
        if self._failures >= self._config.max_consecutive_failures:
            self._state = ExplorationState.FAILED
            self._reason = REASON_TOO_MANY_FAILURES

    def step(self, grid: GridMap, robot: Point2D) -> None:
        """Un ciclo de exploración; no hace nada si no está en ``EXPLORING``.

        Fallo seguro: con pose del robot no finita no se envía ningún objetivo.
        """
        if self._state is not ExplorationState.EXPLORING or self._active_goal is not None:
            return
        if not (math.isfinite(robot.x) and math.isfinite(robot.y)):
            _LOG.warning("robot pose is not finite; skipping cycle")
            return
        cfg = self._config.exploration
        frontier = select_frontier(find_frontiers(grid, cfg), robot, self._blacklist, cfg)
        if frontier is None:
            self._no_frontier_cycles += 1
            if self._no_frontier_cycles >= self._config.no_frontier_confirmations:
                self._finish()
            return
        self._no_frontier_cycles = 0
        self._active_goal = frontier.centroid
        self._navigator.send_goal(frontier.centroid)

    def _finish(self) -> None:
        if self._saver.save(self._config.map_save_name):
            self._state = ExplorationState.DONE
            self._reason = REASON_MAP_SAVED
        else:
            self._state = ExplorationState.FAILED
            self._reason = REASON_SAVE_FAILED
        _LOG.info("exploration finished: %s (%s)", self._state.value, self._reason)
