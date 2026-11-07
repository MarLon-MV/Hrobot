"""Máquina de estados pura de la misión de entrega (sin ROS, sin I/O)."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum, auto

from hrobot_core.domain.geometry import Pose2D


class MissionState(Enum):
    """Estados de una misión de entrega."""

    IDLE = auto()
    NAVIGATING_TO_DESTINATION = auto()
    AWAITING_CONFIRMATION = auto()
    RETURNING_TO_BASE = auto()
    EMERGENCY_STOP = auto()
    FAILED = auto()


class MissionEvent(Enum):
    """Eventos que pueden disparar una transición."""

    START = auto()
    GOAL_REACHED = auto()
    NAV_FAILED = auto()
    DELIVERY_CONFIRMED = auto()
    CONFIRMATION_TIMEOUT = auto()
    EMERGENCY = auto()
    RESET = auto()


class InvalidTransition(Exception):  # noqa: N818 - nombre exigido por la tarea
    """Combinación estado/evento no permitida."""


@dataclass(frozen=True)
class Mission:
    """Misión inmutable.

    Attributes:
        mission_id: identificador de la misión.
        destination: pose de entrega.
        base: pose de la base.
        state: estado actual.
        resume_state: estado previo a una parada de emergencia (informativo;
            nunca se reanuda automáticamente).
        delivered: True solo si la recepción fue confirmada por un humano.
    """

    mission_id: str
    destination: Pose2D
    base: Pose2D
    state: MissionState = MissionState.IDLE
    resume_state: MissionState | None = None
    delivered: bool = False


_S = MissionState
_E = MissionEvent

# (estado, evento) -> (nuevo estado, valor de `delivered` o None si no cambia)
_TABLE: dict[tuple[MissionState, MissionEvent], tuple[MissionState, bool | None]] = {
    (_S.IDLE, _E.START): (_S.NAVIGATING_TO_DESTINATION, False),
    (_S.NAVIGATING_TO_DESTINATION, _E.GOAL_REACHED): (_S.AWAITING_CONFIRMATION, None),
    (_S.AWAITING_CONFIRMATION, _E.DELIVERY_CONFIRMED): (_S.RETURNING_TO_BASE, True),
    (_S.AWAITING_CONFIRMATION, _E.CONFIRMATION_TIMEOUT): (_S.RETURNING_TO_BASE, False),
    (_S.RETURNING_TO_BASE, _E.GOAL_REACHED): (_S.IDLE, None),
    (_S.EMERGENCY_STOP, _E.RESET): (_S.IDLE, False),
}

_NAV_FAILED_EXCLUDED = frozenset({_S.IDLE, _S.FAILED})


def transition(m: Mission, event: MissionEvent) -> Mission:
    """Aplica `event` a `m` y devuelve una nueva misión (no muta).

    Raises:
        InvalidTransition: si la combinación estado/evento no es válida.
    """
    if event is _E.EMERGENCY:
        if m.state is _S.EMERGENCY_STOP:
            return m  # idempotente: conserva el resume_state original
        return replace(m, state=_S.EMERGENCY_STOP, resume_state=m.state)
    if event is _E.NAV_FAILED and m.state not in _NAV_FAILED_EXCLUDED:
        return replace(m, state=_S.FAILED)
    target = _TABLE.get((m.state, event))
    if target is None:
        raise InvalidTransition(f"invalid transition: state={m.state.name} event={event.name}")
    new_state, delivered = target
    new_delivered = m.delivered if delivered is None else delivered
    resume = None if event is _E.RESET else m.resume_state
    return replace(m, state=new_state, delivered=new_delivered, resume_state=resume)
