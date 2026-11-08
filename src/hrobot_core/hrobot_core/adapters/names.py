"""Nombres únicos de tópicos, frames y constantes de hardware (sin imports de ROS)."""

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"  # solo lo publica supervisor_node
NAV_CMD_VEL_TOPIC = "/cmd_vel_nav"  # salida de Nav2, entrada del supervisor
MISSION_STATUS_TOPIC = "/hrobot/mission_status"

DEFAULT_CREATE3_NAMESPACE = "/r2"


def namespaced(namespace: str, topic: str) -> str:
    """Antepone el namespace del Create 3 a un tópico, sin dobles barras.

    Args:
        namespace: Namespace (``"/r2"``, ``"r2"``, ``"/r2/"``); vacío o ``"/"`` = sin namespace.
        topic: Tópico (``"/cmd_vel"`` o ``"cmd_vel"``).

    Returns:
        Tópico absoluto, p. ej. ``"/r2/cmd_vel"``.

    Raises:
        ValueError: Si namespace o tópico contienen espacios en blanco.
    """
    if any(c.isspace() for c in namespace) or any(c.isspace() for c in topic):
        raise ValueError(f"namespace/topic no pueden contener espacios: {namespace!r}, {topic!r}")
    parts = [p for p in (*namespace.split("/"), *topic.split("/")) if p]
    return "/" + "/".join(parts)


FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_LASER = "laser"

LD19_DEFAULT_PORT = "/dev/ttyUSB0"
LD19_BAUDRATE = 230400
