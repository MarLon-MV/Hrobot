"""Nombres únicos de tópicos, frames y constantes de hardware (sin imports de ROS)."""

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"  # solo lo publica supervisor_node
NAV_CMD_VEL_TOPIC = "/cmd_vel_nav"  # salida de Nav2, entrada del supervisor
MISSION_STATUS_TOPIC = "/hrobot/mission_status"
TF_TOPIC = "/tf"  # el Create 3 publica en <namespace>/tf (ver namespaced)
TF_STATIC_TOPIC = "/tf_static"

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

MAP_TOPIC = "/map"  # publicado por slam_toolbox
EXPLORATION_STATUS_TOPIC = "/hrobot/exploration_status"
NAVIGATE_TO_POSE_ACTION = "navigate_to_pose"  # action server de Nav2 (bt_navigator)
MAP_SAVER_SERVICE = "/map_saver/save_map"  # nav2_map_server (nav2_msgs/srv/SaveMap)

# Create 3: se resuelven con namespaced(create3_namespace, ...)
DOCK_STATUS_TOPIC = "dock_status"  # irobot_create_msgs/msg/DockStatus
KIDNAP_STATUS_TOPIC = "kidnap_status"  # irobot_create_msgs/msg/KidnapStatus
BATTERY_STATE_TOPIC = "battery_state"  # sensor_msgs/msg/BatteryState
HAZARD_DETECTION_TOPIC = "hazard_detection"  # irobot_create_msgs/msg/HazardDetectionVector
UNDOCK_ACTION = "undock"  # irobot_create_msgs/action/Undock
