"""Nombres únicos de tópicos, frames y constantes de hardware (sin imports de ROS)."""

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"  # solo lo publica supervisor_node
NAV_CMD_VEL_TOPIC = "/cmd_vel_nav"  # salida de Nav2, entrada del supervisor
MISSION_STATUS_TOPIC = "/hrobot/mission_status"

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_LASER = "laser"

LD19_DEFAULT_PORT = "/dev/ttyUSB0"
LD19_BAUDRATE = 230400
