"""Hardware-basis: robot_state_publisher (xacro) + driver van de SLAMTEC RPLIDAR C1.

LET OP: de iRobot Create 3 wordt hier NIET gestart. Hij draait zijn eigen firmware en wordt via het
netwerk (Wi-Fi/Ethernet-over-USB) bereikt via zijn ingebouwde micro-ROS/Fast DDS- of Zenoh-stack. De RPi5
moet dezelfde ROS_DOMAIN_ID en RMW hebben als de Create 3 (rmw_fastrtps_cpp of rmw_zenoh_cpp, zoals
ingesteld in de webinterface van de Create 3 > Application > ROS 2). De Create 3 leeft in een namespace
(standaard /r2: hij publiceert /r2/odom, /r2/tf, /r2/tf_static en luistert op /r2/cmd_vel); deze launch plaatst
de nodes die hij aanmaakt onder dezelfde `robot_namespace` en remapt TF naar <ns>/tf en <ns>/tf_static
(lege robot_namespace = geen namespace). De namespace wordt hier één keer toegepast (zie
hrobot_namespace.py); aanroepers mogen deze launch niet in een andere namespace wikkelen. Hardwarecontrole
nog te doen.

AANNAME over de RPLIDAR C1-driver: het officiële pakket `sllidar_ros2` van Slamtec (executable
`sllidar_node`). Gekozen omdat het de driver van de fabrikant is en de parameters channel_type /
serial_port / serial_baudrate / frame_id / inverted / angle_compensate / scan_mode ondersteunt. Het moet
vanaf de broncode in de workspace gebouwd worden (niet beschikbaar via rosdep).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from hrobot_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    robot_namespace_argument,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    namespace = resolve_namespace(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    laser_inverted = LaunchConfiguration("laser_inverted")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")

    xacro_file = os.path.join(
        get_package_share_directory("hrobot_description"), "urdf", "hrobot_rplidar_c1.urdf.xacro"
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        remappings=tf_remappings(namespace),
    )

    # Publiceert sensor_msgs/LaserScan op `scan_raw` met frame laser_frame (gedefinieerd in de xacro). De
    # driver publiceert standaard op `scan`; die naam wordt hier naar `scan_raw` geremapt. slam_toolbox
    # (Karto) weigert scans waarvan het aantal stralen verandert, dus scan_resampler publiceert het opnieuw
    # als `scan` (/<ns>/scan; /scan zonder namespace) met een vast aantal stralen.
    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": lidar_port,
                # LaunchConfiguration geeft een string: converteren zodat de node een geheel getal ontvangt.
                "serial_baudrate": ParameterValue(lidar_baudrate, value_type=int),
                "frame_id": "laser_frame",
                "inverted": ParameterValue(laser_inverted, value_type=bool),
                "angle_compensate": True,
                "scan_mode": lidar_scan_mode,
            }
        ],
        remappings=[("scan", "scan_raw")],
    )

    scan_resampler = Node(
        package="hrobot_bringup",
        executable="scan_resampler.py",
        name="scan_resampler",
        output="screen",
        parameters=[{"input_topic": "scan_raw", "output_topic": "scan", "num_beams": 360}],
    )

    return namespaced(namespace, [robot_state_publisher, rplidar, scan_resampler])


def generate_launch_description():
    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/ttyUSB0",
                description="Seriële poort van de RPLIDAR C1 (overweeg een udev-symlink, bijv. /dev/rplidar).",
            ),
            DeclareLaunchArgument("lidar_baudrate", default_value="460800"),
            DeclareLaunchArgument(
                "laser_inverted",
                default_value="false",
                description="Scanrichting van de RPLIDAR C1 (true|false). Zet op true als de scan tegengesteld aan de robot draait.",
            ),
            DeclareLaunchArgument(
                "lidar_scan_mode",
                default_value="Standard",
                description="Scanmodus van de RPLIDAR C1 (Standard of DenseBoost; leeg = standaardmodus van de driver).",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
