"""Lanza el LiDAR LD19 (ld19_node) y el TF estático base_link -> laser."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LIDAR_BAUDRATE = 230400
FRAME_BASE = "base_link"
FRAME_LASER = "laser"
SCAN_TOPIC = "/scan"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    ld19 = Node(
        package="hrobot_core",
        executable="ld19_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("lidar_port"),
                "baudrate": LIDAR_BAUDRATE,
                "frame_id": FRAME_LASER,
                "scan_topic": SCAN_TOPIC,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        arguments=[
            "--x",
            LaunchConfiguration("laser_x"),
            "--y",
            LaunchConfiguration("laser_y"),
            "--z",
            LaunchConfiguration("laser_z"),
            "--yaw",
            LaunchConfiguration("laser_yaw"),
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            FRAME_BASE,
            "--child-frame-id",
            FRAME_LASER,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "laser_x", default_value="0.0", description="Offset x base_link->laser [m]"
            ),
            DeclareLaunchArgument(
                "laser_y", default_value="0.0", description="Offset y base_link->laser [m]"
            ),
            DeclareLaunchArgument(
                "laser_z", default_value="0.0", description="Offset z base_link->laser [m]"
            ),
            DeclareLaunchArgument(
                "laser_yaw", default_value="0.0", description="Yaw base_link->laser [rad]"
            ),
            ld19,
            laser_tf,
        ]
    )
