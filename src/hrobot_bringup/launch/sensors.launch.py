"""Lanza el LiDAR LD19 (ld19_node) y el TF estático base_link -> laser."""

from hrobot_core.adapters.names import TF_STATIC_TOPIC, TF_TOPIC, namespaced
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LIDAR_BAUDRATE = 230400
FRAME_BASE = "base_link"
FRAME_LASER = "laser"
SCAN_TOPIC = "/scan"
TF_CONFIG = "create3_tf_topic"
TF_STATIC_CONFIG = "create3_tf_static_topic"


def _set_tf_topics(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        SetLaunchConfiguration(TF_CONFIG, namespaced(namespace, TF_TOPIC)),
        SetLaunchConfiguration(TF_STATIC_CONFIG, namespaced(namespace, TF_STATIC_TOPIC)),
    ]


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
        remappings=[
            ("tf", LaunchConfiguration(TF_CONFIG)),
            ("tf_static", LaunchConfiguration(TF_STATIC_CONFIG)),
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("create3_namespace", default_value="/r2"),
            OpaqueFunction(function=_set_tf_topics),
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
