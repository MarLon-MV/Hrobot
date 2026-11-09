"""Lanza rviz2 con hrobot_nav.rviz. Solo para un equipo con pantalla (no la RPi5 headless)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hrobot_core.adapters.names import TF_STATIC_TOPIC, TF_TOPIC, namespaced
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TF_CONFIG = "create3_tf_topic"
TF_STATIC_CONFIG = "create3_tf_static_topic"


def _set_tf_topics(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        SetLaunchConfiguration(TF_CONFIG, namespaced(namespace, TF_TOPIC)),
        SetLaunchConfiguration(TF_STATIC_CONFIG, namespaced(namespace, TF_STATIC_TOPIC)),
    ]


def generate_launch_description() -> LaunchDescription:
    rviz_config = Path(get_package_share_directory("hrobot_bringup")) / "rviz" / "hrobot_nav.rviz"
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config)],
        parameters=[
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
        ],
        remappings=[
            ("tf", LaunchConfiguration(TF_CONFIG)),
            ("tf_static", LaunchConfiguration(TF_STATIC_CONFIG)),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("create3_namespace", default_value="/r2"),
            OpaqueFunction(function=_set_tf_topics),
            rviz,
        ]
    )
