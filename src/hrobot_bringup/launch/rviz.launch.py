"""Lanza rviz2 con hrobot_nav.rviz. Solo para un equipo con pantalla (no la RPi5 headless)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rviz_config = Path(get_package_share_directory("hrobot_bringup")) / "rviz" / "hrobot_nav.rviz"
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config)],
        parameters=[
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
        ],
        output="screen",
    )
    return LaunchDescription([DeclareLaunchArgument("use_sim_time", default_value="false"), rviz])
