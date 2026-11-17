"""Robot real: hardware_base (robot_state_publisher + LiDAR LD19) + slam_toolbox + RViz.

El robot se mueve con teleop (ver README). Cada include aplica el namespace por su cuenta, por eso
este launch NO lo envuelve en otro namespace.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from create3_namespace import robot_namespace_argument  # noqa: E402


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("create3_bot_bringup"), "launch")
    namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "hardware_base.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz": LaunchConfiguration("rviz"),
            "maps_dir": LaunchConfiguration("maps_dir"),
            "map": LaunchConfiguration("map"),
            "slam_lifecycle": LaunchConfiguration("slam_lifecycle"),
        }.items(),
    )

    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/ttyUSB0",
                description="Serial port of the LD19; e.g. /dev/ldlidar.",
            ),
            DeclareLaunchArgument("lidar_baudrate", default_value="230400"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument("map", default_value="", description="Map to extend; empty = new."),
            DeclareLaunchArgument("slam_lifecycle", default_value="true"),
            # slam first: it validates map:= before any process starts.
            slam,
            hardware,
        ]
    )
