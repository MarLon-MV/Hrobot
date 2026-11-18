"""Robot real: mapear y navegar a la vez. hardware_base + slam_toolbox + Nav2 (sin AMCL ni map_server) + RViz.

slam_toolbox (modo mapping) publica `map -> odom` y el `/map` vivo; Nav2 planifica sobre ese mapa mientras
crece (nav.launch.py con localization:=slam, planner con allow_unknown). El robot se mueve por teleop o con
metas "2D Goal Pose" de RViz; no hay exploración autónoma. Guarda el mapa con src/scripts/save_map.sh.

No lo combines con `mapping` ni con `navigation`: los tres publican `map -> odom`.
Cada include aplica el namespace por su cuenta: este launch NO lo envuelve. RViz es el de navegación
(navigation.rviz, un solo RViz), así que slam se lanza con rviz:=false.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from hrobot_namespace import robot_namespace_argument  # noqa: E402


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("hrobot_bringup"), "launch")
    namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "hardware_base.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
            "laser_scan_dir": LaunchConfiguration("laser_scan_dir"),
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz": "false",  # el RViz es el de nav.launch.py
            "maps_dir": LaunchConfiguration("maps_dir"),
            "map": LaunchConfiguration("map"),
            "slam_lifecycle": LaunchConfiguration("slam_lifecycle"),
        }.items(),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "localization": "slam",
            "rviz": LaunchConfiguration("rviz"),
            "odom_relay": "true",  # el Create 3 publica odom BEST_EFFORT
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
            DeclareLaunchArgument("laser_scan_dir", default_value="true", description="true|false"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument("map", default_value="", description="Map to extend; empty = new."),
            DeclareLaunchArgument("slam_lifecycle", default_value="true"),
            # slam first: it validates map:= before any process starts.
            slam,
            hardware,
            nav,
        ]
    )
