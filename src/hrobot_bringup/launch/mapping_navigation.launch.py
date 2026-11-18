"""Echte robot: tegelijk kaarten en navigeren. hardware_base + slam_toolbox + Nav2 (zonder AMCL of map_server) + RViz.

slam_toolbox (mapping-modus) publiceert `map -> odom` en de levende `/map`; Nav2 plant op die kaart terwijl
hij groeit (nav.launch.py met localization:=slam, planner met allow_unknown). De robot wordt met teleop of met
"2D Goal Pose"-doelen in RViz bestuurd; er is geen autonome exploratie. Sla de kaart op met src/scripts/save_map.sh.

Combineer dit niet met `mapping` of `navigation`: alle drie publiceren `map -> odom`.
Elke include past de namespace zelf toe: deze launch wikkelt hem NIET. RViz is die van navigatie
(navigation.rviz, één enkele RViz), dus slam wordt gestart met rviz:=false.
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
            "laser_inverted": LaunchConfiguration("laser_inverted"),
            "lidar_scan_mode": LaunchConfiguration("lidar_scan_mode"),
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz": "false",  # de RViz is die van nav.launch.py
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
            "odom_relay": "true",  # de Create 3 publiceert odom als BEST_EFFORT
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
                description="Seriële poort van de RPLIDAR C1; bijv. /dev/rplidar.",
            ),
            DeclareLaunchArgument("lidar_baudrate", default_value="460800"),
            DeclareLaunchArgument("laser_inverted", default_value="false", description="true|false"),
            DeclareLaunchArgument("lidar_scan_mode", default_value="Standard"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument("map", default_value="", description="Uit te breiden kaart; leeg = nieuwe kaart."),
            DeclareLaunchArgument("slam_lifecycle", default_value="true"),
            # slam eerst: het valideert map:= voordat een proces start.
            slam,
            hardware,
            nav,
        ]
    )
