"""Echte robot: hardware_base (robot_state_publisher + RPLIDAR C1) + map_server + AMCL + Nav2 + RViz.

De kaart wordt eerst gemaakt met `mapping` (teleop) en opgeslagen met save_map. Hier wordt er met AMCL op
genavigeerd. Net als in de simulatie start AMCL al gepositioneerd op initial_x/y/yaw (standaard 0,0,0 = de
oorsprong van de kaart, dus het dock waarvandaan gekaart is): zet de robot in het dock voor het starten. Staat
hij elders, geef dan initial_x/y/yaw mee, of set_initial_pose:=false en gebruik "2D Pose Estimate" in RViz.
Elke include past de namespace zelf toe: deze launch wikkelt hem NIET.
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
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "map_yaml": LaunchConfiguration("map_yaml"),
            "rviz": LaunchConfiguration("rviz"),
            "odom_relay": "true",  # de Create 3 publiceert odom als BEST_EFFORT
            "set_initial_pose": LaunchConfiguration("set_initial_pose"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )

    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_yaml", description="Absoluut pad naar het .yaml-bestand van de kaart."),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_baudrate", default_value="460800"),
            DeclareLaunchArgument("laser_inverted", default_value="false", description="true|false"),
            DeclareLaunchArgument("lidar_scan_mode", default_value="Standard"),
            DeclareLaunchArgument(
                "set_initial_pose",
                default_value="true",
                description="true (standaard, zoals in de simulatie) = AMCL start op initial_x/y/yaw, "
                "standaard de oorsprong van de kaart (het dock). false = stel de pose in met 2D Pose Estimate.",
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            nav,  # eerst: valideert map_yaml voordat processen starten
            hardware,
        ]
    )
