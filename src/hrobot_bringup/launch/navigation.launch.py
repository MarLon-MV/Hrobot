"""Robot real: hardware_base (robot_state_publisher + LiDAR LD19) + map_server + AMCL + Nav2 + RViz.

El mapa se crea antes con `mapping` (teleop) y se guarda con save_map. Aquí se navega sobre él:
fija la pose inicial con "2D Pose Estimate" en RViz (o set_initial_pose:=true initial_x/y/yaw).
Cada include aplica el namespace por su cuenta: este launch NO lo envuelve.
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
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav.launch.py")),
        launch_arguments={
            "robot_namespace": namespace,
            "use_sim_time": use_sim_time,
            "map_yaml": LaunchConfiguration("map_yaml"),
            "rviz": LaunchConfiguration("rviz"),
            "odom_relay": "true",  # el Create 3 publica odom BEST_EFFORT
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
            DeclareLaunchArgument("map_yaml", description="Ruta absoluta al .yaml del mapa."),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_baudrate", default_value="230400"),
            DeclareLaunchArgument("laser_scan_dir", default_value="true", description="true|false"),
            DeclareLaunchArgument("set_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            nav,  # primero: valida map_yaml antes de arrancar procesos
            hardware,
        ]
    )
