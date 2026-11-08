"""Modo mapeo: sensores + seguridad + slam_toolbox (online_async)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("hrobot_bringup"))
    slam = Path(get_package_share_directory("slam_toolbox"))
    use_sim_time = LaunchConfiguration("use_sim_time")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "sensors.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "safety.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam / "launch" / "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": str(bringup / "config" / "slam_toolbox_params.yaml"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            sensors,
            safety,
            slam_toolbox,
        ]
    )
