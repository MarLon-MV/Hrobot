"""Modo mapeo: sensores + seguridad + slam_toolbox (online_async)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hrobot_core.adapters.names import TF_STATIC_TOPIC, TF_TOPIC, namespaced
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

TF_CONFIG = "create3_tf_topic"
TF_STATIC_CONFIG = "create3_tf_static_topic"


def _set_tf_topics(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        SetLaunchConfiguration(TF_CONFIG, namespaced(namespace, TF_TOPIC)),
        SetLaunchConfiguration(TF_STATIC_CONFIG, namespaced(namespace, TF_STATIC_TOPIC)),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("hrobot_bringup"))
    slam = Path(get_package_share_directory("slam_toolbox"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_args = {
        "use_sim_time": use_sim_time,
        "create3_namespace": LaunchConfiguration("create3_namespace"),
    }

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "sensors.launch.py")),
        launch_arguments=common_args.items(),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "safety.launch.py")),
        launch_arguments=common_args.items(),
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam / "launch" / "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": str(bringup / "config" / "slam_toolbox_params.yaml"),
        }.items(),
    )
    # online_async_launch.py no acepta remapeos: SetRemap aplica a los nodos del grupo.
    slam_group = GroupAction(
        [
            SetRemap(src="tf", dst=LaunchConfiguration(TF_CONFIG)),
            SetRemap(src="tf_static", dst=LaunchConfiguration(TF_STATIC_CONFIG)),
            slam_toolbox,
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("create3_namespace", default_value="/r2"),
            OpaqueFunction(function=_set_tf_topics),
            sensors,
            safety,
            slam_group,
        ]
    )
