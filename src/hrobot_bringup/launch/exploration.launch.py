"""Modo "mapear explorando": mapping (sensores, seguridad, slam_toolbox) + Nav2 sin AMCL
+ exploration_node + map_saver_server.

Nav2 se incluye con use_slam:=true (sin amcl ni map_server estatico): slam_toolbox es la
fuente de map->odom. Nadie salvo supervisor_node publica /cmd_vel: Nav2 usa nav_cmd_vel_topic.
No usar en el robot hasta TASK-029 (dock/secuestro/bateria/peligros).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hrobot_core.adapters.names import TF_STATIC_TOPIC, TF_TOPIC, namespaced
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TF_CONFIG = "create3_tf_topic"
TF_STATIC_CONFIG = "create3_tf_static_topic"
MAP_SAVE_NAME = "map"  # <share>/hrobot_bringup/maps/map.{yaml,pgm}


def _set_tf_topics(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        SetLaunchConfiguration(TF_CONFIG, namespaced(namespace, TF_TOPIC)),
        SetLaunchConfiguration(TF_STATIC_CONFIG, namespaced(namespace, TF_STATIC_TOPIC)),
    ]


def _tf_remappings() -> list:
    """El Create 3 publica TF en <namespace>/tf y <namespace>/tf_static."""
    return [
        ("tf", LaunchConfiguration(TF_CONFIG)),
        ("tf_static", LaunchConfiguration(TF_STATIC_CONFIG)),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("hrobot_bringup"))
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params = [str(bringup / "config" / "exploration_params.yaml"), {"use_sim_time": use_sim_time}]
    common_args = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "create3_namespace": LaunchConfiguration("create3_namespace"),
    }

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "mapping.launch.py")),
        # exploration_node no publica MissionStatus: sin esto el supervisor frena siempre
        # (NO_MISSION_STATUS). Siguen activas las reglas de scan, distancia y emergencia.
        launch_arguments={
            **common_args,
            "require_mission_status": LaunchConfiguration("require_mission_status"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch" / "navigation.launch.py")),
        launch_arguments={
            **common_args,
            "use_slam": "true",
            "map": "",  # no se usa en modo SLAM
            "nav_cmd_vel_topic": LaunchConfiguration("nav_cmd_vel_topic"),
        }.items(),
    )
    exploration = Node(
        package="hrobot_core",
        executable="exploration_node",
        name="exploration_node",
        output="screen",
        # Despues del YAML: sobrescribe map_save_name con la ruta en maps/.
        parameters=[*params, {"map_save_name": LaunchConfiguration("map_save_path")}],
        remappings=_tf_remappings(),
    )
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=params,
        remappings=_tf_remappings(),
    )
    map_saver_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=params,
        remappings=_tf_remappings(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("create3_namespace", default_value="/r2"),
            DeclareLaunchArgument("nav_cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("require_mission_status", default_value="false"),
            DeclareLaunchArgument(
                "map_save_path", default_value=str(bringup / "maps" / MAP_SAVE_NAME)
            ),
            OpaqueFunction(function=_set_tf_topics),
            mapping,
            navigation,
            exploration,
            map_saver,
            map_saver_lifecycle,
        ]
    )
