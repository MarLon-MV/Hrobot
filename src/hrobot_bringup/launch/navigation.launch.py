"""Lanza localizacion (map_server + AMCL) y navegacion Nav2 nodo a nodo.

No se usa nav2_bringup/navigation_launch.py: la salida final de Nav2 se remapea a
``nav_cmd_vel_topic`` (default /cmd_vel_nav) para que solo supervisor_node publique /cmd_vel.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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

CONTROLLER_RAW_TOPIC = "cmd_vel_nav_raw"


def validate_nav_cmd_vel_topic(topic: str) -> str:
    """Devuelve el topico normalizado (sin espacios) si es seguro; ValueError si no.

    Rechaza vacio, /cmd_vel y el topico interno controller->smoother (CONTROLLER_RAW_TOPIC).
    """
    normalized = topic.strip()
    parts = [p for p in normalized.split("/") if p]
    if not parts:
        raise ValueError("nav_cmd_vel_topic no puede estar vacio")
    if parts == ["cmd_vel"]:
        raise ValueError(
            f"nav_cmd_vel_topic '{topic}' resuelve a /cmd_vel: solo supervisor_node "
            "puede publicar /cmd_vel; use /cmd_vel_nav"
        )
    if parts == [CONTROLLER_RAW_TOPIC]:
        raise ValueError(
            f"nav_cmd_vel_topic '{topic}' coincide con el topico interno "
            f"'{CONTROLLER_RAW_TOPIC}' (entrada del velocity_smoother)"
        )
    return normalized


def _check_topic(context: LaunchContext) -> list:
    topic = context.perform_substitution(LaunchConfiguration("nav_cmd_vel_topic"))
    try:
        normalized = validate_nav_cmd_vel_topic(topic)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    # Sobrescribe el argumento con el valor normalizado: el remapeo lo resuelve despues.
    return [SetLaunchConfiguration("nav_cmd_vel_topic", normalized)]


def _nav_node(package: str, executable: str, params: list, remappings: list | None = None) -> Node:
    return Node(
        package=package,
        executable=executable,
        name=executable,
        output="screen",
        parameters=params,
        remappings=remappings or [],
    )


def _lifecycle(name: str, params: list) -> Node:
    return Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=name,
        output="screen",
        parameters=params,
    )


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("hrobot_bringup"))
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params = [LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / f"{name}.launch.py")),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        )
        for name in ("sensors", "safety")
    ]
    localization = [
        _nav_node(
            "nav2_map_server",
            "map_server",
            [*params, {"yaml_filename": LaunchConfiguration("map")}],
        ),
        _nav_node("nav2_amcl", "amcl", params),
        _lifecycle("lifecycle_manager_localization", params),
    ]
    # VERIFY (human): ros2 topic info /cmd_vel -v debe mostrar solo supervisor_node como publisher
    navigation = [
        _nav_node(
            "nav2_controller",
            "controller_server",
            params,
            [("cmd_vel", CONTROLLER_RAW_TOPIC)],
        ),
        _nav_node("nav2_planner", "planner_server", params),
        _nav_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            params,
            [
                ("cmd_vel", CONTROLLER_RAW_TOPIC),
                ("cmd_vel_smoothed", LaunchConfiguration("nav_cmd_vel_topic")),
            ],
        ),
        _nav_node("nav2_bt_navigator", "bt_navigator", params),
        _lifecycle("lifecycle_manager_navigation", params),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", description="Ruta al .yaml del mapa (obligatorio)"),
            DeclareLaunchArgument("nav_cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument(
                "params_file", default_value=str(bringup / "config" / "nav2_params.yaml")
            ),
            OpaqueFunction(function=_check_topic),
            *includes,
            *localization,
            *navigation,
        ]
    )
