"""Lanza localizacion (map_server + AMCL) y navegacion Nav2 nodo a nodo.

No se usa nav2_bringup/navigation_launch.py: la salida final de Nav2 se remapea a
``nav_cmd_vel_topic`` (default /cmd_vel_nav) para que solo supervisor_node publique /cmd_vel.
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
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BT_XML_NAME = "bt_navigate_no_recovery.xml"
ODOM_CONFIG = "create3_odom_topic"
TF_CONFIG = "create3_tf_topic"
TF_STATIC_CONFIG = "create3_tf_static_topic"
FORBIDDEN_CMD_VEL = ("/cmd_vel",)


def validate_nav_cmd_vel_topic(topic: str, forbidden: tuple[str, ...] = ("/cmd_vel",)) -> str:
    """Devuelve el topico normalizado (sin espacios) si es seguro; ValueError si no.

    Rechaza vacio, los topicos de `forbidden` (el launch anade /<create3_namespace>/cmd_vel,
    entrada real del Create 3).
    """
    normalized = topic.strip()
    parts = [p for p in normalized.split("/") if p]
    if not parts:
        raise ValueError("nav_cmd_vel_topic no puede estar vacio")
    for banned in forbidden:
        if parts == [p for p in banned.split("/") if p]:
            raise ValueError(
                f"nav_cmd_vel_topic '{topic}' resuelve a {banned}: solo supervisor_node "
                "puede publicar cmd_vel del robot; use /cmd_vel_nav"
            )
    return normalized


def _check_topic(context: LaunchContext) -> list:
    topic = context.perform_substitution(LaunchConfiguration("nav_cmd_vel_topic"))
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    try:
        forbidden = (*FORBIDDEN_CMD_VEL, namespaced(namespace, "/cmd_vel"))
        normalized = validate_nav_cmd_vel_topic(topic, forbidden)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    # Sobrescribe el argumento con el valor normalizado: el remapeo lo resuelve despues.
    return [SetLaunchConfiguration("nav_cmd_vel_topic", normalized)]


def _set_odom_topic(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        SetLaunchConfiguration(ODOM_CONFIG, namespaced(namespace, "/odom")),
        SetLaunchConfiguration(TF_CONFIG, namespaced(namespace, TF_TOPIC)),
        SetLaunchConfiguration(TF_STATIC_CONFIG, namespaced(namespace, TF_STATIC_TOPIC)),
    ]


def _tf_remappings() -> list:
    """El Create 3 publica TF en <namespace>/tf y <namespace>/tf_static."""
    return [
        ("tf", LaunchConfiguration(TF_CONFIG)),
        ("tf_static", LaunchConfiguration(TF_STATIC_CONFIG)),
    ]


def _nav_node(
    package: str,
    executable: str,
    params: list,
    remappings: list | None = None,
    condition: UnlessCondition | None = None,
) -> Node:
    return Node(
        condition=condition,
        package=package,
        executable=executable,
        name=executable,
        output="screen",
        parameters=params,
        remappings=[*(remappings or []), *_tf_remappings()],
    )


def _lifecycle(name: str, params: list, condition: UnlessCondition | None = None) -> Node:
    return Node(
        condition=condition,
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=name,
        output="screen",
        parameters=params,
        remappings=_tf_remappings(),
    )


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("hrobot_bringup"))
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params = [LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]
    # Va despues de params_file: sobrescribe odom_topic del YAML con <namespace>/odom.
    odom = {"odom_topic": LaunchConfiguration(ODOM_CONFIG)}

    # use_slam:=true (modo exploracion): slam_toolbox da map->odom; sin AMCL ni map_server
    # y sin sensors/safety (los lanza mapping.launch.py).
    not_slam = UnlessCondition(LaunchConfiguration("use_slam"))
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / f"{name}.launch.py")),
            condition=not_slam,
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "create3_namespace": LaunchConfiguration("create3_namespace"),
            }.items(),
        )
        for name in ("sensors", "safety")
    ]
    localization = [
        _nav_node(
            "nav2_map_server",
            "map_server",
            [*params, {"yaml_filename": LaunchConfiguration("map")}],
            condition=not_slam,
        ),
        _nav_node("nav2_amcl", "amcl", params, condition=not_slam),
        _lifecycle("lifecycle_manager_localization", params, condition=not_slam),
    ]
    # VERIFY (human): ros2 topic info /cmd_vel -v debe mostrar solo supervisor_node como publisher
    navigation = [
        _nav_node(
            "nav2_controller",
            "controller_server",
            params,
            [("cmd_vel", LaunchConfiguration("nav_cmd_vel_topic"))],
        ),
        _nav_node("nav2_planner", "planner_server", params),
        _nav_node(
            "nav2_bt_navigator",
            "bt_navigator",
            # Va despues de params_file para sobrescribir el valor vacio del YAML.
            [
                *params,
                {"default_nav_to_pose_bt_xml": str(bringup / "config" / BT_XML_NAME)},
                odom,
            ],
        ),
        _lifecycle("lifecycle_manager_navigation", params),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", description="Ruta al .yaml del mapa (obligatorio)"),
            DeclareLaunchArgument("use_slam", default_value="false"),
            DeclareLaunchArgument("nav_cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("create3_namespace", default_value="/r2"),
            DeclareLaunchArgument(
                "params_file", default_value=str(bringup / "config" / "nav2_params.yaml")
            ),
            OpaqueFunction(function=_check_topic),
            OpaqueFunction(function=_set_odom_topic),
            *includes,
            *localization,
            *navigation,
        ]
    )
