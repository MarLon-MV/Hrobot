"""Navegación autónoma con Nav2 + RViz. Sin hardware.

Lo comparten `navigation`, `mapping_navigation` (robot real) y `sim` (con nav:=true).

`localization` elige quién publica `map -> odom` y `/map`:
  * amcl (por defecto): map_server + AMCL sobre un mapa guardado (`map_yaml` obligatorio). No corre slam_toolbox.
  * slam: no lanza map_server ni AMCL; Nav2 usa el `/map` vivo de un slam_toolbox que corre aparte (lo lanza
    `mapping_navigation`). El planner pasa a `allow_unknown: true` porque el mapa aún tiene zonas sin explorar.

Arguments: robot_namespace (env CREATE3_ROBOT_NAMESPACE or 'r2'; empty = none), use_sim_time (false),
localization (amcl|slam), map_yaml (ruta absoluta al .yaml del mapa; solo con localization:=amcl), rviz (true),
params_file, autostart (true), odom_relay (false; true en el robot real: Nav2 lee odom_reliable),
set_initial_pose (false), initial_x/initial_y/initial_yaw (0.0) (estos tres solo con AMCL).

Namespace: igual que slam.launch.py (PushRosNamespace, TF relativo, RewrittenYaml). Los nodos Nav2 se
lanzan con el mismo patrón que el stack anterior probado en la Pi.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from hrobot_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    rewrite_yaml,
    robot_namespace_argument,
    substitute_namespace_markers,
    tf_remappings,
)


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1", "yes")


def _setup(context, *args, **kwargs):
    namespace = resolve_namespace(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    localization = LaunchConfiguration("localization").perform(context).strip().lower()
    if localization not in ("amcl", "slam"):
        raise RuntimeError(f"localization:='{localization}' no es válido: usa 'amcl' o 'slam'.")
    use_amcl = localization == "amcl"

    map_yaml = ""
    if use_amcl:
        map_yaml_arg = LaunchConfiguration("map_yaml").perform(context).strip()
        map_yaml = os.path.abspath(os.path.expanduser(map_yaml_arg)) if map_yaml_arg else ""
        if not os.path.isfile(map_yaml):
            raise RuntimeError(
                f"map_yaml:='{map_yaml_arg}' no existe o está vacío. Con localization:=amcl pasa la ruta "
                "absoluta al .yaml del mapa (p. ej. map_yaml:=$PWD/src/maps/mapa1.yaml; se genera con "
                "save_map, ver README)."
            )

    # Orden: '/<robot_namespace>' -> '/<ns>' en los topics de los costmaps y luego RewrittenYaml.
    params_file = rewrite_yaml(
        substitute_namespace_markers(LaunchConfiguration("params_file"), namespace), namespace
    )
    odom_topic = "odom_reliable" if _truthy(context, "odom_relay") else "odom"
    tf_remaps = tf_remappings(namespace)

    common = [params_file, {"use_sim_time": use_sim_time}]
    odom_override = {"odom_topic": odom_topic}

    def nav2_node(package, executable, name=None, extra=None, remappings=tf_remaps):
        return Node(
            package=package,
            executable=executable,
            name=name or executable,
            output="screen",
            parameters=common + ([extra] if extra else []),
            remappings=remappings,
        )

    initial_pose = {
        "set_initial_pose": _truthy(context, "set_initial_pose"),
        "initial_pose": {
            "x": float(LaunchConfiguration("initial_x").perform(context)),
            "y": float(LaunchConfiguration("initial_y").perform(context)),
            "z": 0.0,
            "yaw": float(LaunchConfiguration("initial_yaw").perform(context)),
        },
    }

    actions = []
    if use_amcl:
        actions += [
            nav2_node("nav2_map_server", "map_server", extra={"yaml_filename": map_yaml}),
            nav2_node("nav2_amcl", "amcl", extra=initial_pose),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=common + [{"autostart": autostart}],
            ),
        ]
    # Con slam_toolbox el mapa crece mientras se navega: hay que poder planificar por zonas aún desconocidas.
    planner_extra = None if use_amcl else {"GridBased": {"allow_unknown": True}}
    actions += [
        nav2_node("nav2_controller", "controller_server", extra=odom_override),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server", extra=planner_extra),
        nav2_node("nav2_behaviors", "behavior_server"),
        nav2_node("nav2_bt_navigator", "bt_navigator", extra=odom_override),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=common + [{"autostart": autostart}],
        ),
    ]
    if _truthy(context, "odom_relay"):
        actions.append(
            Node(
                package="hrobot_bringup",
                executable="odom_relay.py",
                name="odom_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    description_share = get_package_share_directory("hrobot_description")
    rviz_config = substitute_namespace_markers(
        os.path.join(description_share, "rviz", "navigation.rviz"), namespace
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=tf_remaps,
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return namespaced(namespace, actions)


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("hrobot_bringup"), "config", "nav2_params.yaml"
    )
    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "localization",
                default_value="amcl",
                description="amcl = map_server + AMCL sobre un mapa guardado; slam = usar el /map de "
                "un slam_toolbox que corre aparte (mapear y navegar a la vez).",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="",
                description="Ruta absoluta al .yaml del mapa (map_server). Obligatorio con localization:=amcl.",
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "odom_relay",
                default_value="false",
                description="true = relé BEST_EFFORT->RELIABLE de odom (robot real).",
            ),
            DeclareLaunchArgument(
                "set_initial_pose",
                default_value="false",
                description="true = AMCL arranca en initial_x/y/yaw; false = usar 2D Pose Estimate en RViz.",
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
