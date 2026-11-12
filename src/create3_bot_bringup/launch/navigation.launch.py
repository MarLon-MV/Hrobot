"""Nav2 navigation servers + map_saver_server (no amcl, no map_server).

The map and map->odom come from slam_toolbox (master_explore.launch.py). Commands are published
directly on `cmd_vel` (/<robot_namespace>/cmd_vel, i.e. /r2/cmd_vel; no velocity_smoother: RPP
already limits acceleration, and the Create 3 applies its own safety limits).

Namespace: the nodes are put under `robot_namespace` here (applied once; callers must not wrap this
launch in another namespace), the YAML is rewritten with RewrittenYaml(root_key=<ns>) and TF is
remapped to <ns>/tf and <ns>/tf_static. Empty robot_namespace = no namespace, YAML untouched.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from create3_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    rewrite_yaml,
    robot_namespace_argument,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    namespace = resolve_namespace(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # With a namespace, keys like `controller_server:` are wrapped under `<ns>:`.
    params_file = rewrite_yaml(LaunchConfiguration("params_file"), namespace)

    # The params file first, then use_sim_time override (a later dict wins).
    # Costmaps are sub-nodes of controller_server/planner_server and read use_sim_time from them.
    node_params = [params_file, {"use_sim_time": use_sim_time}]
    tf_remaps = tf_remappings(namespace)

    def nav2_node(package, executable, **kwargs):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=node_params,
            remappings=tf_remaps,
            **kwargs,
        )

    controller = nav2_node("nav2_controller", "controller_server")
    smoother = nav2_node("nav2_smoother", "smoother_server")
    planner = nav2_node("nav2_planner", "planner_server")
    behaviors = nav2_node("nav2_behaviors", "behavior_server")
    bt_navigator = nav2_node("nav2_bt_navigator", "bt_navigator")

    lifecycle_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "autostart": autostart}],
    )

    # Node name must be `map_saver` so the service is <ns>/map_saver/save_map (map_manager_node).
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_map_saver = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "autostart": autostart}],
    )

    return namespaced(
        namespace,
        [
            controller,
            smoother,
            planner,
            behaviors,
            bt_navigator,
            lifecycle_nav,
            map_saver,
            lifecycle_map_saver,
        ],
    )


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("create3_bot_bringup"), "config", "nav2_params.yaml"
    )
    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
