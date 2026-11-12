"""Nav2 navigation servers + map_saver_server (no amcl, no map_server).

The map and map->odom come from slam_toolbox (master_explore.launch.py). Commands are published
directly on /cmd_vel (no velocity_smoother: RPP already limits acceleration, and the Create 3
applies its own safety limits).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    default_params = os.path.join(
        get_package_share_directory("create3_bot_bringup"), "config", "nav2_params.yaml"
    )
    # The params file first, then use_sim_time override (a later dict wins).
    # Costmaps are sub-nodes of controller_server/planner_server and read use_sim_time from them.
    node_params = [params_file, {"use_sim_time": use_sim_time}]

    def nav2_node(package, executable):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=node_params,
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

    # Node name must be `map_saver` so the service is /map_saver/save_map (map_manager_node default).
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            controller,
            smoother,
            planner,
            behaviors,
            bt_navigator,
            lifecycle_nav,
            map_saver,
            lifecycle_map_saver,
        ]
    )
