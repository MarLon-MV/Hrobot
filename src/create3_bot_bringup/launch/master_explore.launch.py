"""Master launch: hardware base + Nav2 + slam_toolbox + explorer + map manager + RViz.

Arguments: use_sim_time (false), rviz (true), maps_dir (~/.create3_maps), map ("").
With map:=<name>, slam_toolbox loads <maps_dir>/<name>.posegraph and keeps mapping to EXTEND it.
If that pose graph is missing the launch aborts with an explicit error (it never silently starts
a new map).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def _setup(context, *args, **kwargs):
    """Resolve maps_dir/map, validate the pose graph and build slam_toolbox + map_manager_node."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    maps_dir = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("maps_dir").perform(context))
    )
    map_name = LaunchConfiguration("map").perform(context).strip()
    slam_lifecycle = LaunchConfiguration("slam_lifecycle").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    bringup_share = get_package_share_directory("create3_bot_bringup")
    slam_yaml = os.path.join(bringup_share, "config", "slam_params.yaml")
    explorer_yaml = os.path.join(bringup_share, "config", "explorer_params.yaml")

    slam_params = [slam_yaml, {"use_sim_time": use_sim_time}]
    actions = []

    if map_name:
        if os.sep in map_name or (os.altsep and os.altsep in map_name) or ".." in map_name:
            raise RuntimeError(
                f"map:='{map_name}' is invalid: use only the map name (no path); "
                f"maps live in maps_dir ({maps_dir})."
            )
        map_base = os.path.join(maps_dir, map_name)  # no extension
        posegraph = map_base + ".posegraph"
        data_file = map_base + ".data"
        missing = [p for p in (posegraph, data_file) if not os.path.isfile(p)]
        if missing:
            # LogInfo is not enough on its own (it would be printed and the launch would go on),
            # so abort explicitly. Raising here happens before any process is started because this
            # OpaqueFunction is the first entity of the LaunchDescription.
            raise RuntimeError(
                f"map:='{map_name}' requested but slam_toolbox pose graph not found: "
                f"{', '.join(missing)}. Refusing to start a NEW map silently. "
                f"Check maps_dir ({maps_dir}) and the map name, or launch without map:=."
            )
        actions.append(
            LogInfo(
                msg=f"[master_explore] Extending existing map '{map_name}': "
                f"slam_toolbox will load {map_base}.posegraph (mapping mode)."
            )
        )
        slam_params.append({"map_file_name": map_base, "map_start_at_dock": True})
    else:
        actions.append(
            LogInfo(msg="[master_explore] No map:= given: starting a NEW map with slam_toolbox.")
        )

    if slam_lifecycle:
        # slam_toolbox in ROS 2 Jazzy ships async_slam_toolbox_node as a lifecycle node
        # (configure + activate needed, as in its own online_async_launch.py).
        slam_node = LifecycleNode(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace="",
            output="screen",
            parameters=slam_params,
        )
        actions.append(slam_node)
        actions.append(
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        )
        actions.append(
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=slam_node,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        LogInfo(msg="[master_explore] slam_toolbox configured, activating."),
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(slam_node),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        ),
                    ],
                )
            )
        )
    else:
        actions.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=slam_params,
            )
        )

    actions.append(
        Node(
            package="create3_autonomous_explorer",
            executable="map_manager_node",
            name="map_manager_node",
            output="screen",
            # Later dict overrides maps_dir from the YAML.
            parameters=[explorer_yaml, {"maps_dir": maps_dir, "use_sim_time": use_sim_time}],
        )
    )
    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("create3_bot_bringup")
    description_share = get_package_share_directory("create3_bot_description")
    use_sim_time = LaunchConfiguration("use_sim_time")
    explorer_yaml = os.path.join(bringup_share, "config", "explorer_params.yaml")
    rviz_config = os.path.join(description_share, "rviz", "exploration.rviz")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "hardware_base.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "navigation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    explorer = Node(
        package="create3_autonomous_explorer",
        executable="explorer_node",
        name="explorer_node",
        output="screen",
        parameters=[explorer_yaml, {"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/ttyUSB0",
                description="Serial port of the LD19 (forwarded to hardware_base); e.g. /dev/ldlidar.",
            ),
            DeclareLaunchArgument("lidar_baudrate", default_value="230400"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map name (no extension) in maps_dir to extend; empty = new map.",
            ),
            DeclareLaunchArgument(
                "slam_lifecycle",
                default_value="true",
                description="Launch slam_toolbox as a lifecycle node (Jazzy). Set false if the "
                "installed slam_toolbox exposes a plain node.",
            ),
            # First entity: validates map:= before any process starts.
            OpaqueFunction(function=_setup),
            hardware,
            navigation,
            explorer,
            rviz,
        ]
    )
