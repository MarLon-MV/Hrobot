"""Master launch: hardware base + Nav2 + slam_toolbox + explorer + map manager + RViz.

Arguments: robot_namespace (env CREATE3_ROBOT_NAMESPACE or 'r2'; empty = none), use_sim_time (false),
rviz (true), maps_dir (~/.create3_maps), map ("").
With map:=<name>, slam_toolbox loads <maps_dir>/<name>.posegraph and keeps mapping to EXTEND it.
If that pose graph is missing the launch aborts with an explicit error (it never silently starts
a new map).

Namespace decision (applied exactly once, never /r2/r2/...): this launch pushes the namespace
(PushRosNamespace in a GroupAction) ONLY around the nodes it creates itself (slam_toolbox,
map_manager_node, explorer_node, rviz2). The hardware_base and navigation includes are OUTSIDE that
group and receive `robot_namespace`, because each of them applies the namespace on its own.
TF is remapped to <ns>/tf and <ns>/tf_static in every node that uses TF and YAMLs are rewritten
with RewrittenYaml(root_key=<ns>); with an empty namespace nothing is pushed or rewritten.
"""

import os
import sys

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
from nav2_common.launch import ReplaceString

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from create3_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    rewrite_yaml,
    robot_namespace_argument,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    """Resolve maps_dir/map, validate the pose graph and build the namespaced group:
    slam_toolbox + map_manager_node + explorer_node + rviz2."""
    namespace = resolve_namespace(context)
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
    description_share = get_package_share_directory("create3_bot_description")
    slam_yaml = rewrite_yaml(os.path.join(bringup_share, "config", "slam_params.yaml"), namespace)
    explorer_yaml = rewrite_yaml(
        os.path.join(bringup_share, "config", "explorer_params.yaml"), namespace
    )
    tf_remaps = tf_remappings(namespace)

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
            namespace="",  # the namespace comes from PushRosNamespace (LifecycleNode needs a str)
            output="screen",
            parameters=slam_params,
            remappings=tf_remaps,
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
                remappings=tf_remaps,
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

    actions.append(
        Node(
            package="create3_autonomous_explorer",
            executable="explorer_node",
            name="explorer_node",
            output="screen",
            parameters=[explorer_yaml, {"use_sim_time": use_sim_time}],
            # Absolute target: its tf2_ros::TransformListener runs in an internal root-namespace
            # node that only sees global remaps (see create3_namespace.tf_remappings).
            remappings=tf_remappings(namespace, absolute=True),
        )
    )

    # RViz: '/<robot_namespace>' in the .rviz is replaced by '/<ns>' (or '' for no namespace, so
    # topics stay '/map', '/scan', ... without a double slash).
    rviz_config = ReplaceString(
        source_file=os.path.join(description_share, "rviz", "exploration.rviz"),
        replacements={"/<robot_namespace>": f"/{namespace}" if namespace else ""},
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
    bringup_share = get_package_share_directory("create3_bot_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")

    # Outside the namespaced group on purpose: each include applies the namespace itself.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "hardware_base.launch.py")),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "navigation.launch.py")),
        launch_arguments={"robot_namespace": robot_namespace, "use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            robot_namespace_argument(),
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
        ]
    )
