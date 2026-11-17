"""slam_toolbox (online async, mapping) + RViz. Sin hardware: lo comparten `mapping` (robot real) y `sim`.

Arguments: robot_namespace (env CREATE3_ROBOT_NAMESPACE or 'r2'; empty = none), use_sim_time (false),
rviz (true), maps_dir (~/.create3_maps), map (""), slam_lifecycle (true).
With map:=<name>, slam_toolbox loads <maps_dir>/<name>.posegraph and keeps mapping to EXTEND it.
If that pose graph is missing the launch aborts with an explicit error (it never silently starts
a new map).

Namespace: PushRosNamespace (GroupAction) around slam_toolbox and rviz2, which are the nodes created
here. TF is remapped to <ns>/tf and <ns>/tf_static and the YAML is rewritten with
RewrittenYaml(root_key=<ns>); with an empty namespace nothing is pushed or rewritten.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from create3_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    rewrite_yaml,
    robot_namespace_argument,
    substitute_namespace_markers,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    """Resolve maps_dir/map, validate the pose graph and build the namespaced group:
    slam_toolbox + rviz2."""
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
    tf_remaps = tf_remappings(namespace)
    # slam_toolbox publishes the absolute topics /map and /map_metadata, so PushRosNamespace does
    # not move them under the robot namespace. Remap them to /<ns>/map.
    slam_remaps = list(tf_remaps)
    if namespace:
        slam_remaps += [
            ("/map", f"/{namespace}/map"),
            ("/map_metadata", f"/{namespace}/map_metadata"),
        ]

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
            # Raising here happens before any process is started because this OpaqueFunction is
            # the first entity of the LaunchDescription.
            raise RuntimeError(
                f"map:='{map_name}' requested but slam_toolbox pose graph not found: "
                f"{', '.join(missing)}. Refusing to start a NEW map silently. "
                f"Check maps_dir ({maps_dir}) and the map name, or launch without map:=."
            )
        actions.append(
            LogInfo(
                msg=f"[slam] Extending existing map '{map_name}': "
                f"slam_toolbox will load {map_base}.posegraph (mapping mode)."
            )
        )
        slam_params.append({"map_file_name": map_base, "map_start_at_dock": True})
    else:
        actions.append(LogInfo(msg="[slam] No map:= given: starting a NEW map with slam_toolbox."))

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
            remappings=slam_remaps,
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
                        LogInfo(msg="[slam] slam_toolbox configured, activating."),
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
                remappings=slam_remaps,
            )
        )

    # RViz: '/<robot_namespace>' in the .rviz is replaced by '/<ns>' (or '' for no namespace, so
    # topics stay '/map', '/scan', ... without a double slash).
    rviz_config = substitute_namespace_markers(
        os.path.join(description_share, "rviz", "mapping.rviz"), namespace
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
    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
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
        ]
    )
