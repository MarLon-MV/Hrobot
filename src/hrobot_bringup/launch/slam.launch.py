"""slam_toolbox (online async, mapping) + RViz. Zonder hardware: gedeeld door `mapping` (echte robot) en `sim`.

Argumenten: robot_namespace (env CREATE3_ROBOT_NAMESPACE of 'r2'; leeg = geen), use_sim_time (false),
rviz (true), maps_dir (~/.create3_maps), map (""), slam_lifecycle (true).
Met map:=<naam> laadt slam_toolbox <maps_dir>/<naam>.posegraph en blijft kaarten om die UIT TE BREIDEN.
Ontbreekt die pose graph, dan stopt de launch met een expliciete fout (er start nooit stilzwijgend
een nieuwe kaart).

Namespace: PushRosNamespace (GroupAction) rond slam_toolbox en rviz2, de nodes die hier worden aangemaakt.
TF wordt geremapt naar <ns>/tf en <ns>/tf_static en de YAML wordt herschreven met
RewrittenYaml(root_key=<ns>); met een lege namespace wordt niets gepusht of herschreven.
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
from hrobot_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    rewrite_yaml,
    robot_namespace_argument,
    substitute_namespace_markers,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    """Bepaal maps_dir/map, valideer de pose graph en bouw de groep met namespace:
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

    bringup_share = get_package_share_directory("hrobot_bringup")
    description_share = get_package_share_directory("hrobot_description")
    slam_yaml = rewrite_yaml(os.path.join(bringup_share, "config", "slam_params.yaml"), namespace)
    tf_remaps = tf_remappings(namespace)
    # slam_toolbox publiceert de absolute topics /map en /map_metadata, dus PushRosNamespace
    # verplaatst ze niet naar de robot-namespace. Remap ze naar /<ns>/map.
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
                f"map:='{map_name}' is ongeldig: gebruik alleen de kaartnaam (geen pad); "
                f"kaarten staan in maps_dir ({maps_dir})."
            )
        map_base = os.path.join(maps_dir, map_name)  # no extension
        posegraph = map_base + ".posegraph"
        data_file = map_base + ".data"
        missing = [p for p in (posegraph, data_file) if not os.path.isfile(p)]
        if missing:
            # De fout treedt hier op voordat een proces start, omdat deze OpaqueFunction het
            # eerste element van de LaunchDescription is.
            raise RuntimeError(
                f"map:='{map_name}' gevraagd maar de pose graph van slam_toolbox is niet gevonden: "
                f"{', '.join(missing)}. Er wordt niet stilzwijgend een NIEUWE kaart gestart. "
                f"Controleer maps_dir ({maps_dir}) en de kaartnaam, of start zonder map:=."
            )
        actions.append(
            LogInfo(
                msg=f"[slam] Bestaande kaart '{map_name}' wordt uitgebreid: "
                f"slam_toolbox laadt {map_base}.posegraph (mapping-modus)."
            )
        )
        slam_params.append({"map_file_name": map_base, "map_start_at_dock": True})
    else:
        actions.append(LogInfo(msg="[slam] Geen map:= opgegeven: er start een NIEUWE kaart met slam_toolbox."))

    if slam_lifecycle:
        # slam_toolbox in ROS 2 Jazzy levert async_slam_toolbox_node als lifecycle-node
        # (configure + activate nodig, zoals in zijn eigen online_async_launch.py).
        slam_node = LifecycleNode(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace="",  # de namespace komt van PushRosNamespace (LifecycleNode vereist een str)
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
                        LogInfo(msg="[slam] slam_toolbox is geconfigureerd, wordt geactiveerd."),
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

    # RViz: '/<robot_namespace>' in de .rviz wordt vervangen door '/<ns>' (of '' zonder namespace, zodat
    # topics '/map', '/scan', ... blijven, zonder dubbele slash).
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
                description="Kaartnaam (zonder extensie) in maps_dir om uit te breiden; leeg = nieuwe kaart.",
            ),
            DeclareLaunchArgument(
                "slam_lifecycle",
                default_value="true",
                description="Start slam_toolbox als lifecycle-node (Jazzy). Zet op false als de "
                "geïnstalleerde slam_toolbox een gewone node aanbiedt.",
            ),
            # Eerste element: valideert map:= voordat een proces start.
            OpaqueFunction(function=_setup),
        ]
    )
