"""Lanza el supervisor de seguridad (supervisor_node) con sus parámetros por defecto."""

from hrobot_core.adapters.names import namespaced
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

REQUIRE_MISSION_DESCRIPTION = (
    "Si es 'true' (default) el robot NO se mueve hasta que un nodo de mision publique "
    "estado de navegacion. 'false' es modo banco: SOLO para pruebas controladas."
)
NAMESPACE_DESCRIPTION = (
    "Namespace del Create 3 (default /r2): el supervisor publica en <namespace>/cmd_vel."
)


def _supervisor(context: LaunchContext) -> list:
    namespace = context.perform_substitution(LaunchConfiguration("create3_namespace"))
    return [
        Node(
            package="hrobot_core",
            executable="supervisor_node",
            output="screen",
            parameters=[
                {
                    "require_mission_status": ParameterValue(
                        LaunchConfiguration("require_mission_status"), value_type=bool
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "cmd_vel_topic": namespaced(namespace, "/cmd_vel"),
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "require_mission_status",
                default_value="true",
                description=REQUIRE_MISSION_DESCRIPTION,
            ),
            DeclareLaunchArgument(
                "create3_namespace",
                default_value="/r2",
                description=NAMESPACE_DESCRIPTION,
            ),
            OpaqueFunction(function=_supervisor),
        ]
    )
