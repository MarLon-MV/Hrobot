"""Lanza el supervisor de seguridad (supervisor_node) con sus parámetros por defecto."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

REQUIRE_MISSION_DESCRIPTION = (
    "Si es 'true' (default) el robot NO se mueve hasta que un nodo de mision publique "
    "estado de navegacion. 'false' es modo banco: SOLO para pruebas controladas."
)


def generate_launch_description() -> LaunchDescription:
    supervisor = Node(
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
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "require_mission_status",
                default_value="true",
                description=REQUIRE_MISSION_DESCRIPTION,
            ),
            supervisor,
        ]
    )
