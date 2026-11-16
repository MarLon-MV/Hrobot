"""Gazebo (Harmonic) + robot Create 3/LD19 simulado + slam_toolbox + RViz.

Arguments: world (room.sdf), headless (false), rviz (true), maps_dir, map, slam_lifecycle,
x/y/yaw (pose de spawn).

La simulacion corre SIN namespace (/cmd_vel, /scan, /odom, /map, /tf): igual que el robot real con
`robot_namespace:=''`. Para mover el robot: teleop (ver README del paquete):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Todos los nodos ROS usan use_sim_time:=true (reloj de Gazebo via /clock).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("create3_bot_simulation")
    bringup_share = get_package_share_directory("create3_bot_bringup")
    description_share = get_package_share_directory("create3_bot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    xacro_file = os.path.join(description_share, "urdf", "create3_ld19.urdf.xacro")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " sim:=true"]), value_type=str
    )

    # Gazebo server + GUI (-r: start running). headless: server only (-s), no GUI.
    gz_sim_launch = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=UnlessCondition(headless),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s --headless-rendering ", world]}.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "create3",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", "0.01",
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(sim_share, "config", "gz_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "slam.launch.py")),
        launch_arguments={
            "robot_namespace": "",
            "use_sim_time": "true",
            "rviz": LaunchConfiguration("rviz"),
            "maps_dir": LaunchConfiguration("maps_dir"),
            "map": LaunchConfiguration("map"),
            "slam_lifecycle": LaunchConfiguration("slam_lifecycle"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([sim_share, "worlds", "room.sdf"]),
                description="Path to the Gazebo world (.sdf).",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", description="true = Gazebo without GUI."
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument("map", default_value="", description="Map to extend; empty = new."),
            DeclareLaunchArgument("slam_lifecycle", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # Gazebo resolves package://create3_bot_description/meshes/... through this path.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [os.path.dirname(description_share), ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
            ),
            slam,  # first: it validates map:= before any process starts
            gz_gui,
            gz_headless,
            robot_state_publisher,
            spawn,
            bridge,
        ]
    )
