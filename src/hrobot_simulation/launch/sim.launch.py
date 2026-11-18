"""Gazebo (Harmonic) + gesimuleerde Create 3/RPLIDAR C1-robot + slam_toolbox + RViz.

Argumenten: world (room.sdf), headless (false), rviz (true), maps_dir, map, slam_lifecycle,
x/y/yaw (spawn-pose), nav (false), localization (amcl), map_yaml.

nav:=false (standaard): slam_toolbox om met teleop te kaarten.
nav:=true: autonome navigatie op een opgeslagen kaart (map_server + AMCL + Nav2) in plaats van slam_toolbox;
vereist map_yaml:=<absoluut pad naar het .yaml>. AMCL start op de spawn-pose (x/y/yaw), die samenvalt met
de oorsprong van de kaart als die is gemaakt door de simulatie te starten zonder x/y/yaw te wijzigen.
nav:=true localization:=slam: tegelijk kaarten en navigeren (slam_toolbox + Nav2 op de levende /map, zonder
AMCL of map_yaml; één enkele RViz, die van navigatie).

De simulatie draait ZONDER namespace (/cmd_vel, /scan, /odom, /map, /tf): gelijk aan de echte robot met
`robot_namespace:=''`. Om de robot te bewegen: teleop (zie README van het pakket):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Alle ROS-nodes gebruiken use_sim_time:=true (Gazebo-klok via /clock).
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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("hrobot_simulation")
    bringup_share = get_package_share_directory("hrobot_bringup")
    description_share = get_package_share_directory("hrobot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    xacro_file = os.path.join(description_share, "urdf", "hrobot_rplidar_c1.urdf.xacro")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " sim:=true laser_yaw:=0"]), value_type=str
    )

    # Gazebo-server + GUI (-r: meteen draaien). headless: alleen server (-s), geen GUI.
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

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav.launch.py")),
        launch_arguments={
            "robot_namespace": "",
            "use_sim_time": "true",
            "localization": LaunchConfiguration("localization"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "rviz": LaunchConfiguration("rviz"),
            "odom_relay": "false",  # de Gazebo-brug publiceert odom al als RELIABLE
            "set_initial_pose": "true",
            "initial_x": LaunchConfiguration("x"),
            "initial_y": LaunchConfiguration("y"),
            "initial_yaw": LaunchConfiguration("yaw"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav")),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "slam.launch.py")),
        # slam draait als er niet genavigeerd wordt (nav:=false) of als er op de levende kaart genavigeerd wordt (localization:=slam).
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("nav"), "' != 'true' or '", LaunchConfiguration("localization"), "' == 'slam'"]
            )
        ),
        launch_arguments={
            "robot_namespace": "",
            "use_sim_time": "true",
            # met nav:=true is de RViz die van nav.launch.py (slechts één)
            "rviz": PythonExpression(
                ["'false' if '", LaunchConfiguration("nav"), "' == 'true' else '", LaunchConfiguration("rviz"), "'"]
            ),
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
                description="Pad naar de Gazebo-wereld (.sdf).",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", description="true = Gazebo zonder GUI."
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="true|false"),
            DeclareLaunchArgument("maps_dir", default_value="~/.create3_maps"),
            DeclareLaunchArgument("map", default_value="", description="Uit te breiden kaart; leeg = nieuwe kaart."),
            DeclareLaunchArgument("slam_lifecycle", default_value="true"),
            DeclareLaunchArgument(
                "nav", default_value="false", description="true = AMCL + Nav2 in plaats van slam_toolbox."
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="amcl",
                description="Met nav:=true: amcl (opgeslagen kaart, map_yaml) of slam (tegelijk kaarten en navigeren).",
            ),
            DeclareLaunchArgument(
                "map_yaml", default_value="", description="Met nav:=true localization:=amcl: absoluut pad naar het .yaml."
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # Gazebo lost package://hrobot_description/meshes/... op via dit pad.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [os.path.dirname(description_share), ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
            ),
            slam,  # eerst: valideert map:= voordat een proces start
            nav_launch,  # idem: valideert map_yaml voordat processen starten
            gz_gui,
            gz_headless,
            robot_state_publisher,
            spawn,
            bridge,
        ]
    )
