"""Hardware base: robot_state_publisher (xacro) + LiDAR LD19 driver.

NOTE: the iRobot Create 3 is NOT launched here. It runs its own firmware and is reached over the
network (Wi-Fi/Ethernet-over-USB) through its embedded micro-ROS/Fast DDS or Zenoh stack. The RPi5
must share the Create 3 ROS_DOMAIN_ID and RMW (rmw_fastrtps_cpp or rmw_zenoh_cpp, as configured in
the Create 3 web UI > Application > ROS 2). The Create 3 lives in a namespace (default /r2: it
publishes /r2/odom, /r2/tf, /r2/tf_static and listens on /r2/cmd_vel); this launch puts the
nodes it creates under the same `robot_namespace` and remaps TF to <ns>/tf and <ns>/tf_static
(empty robot_namespace = no namespace). The namespace is applied here once (see
hrobot_namespace.py); callers must not wrap this launch in another namespace. Pending hardware
check.

LD19 driver ASSUMPTION: LDRobot's official `ldlidar_stl_ros2` package (node executable
`ldlidar_stl_ros2_node`). Chosen because it is the vendor driver for the LD19 and exposes
frame_id / topic_name / port_name / port_baudrate parameters. It must be built from source in the
workspace (not available through rosdep).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from hrobot_namespace import (  # noqa: E402
    namespaced,
    resolve_namespace,
    robot_namespace_argument,
    tf_remappings,
)


def _setup(context, *args, **kwargs):
    namespace = resolve_namespace(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")

    xacro_file = os.path.join(
        get_package_share_directory("hrobot_description"), "urdf", "hrobot_ld19.urdf.xacro"
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        remappings=tf_remappings(namespace),
    )

    # Publishes sensor_msgs/LaserScan on `scan_raw` with frame laser_frame (defined in the xacro). The LD19
    # returns 500-504 beams per turn, which slam_toolbox rejects, so scan_resampler republishes it as
    # `scan` (/<ns>/scan; /scan without namespace) with a fixed beam count.
    ldlidar = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="ldlidar_publisher_ld19",
        output="screen",
        parameters=[
            {
                "product_name": "LDLiDAR_LD19",
                "topic_name": "scan_raw",
                "frame_id": "laser_frame",
                "port_name": lidar_port,
                # LaunchConfiguration yields a string: convert so the node receives an integer.
                "port_baudrate": ParameterValue(lidar_baudrate, value_type=int),
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
                "angle_crop_min": 135.0,
                "angle_crop_max": 225.0,
            }
        ],
    )

    scan_resampler = Node(
        package="hrobot_bringup",
        executable="scan_resampler.py",
        name="scan_resampler",
        output="screen",
        parameters=[{"input_topic": "scan_raw", "output_topic": "scan", "num_beams": 504}],
    )

    return namespaced(namespace, [robot_state_publisher, ldlidar, scan_resampler])


def generate_launch_description():
    return LaunchDescription(
        [
            robot_namespace_argument(),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/ttyUSB0",
                description="Serial port of the LD19 (consider a udev symlink, e.g. /dev/ldlidar).",
            ),
            DeclareLaunchArgument("lidar_baudrate", default_value="230400"),
            OpaqueFunction(function=_setup),
        ]
    )
