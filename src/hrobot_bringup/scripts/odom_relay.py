#!/usr/bin/env python3
"""Publiceert nav_msgs/Odometry opnieuw van BEST_EFFORT naar RELIABLE.

De Create 3 publiceert /<ns>/odom als BEST_EFFORT met depth 1; controller_server en bt_navigator van Nav2
abonneren zich als RELIABLE en koppelen daardoor nooit met die publisher. Deze node abonneert zich met
sensor-QoS en publiceert hetzelfde bericht opnieuw op `odom_reliable` (relatief: respecteert de namespace).
Wordt alleen met de echte robot gebruikt.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class OdomRelay(Node):
    def __init__(self):
        super().__init__("odom_relay")
        self.declare_parameter("input_topic", "odom")
        self.declare_parameter("output_topic", "odom_reliable")
        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, out_qos
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._pub.publish,
            qos_profile_sensor_data,
        )


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
