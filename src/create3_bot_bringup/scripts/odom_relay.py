#!/usr/bin/env python3
"""Republica nav_msgs/Odometry de BEST_EFFORT a RELIABLE.

El Create 3 publica /<ns>/odom BEST_EFFORT depth 1; controller_server y bt_navigator de Nav2 suscriben
RELIABLE y nunca emparejan con ese publicador. Este nodo suscribe con QoS de sensor y republica el mismo
mensaje en `odom_reliable` (relativo: respeta el namespace). Solo se usa con el robot real.
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
