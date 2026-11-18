#!/usr/bin/env python3
"""Resamplet sensor_msgs/LaserScan naar een VAST aantal stralen.

De RPLIDAR C1 levert per omwenteling een wisselend aantal metingen (~500 ruwe punten; met hoekcompensatie
van de driver een vast aantal). slam_toolbox (Karto) legt de grootte vast met de eerste scan en meldt
"LaserRangeScan contains N range readings, expected M" bij elke andere. Deze node leest `scan_raw` (driver)
en publiceert `scan` met altijd `num_beams` stralen en dezelfde hoeken. Gebruikt de dichtstbijzijnde
buur (geen interpolatie, om geen afstanden aan de randen van obstakels te verzinnen). Alleen echte robot.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TWO_PI = 2.0 * math.pi


def resample_arrays(ranges, intensities, angle_min, angle_increment, num_beams):
    """Geeft (ranges, intensities) terug met `num_beams` stralen verdeeld over 0..2*pi (360 graden).

    Uitvoerstraal k = hoek k*2*pi/num_beams. De dichtstbijzijnde invoermeting wordt genomen; als de
    invoerscan geen volledige omwenteling dekt, blijven stralen buiten het bereik op inf (geen meting).
    `intensities` kan None zijn of een andere lengte hebben dan `ranges`: dan bevat de uitvoer geen intensiteiten.
    Pure Python: ~500 stralen bij 10 Hz rechtvaardigen geen afhankelijkheid van numpy.
    """
    n_in = len(ranges)
    out_inc = TWO_PI / num_beams
    full_circle = n_in * angle_increment >= TWO_PI - 1.5 * angle_increment
    use_int = intensities is not None and len(intensities) == n_in
    out_ranges = []
    out_int = [] if use_int else None
    for k in range(num_beams):
        phi = (k * out_inc - angle_min) % TWO_PI
        i = int(round(phi / angle_increment))
        if i >= n_in:
            if full_circle:
                i -= n_in
            else:
                out_ranges.append(math.inf)
                if use_int:
                    out_int.append(0.0)
                continue
        i = min(max(i, 0), n_in - 1)
        out_ranges.append(float(ranges[i]))
        if use_int:
            out_int.append(float(intensities[i]))
    return out_ranges, out_int


class ScanResampler(Node):
    def __init__(self):
        super().__init__("scan_resampler")
        self.declare_parameter("input_topic", "scan_raw")
        self.declare_parameter("output_topic", "scan")
        self.declare_parameter("num_beams", 360)
        self._n = int(self.get_parameter("num_beams").value)
        if self._n < 2:
            raise ValueError("num_beams moet >= 2 zijn")
        # RELIABLE koppelt met RELIABLE-abonnees (Nav2) en BEST_EFFORT-abonnees (slam_toolbox, RViz).
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value, out_qos
        )
        # Sensor-data (BEST_EFFORT) koppelt met een RELIABLE- of BEST_EFFORT-driver.
        self.create_subscription(
            LaserScan,
            self.get_parameter("input_topic").value,
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, msg):
        if len(msg.ranges) < 2 or msg.angle_increment <= 0.0:
            return
        ranges, intensities = resample_arrays(
            msg.ranges, msg.intensities, msg.angle_min, msg.angle_increment, self._n
        )
        out = LaserScan()
        out.header = msg.header
        out.angle_min = 0.0
        out.angle_increment = TWO_PI / self._n
        out.angle_max = out.angle_increment * (self._n - 1)
        out.scan_time = msg.scan_time
        out.time_increment = msg.scan_time / self._n if msg.scan_time > 0.0 else 0.0
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = intensities if intensities is not None else []
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ScanResampler()
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
