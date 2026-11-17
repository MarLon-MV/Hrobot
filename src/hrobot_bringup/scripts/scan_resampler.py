#!/usr/bin/env python3
"""Remuestrea sensor_msgs/LaserScan a un numero FIJO de rayos.

El LD19 entrega entre ~500 y 504 lecturas por vuelta. slam_toolbox (Karto) fija el tamano con el primer
scan y avisa "LaserRangeScan contains N range readings, expected 504" con cualquier otro. Este nodo lee
`scan_raw` (driver) y publica `scan` con siempre `num_beams` rayos y los mismos angulos. Usa el vecino mas
cercano (no interpola, para no inventar distancias en los bordes de los obstaculos). Solo robot real.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TWO_PI = 2.0 * math.pi


def resample_arrays(ranges, intensities, angle_min, angle_increment, num_beams):
    """Devuelve (ranges, intensities) con `num_beams` rayos repartidos en 0..2*pi (360 grados).

    Rayo de salida k = angulo k*2*pi/num_beams. Se toma la lectura de entrada mas cercana; si el scan de
    entrada no cubre una vuelta completa, los rayos fuera de su rango quedan en inf (sin lectura).
    `intensities` puede ser None o de otro largo que `ranges`: entonces la salida no lleva intensidades.
    Python puro: ~500 rayos a 10 Hz no justifican depender de numpy.
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
        self.declare_parameter("num_beams", 504)
        self._n = int(self.get_parameter("num_beams").value)
        if self._n < 2:
            raise ValueError("num_beams debe ser >= 2")
        # RELIABLE empareja con suscriptores RELIABLE (Nav2) y BEST_EFFORT (slam_toolbox, RViz).
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value, out_qos
        )
        # Sensor-data (BEST_EFFORT) empareja con un driver RELIABLE o BEST_EFFORT.
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
