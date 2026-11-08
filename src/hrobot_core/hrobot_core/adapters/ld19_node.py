"""Nodo ROS 2 del LiDAR LD19: lee el UART, decodifica y publica ``/scan``.

Capa delgada: la decodificación vive en ``Ld19Parser`` y la rejilla fija en ``ScanBuilder``.
"""

from __future__ import annotations

from typing import Any

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from hrobot_core.adapters.ld19_parser import Ld19Parser
from hrobot_core.adapters.names import (
    FRAME_LASER,
    LD19_BAUDRATE,
    LD19_DEFAULT_PORT,
    SCAN_TOPIC,
)
from hrobot_core.adapters.scan_builder import ScanBuilder, ScanBuilderConfig, ScanFrame

NODE_NAME = "ld19_node"
READ_CHUNK_BYTES = 4096
NS_PER_S = 1_000_000_000
NO_DATA_FACTOR = 3.0  # advertencia tras NO_DATA_FACTOR * reconnect_period_s sin bytes

DEFAULT_BINS = 360
DEFAULT_RANGE_MIN_M = 0.02
DEFAULT_RANGE_MAX_M = 12.0  # alcance nominal del LD19: verificar con datasheet
DEFAULT_READ_PERIOD_S = 0.01
DEFAULT_RECONNECT_PERIOD_S = 2.0
DEFAULT_STATS_PERIOD_S = 10.0


class Ld19Node(Node):
    """Publica ``sensor_msgs/LaserScan`` a partir del flujo serie del LD19."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self._declare_parameters()
        # ScanBuilderConfig lanza ValueError con parámetros inválidos: no se captura.
        self._cfg = ScanBuilderConfig(
            bins=int(self._param("bins")),
            range_min=float(self._param("range_min")),
            range_max=float(self._param("range_max")),
        )
        self._port = str(self._param("port"))
        self._baudrate = int(self._param("baudrate"))
        self._frame_id = str(self._param("frame_id"))
        self._reconnect_period_s = float(self._param("reconnect_period_s"))

        self._parser = Ld19Parser()
        self._builder = ScanBuilder(self._cfg)
        self._serial: Any = None
        self._crc_base = 0
        self._scans_in_period = 0
        self._last_data_ns = 0
        self._no_data_warned = False

        self._pub = self.create_publisher(
            LaserScan, str(self._param("scan_topic")), qos_profile_sensor_data
        )
        self._open_port()
        self.create_timer(float(self._param("read_period_s")), self._on_read_timer)
        self.create_timer(self._reconnect_period_s, self._on_reconnect_timer)
        self.create_timer(float(self._param("stats_period_s")), self._on_stats_timer)

    # ------------------------------------------------------------------ parámetros
    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "port": LD19_DEFAULT_PORT,
            "baudrate": LD19_BAUDRATE,
            "frame_id": FRAME_LASER,
            "scan_topic": SCAN_TOPIC,
            "bins": DEFAULT_BINS,
            "range_min": DEFAULT_RANGE_MIN_M,
            "range_max": DEFAULT_RANGE_MAX_M,
            "read_period_s": DEFAULT_READ_PERIOD_S,
            "reconnect_period_s": DEFAULT_RECONNECT_PERIOD_S,
            "stats_period_s": DEFAULT_STATS_PERIOD_S,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ puerto serie
    def _open_port(self) -> None:
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=0)
        except (serial.SerialException, OSError) as exc:
            self._serial = None
            self.get_logger().error(f"cannot open LD19 port {self._port}: {exc}")
            return
        self._last_data_ns = self._now_ns()
        self._no_data_warned = False
        self.get_logger().info(f"LD19 port {self._port} opened at {self._baudrate} baud")

    def close_port(self) -> None:
        """Cierra el puerto serie si está abierto."""
        port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f"error closing LD19 port: {exc}")

    def _reset_pipeline(self) -> None:
        self._crc_base += self._parser.crc_errors
        self._parser = Ld19Parser()
        self._builder.reset()

    def _on_reconnect_timer(self) -> None:
        if self._serial is None:
            self._open_port()

    # ------------------------------------------------------------------ lectura
    def _on_read_timer(self) -> None:
        if self._serial is None:
            return
        try:
            data = self._serial.read(READ_CHUNK_BYTES)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"LD19 read failed, will reconnect: {exc}")
            self.close_port()
            self._reset_pipeline()
            return
        now_ns = self._now_ns()
        if data:
            self._last_data_ns = now_ns
            self._no_data_warned = False
            self._process(bytes(data), now_ns)
        elif not self._no_data_warned and (
            now_ns - self._last_data_ns > NO_DATA_FACTOR * self._reconnect_period_s * NS_PER_S
        ):
            self._no_data_warned = True
            self.get_logger().warning("no data received from LD19")

    def _process(self, data: bytes, now_ns: int) -> None:
        for packet in self._parser.feed(data):
            for frame in self._builder.add_packet(packet):
                self._publish(frame, now_ns)

    def _publish(self, frame: ScanFrame, close_ns: int) -> None:
        bins = len(frame.ranges)
        start_ns = close_ns - round(frame.scan_time_s * NS_PER_S)
        msg = LaserScan()
        msg.header.frame_id = self._frame_id
        msg.header.stamp.sec = start_ns // NS_PER_S
        msg.header.stamp.nanosec = start_ns % NS_PER_S
        msg.angle_min = float(frame.angle_min)
        msg.angle_max = float(frame.angle_max)
        msg.angle_increment = float(frame.angle_increment)
        msg.time_increment = float(frame.scan_time_s / bins)
        msg.scan_time = float(frame.scan_time_s)
        msg.range_min = float(frame.range_min)
        msg.range_max = float(frame.range_max)
        msg.ranges = [float(r) for r in frame.ranges]
        msg.intensities = [float(i) for i in frame.intensities]
        self._pub.publish(msg)
        self._scans_in_period += 1

    # ------------------------------------------------------------------ estadísticas
    def _on_stats_timer(self) -> None:
        crc = self._crc_base + self._parser.crc_errors
        self.get_logger().info(
            f"ld19 stats: crc_errors={crc} "
            f"dropped_revolutions={self._builder.dropped_revolutions} "
            f"scans={self._scans_in_period}"
        )
        self._scans_in_period = 0

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)


def main(args: list[str] | None = None) -> None:
    """Punto de entrada del ejecutable ``ld19_node``."""
    rclpy.init(args=args)
    node: Ld19Node | None = None
    try:
        node = Ld19Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.close_port()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
