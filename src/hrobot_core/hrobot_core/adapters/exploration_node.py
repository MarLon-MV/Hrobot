"""Nodo de exploración autónoma: capa delgada sobre ``ExplorationManager``.

Lee ``/map``, obtiene la pose por tf2 (``map -> base_link``), envía objetivos a Nav2 por la
acción ``navigate_to_pose`` y guarda el mapa con ``nav2_map_server``. Nunca publica
velocidades: Nav2 usa ``/cmd_vel_nav`` y solo ``supervisor_node`` publica ``/cmd_vel``.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from functools import partial
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus  # type: ignore[import-not-found]  # paquete ROS sin stubs
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock  # type: ignore[import-not-found]  # paquete ROS sin stubs
from irobot_create_msgs.msg import (  # type: ignore[import-not-found]  # paquete ROS sin stubs
    DockStatus,
    HazardDetection,
    HazardDetectionVector,
    KidnapStatus,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid  # type: ignore[import-not-found]  # paquete ROS sin stubs
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger  # type: ignore[import-not-found]  # paquete ROS sin stubs
from tf2_ros import (  # type: ignore[import-not-found]  # paquete ROS sin stubs
    Buffer,
    TransformException,
    TransformListener,
)

from hrobot_core.adapters.names import (
    BATTERY_STATE_TOPIC,
    DEFAULT_CREATE3_NAMESPACE,
    DOCK_STATUS_TOPIC,
    EXPLORATION_STATUS_TOPIC,
    FRAME_BASE,
    FRAME_MAP,
    HAZARD_DETECTION_TOPIC,
    KIDNAP_STATUS_TOPIC,
    MAP_SAVER_SERVICE,
    MAP_TOPIC,
    NAVIGATE_TO_POSE_ACTION,
    UNDOCK_ACTION,
    namespaced,
)
from hrobot_core.domain.exploration import ExplorationConfig, GridMap, Point2D
from hrobot_core.services.exploration_manager import (
    ExplorationManager,
    InvalidTransitionError,
    ManagerConfig,
    RobotStatus,
)

NODE_NAME = "exploration_node"
START_SERVICE = "~/start"
STOP_SERVICE = "~/stop"
QOS_DEPTH = 1
MAX_ORIGIN_YAW_RAD = 1e-3  # la conversión a GridMap no soporta orígenes rotados
GOAL_ORIENTATION_W = 1.0  # orientación identidad: Nav2 elige el rumbo final libremente
SAVE_MAP_MODE = "trinary"
SAVE_MAP_FORMAT = ""  # vacío: formato por defecto de map_saver
SAVE_FREE_THRESH = 0.25  # coherente con ExplorationConfig.free_max (25)
SAVE_OCCUPIED_THRESH = 0.65  # coherente con ExplorationConfig.occupied_min (65)
USE_SIM_TIME_PARAM = "use_sim_time"
NS_PER_S = 1_000_000_000
PERCENT_PER_FRACTION = 100.0  # BatteryState.percentage es 0..1

_EXPLORATION_DEFAULTS = ExplorationConfig()
_MANAGER_DEFAULTS = ManagerConfig()
_PARAM_DEFAULTS: dict[str, Any] = {
    "min_frontier_size": _EXPLORATION_DEFAULTS.min_frontier_size,
    "blacklist_radius": _EXPLORATION_DEFAULTS.blacklist_radius,
    "min_goal_distance": _EXPLORATION_DEFAULTS.min_goal_distance,
    "exploration_period_s": 2.0,
    "max_consecutive_failures": _MANAGER_DEFAULTS.max_consecutive_failures,
    "no_frontier_confirmations": _MANAGER_DEFAULTS.no_frontier_confirmations,
    "map_save_name": _MANAGER_DEFAULTS.map_save_name,
    "map_save_timeout_s": 10.0,
    "create3_namespace": DEFAULT_CREATE3_NAMESPACE,
    "undock_timeout_s": _MANAGER_DEFAULTS.undock_timeout_s,
    "status_timeout_s": _MANAGER_DEFAULTS.status_timeout_s,
    "kidnap_clear_cycles": _MANAGER_DEFAULTS.kidnap_clear_cycles,
    "min_battery_pct": _MANAGER_DEFAULTS.min_battery_pct,
}
_INT_PARAMS = (
    "min_frontier_size",
    "max_consecutive_failures",
    "no_frontier_confirmations",
    "kidnap_clear_cycles",
)
_FLOAT_PARAMS = (
    "blacklist_radius",
    "min_goal_distance",
    "exploration_period_s",
    "map_save_timeout_s",
    "undock_timeout_s",
    "status_timeout_s",
)
HAZARD_TYPE_NAMES = ("BUMP", "CLIFF", "STUCK")  # el resto (BACKUP_LIMIT, ...) se ignora


def occupancy_to_grid(msg: Any) -> GridMap:
    """Convierte un ``nav_msgs/OccupancyGrid`` (duck-typed) en ``GridMap``.

    Raises:
        ValueError: si las dimensiones/resolución son inválidas, si el origen está
            rotado o si ``data`` no coincide con ``width*height``.
    """
    info = msg.info
    q = info.origin.orientation
    yaw = 2.0 * math.atan2(float(q.z), float(q.w))
    if not math.isfinite(yaw) or abs(yaw) > MAX_ORIGIN_YAW_RAD:
        raise ValueError("OccupancyGrid con origen rotado no soportado")
    return GridMap(
        width=int(info.width),
        height=int(info.height),
        resolution=float(info.resolution),
        origin_x=float(info.origin.position.x),
        origin_y=float(info.origin.position.y),
        data=tuple(int(v) for v in msg.data),
    )


def build_configs(values: dict[str, Any]) -> tuple[ManagerConfig, float, float]:
    """Valida los parámetros y construye la configuración.

    Returns:
        ``(ManagerConfig, exploration_period_s, map_save_timeout_s)``.

    Raises:
        ValueError: si un parámetro tiene tipo o rango inválido.
    """
    for name in _INT_PARAMS:
        if isinstance(values[name], bool) or not isinstance(values[name], int):
            raise ValueError(f"parameter {name} must be an int, got {values[name]!r}")
    for name in _FLOAT_PARAMS:
        v = values[name]
        if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v) or v <= 0:
            raise ValueError(f"parameter {name} must be a finite number > 0, got {v!r}")
    if not isinstance(values["map_save_name"], str):
        raise ValueError("parameter map_save_name must be a str")
    if not isinstance(values["create3_namespace"], str):
        raise ValueError("parameter create3_namespace must be a str")
    namespaced(values["create3_namespace"], UNDOCK_ACTION)  # ValueError si es inválido
    battery = values["min_battery_pct"]
    if isinstance(battery, bool) or not isinstance(battery, int | float):
        raise ValueError(f"parameter min_battery_pct must be a number, got {battery!r}")
    if values["min_frontier_size"] < 1:
        raise ValueError("parameter min_frontier_size must be >= 1")
    cfg = ManagerConfig(
        exploration=ExplorationConfig(
            min_frontier_size=values["min_frontier_size"],
            blacklist_radius=float(values["blacklist_radius"]),
            min_goal_distance=float(values["min_goal_distance"]),
        ),
        max_consecutive_failures=values["max_consecutive_failures"],
        no_frontier_confirmations=values["no_frontier_confirmations"],
        map_save_name=values["map_save_name"],
        undock_timeout_s=float(values["undock_timeout_s"]),
        status_timeout_s=float(values["status_timeout_s"]),
        kidnap_clear_cycles=values["kidnap_clear_cycles"],
        min_battery_pct=float(battery),
    )
    return cfg, float(values["exploration_period_s"]), float(values["map_save_timeout_s"])


class Nav2Navigator:
    """``NavigatorPort`` sobre la acción ``navigate_to_pose`` (resultados asíncronos)."""

    def __init__(
        self, node: Any, client: Any, on_result: Callable[[bool], None], frame_id: str
    ) -> None:
        self._node = node
        self._client = client
        self._on_result = on_result
        self._frame_id = frame_id
        self._seq = 0  # invalida callbacks de objetivos ya cancelados o reemplazados
        self._handle: Any = None

    def send_goal(self, goal: Point2D) -> None:
        """Envía ``goal``; si el servidor no está listo, informa un fallo."""
        self._seq += 1
        self._handle = None
        if not self._client.server_is_ready():
            self._node.get_logger().warning("navigate_to_pose action server not ready")
            self._on_result(False)
            return
        msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.w = GOAL_ORIENTATION_W
        msg.pose = pose
        future = self._client.send_goal_async(msg)
        future.add_done_callback(partial(self._on_response, self._seq))

    def cancel(self) -> None:
        """Cancela el objetivo activo y descarta sus resultados pendientes."""
        self._seq += 1
        handle, self._handle = self._handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def _on_response(self, seq: int, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # cualquier error del futuro cuenta como fallo
            self._node.get_logger().warning(f"send_goal failed: {exc}")
            if seq == self._seq:
                self._on_result(False)
            return
        if seq != self._seq:
            if handle.accepted:
                handle.cancel_goal_async()  # objetivo obsoleto: no debe quedar en curso
            return
        if not handle.accepted:
            self._on_result(False)
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    def _on_done(self, seq: int, future: Any) -> None:
        if seq != self._seq:
            return
        self._handle = None
        try:
            status = future.result().status
        except Exception as exc:  # cualquier error del futuro cuenta como fallo
            self._node.get_logger().warning(f"goal result failed: {exc}")
            self._on_result(False)
            return
        self._on_result(status == GoalStatus.STATUS_SUCCEEDED)


class Nav2MapSaver:
    """``MapSaverPort`` sobre el servicio de ``nav2_map_server``.

    Espera la respuesta con un ``threading.Event``: requiere un ejecutor multihilo y que el
    cliente esté en un grupo de callbacks distinto del que ejecuta la llamada.
    """

    def __init__(self, node: Any, client: Any, timeout_s: float) -> None:
        self._node = node
        self._client = client
        self._timeout_s = timeout_s

    def save(self, name: str) -> bool:
        """Guarda el mapa; ``False`` si el servicio no está, expira o falla."""
        if not self._client.service_is_ready():
            self._node.get_logger().warning("map saver service not available")
            return False
        req = SaveMap.Request()
        req.map_topic = MAP_TOPIC
        req.map_url = name
        req.image_format = SAVE_MAP_FORMAT
        req.map_mode = SAVE_MAP_MODE
        req.free_thresh = SAVE_FREE_THRESH
        req.occupied_thresh = SAVE_OCCUPIED_THRESH
        done = threading.Event()
        future = self._client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout_s):
            self._node.get_logger().warning("map save timed out")
            future.cancel()
            return False
        try:
            return bool(future.result().result)
        except Exception as exc:  # cualquier error del futuro cuenta como fallo
            self._node.get_logger().warning(f"map save failed: {exc}")
            return False


class Create3Dock:
    """``DockPort`` sobre la acción ``undock`` del Create 3 (resultado asíncrono).

    Es la única acción del Create 3 que se usa: no hay ``drive_*`` ni ``cmd_vel``.
    """

    def __init__(self, node: Any, client: Any, on_result: Callable[[bool], None]) -> None:
        self._node = node
        self._client = client
        self._on_result = on_result
        self._seq = 0  # invalida callbacks de undocks ya cancelados o reemplazados
        self._handle: Any = None

    def undock(self) -> None:
        """Envía el undock; si el servidor no está listo, informa un fallo."""
        self._seq += 1
        self._handle = None
        if not self._client.server_is_ready():
            self._node.get_logger().warning("undock action server not ready")
            self._on_result(False)
            return
        future = self._client.send_goal_async(Undock.Goal())
        future.add_done_callback(partial(self._on_response, self._seq))

    def cancel(self) -> None:
        """Cancela el undock activo y descarta sus resultados pendientes."""
        self._seq += 1
        handle, self._handle = self._handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def _on_response(self, seq: int, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # cualquier error del futuro cuenta como fallo
            self._node.get_logger().warning(f"undock send_goal failed: {exc}")
            if seq == self._seq:
                self._on_result(False)
            return
        if seq != self._seq:
            if handle.accepted:
                handle.cancel_goal_async()  # undock obsoleto: no debe quedar en curso
            return
        if not handle.accepted:
            self._on_result(False)
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    def _on_done(self, seq: int, future: Any) -> None:
        if seq != self._seq:
            return
        self._handle = None
        try:
            status = future.result().status
        except Exception as exc:  # cualquier error del futuro cuenta como fallo
            self._node.get_logger().warning(f"undock result failed: {exc}")
            self._on_result(False)
            return
        self._on_result(status == GoalStatus.STATUS_SUCCEEDED)


class ExplorationNode(Node):  # type: ignore[misc]  # Node es Any (rclpy sin stubs)
    """Nodo ROS 2 de exploración autónoma."""

    def __init__(self) -> None:
        """Declara parámetros, crea clientes, servicios, suscripción y temporizador.

        Raises:
            ValueError: si algún parámetro es inválido (el nodo falla al arrancar).
        """
        super().__init__(NODE_NAME)
        values = {n: self.declare_parameter(n, d).value for n, d in _PARAM_DEFAULTS.items()}
        if not self.has_parameter(USE_SIM_TIME_PARAM):  # rclpy ya lo declara normalmente
            self.declare_parameter(USE_SIM_TIME_PARAM, False)
        cfg, period_s, save_timeout_s = build_configs(values)
        self._warned: set[str] = set()
        self._grid: GridMap | None = None
        # (valor, hora de recepción en s); None = aún sin datos (política conservadora)
        # lock de las muestras: las suscripciones y el temporizador corren en hilos distintos
        self._sample_lock = threading.Lock()
        self._docked: tuple[bool, float] | None = None
        self._kidnapped: tuple[bool, float] | None = None
        self._battery_pct: tuple[float, float] | None = None
        self._status_max_age_s = cfg.status_timeout_s  # pasado este tiempo el dato caduca
        ns = str(values["create3_namespace"])
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        navigator = Nav2Navigator(
            self,
            ActionClient(self, NavigateToPose, NAVIGATE_TO_POSE_ACTION),
            self._on_goal_result,
            FRAME_MAP,
        )
        save_client = self.create_client(
            SaveMap, MAP_SAVER_SERVICE, callback_group=ReentrantCallbackGroup()
        )
        saver = Nav2MapSaver(self, save_client, save_timeout_s)
        dock = Create3Dock(
            self,
            ActionClient(self, Undock, namespaced(ns, UNDOCK_ACTION)),
            self._on_undock_result,
        )
        self._manager = ExplorationManager(navigator, saver, dock, cfg, self._now_s)
        self._create_robot_subscriptions(ns)
        latched = QoSProfile(
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, latched)
        self._status_pub = self.create_publisher(String, EXPLORATION_STATUS_TOPIC, latched)
        self.create_service(Trigger, START_SERVICE, self._on_start)
        self.create_service(Trigger, STOP_SERVICE, self._on_stop)
        self._timer = self.create_timer(period_s, self._on_timer)

    # ---- avisos sin spam: se registra al aparecer la condición y de nuevo tras resolverse
    def _warn_once(self, key: str, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(text)

    def _clear_warning(self, key: str) -> None:
        self._warned.discard(key)

    def _now_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) / NS_PER_S

    def _create_robot_subscriptions(self, ns: str) -> None:
        qos = qos_profile_sensor_data  # best effort, como los sensores del Create 3
        self.create_subscription(DockStatus, namespaced(ns, DOCK_STATUS_TOPIC), self._on_dock, qos)
        self.create_subscription(
            KidnapStatus, namespaced(ns, KIDNAP_STATUS_TOPIC), self._on_kidnap, qos
        )
        self.create_subscription(
            BatteryState, namespaced(ns, BATTERY_STATE_TOPIC), self._on_battery, qos
        )
        self.create_subscription(
            HazardDetectionVector, namespaced(ns, HAZARD_DETECTION_TOPIC), self._on_hazards, qos
        )

    def _fresh(self, sample: tuple[Any, float] | None) -> Any:
        """Valor de la muestra, o ``None`` si falta o superó ``status_timeout_s`` (caducada)."""
        if sample is None:
            return None
        value, stamp = sample
        return value if self._now_s() - stamp <= self._status_max_age_s else None

    def _robot_status(self) -> RobotStatus:
        with self._sample_lock:
            return RobotStatus(
                self._fresh(self._docked),
                self._fresh(self._kidnapped),
                self._fresh(self._battery_pct),
            )

    def _on_dock(self, msg: Any) -> None:
        sample = (bool(msg.is_docked), self._now_s())
        with self._sample_lock:
            self._docked = sample

    def _on_kidnap(self, msg: Any) -> None:
        sample = (bool(msg.is_kidnapped), self._now_s())
        with self._sample_lock:
            self._kidnapped = sample

    def _on_battery(self, msg: Any) -> None:
        pct = float(msg.percentage) * PERCENT_PER_FRACTION  # NaN => desconocido
        sample = (pct, self._now_s())
        with self._sample_lock:
            self._battery_pct = sample

    def _on_hazards(self, msg: Any) -> None:
        hazard_types = {getattr(HazardDetection, name) for name in HAZARD_TYPE_NAMES}
        if any(d.type in hazard_types for d in msg.detections):
            self._manager.on_hazard(self._lookup_robot())

    def _on_goal_result(self, success: bool) -> None:
        self._manager.on_goal_result(success)

    def _on_undock_result(self, success: bool) -> None:
        self._manager.on_undock_result(success)

    def _on_map(self, msg: Any) -> None:
        try:
            self._grid = occupancy_to_grid(msg)
        except ValueError as exc:
            self._grid = None  # fallo seguro: no explorar con un mapa inválido
            self._warn_once("map_invalid", f"invalid /map: {exc}")
            return
        self._clear_warning("map_invalid")

    def _lookup_robot(self) -> Point2D | None:
        try:
            tf = self._buffer.lookup_transform(FRAME_MAP, FRAME_BASE, Time())
        except TransformException as exc:
            self._warn_once("tf", f"TF {FRAME_MAP}->{FRAME_BASE} unavailable: {exc}")
            return None
        self._clear_warning("tf")
        return Point2D(float(tf.transform.translation.x), float(tf.transform.translation.y))

    def _on_timer(self) -> None:
        grid = self._grid
        if grid is None:
            self._warn_once("map", f"waiting for {MAP_TOPIC}")
        else:
            self._clear_warning("map")
            robot = self._lookup_robot()
            if robot is not None:
                self._manager.step(grid, robot, self._robot_status())
        self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        reason = self._manager.reason
        msg.data = self._manager.state.value + (f": {reason}" if reason else "")
        self._status_pub.publish(msg)

    def _on_start(self, _request: Any, response: Any) -> Any:
        try:
            self._manager.start(self._robot_status())
        except InvalidTransitionError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = "exploration started"
        return response

    def _on_stop(self, _request: Any, response: Any) -> Any:
        self._manager.stop()
        response.success = True
        response.message = "exploration stopped"
        return response

    def shutdown_cancel(self) -> None:
        """Cancela el objetivo activo (se usa al terminar el proceso)."""
        self._manager.stop()


def main(args: list[str] | None = None) -> None:
    """Punto de entrada: ejecutor multihilo para que el guardado de mapa no bloquee."""
    rclpy.init(args=args)
    node: ExplorationNode | None = None
    try:
        node = ExplorationNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        try:
            if node is not None:
                node.shutdown_cancel()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
