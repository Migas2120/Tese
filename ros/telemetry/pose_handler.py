from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler
import math

class PoseHandler(TelemetryHandler):
    DISTANCE_WARN = 150  # meters
    ALTITUDE_CRIT = 120  # meters (example: local drone regs)

    def __init__(self, node, logger=None):
        super().__init__(logger)
        self._latest = None
        self._home_position = None
        self._distance_from_home = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = node.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self._callback,
            qos
        )

        self.logger.info("[PoseHandler] Subscribed to /ap/pose/filtered.")

    def _callback(self, msg):
        self._latest = msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self._home_position is None:
            self._home_position = (x, y, z)
            self.logger.info(f"[PoseHandler] Home position set: {x}, {y}, {z}")

        self._distance_from_home = math.sqrt(
            (x - self._home_position[0])**2 +
            (y - self._home_position[1])**2 +
            (z - self._home_position[2])**2
        )

        self.logger.debug("[PoseHandler] Pose message received.")
        self.logger.debug(
            f"[PoseHandler] Position: ({x:.2f}, {y:.2f}, {z:.2f}), "
            f"Orientation: ({msg.pose.orientation.x:.2f}, {msg.pose.orientation.y:.2f}, "
            f"{msg.pose.orientation.z:.2f}, {msg.pose.orientation.w:.2f})"
        )
        self.logger.debug(f"[PoseHandler] Distance from home: {self._distance_from_home:.2f} m")

    def get_serialized(self):
        if not self._latest:
            return None
        pose = self._latest.pose
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            },
            "distance_from_home": self._distance_from_home
        }

    def get_health_status(self):
        if not self._latest:
            return "UNKNOWN"

        pose = self._latest.pose
        z = pose.position.z
        # Altitude check (optional, based on your flight limits)
        if z > self.ALTITUDE_CRIT:
            return "CRIT"

        # Distance from home check
        if self._distance_from_home > self.DISTANCE_WARN:
            return "WARN"
        else:
            return "OK"