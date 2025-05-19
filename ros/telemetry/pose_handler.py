from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler

class PoseHandler(TelemetryHandler):
    def __init__(self, node, logger=None):
        super().__init__(logger)
        self._latest = None

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
        self.logger.debug("[PoseHandler] Pose message received.")
        self.logger.debug(
            f"[PoseHandler] Position: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}), "
            f"Orientation: ({msg.pose.orientation.x:.2f}, {msg.pose.orientation.y:.2f}, "
            f"{msg.pose.orientation.z:.2f}, {msg.pose.orientation.w:.2f})"
        )

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
            }
        }
