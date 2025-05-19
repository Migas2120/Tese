from geographic_msgs.msg import GeoPoseStamped  # Correct message type
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy
from rclpy.node import Node

class GpsPoseHandler(TelemetryHandler):
    def __init__(self, node, logger=None):
        super().__init__(logger)  
        self._latest = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = node.create_subscription(
            GeoPoseStamped,  # Correct type here
            '/ap/geopose/filtered',  # Correct topic
            self._pose_callback,
            qos
        )
        self.logger.info("[GpsPoseHandler] Subscribed to /ap/geopose/filtered")

    def _pose_callback(self, msg):
        self._latest = msg
        # Correct access: msg.pose.position
        self.logger.debug(f"[GpsPoseHandler] Updated GPS pose: {msg.pose.position.latitude}, {msg.pose.position.longitude}, {msg.pose.position.altitude}")

    def get_serialized(self) -> dict:
        if not self._latest:
            return {"status": "no_data"}

        return {
            "latitude": self._latest.pose.position.latitude,
            "longitude": self._latest.pose.position.longitude,
            "altitude": self._latest.pose.position.altitude
        }