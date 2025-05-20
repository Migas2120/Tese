from geographic_msgs.msg import GeoPoseStamped  # Correct message type
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy
from rclpy.node import Node

import math

SAFA_DISTANCE = 150  # meters

def haversine(lat1, lon1, lat2, lon2):
    # Earth radius in meters
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.asin(math.sqrt(a))

class GpsPoseHandler(TelemetryHandler):
    def __init__(self, node, logger=None):
        super().__init__(logger)  
        self._latest = None
        self._home_position = None
        self._distance_from_home = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = node.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self._pose_callback,
            qos
        )
        self.logger.info("[GpsPoseHandler] Subscribed to /ap/geopose/filtered")

    def _pose_callback(self, msg):
        self._latest = msg
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        alt = msg.pose.position.altitude

        if self._home_position is None:
            self._home_position = (lat, lon, alt)
            self.logger.info(f"[GpsPoseHandler] Home position set: {lat}, {lon}, {alt}")

        self._distance_from_home = haversine(self._home_position[0], self._home_position[1], lat, lon)
        self.logger.debug(f"[GpsPoseHandler] Distance from home: {self._distance_from_home:.2f} m")

    def get_serialized(self) -> dict:
        if not self._latest:
            return {"status": "no_data"}

        return {
            "latitude": self._latest.pose.position.latitude,
            "longitude": self._latest.pose.position.longitude,
            "altitude": self._latest.pose.position.altitude,
            "distance_from_home": self._distance_from_home
        }

    def get_health_status(self):
        if not self._latest:
            return "UNKNOWN"

        # No GPS fix would typically be handled differently (this msg always comes if there's a fix)
        if self._distance_from_home > SAFA_DISTANCE:
            return "WARN"  # Or "CRIT" if you want to be strict
        else:
            return "OK"