from sensor_msgs.msg import BatteryState
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Thresholds can be moved to config/constants
BATTERY_WARN = 0.30
BATTERY_CRIT = 0.15

class BatteryHandler(TelemetryHandler):
    def __init__(self, node, logger=None):
        super().__init__(logger)
        self._latest = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = node.create_subscription(
            BatteryState,
            '/ap/battery',
            self._callback,
            qos
        )

        self.logger.info("[BatteryHandler] Subscribed to /ap/battery.")

    def _callback(self, msg):
        self._latest = msg
        self.logger.debug(f"[BatteryHandler] Battery message received: Voltage={msg.voltage:.2f}V, Percentage={msg.percentage:.2%}")

    def get_serialized(self):
        if not self._latest:
            return None
        return {
            "battery": {
                "voltage": self._latest.voltage,
                "percentage": self._latest.percentage
            }
        }

    def get_health_status(self):
        """
        Returns: "OK", "WARN", or "CRIT"
        """
        if not self._latest:
            return "UNKNOWN"
        pct = self._latest.percentage
        if pct < BATTERY_CRIT:
            return "CRIT"
        elif pct < BATTERY_WARN:
            return "WARN"
        else:
            return "OK"
