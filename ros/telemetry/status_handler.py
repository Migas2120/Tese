from rosidl_runtime_py.utilities import get_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ros_2_server.ros.telemetry.telemetry_handler import TelemetryHandler

class StatusHandler(TelemetryHandler):
    def __init__(self, node, logger=None):
        super().__init__(logger)
        self._latest = None

        msg_type = get_message('ardupilot_msgs/msg/Status')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub = node.create_subscription(
            msg_type,
            '/ap/status',
            self._callback,
            qos
        )

        self.logger.info("[StatusHandler] Subscribed to /ap/status with dynamic type.")

    def _callback(self, msg):
        self._latest = msg
        self.logger.debug("[StatusHandler] Status message received.")
        self.logger.debug(f"[StatusHandler] Message contents: {msg}")
        # Optional: log parsed fields for debug purposes
        self.logger.debug(f"[StatusHandler] Armed: {msg.armed}, Mode: {msg.mode}, Failsafe: {list(msg.failsafe)}")

    def get_serialized(self):
        if not self._latest:
            return None
        return {
            "status": {
                "armed": self._latest.armed,
                "flying": self._latest.flying,
                "mode": self._latest.mode,
                "external_control": self._latest.external_control,
                "failsafe": list(self._latest.failsafe)  # convert from uint8[] to list
            }
        }
    
    def get_health_status(self):
        if not self._latest:
            return "UNKNOWN"

        # If any failsafe flag is set, CRIT
        if self._latest.failsafe and any(self._latest.failsafe):
            return "CRIT"

        # (Optional) Check for certain problematic modes
        failsafe_modes = ["RTL", "LAND", "BRAKE", "FAILSAFE"]  # Add as needed
        if hasattr(self._latest, "mode") and str(self._latest.mode).upper() in failsafe_modes:
            return "WARN"

        # (Optional) Logic for inconsistent armed/flying status
        if self._latest.flying and not self._latest.armed:
            return "WARN"

        return "OK"