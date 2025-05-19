"""
telemetry_manager.py

Periodically collects telemetry data from ROS 2 (status, pose, battery)
and sends it to Unity via a TCP client. This class manages the timing loop
and JSON payload formatting.

Each data source is handled by a dedicated handler:
- StatusHandler → operational mode, armed state, etc.
- PoseHandler → drone's current position
- BatteryHandler → remaining battery and voltage

Telemetry is sent out at a configurable interval (default: 1 second).
"""

import json
import time
import logging
from threading import Timer

from ros_2_server.ros.telemetry.status_handler import StatusHandler
from ros_2_server.ros.telemetry.pose_handler import PoseHandler
from ros_2_server.ros.telemetry.battery_handler import BatteryHandler
from ros_2_server.ros.telemetry.gps_pose_handler import GpsPoseHandler  

class TelemetryManager:
    """
    Collects and sends drone telemetry data over TCP at regular intervals.
    """

    def __init__(self, node, tcp_client, logger=None, interval_sec=1):
        """
        Parameters:
        - node: ROS 2 Node used by handlers to subscribe to data
        - tcp_client: TCP client responsible for sending data to Unity
        - logger: optional logging.Logger instance
        - interval_sec: how often to send telemetry (in seconds)
        """
        self.node = node
        self.tcp_client = tcp_client
        self.interval_sec = interval_sec
        self.logger = logger or logging.getLogger(__name__)

        # Debug-only dev tool: throttle telemetry logging
        self._last_info_log_time = 0
        self._info_log_interval = 3  # seconds

        # Telemetry handlers
        self.status = StatusHandler(node)
        self.pose = PoseHandler(node)
        self.battery = BatteryHandler(node)
        self.gps = GpsPoseHandler(node)

        self.logger.info("[TelemetryManager] Initialized.")
        self._start_timer()

    def _start_timer(self):
        """
        Starts the periodic telemetry loop using threading.Timer.
        Restarts itself after each run.
        """
        self.timer = Timer(self.interval_sec, self._send_telemetry)
        self.timer.daemon = True
        self.timer.start()
        self.logger.debug("[TelemetryManager] Telemetry timer started.")

    def _send_telemetry(self):
        """
        Gathers telemetry from handlers and sends a structured JSON payload
        to Unity via TCP. Logs the result for development/debug purposes.
        """
        status_data = self.status.get_serialized()
        pose_data = self.pose.get_serialized()
        battery_data = self.battery.get_serialized()
        gps_data = self.gps.get_serialized()

        payload = {
            "status": status_data.get("status") if status_data else None,
            "pose": pose_data,
            "battery": battery_data.get("battery") if battery_data else None,
            "gps_pose": gps_data
        }

        try:
            json_str = json.dumps(payload, indent=2)
            self.logger.debug("[TelemetryManager] Payload to Unity:")
            self.logger.debug(json_str)

            # Uncomment to activate real data transmission:
            # self.tcp_client.sendall(json_str.encode())

        except Exception as e:
            self.logger.error(f"[TelemetryManager] Failed to serialize/send telemetry: {e}")

        # Restart the timer to keep looping
        self._start_timer()
