from pymavlink import mavutil
from ros_2_server.ros.commands.base_command import BaseCommand

class GpsGoalCommand(BaseCommand):
    """
    ✅ Direct MAVLink injection version of GPS Goal command.

    This command connects once to the vehicle via MAVLink (bypassing ROS 2, DDS, MAVROS) and sends
    SET_POSITION_TARGET_GLOBAL_INT commands using pymavlink multiple times.

    Requirements:
    - Ensure ArduPilot SITL is exposing MAVLink on udp:127.0.0.1:14550
    """

    def __init__(self, logger=None):
        self.logger = logger
        self.master = None
        self._connect()

    def _connect(self):
        try:
            # Connect to vehicle once using UDP (adjust if needed)
            self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
            self.master.wait_heartbeat(timeout=5)
            if self.logger:
                self.logger.info(f"[GPS_GOAL_MAVLINK] Connected to system {self.master.target_system} component {self.master.target_component}")
            else:
                print(f"[GPS_GOAL_MAVLINK] Connected to system {self.master.target_system} component {self.master.target_component}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"[GPS_GOAL_MAVLINK] Connection failed: {e}")
            else:
                print(f"[GPS_GOAL_MAVLINK] Connection failed: {e}")
            self.master = None

    def execute(self, ros, data: dict):
        if not self.master:
            self._connect()
            if not self.master:
                if self.logger:
                    self.logger.error("[GPS_GOAL_MAVLINK] No connection to vehicle.")
                return

        lat = float(data.get("latitude", 0.0))
        lon = float(data.get("longitude", 0.0))
        alt = float(data.get("altitude", 10.0))
        self._send_gps_goal(lat, lon, alt)

    def _send_gps_goal(self, lat, lon, alt):
        try:
            # Send SET_POSITION_TARGET_GLOBAL_INT
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (ignored)
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # Only position enabled
                int(lat * 1e7),
                int(lon * 1e7),
                alt, 
                0, 0, 0,  # velocities ignored
                0, 0, 0,  # acceleration ignored
                0, 0      # yaw and yaw rate ignored
            )
            if self.logger:
                self.logger.info(f"[GPS_GOAL_MAVLINK] Command sent to {lat},{lon},{alt}")
            else:
                print(f"[GPS_GOAL_MAVLINK] Command sent to {lat},{lon},{alt}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"[GPS_GOAL_MAVLINK] Failed to send goal: {e}")
            else:
                print(f"[GPS_GOAL_MAVLINK] Failed to send goal: {e}")

    def close(self):
        if self.master:
            try:
                self.master.close()
                if self.logger:
                    self.logger.info("[GPS_GOAL_MAVLINK] Connection closed.")
                else:
                    print("[GPS_GOAL_MAVLINK] Connection closed.")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[GPS_GOAL_MAVLINK] Failed to close connection: {e}")
                else:
                    print(f"[GPS_GOAL_MAVLINK] Failed to close connection: {e}")
            self.master = None