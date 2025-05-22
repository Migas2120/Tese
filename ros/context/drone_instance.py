import threading
import rclpy
import os, math
import datetime
from rclpy.executors import SingleThreadedExecutor
from ros_2_server.ros.node_wrapper import MiddleManNode
from ros_2_server.tcp.client import TCPClient
from ros_2_server.ros.telemetry.telemetry_manager import TelemetryManager
from ros_2_server.ros.health.health_manager import DroneHealthManager


class DroneInstance:
    """
    Represents a single drone running inside its own ROS_DOMAIN_ID,
    with its own ROS context, MiddleManNode, TelemetryManager, and TCP client.
    """

    def __init__(self, domain_id, tcp_host, tcp_port, logger):
        """
        Initialize a new drone instance inside the specified ROS_DOMAIN_ID.

        :param domain_id: Unique ROS_DOMAIN_ID for this drone.
        :param tcp_host: IP address of Unity (or telemetry receiver).
        :param tcp_port: TCP port Unity expects telemetry.
        :param logger: Logger instance.
        """
        self.domain_id = domain_id
        self.logger = logger
        self.logfile = f"logs/telemetry_drone0_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl"

         # === Set environment variable for domain ID (correct way for context isolation)
        os.environ['ROS_DOMAIN_ID'] = str(self.domain_id)

        # === ROS context (per domain) ===
        self.ros_context = rclpy.Context()
        self.ros_context.init()
        self._log("info", f"ROS context created in domain {self.domain_id}.")

        # === MiddleManNode ===
        self.node = MiddleManNode(logger=self.logger, context=self.ros_context, drone_id=self.domain_id)
        self._log("debug", f"MiddleManNode created.")

        # === TCP Client ===
        self.tcp_client = TCPClient(host=tcp_host, port=tcp_port, logger=self.logger)
        self.tcp_thread = threading.Thread(target=self.tcp_client.run, daemon=True)
        self.tcp_thread.start()
        self._log("debug", f"TCP client thread started.")

        # Link TCP client to ROS node
        self.node.set_tcp_client(self.tcp_client)

        # === Telemetry Manager ===
        self.telemetry_manager = TelemetryManager(self.node, self.tcp_client, logger=self.logger, telemetry_log_file=self.logfile)
        self.node.set_pose_handler(self.telemetry_manager.gps)
        self._log("info", f"TelemetryManager initialized.")

        # === Drone Health MOnotoring ===
        battery_handler = self.telemetry_manager.battery
        gps_handler = self.telemetry_manager.gps
        pose_handler = self.telemetry_manager.pose
        status_handler = self.telemetry_manager.status

        self.health_manager = DroneHealthManager(
            battery_handler, gps_handler, pose_handler, status_handler, logger=self.logger
        )
        self._log("info", f"DroneHealthManager initialized.")

        # === ROS Node thread ===
        self.node_thread = threading.Thread(target=self._spin_node, daemon=True)
        self.node_thread.start()
        self._log("info", f"ROS node spinning in separate thread.")

    
    def _spin_node(self):
        """
        Spin the ROS node in a separate thread using SingleThreadedExecutor.
        This allows the node to process callbacks and handle incoming messages.
        """
        executor = SingleThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        self._log("debug", f"Executor created and node added.")
        executor.spin()

    def tick_all(self, mission_planner):
        """
        Tick all components of the drone instance, including telemetry and health checks.
        This method is called periodically to update the drone's state and handle any necessary actions.
        """

        # Block re-entry if landing is in progress
        if getattr(self, "land_mode_active", False):
            landed = not getattr(self.node.state, "in_air", True)
            current_alt = self.telemetry_manager.gps._latest.pose.position.altitude
            home_alt = self.telemetry_manager.gps._home_position[2]
            alt_diff = abs(current_alt - home_alt)

            if landed and alt_diff < 0.5:  # Threshold in meters, adjust as needed
                self._log("info", f"Landing complete (alt diff {alt_diff:.2f}m). Resetting land_mode_active.")
                self.land_mode_active = False
                # Optionally: self.node.dispatcher.dispatch({"command": "disarm"})
            return

        # Send LAND after RTL if home and in RTL mode
        if getattr(self, "rth_active", False) and self._is_at_home_and_in_rtl():
            self._log("info", "RTL complete and drone is home, sending LAND command.")
            LAND_MODE = 9  # ArduPilot LAND mode
            self.node.dispatcher.dispatch({
                "command": "mode",
                "mode": LAND_MODE
            })
            self.rth_active = False  # Prevent retriggering LAND
            self.land_mode_active = True  # Indicate landing in progress
            return  # Prevent further action this tick

        # Not in failsafe; normal logic
        self.check_and_trigger_rth()
        health = self.get_health_status()
        if hasattr(self.node, "tick_with_planner"):
            self.node.tick_with_planner(mission_planner, health_status=health)


    def get_health_status(self):
        return self.health_manager.get_health_status()
    
    def check_and_trigger_rth(self):
        """
        Check health and trigger Return-To-Home (RTL) if battery is CRIT or WARN and drone is flying.
        """
        if getattr(self, "rth_active", False):
            return
    
        health = self.get_health_status()
        # You may want to trigger on "CRIT" only, or "WARN" as well.
        if health and health.get("battery") in ("CRIT",):  # Add "WARN" here if you want RTH on low battery too
            # Ask state from drone's state manager
            in_air = getattr(self.node.state, "in_air", False)
            if in_air:
                self._log("warning", f"Battery {health['battery']} while in air! Triggering Return-to-Home (RTL).")
                # Abort current mission
                try:
                    executor_manager = getattr(self.node, "executor_manager", None)
                    if executor_manager and hasattr(executor_manager, "abort_current_mission"):
                        executor_manager.abort_current_mission()
                        self._log("info", f"Aborted current mission for drone {self.domain_id}.")
                except Exception as e:
                    self._log("error", f"Error aborting mission: {e}")

                # Send RTL mode
                if hasattr(self.node, "dispatcher"):
                    RTL_MODE = 6
                    self.node.dispatcher.dispatch({
                        "command": "mode",
                        "mode": RTL_MODE
                    })
                    self._log("info", f"Sent RTL command to drone {self.domain_id}.")
                    self.node.state.set_mode(RTL_MODE)
                self.rth_active = True
            else:
                self._log("info", "Battery is CRIT, but drone is not in air. Not triggering RTH.")
        else:
            self.rth_active = False  # Reset flag if healthy again


    def _is_at_home_and_in_rtl(self):
        """
        Check if the drone is at home and in RTL mode.
        Returns True if the drone is at home and in RTL mode, False otherwise.
        """
        # Check current mode
        mode = getattr(self.node.state, "mode", None)
        RTL_MODE = 6  # ArduPilot RTL mode
        if mode != RTL_MODE:
            return False

        # Check position
        gps_handler = self.telemetry_manager.gps
        if not gps_handler or not gps_handler._latest or not gps_handler._home_position:
            return False    
        current = gps_handler._latest.pose.position
        home = gps_handler._home_position

        # Compute simple distance in meters (lat/lon approx, or use geopy for accuracy)
        dx = (current.latitude - home[0]) * 111320
        dy = (current.longitude - home[1]) * 111320 * math.cos(math.radians(current.latitude))
        distance = math.sqrt(dx*dx + dy*dy)
        return distance < 1.0

    def shutdown(self):
        """Shutdown this drone instance cleanly."""
        self._log("info", f"Shutting down...")
        if self.node:
            self.node.destroy_node()
        if self.ros_context:
            self.ros_context.shutdown()
        self._log("info", f"Shutdown complete.")
    
    def _log(self, level: str, msg: str):
        """
        Log with a specific level (info, warning, debug, error). Falls back to print if logger not provided.
        """
        tag = "[DroneInstance]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
