import threading
import rclpy
import os
from rclpy.executors import SingleThreadedExecutor
from ros_2_server.ros.node_wrapper import MiddleManNode
from ros_2_server.tcp.client import TCPClient
from ros_2_server.ros.telemetry.telemetry_manager import TelemetryManager


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
        self.telemetry_manager = TelemetryManager(self.node, self.tcp_client, logger=self.logger)
        self.node.set_pose_handler(self.telemetry_manager.gps)
        self._log("info", f"TelemetryManager initialized.")

        # === ROS Node thread ===
        self.node_thread = threading.Thread(target=self._spin_node, daemon=True)
        self.node_thread.start()
        self._log("info", f"ROS node spinning in separate thread.")

    
    def _spin_node(self):
        executor = SingleThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        self._log("debug", f"Executor created and node added.")
        executor.spin()

    def tick_all(self, mission_planner):
        if hasattr(self.node, "tick_with_planner"):
            self.node.tick_with_planner(mission_planner)

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
