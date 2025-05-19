import time
import threading
import json

from ros_2_server.ros.context.drone_instance import DroneInstance
from ros_2_server.ros.mission.mission_planner import MissionPlanner
from ros_2_server.ros.mission.mission_manager import MissionManager
from ros_2_server.ros.mission.mission_factory import MissionFactory
from ros_2_server.tcp.server import TCPServer

class AppRunner:
    """
    Bootstraps and runs multiple DroneInstance objects and a single TCP server.
    Routes incoming commands from TCP to the correct drone based on 'domain_id' in payload.
    Also manages a global MissionPlanner shared by all drones.
    """

    def __init__(self, logger):
        self.logger = logger
        self.drones = []
        self.tcp_server = None
        self.tcp_server_thread = None

        # Global mission coordination
        self.mission_planner = MissionPlanner(logger=self.logger)
        self.mission_manager = MissionManager(self.mission_planner, logger=self.logger)

        # Maps domain_id to DroneInstance
        self.domain_to_drone = {}

    def add_drone(self, drone):
        """
        Adds a DroneInstance to the list and updates the domain-to-drone mapping.
        """
        self.drones.append(drone)
        self.domain_to_drone[drone.domain_id] = drone
        self._log("info", f"Drone with domain_id {drone.domain_id} added.")

    def start(self):
        self._log("info", "Starting multi-drone system...")

        # === Example: Launch drones ===
        drone1 = DroneInstance(domain_id=0, tcp_host='192.168.1.196', tcp_port=12345, logger=self.logger)
        # drone2 = DroneInstance(domain_id=12, tcp_host='192.168.1.196', tcp_port=12346, logger=self.logger)

        self.add_drone(drone1)
        # self.drones.append(drone2)


        self.domain_to_drone = {drone.domain_id: drone for drone in self.drones}
        # === Start TCP Server ===
        self._log("info", "Starting central TCP server...")
        self.tcp_server = TCPServer(ros_node=self, logger=self.logger, host='127.0.0.1', port=65432)
        self.tcp_server_thread = threading.Thread(target=self.tcp_server.start, daemon=True)
        self.tcp_server_thread.start()

        self._log("info", "All drones and TCP server initialized. Running...")
        try:
            while True:
                for drone in self.drones:
                    drone.tick_all(self.mission_planner)
                time.sleep(1)
        except KeyboardInterrupt:
            self._log("warning", "Shutdown requested by user.")
            self.shutdown()

    def publish_from_unity(self, message_json):
        """
        Called by the TCP server when a message arrives from Unity.
        Routes to the correct DroneInstance by 'domain_id'.
        Handles both per-drone and global mission creation.
        """
        try:
            data = json.loads(message_json)

            # === Handle mission creation ===
            if data.get("type") == "add_mission":
                try:
                    mission_id = data["mission_id"]
                    drone_ids = data.get("drone_ids", [])
                    waypoints = data["waypoints"]
                    priority = int(data.get("priority", 5))
                    description = data.get("description", "")
                    start_time = data.get("start_time")

                    if not isinstance(waypoints, list) or not all(isinstance(wp, list) and len(wp) == 3 for wp in waypoints):
                        self._log("warning", "Invalid waypoint format. Must be list of [x, y, z]")
                        return

                    points = [(float(x), float(y), float(z)) for x, y, z in waypoints]

                    # --- Create a mission object using your MissionFactory/Planner ---
                    # Note: Adjust as needed for your real Mission type/factory

                    if drone_ids:
                        # Assign directly to each drone's queue
                        for drone_id in drone_ids:
                            mission = MissionFactory.create_inspection_mission(
                                mission_id=mission_id,
                                drone_id=drone_id,
                                points=points,  # points already validated and converted above
                                priority=priority,
                                description=description,
                                start_time=start_time
                            )
                            drone = self.domain_to_drone.get(drone_id)
                            if not drone:
                                self._log("warning", f"No drone found for domain_id {drone_id}.")
                                continue
                            executor = drone.node.executor_manager.executors.get(drone_id)
                            if executor:
                                executor.assign_mission(mission)
                                self._log("info", f"Assigned mission '{mission_id}' directly to drone {drone_id}.")
                            else:
                                self._log("warning", f"No executor found for drone {drone_id}.")
                    else:
                        # Add to global mission pool
                        success = self.mission_manager.create_inspection_mission(
                            mission_id=mission_id,
                            drone_id=None,
                            points=points,
                            priority=priority,
                            description=description,
                            start_time=start_time 
                        )
                        if success:
                            self._log("info", f"Mission '{mission_id}' added to global mission pool.")
                        else:
                            self._log("warning", f"Failed to create mission '{mission_id}' in global pool.")


                except KeyError as e:
                    self._log("error", f"Missing required field: {e}")
                except Exception as e:
                    self._log("error", f"Exception during mission creation: {e}")
                return

            if data.get("type") == "cancel_mission":
                try:
                    mission_id = data["mission_id"]
                    # Remove from global pool
                    success = self.mission_manager.cancel_mission(mission_id)
                    # Also remove from all executors (per-drone queues)
                    for drone in self.drones:
                        for executor in drone.node.executor_manager.executors.values():
                            # Remove from queue if queued, or interrupt if currently running
                            if executor.current_mission and executor.current_mission.mission_id == mission_id:
                                executor._log("info", f"Cancelling currently executing mission '{mission_id}'.")
                                executor.current_mission = None
                            # Remove from queue (for queued, not yet running)
                            executor.missions_queue = [m for m in executor.missions_queue if m.mission_id != mission_id]
                    if success:
                        self._log("info", f"Mission '{mission_id}' cancelled everywhere.")
                    else:
                        self._log("warning", f"Mission '{mission_id}' not found to cancel.")
                except Exception as e:
                    self._log("error", f"Exception during mission cancellation: {e}")
                return
            # === Default: Route by domain_id (non-mission messages) ===
            target_domain = data.get("domain_id")
            for drone in self.drones:
                if drone.domain_id == target_domain:
                    drone.node.publish_from_unity(message_json)
                    return

            self._log("warning", f"No drone matched 'domain_id': {target_domain}")

        except json.JSONDecodeError:
            self._log("error", "Invalid JSON received by AppRunner.")
        except Exception as e:
            self._log("error", f"Error in routing message: {e}")


    def shutdown(self):
        self._log("info", "Shutting down all drones and TCP server...")
        if self.tcp_server:
            self.tcp_server.stop()

        for drone in self.drones:
            drone.shutdown()
        self._log("info", "AppRunner shutdown complete.")

    def _log(self, level: str, msg: str):
        tag = "[AppRunner]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
