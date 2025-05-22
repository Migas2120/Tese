import time
import threading
import json

from ros_2_server.ros.context.drone_instance import DroneInstance
from ros_2_server.ros.mission.mission_planner import MissionPlanner
from ros_2_server.ros.mission.mission_manager import MissionManager
from ros_2_server.ros.mission.mission_factory import MissionFactory
from ros_2_server.ros.swarm.swarm_controller import SwarmController 
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

        self.swarm_controller = None

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
        # self.add_drone(drone2)

        self.domain_to_drone = {drone.domain_id: drone for drone in self.drones}

        # === Swarm Controller ===
        # Pass all executor managers as a dict to SwarmController (not just one manager)
        self.swarm_controller = SwarmController(
            drone_instances={d.domain_id: d for d in self.drones},
            executor_managers={d.domain_id: d.node.executor_manager for d in self.drones}
        )
        
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

                    if drone_ids and len(drone_ids) == 1:
                        # === Single drone: Assign directly as before ===
                        drone_id = drone_ids[0]
                        drone = self.domain_to_drone.get(drone_id)
                        if not drone:
                            self._log("warning", f"No drone found for domain_id {drone_id}.")
                            return

                        health = drone.get_health_status()
                        if health.get("system") == "CRIT":
                            self._log("warning", f"Drone {drone_id} is in CRIT health state. Skipping mission assignment.")
                            return

                        mission = MissionFactory.create_inspection_mission(
                            mission_id=mission_id,
                            drone_id=drone_id,
                            points=points,
                            priority=priority,
                            description=description,
                            start_time=start_time
                        )

                        # === Change: Now assign directly to the executor_manager (single-drone) ===
                        executor_manager = drone.node.executor_manager
                        if executor_manager:
                            executor_manager.assign_mission(mission)
                            self._log("info", f"Assigned mission '{mission_id}' directly to drone {drone_id}.")
                        else:
                            self._log("warning", f"No executor manager found for drone {drone_id}.")

                    elif drone_ids and len(drone_ids) > 1:
                        # === Multi-drone: Route to SwarmController ===
                        # NOTE: You may want to include formation type or mission type in the data dict
                        self.swarm_controller.handle_multi_drone_mission(
                            mission_id=mission_id,
                            drone_ids=drone_ids,
                            points=points,
                            priority=priority,
                            description=description,
                            start_time=start_time,
                            extra=data  # Can pass the whole dict for more metadata (formation type, etc)
                        )
                        self._log("info", f"Multi-drone mission '{mission_id}' routed to SwarmController.")
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
                        executor_manager = drone.node.executor_manager
                        if executor_manager:
                            executor = executor_manager.executor  # If you have an 'executor' attribute
                            if executor.current_mission and executor.current_mission.mission_id == mission_id:
                                executor._log("info", f"Cancelling currently executing mission '{mission_id}'.")
                                executor.current_mission = None
                            executor.missions_queue = [m for m in executor.missions_queue if m.mission_id != mission_id]
                    if success:
                        self._log("info", f"Mission '{mission_id}' cancelled everywhere.")
                    else:
                        self._log("warning", f"Mission '{mission_id}' not found to cancel.")
                except Exception as e:
                    self._log("error", f"Exception during mission cancellation: {e}")
                return
            
            if data.get("type") == "get_health_status":
                drone_id = data.get("drone_id")
                if drone_id:
                    drone = self.domain_to_drone.get(drone_id)
                    if not drone:
                        self._log("warning", f"No drone found for domain_id {drone_id}.")
                        # Optionally send error response to Unity
                        return
                    health = drone.get_health_status()
                    # Send health back to Unity (implement TCP response logic here)
                    self.tcp_client.send(json.dumps({"type": "health_status", "drone_id": drone_id, "health": health}))
                else:
                    # Send health for all drones
                    health_summary = {id: d.get_health_status() for id, d in self.domain_to_drone.items()}
                    self.tcp_client.send(json.dumps({"type": "health_status_all", "health": health_summary}))
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
