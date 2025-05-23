from ros_2_server.ros.mission.mission_planner import Mission, Waypoint

class SwarmController:
    """
    SwarmController: Handles all multi-drone (swarm) missions.
    Integrates with ExecutorManagers (one per drone).
    """

    def __init__(self, drone_instances, executor_managers):
        """
        :param drone_instances: dict {drone_id: DroneInstance}
        :param executor_managers: dict {drone_id: ExecutorManager}
        """
        self.drone_instances = drone_instances
        self.executor_managers = executor_managers

    def handle_multi_drone_mission(
        self, mission_id, drone_ids, points, priority, description, start_time, formation=None
    ):
        """
        Handles a multi-drone (swarm) mission.
        Supports:
          - Formation flight
          - Scattered/individual mission allocation
        """
        # 1. Formation flight (e.g., all drones move in a shape)
        if formation:
            self._assign_formation_mission(
                mission_id, drone_ids, points, priority, description, start_time, formation
            )
        else:
            # 2. Scattered: Distribute points to drones (one per drone)
            self._assign_scattered_mission(
                mission_id, drone_ids, points, priority, description, start_time
            )

    def _assign_formation_mission(self, mission_id, drone_ids, points, priority, description, start_time, formation_type):
        """
        Assigns the same path (or a shape) to all drones, with appropriate offsets based on formation_type.
        """
        # Example: Just assign the exact same mission to all, but you could shift/offset points per drone here.
        for idx, drone_id in enumerate(drone_ids):
            # (Optionally, calculate offset based on formation_type and idx)
            drone_points = points  # TODO: adjust per-drone for actual formation
            mission = Mission(
                mission_id=f"{mission_id}_drone{drone_id}",
                waypoints=[Waypoint(*pt) for pt in drone_points],
                drone_ids=[str(drone_id)],
                priority=priority,
                mission_type="formation",
                start_time=start_time,
                description=f"{description} [formation: {formation_type}]"
            )
            self.executor_managers[drone_id].assign_mission(mission)
            self._log("info", f"Assigned FORMATION mission to drone {drone_id} ({formation_type})")

    def _assign_scattered_mission(self, mission_id, drone_ids, points, priority, description, start_time):
        """
        Distributes unique points to each drone (1 point per drone for now).
        """
        
        num_drones = len(drone_ids)
        if len(points) < num_drones:
            self._log("warning", f"Not enough points for all drones (have {len(points)}, need {num_drones})")
            return

        for idx, drone_id in enumerate(drone_ids):
            pt = points[idx]  # Just assign one point per drone
            mission = Mission(
                mission_id=f"{mission_id}_drone{drone_id}",
                waypoints=[Waypoint(*pt)],
                drone_ids=[str(drone_id)],
                priority=priority,
                mission_type="scattered",
                start_time=start_time,
                description=f"{description} [scattered mission]"
            )
            self.executor_managers[drone_id].assign_mission(mission)
            self._log("info", f"Assigned SCATTERED mission to drone {drone_id}")

    def _log(self, level, msg):
        print(f"[SwarmController:{level}] {msg}")
