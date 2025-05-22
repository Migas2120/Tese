from ros_2_server.ros.mission.mission_executor import MissionExecutor

class ExecutorManager:
    """
    ExecutorManager (single-drone version)
    - Manages a single MissionExecutor for one drone_id.
    - Provides methods to assign, tick, and abort missions for its drone.
    """

    def __init__(self, drone_id, logger=None):
        self.drone_id = drone_id
        self.logger = logger

        # Initialize the MissionExecutor for this drone
        self.executor = MissionExecutor(drone_id=drone_id, logger=logger)

    def set_command_sender(self, command_sender):
        self.executor.set_command_sender(command_sender)

    def set_pose_handler(self, pose_handler):
        self.executor.set_pose_handler(pose_handler)

    def set_state_manager(self, state_manager):
        self.executor.set_state_manager(state_manager)

    def set_path_planner(self, path_planner):
        self.executor.set_path_planner(path_planner)

    def tick(self, mission_planner=None, health_status=None):
        """
        Tick the managed executor.
        Optionally assign a new mission if idle and permitted.
        """
        if self.executor.is_idle():
            if getattr(self.executor, "rth_active", False):
                self._log("warning", f"RTH active for drone {self.drone_id}. Skipping mission assignment.")
                return

            if health_status and health_status.get("system") == "CRIT":
                self._log("warning", f"Drone {self.drone_id} in CRIT state, skipping mission assignment.")
                return

            # Assign a mission if available and allowed
            if mission_planner:
                mission = mission_planner.get_next_priority_mission()
                if mission and (not mission.drone_ids or str(self.drone_id) in mission.drone_ids):
                    self.executor.assign_mission(mission)
                    self._log("info", f"Assigned mission '{mission.mission_id}' to drone {self.drone_id}.")

        # Always tick the executor (to run its state machine)
        self.executor.tick()
    
    def assign_mission(self, mission):
        self.executor.assign_mission(mission)

    def abort_mission(self):
        self.executor.abort_current_mission()
        self._log("info", f"Abort mission for drone {self.drone_id}.")

    def _log(self, level, msg):
        """
        Internal logger with fallback to print().
        """
        tag = "[ExecutorManager]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
