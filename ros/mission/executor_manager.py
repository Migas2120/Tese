from ros_2_server.ros.mission.mission_executor import MissionExecutor
from ros_2_server.ros.mission.mission_manager import MissionManager

class ExecutorManager:
    """
    ExecutorManager
    - Manages multiple MissionExecutor instances, one per drone_id.
    - Provides methods to add, remove, and tick all executors.
    """

    def __init__(self, logger=None):
        self.executors = {}  # {drone_id: MissionExecutor}
        self.logger = logger

    def add_executor(self, drone_id, planner, command_sender, pose_handler, state_manager, path_planner):
        """
        Create and add a MissionExecutor for the given drone_id.
        NOTE: planner is no longer stored here — it is passed at runtime during tick.
        """
        if drone_id in self.executors:
            self._log("warning", f"Executor for '{drone_id}' already exists. Skipping.")
            return

        executor = MissionExecutor(drone_id=drone_id, logger=self.logger)
        executor.set_command_sender(command_sender)
        executor.set_pose_handler(pose_handler)
        executor.set_state_manager(state_manager)
        executor.set_path_planner(path_planner)

        self.executors[drone_id] = executor
        self._log("info", f"Executor for '{drone_id}' added successfully.")

    def remove_executor(self, drone_id):
        """
        Remove the MissionExecutor for the given drone_id.
        """
        if drone_id in self.executors:
            del self.executors[drone_id]
            self._log("info", f"Executor for '{drone_id}' removed.")
        else:
            self._log("warning", f"No executor found for '{drone_id}' to remove.")

    def tick_all(self, mission_planner, health_status=None):
        """
        Tick all managed executors and assign missions if idle and permitted.
        get_health_status: function or lambda taking drone_id and returning its health dict.
        """
        for id, executor in self.executors.items():
            if executor.is_idle():
                # Check health before assignment
                if health_status and health_status.get("system") == "CRIT":
                    self._log("warning", f"Drone {id} in CRIT state, skipping mission assignment.")
                    continue

                mission = mission_planner.get_next_priority_mission()
                if mission:
                    if not mission.drone_ids or str(id) in mission.drone_ids:
                        executor.assign_mission(mission)
                        self._log("info", f"Assigned mission '{mission.mission_id}' to drone {id}.")
            executor.tick()

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
