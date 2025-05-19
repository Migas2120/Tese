# mission/mission_manager.py

from ros_2_server.ros.mission.mission_factory import MissionFactory
from ros_2_server.ros.mission.mission_planner import Mission

class MissionManager:
    def __init__(self, mission_planner, logger=None):
        """
        Parameters:
        - mission_planner: the instance of MissionPlanner to store missions
        - logger: optional logger for debug/info/error output
        """
        self.planner = mission_planner
        self.logger = logger

    def create_inspection_mission(self, mission_id: str, drone_id: str, points: list[tuple[float, float, float]], priority: int = 5, description: str = "",start_time: float = None) -> bool:
        """
        Creates an inspection mission with multiple waypoints using the MissionFactory and registers it in the planner.
        """
        mission = MissionFactory.create_inspection_mission(
            mission_id=mission_id,
            drone_id=drone_id,
            points=points,
            priority=priority,
            description=description,
            start_time=start_time
        )
        success = self.planner.add_mission(mission)
        if success:
            self._log("info", f"Inspection mission '{mission_id}' successfully added.")
        else:
            self._log("warning", f"Failed to add inspection mission '{mission_id}'.")
        return success

    def cancel_mission(self, mission_id: str) -> bool:
        """
        Cancels a mission by removing it from the planner and (optionally) from any assigned drone queues.
        Returns True if cancelled, False if not found.
        """
        removed = self.planner.remove_mission(mission_id)
        if removed:
            self._log("info", f"Mission '{mission_id}' cancelled and removed from mission registry.")
        else:
            self._log("warning", f"Attempted to cancel unknown mission '{mission_id}'.")
        return removed

    def _log(self, level: str, msg: str):
        """
        Internal logging helper with fallback to print().
        """
        tag = "[MissionManager]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
