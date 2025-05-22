# mission_manager.py

from typing import List, Optional
from ros_2_server.ros.mission.mission_factory import MissionFactory
from ros_2_server.ros.mission.mission_planner import Mission, MissionPlanner

class MissionManager:
    """
    High-level interface for creating, cancelling, and managing missions.
    Delegates all storage/scheduling to MissionPlanner.
    """
    def __init__(self, mission_planner: MissionPlanner, logger=None):
        self.mission_planner = mission_planner
        self.logger = logger

    def create_inspection_mission(
        self,
        mission_id: str,
        drone_id: str,
        points: List[tuple[float, float, float]],
        priority: int = 5,
        description: str = "",
        start_time: Optional[float] = None
    ) -> bool:
        """Creates and registers a new inspection mission."""
        mission = MissionFactory.create_inspection_mission(
            mission_id=mission_id,
            drone_id=drone_id,
            points=points,
            priority=priority,
            description=description,
            start_time=start_time
        )
        if not mission.is_valid():
            self._log("warning", f"Attempted to create invalid inspection mission '{mission_id}'.")
            return False
        return self._add_mission(mission)

    def cancel_mission(self, mission_id: str) -> bool:
        """Cancels and removes a mission by ID."""
        removed = self.mission_planner.remove_mission(mission_id)
        if removed:
            self._log("info", f"Mission '{mission_id}' cancelled.")
        else:
            self._log("warning", f"Tried to cancel unknown mission '{mission_id}'.")
        return removed

    def _add_mission(self, mission: Mission) -> bool:
        """Internal helper to add mission to planner, with logging."""
        added = self.mission_planner.add_mission(mission)
        if added:
            self._log("info", f"Mission '{mission.mission_id}' registered with planner.")
        else:
            self._log("warning", f"Failed to add mission '{mission.mission_id}' to planner.")
        return added

    def _log(self, level: str, msg: str):
        tag = "[MissionManager]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
