from typing import List, Optional
from dataclasses import dataclass, field
import heapq
import time

# Mission types supported by the system
VALID_MISSION_TYPES = {
    "patrol",       # Loop over waypoints
    "survey",       # Area scan
    "inspection",   # Visit single or specific points
    "formation",    # Formation flight
    "follow",       # Follow a dynamic target
    "delivery",     # Go from A to B with optional payload
    "hold"          # Hold position
}

# Defines a waypoint in 3D space with optional hold time
@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    hold_time: float = 0.0

# Mission with comparable priority support
@dataclass(order=True)
class Mission:
    priority: int
    mission_type: str = field(compare=False)  # Must be one of VALID_MISSION_TYPES
    mission_id: str = field(compare=False)
    drone_ids: List[str] = field(compare=False)
    waypoints: List[Waypoint] = field(compare=False)
    start_time: Optional[float] = field(default=None, compare=False)
    description: Optional[str] = field(default="", compare=False)
    start_time: Optional[float] = field(default=None, compare=False)

    def is_valid(self) -> bool:
        """
        A mission is valid if:
        - It has a non-empty ID
        - At least one drone
        - At least one waypoint
        - A known mission type
        """
        return (
            bool(self.mission_id and self.drone_ids and self.waypoints) and
            self.mission_type in VALID_MISSION_TYPES
        )

# MissionPlanner manages missions by ID and also supports priority-based scheduling
class MissionPlanner:
    def __init__(self, logger=None):
        self.missions: dict[str, Mission] = {}  # Mission lookup by ID
        self.mission_queue: List[Mission] = []  # Priority queue (heap) of missions
        self.logger = logger

    def add_mission(self, mission: Mission) -> bool:
        """
        Add a mission to both the lookup dictionary and the priority queue.
        Returns True if added, False if the mission is invalid.
        """
        if not mission.is_valid():
            self._log("Invalid mission attempted to be added.")
            return False

        if mission.mission_type not in VALID_MISSION_TYPES:
            self._log(f"Rejected mission '{mission.mission_id}': Unknown mission type '{mission.mission_type}'.")
            return False  # Important! Stop here if the type is invalid

        self.missions[mission.mission_id] = mission
        heapq.heappush(self.mission_queue, (-mission.priority, mission))  # Use negative for max-heap behavior
        self._log(f"Mission '{mission.mission_id}' added with priority {mission.priority} for drones: {mission.drone_ids}")
        return True

    def get_mission(self, mission_id: str) -> Optional[Mission]:
        """
        Retrieve a mission by ID. Returns None if not found.
        """
        return self.missions.get(mission_id)

    def update_mission(self, updated_mission: Mission) -> bool:
        """
        Update an existing mission with new data.
        Replaces the mission in the dictionary and re-inserts it into the priority queue.
        Returns True if updated, False if the mission ID does not exist or is invalid.
        """
        if not updated_mission.is_valid():
            self._log("Attempted to update with an invalid mission.")
            return False

        if updated_mission.mission_id not in self.missions:
            self._log(f"Cannot update non-existent mission '{updated_mission.mission_id}'.")
            return False

        # Replace the mission in the dictionary
        self.missions[updated_mission.mission_id] = updated_mission

        # Push the updated mission into the priority queue
        heapq.heappush(self.mission_queue, (-updated_mission.priority, updated_mission))

        self._log(f"Mission '{updated_mission.mission_id}' updated with new priority {updated_mission.priority}.")
        return True
    
    def remove_mission(self, mission_id: str) -> bool:
        """
        Remove a mission by ID. This will not immediately remove it from the priority queue,
        but it will be skipped on dispatch.
        """
        if mission_id in self.missions:
            del self.missions[mission_id]
            self._log(f"Mission '{mission_id}' removed from mission registry.")
            return True
        self._log(f"Attempted to remove unknown mission '{mission_id}'.")
        return False

    def list_missions(self) -> List[str]:
        """
        List all mission IDs currently registered.
        """
        return list(self.missions.keys())

    def get_next_priority_mission(self) -> Optional[Mission]:
        """
        Retrieve the highest priority mission that is eligible to start.
        """
        now = time.time()
        while self.mission_queue:
            _, mission = heapq.heappop(self.mission_queue)
            if mission.mission_id in self.missions:
                # Only dispatch if scheduled time has arrived
                if mission.start_time is None or mission.start_time <= now:
                    self._log(f"Dispatching mission '{mission.mission_id}' with priority {mission.priority}.")
                    return mission
                else:
                    # Not ready yet, re-queue it and keep looking
                    heapq.heappush(self.mission_queue, (-mission.priority, mission))
                    break  # Stop searching, all later missions are lower priority
        self._log("No valid missions left in the priority queue.")
        return None


    def _log(self, msg: str):
        """
        Internal logging utility. Uses an external logger if provided; falls back to print().
        """
        if self.logger:
            self.logger.debug(f"[MissionPlanner] {msg}")
        else:
            print(f"[MissionPlanner] {msg}")
