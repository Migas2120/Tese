from typing import List
from ros_2_server.ros.mission.mission_planner import Waypoint
import math

class PathPlanner:
    """
    Converts a sequence of relative waypoints into GPS absolute waypoints.
    - First point must be the drone's current GPS position (lat, lon, alt) and acts as origin (not included in output).
    - Subsequent waypoints are treated as relative (meters) offsets from the origin.
    - Returns absolute GPS waypoints only for the relative points given.
    """

    def __init__(self, logger=None):
        self.logger = logger
        self._log("debug", "PathPlanner initialized.")

    def generate_path(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        if not waypoints or len(waypoints) < 2:
            self._log("error", "At least one GPS starting point and one relative waypoint are required.")
            raise ValueError("At least one GPS starting point and one relative waypoint are required.")

        # The first point is origin (absolute GPS), don't include it in output path
        origin = waypoints[0]

        # Convert all subsequent relative waypoints to absolute GPS
        path = []
        for relative_wp in waypoints[1:]:
            abs_wp = self._shift_gps(origin, relative_wp.x, relative_wp.y, relative_wp.z)
            path.append(abs_wp)

        self._log("info", f"Generated {len(path)} absolute GPS waypoints from relative inputs.")
        return path

    def _shift_gps(self, origin: Waypoint, dx_m: float, dy_m: float, dz_m: float) -> Waypoint:
        # Shift latitude (meters to degrees)
        delta_lat = dx_m / 111320.0
        # Shift longitude, compensating for latitude
        delta_lon = dy_m / (111320.0 * math.cos(math.radians(origin.x)))

        return Waypoint(
            origin.x + delta_lat,
            origin.y + delta_lon,
            dz_m  # Add relative altitude offset to origin altitude
        )

    def _log(self, level, msg):
        tag = "[PathPlanner]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")