
from ros_2_server.ros.mission.mission_planner import Mission, Waypoint

class MissionFactory:
    @staticmethod
    def create_inspection_mission(
        mission_id: str,
        drone_id: str,
        points: list[tuple[float, float, float]],
        priority: int = 5,
        description: str = "",
        start_time: float = None
    ) -> Mission:
        waypoints = [Waypoint(x, y, z) for x, y, z in points]

        return Mission(
            mission_id=mission_id,
            mission_type="inspection",
            drone_ids=[drone_id],
            waypoints=waypoints,
            priority=priority,
            description=description,
            start_time=start_time
        )

    @staticmethod
    def create_patrol_mission(
        mission_id: str,
        drone_id: str,
        path: list[tuple[float, float, float]],
        priority: int = 3,
        description: str = ""
    ) -> Mission:
        waypoints = [Waypoint(x, y, z) for (x, y, z) in path]
        return Mission(
            mission_id=mission_id,
            mission_type="patrol",
            drone_ids=[drone_id],
            waypoints=waypoints,
            priority=priority,
            description=description
        )

    @staticmethod
    def create_survey_mission(
        mission_id: str,
        drone_id: str,
        center: tuple[float, float, float],
        width: float,
        height: float,
        rows: int,
        priority: int = 4,
        description: str = ""
    ) -> Mission:
        """
        Simulates a simple rectangular survey pattern with `rows` horizontal sweeps.
        """
        cx, cy, cz = center
        step = height / max(rows - 1, 1)
        y_start = cy - height / 2
        x_left = cx - width / 2
        x_right = cx + width / 2

        waypoints = []
        for i in range(rows):
            y = y_start + i * step
            if i % 2 == 0:
                waypoints.append(Waypoint(x_left, y, cz))
                waypoints.append(Waypoint(x_right, y, cz))
            else:
                waypoints.append(Waypoint(x_right, y, cz))
                waypoints.append(Waypoint(x_left, y, cz))

        return Mission(
            mission_id=mission_id,
            mission_type="survey",
            drone_ids=[drone_id],
            waypoints=waypoints,
            priority=priority,
            description=description
        )