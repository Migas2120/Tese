import math
from ros_2_server.ros.mission.mission_planner import Mission,Waypoint

class MissionExecutor:
    """
    MissionExecutor for GPS-based missions.
    - Manages the life cycle of a mission (init, monitor, complete).
    - Interacts with path planner, command sender, pose handler, and state manager.
    - Sends GPS goals to the drone and monitors progress.
    """

    def __init__(self, drone_id, logger=None):
        self.drone_id = drone_id
        self.logger = logger

        self.rth_active = False

        self.path_planner = None
        self.command_sender = None
        self.pose_handler = None
        self.state_manager = None

        self.current_mission = None
        self.current_path = []
        self.current_waypoint_index = 0

        self.execution_phase = "waiting_for_ready"
        self._sent_current_goal = False

        self.reference_altitude = None

        self.missions_queue = []

        self._log("debug", f"MissionExecutor initialized for drone {self.drone_id}.")

    def is_idle(self):
        return self.current_mission is None

    def assign_mission(self, mission):
        """
        Assign a mission to the executor.
        - If idle, start the mission.
        - If busy, queue the mission
        """
        if self.is_idle():
            self.current_mission = mission
            self._log("info", f"Started mission '{self.current_mission.mission_id}'.")
            self._prepare_path()
            self.execution_phase = "waiting_for_ready"
        else:
            # Simply queue any incoming mission
            self.missions_queue.append(mission)
            self._log("info", f"Queued mission '{mission.mission_id}' for later execution.")

    def set_path_planner(self, planner):
        self.path_planner = planner

    def set_command_sender(self, sender):
        self.command_sender = sender

    def set_pose_handler(self, pose_handler):
        self.pose_handler = pose_handler

    def set_state_manager(self, state_manager):
        self.state_manager = state_manager

    def _prepare_path(self):
        """
        Prepares the path for the current mission.
        - Captures reference altitude.
        - Generates path including current position.
        - Validates path.
        """

        # Check if path planner is set.
        if not self.path_planner:
            self._log("error", "PathPlanner not set.")
            return

        # Check if command sender is set.
        if not self.pose_handler or not self.pose_handler._latest:
            self._log("error", "No GPS pose available to prepare path.")
            return

        try:
            home_position_for_reference = self.pose_handler._home_position 
            self.reference_altitude = self.pose_handler._home_position[2]
            self._log("debug", f"Captured reference altitude: {self.reference_altitude:.2f}m")

            current_position = Waypoint(home_position_for_reference[0], home_position_for_reference[1], 0)  # Only XY used

            # Prepend current position to mission waypoints
            mission_waypoints = [current_position] + self.current_mission.waypoints

            # Calculate the path using the path planner
            self.current_path = self.path_planner.generate_path(mission_waypoints)
            self.current_waypoint_index = 0
            self._sent_current_goal = False

            if not self.current_path:
                self._log("error", "PathPlanner returned an empty path.")
                self.current_mission = None
                return

            self._log("info", f"Path prepared with {len(self.current_path)} waypoints.")
        except Exception as e:
            self._log("error", f"Failed to prepare path: {e}")
            self.current_mission = None

    def _execute_current_step(self):
        """
        Executes the current mission step:
        - Sends GPS goal if needed.
        - Checks if waypoint is reached.
        - Advances waypoint index when reached.
        """

        # Check if the current waypoint index has passed the path length, if yes, mission is complete
        if self.current_waypoint_index >= len(self.current_path):
            self._log("info", f"Mission '{self.current_mission.mission_id}' completed.")
            self.current_mission = None
            # Start next mission in queue if available
            if self.missions_queue:
                next_mission = self.missions_queue.pop(0)
                self.assign_mission(next_mission)
            return

        # Check if command sender is set.
        if not self.command_sender:
            self._log("error", "CommandSender not set.")
            return

        # Check if pose handler is set and has a valid latest pose.
        if not self.pose_handler or not self.pose_handler._latest:
            self._log("warning", "No current GPS pose available yet.")
            return

        # Get the current target waypoint and the latest pose
        current_target = self.current_path[self.current_waypoint_index]
        current_pose = self.pose_handler._latest

        self._log("debug", f"Current GPS pose: {current_pose.pose.position.latitude}, {current_pose.pose.position.longitude}, {current_pose.pose.position.altitude}")

        try:
             # Check if waypoint is reached
            if self._has_reached_waypoint(current_pose, current_target):
                self._log("info", f"Waypoint {self.current_waypoint_index} reached.")
                self.current_waypoint_index += 1
                self._sent_current_goal = False  # Reset to send next goal
                return

            if not self._sent_current_goal:
                self._log("info", f"Sending GPS goal: {current_target}")
                self.command_sender.dispatch({
                    "command": "gps_goal",
                    "latitude": current_target.x,
                    "longitude": current_target.y,
                    "altitude": current_target.z
                })
                self._sent_current_goal = True

        except Exception as e:
            self._log("error", f"Failed to send GPS goal: {e}")

    def _has_reached_waypoint(self, current_pose, target_waypoint, tolerance=0.5):
        """
        Check if the drone has reached the target waypoint.
        - Calculates distance in meters considering latitude, longitude, and relative altitude.
        """
        dx = (target_waypoint.x - current_pose.pose.position.latitude) * 111320
        dy = (target_waypoint.y - current_pose.pose.position.longitude) * (111320 * math.cos(math.radians(current_pose.pose.position.latitude)))
        dz = target_waypoint.z - (current_pose.pose.position.altitude - self.reference_altitude)

        distance = (dx**2 + dy**2 + dz**2) ** 0.5

        self._log("debug", f"Distance to waypoint: {distance:.2f} m (dx={dx:.2f} m, dy={dy:.2f} m, dz={dz:.2f} m)")

        return distance <= tolerance

    def _ensure_ready_to_fly(self):
        """
        Ensures the drone is:
        - Armed.
        - In GUIDED mode.
        - In the air and at safe altitude.
        """
        if not self.state_manager:
            self._log("error", "DroneStateManager not set.")
            return False

        if not self.command_sender:
            self._log("error", "CommandSender not set.")
            return False

        if not self.pose_handler or not self.pose_handler._latest:
            self._log("warning", "No GPS pose available yet.")
            return False

        if not self.state_manager.armed:
            self._log("info", "Drone not armed. Sending arm command.")
            self.command_sender.dispatch({"command": "arm"})
            self.state_manager.set_armed(True)

        if self.state_manager.mode != 4:
            self._log("info", "Drone not in GUIDED mode. Sending mode change.")
            self.command_sender.dispatch({"command": "mode", "mode": 4})
            self.state_manager.set_mode(4)

        if not self.state_manager.in_air:
            self._log("info", "Drone not in air. Sending takeoff.")
            self.command_sender.dispatch({"command": "takeoff", "altitude": 2})
            self.state_manager.set_in_air(True)
            return False

        if (self.pose_handler._latest.pose.position.altitude - self.reference_altitude) < 2:
            self._log("debug", f"Current altitude {self.pose_handler._latest.pose.position.altitude - self.reference_altitude:.2f}m still below safe threshold relative to reference.")
            return False

        return True

    def abort_current_mission(self):
        """
        Abort the current mission and clear the mission queue.
        """
        if self.current_mission:
            self._log("warning", f"Aborting mission '{self.current_mission.mission_id}' due to abort command or failsafe.")
            self.current_mission = None
            self.execution_phase = "waiting_for_ready"
        if self.missions_queue:
            self._log("info", "Clearing mission queue due to abort/failsafe.")
            self.missions_queue.clear()
        self.clear_rth()

    def clear_rth(self):
        self.rth_active = False

    def tick(self):
        # --- Normal mission handling ---
        if not self.current_mission:
            return
        if self.execution_phase == "waiting_for_ready":
            if self._ensure_ready_to_fly():
                self._log("info", "Drone is ready to fly. Switching to path execution phase.")
                self.execution_phase = "executing_path"
            else:
                return
        if self.execution_phase == "executing_path":
            self._execute_current_step()

    def _log(self, level, msg):
        """
        Unified logger.
        Includes drone_id in the tag for multi-drone clarity.
        """
        tag = f"[MissionExecutor: Drone{self.drone_id}]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
     