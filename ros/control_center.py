import time

class ControlCenter:
    def __init__(self, dispatcher, state_manager, logger):
        """
        Parameters:
        - dispatcher: handles command dispatching (e.g., to ROS or MAVLink)
        - state_manager: keeps internal status of drone state (armed, in_air, mode)
        - logger: optional logger for debug/info output
        """
        self.dispatcher = dispatcher
        self.state = state_manager
        self.logger = logger

    # ======================
    # Incoming Message Router
    # ======================

    def process_message(self, data: dict):
        """
        Routes incoming messages from Unity to appropriate handlers.
        Recognizes:
        - intent = "takeoff" → triggers guided takeoff
        - intent = "hover"   → placeholder for hover logic
        - x/y/z/yaw fields   → interpreted as velocity command
        """
        if not data:
            self._log("warning", "Received empty message.")
            return

        intent = data.get("intent")

        if intent:
            if intent == "takeoff":
                self._handle_takeoff()
                return

            if intent == "hover":
                self._handle_hover()
                return

            self._log("warning", f"Received unknown intent: {intent}")
            return

        if any(k in data for k in ("x", "y", "z", "yaw")):
            self._handle_velocity(data)
            return

        self._log("warning", f"Unrecognized message format: {data}")

    # ======================
    # Manual Velocity Control
    # ======================

    def _handle_velocity(self, data):
        if self.state.control_mode != "manual":
            self._log("warning", "Ignoring velocity input: Control mode is automatic.")
            return

        if not self.state.armed:
            self._log("info", "Drone not armed. Arming before velocity control.")
            self.dispatcher.dispatch({"command": "arm"})
            self.state.set_armed(True)

        if self.state.mode != 4:
            self._log("info", "Switching to GUIDED mode before velocity control.")
            self.dispatcher.dispatch({"command": "mode", "mode": 4})
            self.state.set_mode(4)

        if not self.state.in_air:
            self._log("info", "Drone is grounded. Initiating takeoff.")
            self.dispatcher.dispatch({"command": "takeoff", "altitude": 2})
            time.sleep(10)
            self.state.set_in_air(True)

        self._log("debug", f"Sending velocity command: {data}")
        self.dispatcher.dispatch({**data, "command": "vel"})

    # ======================
    # Takeoff Handler
    # ======================

    def _handle_takeoff(self):
        if self.state.in_air:
            self._log("info", "Takeoff ignored — drone already in air.")
            return

        if not self.state.armed:
            self._log("info", "Arming before takeoff.")
            self.dispatcher.dispatch({"command": "arm"})
            self.state.set_armed(True)

        if self.state.mode != 4:
            self._log("info", "Switching to GUIDED mode.")
            self.dispatcher.dispatch({"command": "mode", "mode": 4})
            self.state.set_mode(4)

        self._log("info", "Initiating takeoff to 2m.")
        self.dispatcher.dispatch({"command": "takeoff", "altitude": 2})
        time.sleep(10)
        self.state.set_in_air(True)

    # ======================
    # Hover Placeholder
    # ======================

    def _handle_hover(self):
        self._log("info", "Entering hover mode (future implementation).")

    # ======================
    # Logging Helper
    # ======================

    def _log(self, level: str, msg: str):
        tag = "[ControlCenter]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")