"""
drone_state_manager.py

Tracks the internal flight status of the drone (armed, flight mode, airborne state),
as well as the high-level control mode (manual vs automatic).

This state is shared across:
- ControlCenter (for safety/logic checks)
- CommandDispatcher (to avoid redundant service calls)
- TelemetryManager (to report current drone status)
"""

import logging

class DroneStateManager:
    """
    Simple data container and access interface for drone state:
    - armed: whether motors are armed
    - mode: current flight mode (int enum, e.g., 4 for GUIDED)
    - in_air: whether the drone is airborne
    - control_mode: 'manual' or 'automatic'
    """

    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.armed = False
        self.mode = None
        self.in_air = False
        self.control_mode = "manual"  # Default is manual control
        self.logger.debug(f"[State] Initialized with armed={self.armed}, mode={self.mode}, in_air={self.in_air}, control_mode={self.control_mode}")

    def set_armed(self, value: bool):
        """Updates armed state (True = motors armed)"""
        self.armed = value
        if self.logger:
            self.logger.debug(f"[State] Armed set to: {value}")

    def set_mode(self, mode: int):
        """Updates the drone's current flight mode (e.g., 4 = GUIDED)"""
        self.mode = mode
        if self.logger:
            self.logger.debug(f"[State] Mode set to: {mode}")

    def set_in_air(self, value: bool):
        """Updates whether the drone is considered flying"""
        self.in_air = value
        if self.logger:
            self.logger.debug(f"[State] In air set to: {value}")

    def set_control_mode(self, mode: str):
        """
        Sets the high-level control mode. Only 'manual' and 'automatic' are valid.
        """
        if mode not in ["manual", "automatic"]:
            raise ValueError(f"Invalid control mode: {mode}")
        self.control_mode = mode
        if self.logger:
            self.logger.info(f"[State] Control mode set to: {mode}")

    def get_summary(self):
        """
        Returns the full state as a dictionary.
        Useful for status dumps or telemetry reporting.
        """
        return {
            "armed": self.armed,
            "mode": self.mode,
            "in_air": self.in_air,
            "control_mode": self.control_mode
        }
