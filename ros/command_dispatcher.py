"""
command_dispatcher.py

Receives parsed command dictionaries and routes them to the appropriate
ROS 2 service or publisher command handlers.

Used by:
- ControlCenter (when interpreting Unity messages)
- TelemetryManager (optional future hook)
"""

import logging

# Command handlers
from ros_2_server.ros.commands.arm_command import ArmCommand
from ros_2_server.ros.commands.mode_command import ModeCommand
from ros_2_server.ros.commands.takeoff_command import TakeoffCommand
from ros_2_server.ros.commands.prearm_command import PrearmCommand
from ros_2_server.ros.commands.vel_command import VelCommand
from ros_2_server.ros.commands.gps_goal_command import GpsGoalCommand

class CommandDispatcher:
    """
    Routes a 'command' string from a data payload to its corresponding
    execution handler (e.g., arm, takeoff, velocity).
    """

    def __init__(self, ros_handler, logger=None):
        """
        Parameters:
        - ros_handler: a ROS 2 Node (e.g., MiddleManNode) used to make service calls
        - logger: optional logger; defaults to ros_handler.get_logger()
        """
        self.ros = ros_handler
        self.logger = logger or ros_handler.get_logger()

        
        # Pre-register all supported commands and their handler objects
        self.command_map = {
            "arm": ArmCommand(logger=self.logger),
            "mode": ModeCommand(logger=self.logger),
            "takeoff": TakeoffCommand(logger=self.logger),
            "prearm_check": PrearmCommand(logger=self.logger),
            "vel": VelCommand(logger=self.logger),
            "gps_goal": GpsGoalCommand(logger=self.logger)
        }

        

        self.logger.debug(f"[CommandDispatcher] Initialized with command map: {self.command_map.keys()}")

    def dispatch(self, data: dict):
        """
        Main entry point for command dispatching.
        
        Parameters:
        - data (dict): must include a 'command' key (e.g., 'takeoff')

        Finds the appropriate command handler and executes it, passing the data along.
        """
        command = data.get("command", "")
        handler = self.command_map.get(command)

        if handler:
            try:
                self.logger.debug(f"Dispatching command '{command}' with data: {data}")
                handler.execute(self.ros, data)
            except Exception as e:
                self.logger.error(f"Error executing '{command}': {e}")
        else:
            self.logger.warning(f"Unknown command: {command}")
