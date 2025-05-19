"""
base_command.py

Defines the abstract interface for all command handlers in the
MiddleMan system. Each command (e.g., arm, mode, takeoff) must
subclass this and implement the `execute()` method.

This ensures all command logic is uniform and pluggable into
CommandDispatcher.
"""

from abc import ABC, abstractmethod
import logging

class BaseCommand(ABC):
    """
    Abstract base class for all ROS command handlers.
    Subclasses must implement the `execute()` method.

    Expected usage:
        class MyCommand(BaseCommand):
            def execute(self, ros, data):
                # your implementation

    Parameters:
    - ros: ROS 2 node instance (e.g., MiddleManNode)
    - data: Dictionary payload with any command-specific parameters
    """

    def __init__(self, logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(__name__)

    @abstractmethod
    def execute(self, ros, data: dict):
        """
        Execute the command logic using the provided ROS node.

        Parameters:
        - ros (Node): the active ROS 2 node for context
        - data (dict): structured command payload, must include all needed fields

        Raises:
        - NotImplementedError if subclass does not override this method
        """
        pass
