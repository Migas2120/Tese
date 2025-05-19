"""
logger.py

Initializes and configures the logging system for the ROS 2 MiddleMan server.

Usage:
    from logger import init_logger
    logger = init_logger(debug_enabled=True)
"""

import logging

def init_logger(debug_enabled: bool = False):
    """
    Initializes a global logger with either DEBUG or INFO level.

    Parameters:
    - debug_enabled (bool): if True, sets logging to DEBUG; otherwise INFO

    Returns:
    - logging.Logger: configured logger instance named "Ros2Server"
    """
    level = logging.DEBUG if debug_enabled else logging.INFO

    # Configure basic logger formatting
    logging.basicConfig(
        level=level,
        format='[%(asctime)s] [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )

    return logging.getLogger("Ros2Server")
