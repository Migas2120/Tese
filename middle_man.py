"""
main.py

Entry point for the MiddleMan ROS 2 server application.

Responsibilities:
- Parse CLI arguments (e.g., --debug)
- Initialize logger
- Instantiate and start the AppRunner, which manages multiple DroneInstances.
"""

import argparse
from ros_2_server.core.logger import init_logger
from ros_2_server.core.app_runner import AppRunner  # Now the refactored AppRunner (multi-drone aware)

def main():
    """
    Parses command-line arguments, initializes the logger,
    and launches the AppRunner to start the multi-drone system.
    """
    parser = argparse.ArgumentParser(description="Start the ROS 2 MiddleMan multi-drone server")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args, _ = parser.parse_known_args()

    # Initialize logger
    logger = init_logger(args.debug)
    logger.info("[Main] Debug mode enabled" if args.debug else "[Main] Running in normal mode")

    # Start AppRunner (which now handles multiple DroneInstances internally)
    app = AppRunner(logger=logger)
    app.start()

if __name__ == '__main__':
    main()