"""
tcp_client.py

Implements a simple TCP client that connects to Unity and sends outbound messages.
Used for delivering telemetry updates or status notifications from ROS to Unity.

This is typically managed by the TelemetryManager, which will eventually
send real JSON payloads instead of the hardcoded string.
"""

import socket
import time
import logging

class TCPClient:
    """
    TCP client for sending outbound messages (e.g., telemetry) to Unity.
    Runs in its own thread, controlled externally by the AppRunner.
    """

    def __init__(self, host='192.168.1.196', port=12345, logger=None):
        """
        Parameters:
        - host: the IP address Unity is listening on
        - port: the TCP port Unity expects telemetry on
        - logger: optional logging.Logger instance
        """
        self.host = host
        self.port = port
        self.socket = None
        self.running = True
        self.logger = logger or logging.getLogger(__name__)

    def run(self):
        """
        Establishes connection to Unity and sends a heartbeat message every second.
        This is a placeholder loop that should be replaced by real data streaming.
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.logger.info(f"[TCPClient] Connected to Unity on {self.host}:{self.port}")

            while self.running:
                message = "telemetry_data_here"  # Placeholder message
                try:
                    self.socket.sendall(message.encode('utf-8'))
                    self.logger.debug(f"[TCPClient] Sent message: {message}")
                    time.sleep(1.0)  # Prevent spamming the Unity server
                except Exception as send_err:
                    self.logger.error(f"[TCPClient] Send failed: {send_err}")
                    self.running = False

        except Exception as e:
            self.logger.error(f"[TCPClient] Connection error: {e}")
        finally:
            if self.socket:
                self.socket.close()
                self.logger.info("[TCPClient] Socket closed")

    def stop(self):
        """Flags the run loop to exit gracefully."""
        self.logger.info("[TCPClient] Stop requested")
        self.running = False
