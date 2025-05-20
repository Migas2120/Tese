class DroneHealthManager:
    """
    Aggregates health status from all relevant handlers for a given drone.
    """

    def __init__(self, battery_handler, gps_handler, pose_handler, status_handler, logger=None):
        self.logger = logger
        self.handlers = {
            "battery": battery_handler,
            "gps": gps_handler,
            "pose": pose_handler,
            "status": status_handler,
        }

    def get_health_status(self):
        """
        Returns a dict of per-subsystem and overall system health.
        """
        health = {}
        levels = []

        for key, handler in self.handlers.items():
            try:
                status = handler.get_health_status()
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[DroneHealthManager] Error checking {key} health: {e}")
                status = "UNKNOWN"
            health[key] = status
            levels.append(status)

        # Determine system (overall) health: CRIT > WARN > OK > UNKNOWN
        if "CRIT" in levels:
            system = "CRIT"
        elif "WARN" in levels:
            system = "WARN"
        elif all(l == "OK" for l in levels if l != "UNKNOWN"):
            system = "OK"
        else:
            system = "UNKNOWN"
        health["system"] = system

        return health

    def get_serialized(self):
        """
        Optionally serialize health summary for sending over TCP/MR UI.
        """
        return self.get_health_status()
