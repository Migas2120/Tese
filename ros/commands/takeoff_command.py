from ros_2_server.ros.commands.base_command import BaseCommand
from ardupilot_msgs.srv import Takeoff

class TakeoffCommand(BaseCommand):
    def execute(self, ros, data: dict):
        alt = float(data.get("altitude", 10.0))
        request = Takeoff.Request()
        request.alt = alt
        future = ros.takeoff_client.call_async(request)
        future.add_done_callback(self._handle_response())

        if self.logger:
            self.logger.debug(f"[TAKEOFF] Sent request with altitude: {alt}")

    def _handle_response(self):
        def callback(future):
            try:
                result = future.result()
                if self.logger:
                    self.logger.info(f"[TAKEOFF] Result: {result.status}")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[TAKEOFF] Failed: {e}")
        return callback