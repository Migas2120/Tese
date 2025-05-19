from std_srvs.srv import Trigger
from ros_2_server.ros.commands.base_command import BaseCommand

class PrearmCommand(BaseCommand):
    def execute(self, ros, data: dict):
        client = ros.prearm_client
        if not client.wait_for_service(timeout_sec=3.0):
            if self.logger:
                self.logger.warning("[PREARM] Service not available")
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(self._handle_response())

        if self.logger:
            self.logger.debug("[PREARM] Sent prearm check request.")

    def _handle_response(self):
        def callback(future):
            try:
                result = future.result()
                if self.logger:
                    self.logger.info(f"[PREARM] Success: {result.success}, Message: {result.message}")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[PREARM] Failed: {e}")
        return callback