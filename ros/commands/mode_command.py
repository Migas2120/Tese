from ros_2_server.ros.commands.base_command import BaseCommand
from ardupilot_msgs.srv import ModeSwitch

class ModeCommand(BaseCommand):
    def execute(self, ros, data: dict):
        mode = int(data.get("mode", 4))
        request = ModeSwitch.Request()
        request.mode = mode
        future = ros.mode_client.call_async(request)
        future.add_done_callback(self._handle_response())

        if self.logger:
            self.logger.debug(f"[MODE] Sent mode switch request with mode: {mode}")

    def _handle_response(self):
        def callback(future):
            try:
                result = future.result()
                if self.logger:
                    self.logger.info(f"[MODE] Result: {result.status}")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[MODE] Failed: {e}")
        return callback