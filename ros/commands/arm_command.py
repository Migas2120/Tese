from ros_2_server.ros.commands.base_command import BaseCommand
from ardupilot_msgs.srv import ArmMotors

class ArmCommand(BaseCommand):
    def execute(self, ros, data: dict):
        request = ArmMotors.Request()
        request.arm = True
        future = ros.arm_client.call_async(request)
        future.add_done_callback(self._handle_response())

        if self.logger:
            self.logger.debug("[ARM] Sent arm request.")

    def _handle_response(self):
        def callback(future):
            try:
                result = future.result()
                if self.logger:
                    self.logger.info(f"[ARM] Result: {result.result}")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[ARM] Failed: {e}")
        return callback
