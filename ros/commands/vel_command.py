import time
from geometry_msgs.msg import TwistStamped
from ros_2_server.ros.commands.base_command import BaseCommand

class VelCommand(BaseCommand):
    def execute(self, ros, data: dict):
        msg = self._build_twist_msg(ros, data)
        ros.vel_pub.publish(msg)
        ros.last_velocity_data = data
        ros.last_velocity_time = time.time()
        if self.logger:
            self.logger.debug(f"[VEL] Published Twist: {msg.twist}")

    def _build_twist_msg(self, ros, data: dict):
        msg = TwistStamped()
        msg.header.stamp = ros.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(data.get("x", 0.0))
        msg.twist.linear.y = float(data.get("y", 0.0))
        msg.twist.linear.z = float(data.get("z", 0.0))
        return msg