import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class ClimbTester(Node):
    def __init__(self):
        super().__init__('climb_test_node')

        # Clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')

        # Publishers
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos)

        # Subscribers
        qos1 = QoSProfile(depth=10)
        qos1.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos1
        )

        # State
        self.latest_z = None
        self.state = "INIT"
        self.count = 0
        self.max_ticks = 300  # 30 seconds if 0.1s tick
        self.timer = self.create_timer(0.1, self.step)

        self.takeoff_target_alt = 2.0
        self.takeoff_reached_time = None

    def step(self):
        if self.state == "INIT":
            self.get_logger().info("Requesting ARM...")
            req = ArmMotors.Request()
            req.arm = True
            future = self.arm_client.call_async(req)
            future.add_done_callback(self.arm_done)
            self.state = "WAIT_ARM"

        elif self.state == "WAIT_FOR_TAKEOFF_REACHED":
            if self.latest_z is not None:
                if self.latest_z >= self.takeoff_target_alt - 0.2:
                    if self.takeoff_reached_time is None:
                        self.takeoff_reached_time = time.time()
                        self.get_logger().info(f"Takeoff altitude reached ({self.latest_z:.2f} m). Waiting 5 seconds...")
                    elif time.time() - self.takeoff_reached_time >= 5.0:
                        self.get_logger().info("Reasserting GUIDED mode to allow velocity control...")
                        req = ModeSwitch.Request()
                        req.mode = 4  # GUIDED
                        future = self.mode_client.call_async(req)
                        future.add_done_callback(self.start_climb)
                        self.state = "WAIT_GUIDED_BEFORE_CLIMB"

        elif self.state == "CLIMBING":
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.z = 3.0  # Climb at 3 m/s
            self.vel_pub.publish(msg)

            if self.latest_z is not None:
                self.get_logger().info(f"[CLIMB] z=3.0 | Altitude: {self.latest_z:.2f} m (tick {self.count})")
            else:
                self.get_logger().info(f"[CLIMB] z=3.0 | Altitude: (unknown) (tick {self.count})")

            self.count += 1
            if self.count >= self.max_ticks:
                self.get_logger().info("Test complete.")
                rclpy.shutdown()

    def arm_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[ARM] Result: {result.result}")
            self.get_logger().info("Switching to GUIDED mode...")
            req = ModeSwitch.Request()
            req.mode = 4  # GUIDED
            future = self.mode_client.call_async(req)
            future.add_done_callback(self.mode_done)
        except Exception as e:
            self.get_logger().error(f"[ARM] Failed: {e}")

    def mode_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[MODE] Result: {result.status}")
            self.get_logger().info("Sending takeoff command...")
            req = Takeoff.Request()
            req.alt = self.takeoff_target_alt
            future = self.takeoff_client.call_async(req)
            future.add_done_callback(self.takeoff_done)
        except Exception as e:
            self.get_logger().error(f"[MODE] Failed: {e}")

    def takeoff_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[TAKEOFF] Result: {result.status}")
            self.get_logger().info("Waiting for drone to reach takeoff height...")
            self.state = "WAIT_FOR_TAKEOFF_REACHED"
        except Exception as e:
            self.get_logger().error(f"[TAKEOFF] Failed: {e}")

    def start_climb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[GUIDED] Reasserted: {result.status}")
            self.get_logger().info("Starting climb...")
            self.state = "CLIMBING"
        except Exception as e:
            self.get_logger().error(f"[GUIDED] Failed before climb: {e}")

    def pose_callback(self, msg):
        self.latest_z = msg.pose.position.z

def main(args=None):
    rclpy.init(args=args)
    node = ClimbTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
