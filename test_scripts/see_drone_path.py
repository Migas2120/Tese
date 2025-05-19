import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import matplotlib.pyplot as plt

class PosePlotter(Node):
    def __init__(self, waypoints):
        super().__init__('pose_plotter')
        self.positions = []
        self.waypoints = waypoints

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos
        )

        plt.ion()
        self.fig, self.ax = plt.subplots()

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.positions.append((x, y, z))
        self.update_plot()

    def update_plot(self):
        if not self.positions:
            return

        self.ax.clear()

        # Plot drone path
        xs, ys, zs = zip(*self.positions)
        self.ax.plot(xs, ys, label='Drone Path')

        # Plot waypoints
        wxs = [wp.x for wp in self.waypoints]
        wys = [wp.y for wp in self.waypoints]
        self.ax.scatter(wxs, wys, c='red', marker='x', label='Waypoints')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Live Drone Path vs Waypoints')
        self.ax.axis('equal')
        self.ax.legend()
        plt.pause(0.01)

def main():
    from ros_2_server.ros.mission.mission_planner import Waypoint
    waypoints = [
        Waypoint(10.0, 20.0, 5.0),
        Waypoint(15.0, 25.0, 5.0),
        Waypoint(20.0, 30.0, 5.0)
    ]

    rclpy.init()
    node = PosePlotter(waypoints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
