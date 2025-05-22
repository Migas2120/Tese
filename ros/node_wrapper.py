"""
node_wrapper.py

Defines the MiddleManNode class, the core ROS 2 node responsible for:
- Managing all communication with ROS services and topics
- Interpreting and dispatching messages from Unity
- Handling control modes, velocity commands, and takeoff/arming

This node integrates:
- A CommandDispatcher for service calls
- A DroneStateManager for internal flight state tracking
- A ControlCenter for message routing and logic
"""


# TODO: Split mission and executor logic into MissionController or MissionManager
# TODO: Extract all velocity command logic to VelocityManager
# TODO: Extract Unity/TCP handling to UnityBridge/TCPBridge
# TODO: Only handle ROS node lifecycle here; delegate all business logic
# TODO: Reduce direct attribute access (Law of Demeter)
# TODO: Add docstrings to every new helper class
# TODO: Add unit tests for MissionController and VelocityManager, independent of ROS
# TODO: Replace direct instantiations with dependency injection for easier testing



import rclpy
import time
import json
import logging

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ROS messages and services
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from ardupilot_msgs.msg import GlobalPosition

# Internal systems
from ros_2_server.ros.command_dispatcher import CommandDispatcher
from ros_2_server.ros.state.drone_state_manager import DroneStateManager
from ros_2_server.path.path_planner import PathPlanner
from ros_2_server.ros.mission.executor_manager import ExecutorManager
from ros_2_server.ros.control_center import ControlCenter
from ros_2_server.ros.commands.vel_command import VelCommand

class MiddleManNode(Node):
    def __init__(self, logger=None, context=None, drone_id=11):
        self.logger = logger or logging.getLogger('MiddleManNode')
        self.logger.info('Initializing MiddleMan ROS 2 node...')
        self.drone_id = drone_id

        if context:
            self.logger.info(f"[MiddleManNode] Using custom ROS context (likely per-domain).")
            super().__init__('middle_man_node', context=context)
        else:
            self.logger.info(f"[MiddleManNode] Using default global ROS context.")
            super().__init__('middle_man_node')


        # === Internal Modules ===
        self.dispatcher = CommandDispatcher(self, self.logger)
        self.state = DroneStateManager(logger=self.logger)
        self.path_planner = PathPlanner(logger=self.logger)

        self.executor_manager = ExecutorManager(drone_id=self.drone_id, logger=self.logger)
        self.executor_manager.set_command_sender(self.dispatcher)
        self.executor_manager.set_state_manager(self.state)
        self.executor_manager.set_path_planner(self.path_planner)

        self.control_center = ControlCenter(
            dispatcher=self.dispatcher,
            state_manager=self.state,
            logger=self.logger
        )

        # === Velocity Handling ===
        self.last_velocity_data = None
        self.last_velocity_time = 0.0
        self.vel_repeater = VelCommand(logger=self.logger)
        self.create_timer(0.1, self._repeat_last_velocity)

        # === ROS Publishers ===
        self.publisher_ = self.create_publisher(String, 'unity_to_ros', 10)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.vel_pub = self.create_publisher(TwistStamped, "/ap/cmd_vel", qos)

        # === ROS Service Clients ===
        self.prearm_client = self.create_client(Trigger, '/ap/prearm_check')
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')

        for client, name in [
            (self.arm_client, 'arm_motors'),
            (self.mode_client, 'mode_switch'),
            (self.takeoff_client, 'takeoff')
        ]:
            if not client.wait_for_service(timeout_sec=2.0):
                self.logger.warning(f"{name} service not available.")

    def set_tcp_client(self, client):
        self.tcp_client = client
        self.logger.debug("TCP client set in MiddleManNode.")

    def publish_from_unity(self, message_json):
        try:
            data = json.loads(message_json)
            self.logger.debug(f"Dispatching message from Unity: {data}")
            self.control_center.process_message(data)
        except json.JSONDecodeError:
            self.logger.error("Invalid JSON received.")

    def set_pose_handler(self, pose_handler):
        self.executor_manager.set_pose_handler(pose_handler)
        self.logger.info(f"[MiddleManNode] PoseHandler injected into executor for drone {self.drone_id}.")



    def _repeat_last_velocity(self):
        if self.last_velocity_data is None:
            return
        if time.time() - self.last_velocity_time > 5.0:
            return
        self.vel_repeater.execute(self, self.last_velocity_data)

    def tick_with_planner(self, mission_planner, health_status=None):
        self.executor_manager.tick(mission_planner, health_status=health_status)
