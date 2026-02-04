#!/usr/bin/env python3

import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header
from geometry_msgs.msg import  PoseStamped,Pose2D, Quaternion
from custom_navigator_interfaces.action import NavigationRequest
# TurtleBot4 Navigator imports (These are available via the turtlebot4 common and turtlebot4_navigation packages)
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


# Configurations:
from custom_navigator.config.nav_configs import NavigationState, NavRequest
from custom_navigator.config.ros_presets import STD_CFG

from robot_common.logging import LogEvent, log_error,log_info
class ExecutorNode(Node):

    def __init__(self):
        super().__init__('navigator_setup_node')
        log_info(self.get_logger(),'Initializing Navigator Setup Node...',self.__init__.__qualname__)

        # ***** THESE ARE TEMPORARY TILL I PROPERLY REWRITE THIS MODULE *****
        self.__initial_x = 1.0
        self.__initial_y = 0.1
        self.__initial_yaw = 0
        self.__initialized = False
        self.__navigating = False


        self._last_log_message: LogEvent = None # Last logged message
        self._ros_config = STD_CFG              # Configuration for ROS2
        self._nav_goal_future = None            # Future for Navigation Requests
        self._nav_result_future = None

        self.__navigator = TurtleBot4Navigator()

        # Subscribe to goal topic
        self.__goal_subscription = self.create_subscription(
            Pose2D,
            self._ros_config.navigation_goal_topic,
            self._goal_received_callback,
            self._ros_config.max_messages
        )

        # Navigation action client:
        self._nav_action_client = ActionClient(
            self, 
            NavigationRequest,
            self._ros_config.navigation_server
        )

        self._initialize()
    #-----------------------------------------------------------------------------
    def _goal_received_callback(self, msg: Pose2D):
        """Callback function for goal topic subscription"""

        # if self.__navigating:
        #     self.get_logger().info('Already navigating to a goal, ignoring new goal.')
        #     return
        
        self._last_log_message=log_info(
            logger=self.get_logger(),
            message='Goal received, initializing navigator setup...',
            last_event=self._last_log_message
        )
        x = msg.x
        y = msg.y
        yaw = int(msg.theta)
        q = Quaternion()
        q.x ,q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        pose.pose.orientation = q
        self._send_goal_to_server(pose)

    #----------------------------------------------------------------------------------
    def _send_goal_to_server(self, pose: PoseStamped):
        goal_msg = NavigationRequest.Goal()
        header = Header()
        request_type:int = NavRequest.NAVIGATE
        request_text:str = f"Request move to location x: {pose.pose.position.x} y:{pose.pose.position.y}"
        position:PoseStamped = pose

        goal_msg.header = header
        goal_msg.request_type = request_type
        goal_msg.request_text = request_text
        goal_msg.position = position

        self._nav_action_client.wait_for_server()
        self._last_log_message = log_info(
            logger=self.get_logger(),
            message=f"Sending goal: x{pose.pose.position.x}, y:{pose.pose.position.y}",
            last_event=self._last_log_message
        )

        self._nav_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        self._nav_goal_future.add_done_callback(self._nav_response_callback)

    #------------------------------------------------------------------------------
    def _nav_feedback_callback(self, msg:NavigationRequest):
        feedback = msg.feedback
        state = feedback.state
        feedback_text = feedback.feedback_text
        position: PoseStamped = feedback.position
        progress = feedback.progress
        # print for now.... Do something later
        print(f"\r x Progress: {feedback.progress}%\t", end="", flush=True)

    #------------------------------------------------------------------------------
    def _nav_response_callback(self, future:NavigationRequest):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Log rejection:
            self._last_log_message = log_info(
                logger=self.get_logger(),
                message='Goal rejected',
                last_event=self._last_log_message
            )
        else:
            self._last_log_message = log_info(
                logger=self.get_logger(),
                message='Goal accepted',
                last_event=self._last_log_message
            )
            self._nav_result_future = goal_handle.get_result_async()
            self._nav_result_future.add_done_callback(self._nav_result_callback)
        
    #------------------------------------------------------------------------------
    def _nav_result_callback(self, future:NavigationRequest):
        """Handle the final result"""
        result = future.result().result
        # Print for now... do something later:
        print(f"\n")
        print(result.result_text)
    #-----------------------------------------------------------------------------
    # def move_to_pose(self, x, y, yaw):
    #     """Moves the robot to the specified pose (x, y, yaw)"""
    #     self.__navigating = True
    #     goal_pose =self.__navigator.getPoseStamped([x, y], yaw)
    #     goal_pose.header.stamp = self.__navigator.get_clock().now().to_msg()

    #     # initial_pose = self.__navigator.getPoseStamped([0, 0], 0.0)
    #     # self.__navigator.setInitialPose(initial_pose)

    #     self.__navigator.info('Waiting for Nav2 to become active')
    #     self.__navigator.waitUntilNav2Active(localizer='slam_toolbox')
    #     self.__navigator.info(f'Moving to pose: x={x}, y={y}, yaw={yaw}')

    #     self.__navigator.goToPose(goal_pose)
    #     self.__navigating = False
    #----------------------------------------------------------------------------------
    def _initialize(self):
        """Sets the initial pose of the robot after undocking"""
        
        if self.__initialized:
            return
        print("Initializing...")
        # Undock if docked
        if self.__navigator.getDockedStatus():
            self.__navigator.info('Undocking')
            self.__navigator.undock()

        # time.sleep(2.0)  # Wait for undocking to complete
        # self.destroy_node()
    #----------------------------------------------------------------------------------
    def __set_initial_pose(self):
        """Sets the initial pose of the robot"""
        initial_pose = self.__navigator.getPoseStamped([self.__initial_x, self.__initial_y], self.__initial_yaw)
        initial_pose.header.stamp = self.__navigator.get_clock().now().to_msg()
        self.__navigator.setInitialPose(initial_pose)

        self.__navigator.info('Waiting for Nav2 to become active')
        self.__navigator.waitUntilNav2Active(localizer='slam_toolbox')
        self.get_logger().info('NavigatorSetupNode initialization complete.')

        self.__initialized = True

    #----------------------------------------------------------------------------------
    def navigator(self):
        """Returns the TurtleBot4Navigator instance"""
        return self.__navigator

#******************************************************************************
def main(args=None):

    rclpy.init(args=args)
    navigator_setup_node = ExecutorNode()
    try:
        rclpy.spin(navigator_setup_node)

    finally:
        navigator_setup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
