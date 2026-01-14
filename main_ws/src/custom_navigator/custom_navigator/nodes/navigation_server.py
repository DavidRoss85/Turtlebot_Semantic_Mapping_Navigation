
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import(
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.action.server import ServerGoalHandle

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_navigator_interfaces.action import NavigationRequest

#These enums need to be moved later to a separate file
class NavRequest(int, Enum):
    PAUSE = 0
    ALIGN = 1
    NAVIGATE = 2
    SEARCH = 3
    APPROACH = 4
    CANCEL = 5

class NavigationState(int, Enum):
    IDLE = 0
    ALIGNING = 1
    NAVIGATING = 2
    SEARCHING = 3
    APPROACHING = 4
    REACHED_GOAL = 5
    CANCELED = 6
    FAILED = 7
    PAUSED = 8

class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')

        self.get_logger().info('Navigation Server Node has been started.')

        self._cb_group = ReentrantCallbackGroup()   # Allows multiple callbacks to run simultaneously
        
        self._state = NavigationState.IDLE  # Initialize the navigation state to IDLE

        self._execute_rate = 100  # Set the execution rate for the navigation logic
        
        # Initialize the action server
        self.get_logger().info('Initializing Action Server...')
        self._action_server = ActionServer(
            self,
            NavigationRequest,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

    #----------------------------------------------------------------------------------
    def goal_callback(self, goal_request: NavigationRequest.Goal):
        self.get_logger().info('Received goal request')

        # Check if the server is idle before accepting a new goal
        if self._state != NavigationState.IDLE:
            self.get_logger().info('Navigation is not idle, rejecting goal request')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        # Add cancel logic here

        return CancelResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        rate = self.create_rate(self._execute_rate)  # Set the rate 

        # Placeholder for the actual navigation logic:
        feedback_msg = NavigationRequest.Feedback()
        result_msg = NavigationRequest.Result()

        while True:
            #  ----- Check if the goal is canceled -----
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                # Set result_msg here...
                self._state = NavigationState.IDLE
                goal_handle.canceled()
                return result_msg
            
            # ----- Pause the navigation if the goal request is to pause -----
            if self._state == NavigationState.PAUSED:
                self.get_logger().info('Navigation paused')
                # Add pause logic here
                feedback_msg.state = NavigationState.PAUSED
                feedback_msg.feedback_text = 'Navigation paused'
                goal_handle.publish_feedback(feedback_msg)
                rate.sleep()
                continue

            self._state = NavigationState.NAVIGATING  # Set the state to NAVIGATING

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Executing goal')
            # Add your navigation logic here
            # Set result_msg here...
            rate.sleep()
            # break
        goal_handle.succeed()
        return result_msg
    
#**************************************************************************************
# Main function to initialize the ROS node and start the action server
def main(args=None):
    rclpy.init(args=args)
    navigation_server = NavigationServer()

    # Create a multi-threaded executor to handle multiple callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(navigation_server)
    executor.spin()

    navigation_server.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()