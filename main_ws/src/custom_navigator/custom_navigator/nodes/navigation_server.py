
from enum import Enum
import time
import threading

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
from custom_navigator.mapping.map_cache import NavMapCache

from custom_navigator.config.ros_presets import STD_CFG

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
    PLANNING = 2
    NAVIGATING = 3
    SEARCHING = 4
    APPROACHING = 5
    REACHED_GOAL = 6
    CANCELED = 7
    FAILED = 8
    PAUSED = 9
    EMERGENCY_STOP = 10

class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')

        self.get_logger().info('Navigation Server Node has been started.')

        self._ros_config = STD_CFG
        self._state = NavigationState.IDLE  # Initialize the navigation state to IDLE
        self._execute_rate = 100  # Set the execution rate for the navigation logic
        self._map_update_interval = 0.05  # Interval to update the map in seconds
        self._map_timeout_s = 5.0  # Timeout for waiting for the map in seconds
        self._nav_timer_interval = 1  # General timer interval in seconds
        self._feedback_timer_interval = 1.0  # Feedback timer interval in seconds
        self._active_goal_handle: ServerGoalHandle = None  # Currently active goal handle
        self._active_feedback: NavigationRequest.Feedback = None  # Currently active feedback message
        self._feedback_state_list = {            
            NavigationState.ALIGNING,
            NavigationState.PLANNING,
            NavigationState.NAVIGATING,
            NavigationState.SEARCHING,
            NavigationState.APPROACHING,
            NavigationState.PAUSED
          }  # List of states where feedback is published

        self._goal_callback_lock = threading.Lock()

        self._cb_group = ReentrantCallbackGroup()   # Allows multiple callbacks to run simultaneously
        self._map_cache = NavMapCache(self, self._ros_config.navigation_grid_topic) # Map cache for occupancy grid
        
        # Initialize timers:
        self._nav_timer = self.create_timer(
            self._nav_timer_interval,
            self._nav_timer_callback,
            callback_group=self._cb_group
        )
        self._feedback_timer = self.create_timer(
            self._feedback_timer_interval,  # Feedback interval in seconds
            self._feedback_timer_callback,
            callback_group=self._cb_group
        )

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
        """Executes upon receiving a goal request."""
        self.get_logger().info('Received goal request')

        with self._goal_callback_lock:
            # Check if the server is idle before accepting a new goal
            if self._state != NavigationState.IDLE:
                # For now it will simply reject new goals if not idle... Later we will add more complex logic
                self.get_logger().info('Navigation is not idle, rejecting goal request')
                return GoalResponse.REJECT
            
            # User requests to navigate to a pose
            if goal_request.request_type == NavRequest.NAVIGATE:
            # Validate the goal here (e.g., check if the position is reachable)
                self.get_logger().info('Accepted NAVIGATE goal request')
            
            
            # Goal will execute in execute_callback
            return GoalResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def cancel_callback(self, goal):
        """Executes upon receiving a cancel request."""
        self.get_logger().info('Received cancel request')
        # Add cancel logic here

        return CancelResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Executes upon receiving a goal request after approved by goal callback."""

        # Local variables
        result_msg = NavigationRequest.Result()
        rate = self.create_rate(self._execute_rate)  # Set the rate 
        goal = goal_handle.request  # The actual goal request
        final_text = ""

        #----- Cleanup function -----
        def _cleanup_execution(text: str  = ""):
            """Cleans up after a navigation failure."""
            self._active_goal_handle = None
            self._active_feedback = None
            result_msg.state = self._state
            result_msg.result_text = text
            self.get_logger().info(text)

        # Initial checks before starting execution
        if self._state == NavigationState.EMERGENCY_STOP:
            goal_handle.abort()
            final_text="Navigation server is not ready to execute goals."
            _cleanup_execution(final_text)
            return result_msg

        self._active_goal_handle = goal_handle # Store the active goal handle for use in other methods

        # ----- For navigation, plan the path -----
        if goal.request_type == NavRequest.NAVIGATE:
            self._state = NavigationState.PLANNING
            map_available = self._wait_for_map(goal_handle, self._map_timeout_s)
            if not map_available:
                goal_handle.abort()
                self._state = NavigationState.FAILED
                final_text="Map not available, navigation aborted."
                _cleanup_execution(final_text)
                return result_msg
            else:
                # ----- Proceed with navigation logic -----
                # <Plan route here> 
                self._state = NavigationState.NAVIGATING # set state to NAVIGATING
                self._active_feedback.state = self._state
                self._active_feedback.feedback_text = "Starting Navigation..."
                pass

        # ----- Main execution loop -----
        while self._state != NavigationState.EMERGENCY_STOP:

            #  ----- Check if the goal is canceled -----
            if goal_handle.is_cancel_requested:
                self._state = NavigationState.CANCELED
                goal_handle.canceled()
                final_text="Navigation canceled by user."
                break
            
            # ----- Chceck if navigation goal failed -----
            if self._state == NavigationState.FAILED:
                goal_handle.abort()
                final_text="Navigation failed."
                break
            
            # ----- Check if navigation goal succeeded -----
            if self._state == NavigationState.REACHED_GOAL:
                goal_handle.succeed()
                final_text="Navigation succeeded."
                break
            
            rate.sleep()
            # Publish feedback - Moved to a designated feedback timer
        
        _cleanup_execution(final_text)
        return result_msg

    
    #----------------------------------------------------------------------------------
    def _wait_for_map(self, goal_handle: ServerGoalHandle, timeout_s: float = 2.0) -> bool:
        """Wait for the map to be available in the map cache."""
        start = time.time()

        while not self._map_cache.has_map():
            # Allow fast cancel / estop while waiting
            if self._state == NavigationState.EMERGENCY_STOP:
                return False
            # Check for goal cancelation
            if goal_handle.is_cancel_requested:
                return False
            # Check for timeout
            if (time.time() - start) > timeout_s:
                return False
            
            time.sleep(self._map_update_interval)

        return True
    
    #----------------------------------------------------------------------------------
    def _nav_timer_callback(self):
        """Timer callback to handle periodic navigation tasks."""

        feedback_msg = NavigationRequest.Feedback()

        match self._state:
            case NavigationState.IDLE:
                pass
            case NavigationState.ALIGNING:
                pass
            case NavigationState.PLANNING:
                pass
            case NavigationState.NAVIGATING:
                pass
            case NavigationState.SEARCHING:
                pass
            case NavigationState.APPROACHING:
                pass
            case NavigationState.PAUSED:    
                # ----- Pause the navigation if the goal request is to pause -----
                # Add pause logic here
                feedback_msg.state = NavigationState.PAUSED
                feedback_msg.feedback_text = 'Navigation paused'

            #----- Return to IDLE States -----
            case NavigationState.CANCELED:
                # -- Handle cancellation logic here --
                # Return to IDLE after handling cancellation:
                self._state = NavigationState.IDLE
                pass
            case NavigationState.REACHED_GOAL:
                # -- Handle goal reached logic here --
                # Return to IDLE after reaching goal:
                self._state = NavigationState.IDLE
            case _:
                pass
        
        # Update active feedback message
        if self._active_feedback is not None:
            self._active_feedback.state = feedback_msg.state
            self._active_feedback.feedback_text = feedback_msg.feedback_text

    #----------------------------------------------------------------------------------
    def _feedback_timer_callback(self):
        """Timer callback to publish feedback periodically."""

        # States to publish feedback for:
        if self._state not in self._feedback_state_list:
            return

        if self._active_goal_handle is None \
            or not self._active_goal_handle.is_active \
            or self._active_feedback is None:
            return
        
        feedback_msg = NavigationRequest.Feedback()
        feedback_msg.state = self._state
        feedback_msg.feedback_text = f'{self._active_feedback.feedback_text}' if self._active_feedback else '-'
        self._active_goal_handle.publish_feedback(feedback_msg)

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