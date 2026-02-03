
from typing import Optional
import numpy as np
from enum import Enum
import time
import threading

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.action import(
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.action.server import ServerGoalHandle

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_navigator_interfaces.action import NavigationRequest
from custom_navigator.mapping.map_cache import NavMapCache, NavGridSnapshot
from custom_navigator.mapping.inflated_costmap import InflatedCostmap
from custom_navigator.planners.astar import AStarPlanner
from custom_navigator.controllers.path_follow import PathFollower

from robot_common.logging import log_error, log_info
from robot_common.robot_model import RobotModel
from robot_common.geometry import quaternion_to_yaw, snap_grid_to_world, snap_world_to_grid

from custom_navigator.config.ros_presets import STD_CFG as ROS_CONFIG
from robot_common.robot_presets import TB4_MODEL as ROBOT_MODEL
from custom_navigator.config.map_presets import DEFAULT_MAP_POLICY as MAP_POLICY

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
        
        self._last_log_event = log_info(self.get_logger(), 'Starting Navigation Server Node...')
    
        self._ros_config = ROS_CONFIG
        self._robot_model = ROBOT_MODEL
        self._map_policy = MAP_POLICY
        self._state = NavigationState.IDLE  # Initialize the navigation state to IDLE

        self._execute_rate = 10  # Set the execution rate for the navigation logic
        self._map_update_interval = 0.05  # Interval to update the map in seconds
        self._map_timeout_s = 5.0  # Timeout for waiting for the map in seconds
        self._pose_timeout_s = 10.0  # Timeout for waiting for the robot pose in seconds
        self._nav_timer_interval = 0.1  # General timer interval in seconds
        self._feedback_timer_interval = 1.0  # Feedback timer interval in seconds
        
        self._active_goal_handle: ServerGoalHandle = None  # Currently active goal handle
        self._active_feedback: NavigationRequest.Feedback = None  # Currently active feedback message
        self._robot_pose: TransformStamped = None

        # List of states where feedback is published: (Move this into config later)
        self._feedback_state_list = {            
            NavigationState.ALIGNING,
            NavigationState.PLANNING,
            NavigationState.NAVIGATING,
            NavigationState.SEARCHING,
            NavigationState.APPROACHING,
            NavigationState.PAUSED
          }  

        self._goal_callback_lock = threading.Lock()
        self._sm_lock = threading.Lock() # State machine lock
        self._follower_lock = threading.Lock()  # Path follower lock

        self._cb_group = ReentrantCallbackGroup()   # Allows multiple callbacks to run simultaneously

        # TF2 Buffer and Listener for robot pose:
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_log_event = log_info(self.get_logger(), 'TF2 Listener initialized.')
        
        # Map cache for occupancy grid:
        self._map_cache = NavMapCache(
            self,                                   # Pass the node instance 
            self._ros_config.navigation_grid_topic, # Topic name for the occupancy grid
            self._ros_config.max_messages           # Max messages to store
        )                                       
        # Planner for path finding:
        self._planner = AStarPlanner() #  >>REMEMBER TO SET THE DEFAULTS LATER
        self._follower = PathFollower()  # Path follower controller

        # Initialize timers:
        # self._nav_timer = self.create_timer(
        #     self._nav_timer_interval,
        #     self._nav_timer_callback,
        #     callback_group=self._cb_group
        # )
        self._feedback_timer = self.create_timer(
            self._feedback_timer_interval,  # Feedback interval in seconds
            self._feedback_timer_callback,
            callback_group=self._cb_group
        )

        # Initialize the action server
        self._last_log_event = log_info(self.get_logger(), 'Initializing Action Server...')
        self._action_server = ActionServer(
            self,
            NavigationRequest,
            self._ros_config.navigation_server,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )
        self._last_log_event = log_info(self.get_logger(), 'Navigation Action Server initialized.')

        # Initialize publisher for velocity commands
        self._last_log_event = log_info(self.get_logger(), 'Initializing Velocity Publisher...')
        self._robot_message_publisher = self.create_publisher(
            Twist,
            self._ros_config.velocity_topic,
            self._ros_config.max_messages
        )
        self._last_log_event = log_info(self.get_logger(), 'Velocity Publisher initialized.')

    #----------------------------------------------------------------------------------
    def goal_callback(self, goal_request: NavigationRequest.Goal):
        """Executes upon receiving a goal request."""
        self._last_log_event = log_info(self.get_logger(), 'Received goal request')

        with self._goal_callback_lock:
            # Check if the server is idle before accepting a new goal
            if self._get_server_state() != NavigationState.IDLE:
                # For now it will simply reject new goals if not idle... Later we will add more complex logic
                self._last_log_event = log_info(self.get_logger(), 'Navigation is not idle, rejecting goal request')
                return GoalResponse.REJECT
            
            # User requests to navigate to a pose
            if goal_request.request_type == NavRequest.NAVIGATE:
            # Validate the goal here (e.g., check if the position is reachable)
                self._last_log_event = log_info(self.get_logger(), 'Accepted NAVIGATE goal request')
            
            
            # Goal will execute in execute_callback
            return GoalResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def cancel_callback(self, goal):
        """Executes upon receiving a cancel request."""
        self._last_log_event = log_info(self.get_logger(), 'Received cancel request')
        # Add cancel logic here

        return CancelResponse.ACCEPT
    
    #----------------------------------------------------------------------------------
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Executes upon receiving a goal request after approved by goal callback."""

        # Local variables
        result_msg = NavigationRequest.Result()
        delay_s = 1.0 / float(self._execute_rate)  # Set the rate 
        goal = goal_handle.request  # The actual goal request
        self._active_feedback = NavigationRequest.Feedback()  # Initialize feedback message
        final_text = "Navigation Terminated Unexpectedly"
        final_state = NavigationState.FAILED

        #----- Cleanup function -----
        def _cleanup_execution(state: NavigationState ,text: str  = ""):
            """Cleans up after a navigation failure."""
            self._active_goal_handle = None
            self._active_feedback = None
            result_msg.state = state
            result_msg.result_text = text
            self._last_log_event = log_info(self.get_logger(), text)
            self._reset_server_to_idle()
        #----------------------------

        # Initial checks before starting execution
    
        if self._get_server_state() == NavigationState.EMERGENCY_STOP:
            goal_handle.abort()
            final_text="Navigation server is not ready to execute goals."
            _cleanup_execution(NavigationState.EMERGENCY_STOP, final_text)
            return result_msg

        self._active_goal_handle = goal_handle # Store the active goal handle for use in other methods

        # ----- For navigation, plan the path -----
        if goal.request_type == NavRequest.NAVIGATE:
            self._set_server_state(NavigationState.PLANNING)
            #----- Wait for map to be available -----
            snapshot = self._wait_for_map(goal_handle, self._map_timeout_s)

            #----- Abort if map is not available -----
            if snapshot is None:
                goal_handle.abort()
                self._set_server_state(NavigationState.FAILED)
                final_text="Map not available, navigation aborted."
                _cleanup_execution(NavigationState.FAILED, final_text)
                return result_msg
            
            else:
                # ----- Proceed with navigation logic -----

                # -- Build the costmap --
                costmap = self._build_inflated_costmap_from_snapshot(
                    snap=snapshot,
                    robot_model=self._robot_model,
                    threshold=self._map_policy.occupancy_threshold
                )
                # -- Get the robot pose --
                pose = self._wait_for_pose(goal_handle, self._pose_timeout_s)
                # -- Abort if robot pose is not available --
                if pose is None:
                    goal_handle.abort()
                    self._set_server_state(NavigationState.FAILED)
                    final_text="Robot pose not available, navigation aborted."
                    _cleanup_execution(NavigationState.FAILED,final_text)
                    return result_msg
                
                # -- Convert robot pose to grid coordinates --
                grid_x, grid_y = snap_world_to_grid(
                    pose.transform.translation.x,
                    pose.transform.translation.y,
                    snapshot
                )
                # -- Get the goal destination in world coordinates --
                dest_x, dest_y = snap_world_to_grid(
                    goal.position.transform.translation.x,
                    goal.position.transform.translation.y,
                    snapshot
                )
                # -- Generate the path plan --
                self._planner.set_grid(costmap)
                grid_plan = self._planner.plan(
                    (grid_x, grid_y), 
                    (dest_x, dest_y)
                )
                # -- Convert plan to world coordinates --
                world_plan = self._convert_plan_to_world(grid_plan, snapshot)

                # -- Start the path follower --
                with self._follower_lock:
                    self._follower.start(world_plan)

                # -- Set state to NAVIGATING --
                self._set_server_state(NavigationState.NAVIGATING)
                    # ----------------------------------------

        # ----- Main execution loop -----
        while True:

            # ----- Check for Emergency Stop -----
            if self._get_server_state() == NavigationState.EMERGENCY_STOP:
                final_state = NavigationState.EMERGENCY_STOP
                final_text = "Emergency Stop"
                goal_handle.abort()
                break

            #  ----- Check if the goal is canceled -----
            if goal_handle.is_cancel_requested:
                self._set_server_state(NavigationState.CANCELED)
                final_state = NavigationState.CANCELED
                goal_handle.canceled()
                final_text="Navigation canceled by user."
                break
            
            # ----- Chceck if navigation goal failed -----
            if self._get_server_state() == NavigationState.FAILED:
                final_state = NavigationState.FAILED
                goal_handle.abort()
                final_text="Navigation failed."
                break
            
            # ----- Check if navigation goal succeeded -----
            if self._get_server_state() == NavigationState.REACHED_GOAL:
                final_state = NavigationState.REACHED_GOAL
                goal_handle.succeed()
                final_text="Navigation succeeded."
                break

            self._nav_execute()
            time.sleep(delay_s)
            

        
        _cleanup_execution(final_state, final_text)
        return result_msg

    
    #----------------------------------------------------------------------------------
    def _nav_execute(self):
        """Single navigation state-machine tick"""

        feedback_msg = NavigationRequest.Feedback()
        state = self._get_server_state()

        match state:
            case NavigationState.IDLE:
                # -- Ahhh... relax... --
                pass
            case NavigationState.ALIGNING:
                # -- Update feedback during aligning --
                feedback_msg.state = NavigationState.ALIGNING
                feedback_msg.feedback_text = 'Aligning robot to goal...'

            case NavigationState.PLANNING:
                # -- Update feedback during planning --
                feedback_msg.state = NavigationState.PLANNING
                feedback_msg.feedback_text = 'Planning path to goal...'

            case NavigationState.NAVIGATING:
                # -- Update feedback during navigation --
                feedback_msg.state = NavigationState.NAVIGATING
                feedback_msg.feedback_text = 'Navigating to goal...'

                # -- Update the path follower with current robot pose --
                pose = self._get_robot_pose()
                if pose is not None:
                    x = pose.transform.translation.x
                    y = pose.transform.translation.y
                    yaw = quaternion_to_yaw(pose.transform.rotation)

                    # -- Tick the follower --
                    cmd = None
                    if self._get_server_state() == NavigationState.NAVIGATING:
                        
                        with self._follower_lock:
                            cmd = self._follower.tick(x, y, yaw)
                    
                    # -- Publish velocity command if available --
                    if cmd is not None:
                        self._publish_velocity_command(cmd)
                    else:
                        self._publish_velocity_command(self._follower.stop_twist())

                        # Check if the path follower has reached the goal
                        if self._follower.has_reached_goal():
                            self._set_server_state(NavigationState.REACHED_GOAL)
                        else:
                            pass
                            #Some logic should go here I suppose if the robot has no command
                            #but has not reached goal
                            

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
            # case NavigationState.CANCELED:
            #     # -- Handle cancellation logic here --
            #     pass
            # case NavigationState.REACHED_GOAL:
            #     # -- Handle goal reached logic here --
            #     pass
            # case _:
            #     pass
        
        # Update active feedback message
        if self._active_feedback is not None:
            self._active_feedback.state = feedback_msg.state
            self._active_feedback.feedback_text = feedback_msg.feedback_text
    #----------------------------------------------------------------------------------
    def _wait_for_map(self, goal_handle: ServerGoalHandle, timeout_s: float = 2.0) -> Optional[NavGridSnapshot]:
        """Wait for the map to be available in the map cache."""
        start = time.time()

        while True:
            # Try to grab snapshot first
            snap = self._map_cache.latest()
            if snap is not None:
                return snap

            # Allow fast cancel / estop while waiting
            if self._get_server_state() == NavigationState.EMERGENCY_STOP:
                return None
            if goal_handle.is_cancel_requested:
                return None
            if (time.time() - start) > timeout_s:
                return None

            time.sleep(self._map_update_interval)
    
    #----------------------------------------------------------------------------------
    def _wait_for_pose(self, goal_handle: ServerGoalHandle, timeout_s: float = 10.0) -> Optional[TransformStamped]:
        """Wait for the robot pose to be available."""
        start = time.time()

        while True:
            # Try to grab pose first
            pose = self._get_robot_pose()
            if pose is not None:
                return pose

            # Allow fast cancel / estop while waiting
            if self._get_server_state() == NavigationState.EMERGENCY_STOP:
                return None
            if goal_handle.is_cancel_requested:
                return None
            if (time.time() - start) > timeout_s:
                return None

            time.sleep(self._map_update_interval)
    
    #--------------------------------------------------------------------------------
    def _convert_snap_to_grid(self, snap: NavGridSnapshot) -> np.ndarray:
        """Convert a NavGridSnapshot to a 2D NumPy array."""
        grid = np.array(snap.data, dtype=np.int16).reshape((snap.height, snap.width))
        grid[grid < 0] = self._map_policy.unknown_value  # Convert unknown cells to specified value
        return grid
    
    #--------------------------------------------------------------------------------
    def _build_inflated_costmap_from_snapshot(self,snap: NavGridSnapshot, robot_model: RobotModel, threshold=50):
        occ = self._convert_snap_to_grid(snap)
        inflation_radius = (robot_model.radius_m + robot_model.safety_margin_m) * self._map_policy.inflation_scale

        return InflatedCostmap(
            occupancy_grid=occ,
            inflation_radius=inflation_radius,
            resolution=snap.resolution,
            threshold=threshold,
        ).get_inflated_costmap()

    #--------------------------------------------------------------------------------
    def _convert_plan_to_world(self, plan: list, snap: NavGridSnapshot) -> list:
        """Convert a list of grid coordinates to world coordinates using a NavGridSnapshot."""
        world_path = []
        for cell in plan:
            i, j = cell
            x, y = snap_grid_to_world(i, j, snap)
            world_path.append([x, y])
        return world_path
    
    #----------------------------------------------------------------------------------
    def _get_robot_pose(self)-> TransformStamped:
        try:
            # Get transform map → base_link
            return self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self._last_log_event = log_error(
                self.get_logger(),
                e,
                self._get_robot_pose.__qualname__,
                'Error performing transform', 
                self._last_log_event
            )
            return None

    #----------------------------------------------------------------------------------
    def _feedback_timer_callback(self):
        """Timer callback to publish feedback periodically."""

        # States to publish feedback for:
        if self._get_server_state() not in self._feedback_state_list:
            return

        if self._active_goal_handle is None \
            or not self._active_goal_handle.is_active \
            or self._active_feedback is None:
            return
        
        feedback_msg = NavigationRequest.Feedback()
        feedback_msg.state = self._get_server_state()
        feedback_msg.feedback_text = f'{self._active_feedback.feedback_text}' if self._active_feedback else '-'
        self._active_goal_handle.publish_feedback(feedback_msg)

    #----------------------------------------------------------------------------------
    def _publish_velocity_command(self, cmd: Twist):
        """Publish a velocity command to the robot."""
        self._robot_message_publisher.publish(cmd)

    #----------------------------------------------------------------------------------
    def _reset_server_to_idle(self):
        """Reset the navigation server to IDLE state."""

        self._set_server_state(NavigationState.IDLE)
        self._follower.stop()
        stop_cmd = Twist()
        self._publish_velocity_command(stop_cmd)

    #----------------------------------------------------------------------------------
    def _get_server_state(self)-> NavigationState:
        with self._sm_lock:
            return self._state
        
    #----------------------------------------------------------------------------------
    def _set_server_state(self, state:NavigationState):
        with self._sm_lock:
            self._state = state
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