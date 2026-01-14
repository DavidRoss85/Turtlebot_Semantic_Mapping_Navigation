# For testing the navigation server action

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped
from custom_navigator_interfaces.action import NavigationRequest

from time import sleep

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigationRequest, 'navigate_to_pose')
        self._goal_handle = None
        
    def send_goal(self, request_type:int, request_text:str, position:TransformStamped ):
        goal_msg = NavigationRequest.Goal()
        goal_msg.request_type = request_type
        goal_msg.request_text = request_text
        goal_msg.position = position

        # Testing sending a goal then immediately canceling it
        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        # if not goal_handle.accepted:
        cancel_future = goal_handle.cancel_goal_async()
        
        self.get_logger().info('Goal canceled.')


def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()

    # Example goal
    req = 1 # ALIGN
    text = "Align to target"
    position = TransformStamped()
    position.transform.translation.x = 2.0

    navigation_client.send_goal(req, text, position)

    rclpy.spin(navigation_client)
    navigation_client.destroy_node()
    rclpy.shutdown()