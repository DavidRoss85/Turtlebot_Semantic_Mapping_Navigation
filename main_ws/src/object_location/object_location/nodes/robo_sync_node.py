#This node will synchronize messages from multiple topics
#  and republish them as a single synchronized message.

# ROS2 Imports
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber #pip3 install message-filters
from sensor_msgs.msg import Image
from object_location_interfaces.msg import RoboSync as RSync

# Configurations:
from robot_common.ros_config import USING_GAZEBO
from object_location.config.ros_presets import STD_CFG, SIM_CFG

class RoboSyncNode(Node):

    #Class Constants
    DEFAULT_SIMULATE_FRAME_LOSS = 0 #frames
    DEFAULT_SIMULATE_LAG_TIME = 0   # seconds


    def __init__(self):
        super().__init__('robo_sync_node')
        self.get_logger().info('Initializing RoboSyncNode...')

        self._rgb_image = None
        self._depth_image = None
        self._robot_pose = None

        self._ros_config = SIM_CFG if USING_GAZEBO else STD_CFG
        
        # Simulate Frame Loss and Lag if enabled:
        self._simulate_frame_loss = self.DEFAULT_SIMULATE_FRAME_LOSS
        self._simulated_lag_time = self.DEFAULT_SIMULATE_LAG_TIME
        self._frame_counter = 0
        self._msg_queue = []
        # Create timer to simulate latency issues if enabled:
        if self._simulated_lag_time > 0:
            self._lag_timer = self.create_timer(
                self._simulated_lag_time,
                self._publish_late_message
            )

        # Screen message logging:
        self._last_screen_message = ['','',0]
        
        self._load_parameters()    # Load external parameters if any
        
        try:
            # Subscribers
            self._image_sub = Subscriber(self, Image, self._ros_config.rgb_topic)
            self.get_logger().info(f'Subscribed to RGB Image topic: {self._ros_config.rgb_topic}')
            
            self._depth_sub = Subscriber(self, Image, self._ros_config.depth_topic)
            self.get_logger().info(f'Subscribed to Depth Image topic: {self._ros_config.depth_topic}')
            
            # TF2 Buffer and Listener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.get_logger().info('TF2 Listener initialized.')

            # Synchronizer
            self._sync = ApproximateTimeSynchronizer(
                [self._image_sub, self._depth_sub],
                queue_size=self._ros_config.max_messages,
                slop=self._ros_config.sync_slop
            )
            self.get_logger().info('ApproximateTimeSynchronizer initialized.')
            self._sync.registerCallback(self._sync_callback)
            self.get_logger().info('Callback registered with synchronizer.')
            
            # Publisher
            self._pub = self.create_publisher(
                RSync,
                self._ros_config.sync_topic,
                self._ros_config.max_messages
            )


            self.get_logger().info(f'Publisher created on topic: {self._ros_config.sync_topic}')
            
            self.get_logger().info('RoboSync Node initialized and ready.')
        except Exception as e:
            self._last_screen_message = self._handle_error(
                e,
                '__init__()',
                'Failed to initialize RoboSync Node.',
                self._last_screen_message
            )
            self.destroy_node()
    #----------------------------------------------------------------------------------
    def _load_parameters(self):
        """Loads external parameters """
        #Enter code here to load parameters
        pass
    #----------------------------------------------------------------------------------
    def _sync_callback(self, image_msg, depth_msg):
        
        # Simulates Frame loss by skipping messages
        self._frame_counter += 1
        if self._frame_counter <= self._simulate_frame_loss:
            return
        else:
            self._frame_counter = 0
        
        # Get images and robot pose
        self._rgb_image = image_msg
        self._depth_image = depth_msg
        self._get_robot_pose()

        sync_msg = RSync()
        sync_msg.header.stamp = self._rgb_image.header.stamp
        sync_msg.rgb_image = self._rgb_image
        sync_msg.depth_image = self._depth_image

        if self._robot_pose is not None:
            sync_msg.robot_pose = self._robot_pose

        # When perfected, should publish only when all three messages are available
        if self._simulated_lag_time <= 0:
            self._pub.publish(sync_msg)
            # self._last_screen_message = self._log_message(
            #     'Published synchronized message.',
            #     '',
            #     self._last_screen_message
            # )
        else:
            self._msg_queue.append(sync_msg)
    #----------------------------------------------------------------------------------
    def _publish_late_message(self):
        # Simulates a delayed message
        if len(self._msg_queue) > 0:
            self._pub.publish(self._msg_queue.pop(0))
            self._last_screen_message = self._log_message(
                'Published delayed synchronized message.',
                '',
                self._last_screen_message
            )

    #----------------------------------------------------------------------------------
    def _get_robot_pose(self):
        try:
            # Get transform map → base_link
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self._robot_pose = trans
        except Exception as e:
            self._last_screen_message = self._handle_error(
                e,
                '__get_robot_pose()',
                'Error performing transform', 
                self._last_screen_message
            )

            
    #----------------------------------------------------------------------------------
    def _handle_error(self, error, function_name, custom_message='', last_message=None):
        """ Handles all errors and error logging"""
        
        if last_message is None: last_message = ['','',0]   # Null check

        prev_message, prev_function, counter = last_message   #unpack last message 

        if prev_function == function_name and\
            prev_message == custom_message:
             
            counter+=1
            print(f'\r x {counter}\t',end='',flush=True)
        
        else:
            print(f'\n')
            self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')
            counter = 0


        return [custom_message,function_name, counter]
    #----------------------------------------------------------------------------------
    def _log_message(self, message, message_type=None, last_log_message=None):
        """Handles displaying messages and logging"""
        if last_log_message is None: last_log_message = ['','',0]
        if message_type is None: message_type = ''
        
        prev_message, prev_type, counter = last_log_message
        if prev_type == message_type and prev_message == message:
            counter +=1
            print(f'\r x {counter}\t',end='',flush=True)
        else:
            print(f'\n')
            self.get_logger().info(message)
            counter = 0
            
        return [message,message_type, counter]

def main(args=None):
    rclpy.init(args=args)
    robo_sync_node = RoboSyncNode()

    try:
        rclpy.spin(robo_sync_node)
    except KeyboardInterrupt:
        print('Shutting down RoboSyncNode...')
    finally:
        robo_sync_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    