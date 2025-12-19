#This node performs object detection using a YOLO model on incoming RGB images
# and publishes detected objects along with annotated images.

# Math Imports:
import math
import numpy as np
import torch

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from object_location_interfaces.msg import RoboSync as RSync, DetectedItem, DetectionList, RSyncDetectionList


# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# YOLO library:
from ultralytics import YOLO #pip3 install typeguard ultralytics
#Ultralytics glitch when attempting to run. Use export to ensure proper import:
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH

# Configurations:
from object_location.config.ros_presets import STD_CFG
from object_location.config.detection_presets import DEFAULT_DETECTION_CONFIG
from object_location.config.yolo_presets import(
    DEFAULT_YOLO_CONFIG, 
    ComputePreference as CPU_Mode,
    YoloConfig,
    DetectionFilterMode as DetecMode
)

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('Initializing Detection Node...')

        #Configurations:
        self._ros_config = STD_CFG
        self._yolo_config = DEFAULT_YOLO_CONFIG
        self._detection_config = DEFAULT_DETECTION_CONFIG
        
        self._pure_image = None    #Stores the pure image returned from the camera
        self._annotated_image = None   #Stores the image with boxes and identifiers
        
        
        self._load_parameters()    #Load external parameters

        self._bridge = CvBridge()   # Initialize CV Bridge
        self.get_logger().info(f'CV Bridge loaded')

        self._process_device = self._select_device(self._yolo_config)   # Select processing device based on config
        self._model = YOLO(self._yolo_config.model.value)   # Load YOLO model

        self.get_logger().info(f'YOLO model loaded from: {self._yolo_config.model}')
        self.get_logger().info(f'Model confidence threshold set to: {self._yolo_config.confidence_threshold}')
        self.get_logger().info(f'Number of items in dataset: {len(self._model.names)}')

        try:
            # Subscriber
            self._rsync_sub = self.create_subscription(
                RSync,
                self._ros_config.sync_topic,
                self._process_detections,
                self._ros_config.max_messages
            )
            self.get_logger().info(f'Subscribed to sync topic: {self._ros_config.sync_topic}')

            # Publisher
            self._detection_pub = self.create_publisher(
                RSyncDetectionList,
                self._ros_config.detections_topic,
                self._ros_config.max_messages
            )
            self.get_logger().info(f'Publisher created on topic: {self._ros_config.detections_topic}')

            self.get_logger().info('Detection Node initialized and ready.')
        except Exception as e:
            self._handle_error(e,'__init__()','Failed to initialize Detection Node')
            self.destroy_node()
    
    #----------------------------------------------------------------------------------
    def _load_parameters(self):
        """Loads external parameters """
        #Enter code here to load parameters
        pass

    #----------------------------------------------------------------------------------
    def _select_device(self, config: YoloConfig) -> str:
        """Selects the device to process on based on the configuration."""
        if config.compute_preference == CPU_Mode.CPU_ONLY or \
            config.compute_preference == CPU_Mode.THROTTLED:
            return 'cpu'
        else:
             return 'cuda:0' if torch.cuda.is_available() else 'cpu'
    #----------------------------------------------------------------------------------
    def _process_detections(self,message:RSync):

        rgb_image = message.rgb_image

        # Convert ros2 message to image:
        cv_image = self._bridge.imgmsg_to_cv2(rgb_image,self._detection_config.image_encoding)
        # Pass frame through model and return results:
        results = self._model(cv_image, verbose=False,device=self._process_device)[0]
        # Save original image:
        self._pure_image = cv_image
        self._annotated_image = cv_image
        #Reset list:
        self._detected_list=[]
        
        
        # Get items and label if meet criteria
        if results.boxes is not None:
            for box in results.boxes:
                if self._meets_critera(box):
                    # Get box coordinates:
                    x1,y1,x2,y2 = map(int,box.xyxy[0].tolist()) #Convert tensor to list
                    box_coords = [x1,y1,x2,y2]

                    xc,yc,w,h = map(int,box.xywh[0].tolist()) #Grab center of box
                    box_center = [xc,yc,w,h]

                    # Get box name:
                    item_name = self._model.names[int(box.cls)]    # Convert item index to name

                    # Create an object to hold detected information
                    detected_object = DetectedObject(
                        xyxy=box_coords,
                        xywh=box_center,
                        name=item_name,
                        confidence=float(box.conf),
                        index=int(box.cls)
                    )
                    
                    # Add to list:
                    self._detected_list.append(detected_object)
                    
                    # Add bounding box to annotated frame
                    self._annotated_image = self._add_bounding_box_to_image(
                        self._annotated_image,
                        xyxy = box_coords,
                        bgr = self._detection_config.bgr_box_color,
                        thickness= self._detection_config.line_thickness
                    )

                    # Add text with item's name/type to annotated frame
                    self._annotated_image=  self._add_text_to_image(
                        self._annotated_image,
                        x = box_coords[0],
                        y = box_coords[1],
                        text = item_name,
                        bgr = self._detection_config.bgr_font_color
                    )

        # Publish message if there are detections:
        if len(self._detected_list) > 0:
            pub_msg = RSyncDetectionList()
            pub_msg.header.stamp = rclpy.time.Time().to_msg()
            pub_msg.robo_sync =  message
            pub_msg.detections = self._generate_detection_list()
            self._publish_data(pub_msg)

        # Show feed if flag is set
        if self._detection_config.show_feed:
            cv2.imshow("Detections",self._annotated_image)
            cv2.waitKey(1)


    #----------------------------------------------------------------------------------
    def _generate_detection_list(self):
        """
        Generates a message to publish based on information
        stored within the object
        returns: DetectionList
        """
        pub_image_raw = self._bridge.cv2_to_imgmsg(self._pure_image)
        pub_image_annotated = self._bridge.cv2_to_imgmsg(self._annotated_image)
        pub_item_list = []
        for item in self._detected_list:
            pub_item = DetectedItem()
            pub_item.name = item.name
            pub_item.xyxy = item.xyxy
            pub_item.xywh = item.xywh
            pub_item.confidence = item.confidence
            pub_item.index = item.index

            pub_item_list.append(pub_item)

        pub_frame_message = DetectionList()
        pub_frame_message.image_raw = pub_image_raw
        pub_frame_message.image_annotated = pub_image_annotated
        pub_frame_message.item_list = pub_item_list

        return pub_frame_message
    #----------------------------------------------------------------------------------
    def _meets_critera(self, box):
        """
        Criteria for identifying an object
        """
        # If confidence is over threshold
        if box.conf < self._yolo_config.confidence_threshold:
            return False
        
        # If there's a list of items to look for then filter by list
        if self._yolo_config.filter_mode == DetecMode.ALLOW\
        and self._model.names[int(box.cls)] not in self._yolo_config.target_classes:
            return False
        
        # Check the reject list:
        elif self._yolo_config.filter_mode == DetecMode.REJECT\
        and self._model.names[int(box.cls)] in self._yolo_config.ignore_classes:
            return False
        
        return True
    #----------------------------------------------------------------------------------
    def _add_bounding_box_to_image(self, image, xyxy=[0,0,0,0],bgr:tuple=(0,0,0),thickness:int=1):
        """Draw a rectangle at the given coordinates"""
        cv2.rectangle(
            image, 
            (xyxy[0],xyxy[1]),
            (xyxy[2],xyxy[3]),
            bgr,
            thickness
        )
        return image
    #----------------------------------------------------------------------------------
    def _add_text_to_image(self, image, x:int=0, y:int=0, text='',bgr:tuple=(0,0,0),font=cv2.FONT_HERSHEY_SIMPLEX,thickness=1):
        """Modify image with text at location"""
        cv2.putText(
            image,
            text,
            (x,y),
            font,
            self._detection_config.font_scale,
            bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA
        )
        return image
 
    #----------------------------------------------------------------------------------
    def _publish_data(self,message:RSyncDetectionList):
        self._detection_pub.publish(message)

    #----------------------------------------------------------------------------------
    def _handle_error(self, error, function_name, custom_message=''):
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')


#*******************************************************
# Generic class to store detected objects data 
# (THIS IS POINTLESS... MAKE NOTE TO REPLACE WITH a normal DetectedItem())
class DetectedObject:
    def __init__(self, xyxy:list=[0,0,0,0], xywh:list =[0,0,0,0], name:str='',confidence:float=0.0, index:int=0):
        self.xyxy = xyxy
        self.xywh = xywh
        self.name = name
        self.confidence = confidence
        self.index = index



def main(args=None):
    rclpy.init()
    detection_node = DetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()