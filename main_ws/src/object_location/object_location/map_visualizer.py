import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


import numpy as np
import cv2
from scipy import ndimage
from collections import defaultdict
from .helpers import (
    YOLO_CLASSES,
    convert_grid_to_world,
    fetch_origin_and_resolution
)

class MapVisualizerNode(Node):

    DEFAULT_MAP_TOPIC = '/grid/navigation'
    DEFAULT_OVERLAY_TOPIC = '/grid/overlay'
    DEFAULT_NAVIGATE_TOPIC = '/navigate/goal'

    DEFAULT_QOS = 10
    DEFAULT_UPDATE_INTERVAL = 0.5
    DEFAULT_OBJECT_OFFSET = 101
    DEFAULT_FONT = cv2.FONT_HERSHEY_SIMPLEX
    DEFAULT_FONT_SCALE = 0.5
    DEFAULT_FONT_THICKNESS = 1
    DEFAULT_FONT_COLOR = (0, 0, 0)
    EVEN_HUE_DISTRIBUTION = 30
    MAX_HUE = 180

    DEFAULT_FREE_SPACE_COLOR = [255, 255, 255]
    DEFAULT_UNKNOWN_SPACE_COLOR = [128, 128, 128]
    DEFAULT_OCCUPIED_SPACE_COLOR = [0, 0, 0]

    DEFAULT_SHOW_BLOBS = False
    DEFAULT_DRAW_TEXT_LABELS = True
    DEFAULT_DRAW_CENTROID_CIRCLE = True
    DEFAULT_DRAW_BOUNDING_BOXES = True

    #-----------------------------------------------------------------------------
    def __init__(self):
        super().__init__('map_visualizer')
        self.get_logger().info('Initializing map visualizer')
        self.get_logger().info('Setting defaults')
        
        self.__qos = self.DEFAULT_QOS
        self.__map_topic = self.DEFAULT_MAP_TOPIC
        self.__overlay_topic = self.DEFAULT_OVERLAY_TOPIC
        self.__navigate_topic = self.DEFAULT_NAVIGATE_TOPIC

        self.__update_interval = self.DEFAULT_UPDATE_INTERVAL
        self.__yolo_classes = YOLO_CLASSES  # YOLO class names
        self.__object_offset = self.DEFAULT_OBJECT_OFFSET
        
        self.__font = self.DEFAULT_FONT
        self.__font_scale = self.DEFAULT_FONT_SCALE
        self.__font_thickness = self.DEFAULT_FONT_THICKNESS
        self.__font_color = self.DEFAULT_FONT_COLOR

        self.__free_space_color = self.DEFAULT_FREE_SPACE_COLOR
        self.__unknown_space_color = self.DEFAULT_UNKNOWN_SPACE_COLOR
        self.__occupied_space_color = self.DEFAULT_OCCUPIED_SPACE_COLOR

        self.__show_blobs = self.DEFAULT_SHOW_BLOBS
        self.__draw_text_labels = self.DEFAULT_DRAW_TEXT_LABELS 
        self.__draw_centroid_circle = self.DEFAULT_DRAW_CENTROID_CIRCLE
        self.__draw_bounding_boxes = self.DEFAULT_DRAW_BOUNDING_BOXES

        self.__latest_map = None
        self.__latest_overlay = None

        self.__navigator = TurtleBot4Navigator()

        self.get_logger().info('Starting timer')
        
        self.__combined_view_timer = self.create_timer(
            self.__update_interval,
            self.__update_combined_view
        )
        self.get_logger().info('Subscribing to topics...')
        
        self.__map_subscription = self.create_subscription(
            OccupancyGrid,
            self.__map_topic,
            self.path_callback,
            self.__qos
        )
        self.__overlay_subscription = self.create_subscription(
            OccupancyGrid,
            self.__overlay_topic,
            self.overlay_callback,
            self.__qos
        )

        # Publisher for navigation
        self.__goal_publisher = self.create_publisher(
            Pose2D,
            self.__navigate_topic,
            self.__qos
        )
        
        
        self.get_logger().info('Map Visualizer Node initialized successfully')

    #-----------------------------------------------------------------------------
    def mouse_callback(self,event, x, y, flags, param):
        if self.__latest_map is None:
            return
        wx, wy = convert_grid_to_world(
            x, y,
            self.__latest_map['info']
        )
        goal_msg = Pose2D()


        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Asking robot to move to map coordinates: x={wx}, y={wy}")
            goal_msg.x = float(wx) 
            goal_msg.y = float(wy)
            goal_msg.theta = float(0) 
            self.__goal_publisher.publish(goal_msg)


        elif event == cv2.EVENT_RBUTTONDOWN:
            print(f"Asking robot to move to map coordinates: x={wy}, y={wx}")
            goal_msg.x = float(wy) 
            goal_msg.y = float(wx)
            goal_msg.theta = float(0)  
            self.__goal_publisher.publish(goal_msg)
            

    #-----------------------------------------------------------------------------
    def path_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        self.__latest_map = {
            'info':msg.info,
            'data':np.array(msg.data, dtype=np.uint16).reshape((height, width)).T
        }
    #-----------------------------------------------------------------------------
    def overlay_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        self.__latest_overlay = {
            'info':msg.info,
            'data':np.array(msg.data, dtype=np.uint16).reshape((height, width)).T
        }
    #-----------------------------------------------------------------------------
    def __update_combined_view(self):
        
        latest_map = self.__latest_map
        latest_overlay = None
        # Check for null values. If overlay is empty, fill with zeros.
        # This ensures that there can be a map to display even if there are no items in overlay
        if latest_map is None or latest_map['info'] is None or latest_map['data'] is None:
            return
        elif self.__latest_overlay is None or self.__latest_overlay['data'] is None:
            latest_overlay = dict()
            latest_overlay['info'] = latest_map['info']
            latest_overlay['data'] = np.zeros_like(latest_map['data'])
        else:
            latest_overlay = self.__latest_overlay
        
        if latest_map['info'].width != latest_overlay['info'].width or \
            latest_map['info'].height != latest_overlay['info'].height:
            return
        
        # Convert layers:
        overlay = latest_overlay['data'].astype(np.uint8)
        map = latest_map['data'].astype(np.uint8)


        # Combine layers
        combine_image = self.create_base_visualization(map, overlay)
        
        # Add labels
        labeled_image = self.find_and_label_objects(overlay, combine_image)

        # Show map window
        cv2.namedWindow("Map Visualizer", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Map Visualizer", self.mouse_callback)
        cv2.imshow("Map Visualizer", labeled_image)
        cv2.waitKey(1)

    #-----------------------------------------------------------------------------
    def create_base_visualization(self, map_data, overlay_data):
        """Create base visualization of the occupancy grid"""
        
        # Create a 3-channel BGR image
        map_image = np.zeros((map_data.shape[0], map_data.shape[1], 3), dtype=np.uint8)
        overlay_image = np.zeros((overlay_data.shape[0], overlay_data.shape[1], 3), dtype=np.uint8)
        
        # Color mapping for base occupancy values
        # Free space (0) - white
        map_image[map_data == 0] = self.__free_space_color
        
        # Unknown space (50) - gray
        map_image[map_data == 50] = self.__unknown_space_color
        
        # Occupied space (100) - black
        map_image[map_data == 100] = self.__occupied_space_color
        
        # Objects (>100) - different colors based on object type
        object_mask = overlay_data >= self.__object_offset
        if np.any(object_mask):
            # Use different colors for different object types
            unique_objects = np.unique(overlay_data[object_mask])
            for obj_val in unique_objects:
                # Generate a color based on the object value
                color = self.get_object_color(int(obj_val))
                overlay_image[overlay_data == obj_val] = color
        
        mask = overlay_data != 0 
        mask3 = np.stack([mask]*3, axis=-1)
        combine_image = map_image.copy()

        if self.__show_blobs:
            combine_image[mask3] = overlay_image[mask3]
                
        return combine_image
    #-----------------------------------------------------------------------------
    def get_object_color(self, obj_value):
        """Generate a distinct color for each object type"""
        # Use HSV to generate distinct colors
        hue = ((obj_value - self.__object_offset) * self.EVEN_HUE_DISTRIBUTION) % self.MAX_HUE  # Spread colors across hue spectrum
        color_hsv = np.array([[[hue, 255, 255]]], dtype=np.uint8)
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return color_bgr.tolist()
    #-----------------------------------------------------------------------------
    def find_and_label_objects(self, overlay_data, combine_image):
        """Find connected components of objects and label them"""
        
        # Dictionary to store object clusters
        object_clusters = defaultdict(list)
        
        # Process each unique object value > 100
        unique_objects = np.unique(overlay_data[overlay_data >= self.__object_offset])
        
        for obj_value in unique_objects:
            # Create binary mask for this object type
            obj_mask = (overlay_data == obj_value).astype(np.uint8)
            
            # Find connected components
            labeled_array, num_features = ndimage.label(obj_mask)
            
            # Process each connected component
            for component_id in range(1, num_features + 1):
                # Get coordinates of this component
                component_coords = np.where(labeled_array == component_id)
                
                if len(component_coords[0]) > 0:
                    # Calculate centroid of the component
                    centroid_y = int(np.mean(component_coords[0]))
                    centroid_x = int(np.mean(component_coords[1]))
                    
                    # Store cluster info
                    cluster_info = {
                        'centroid': (centroid_x, centroid_y),
                        'size': len(component_coords[0]),
                        'value': int(obj_value),
                        'coords': component_coords
                    }
                    object_clusters[obj_value].append(cluster_info)
        
        # Add labels to the image
        labeled_image = self.add_labels_to_image(combine_image, object_clusters)
        
        return labeled_image
    #-----------------------------------------------------------------------------
    def add_labels_to_image(self, image, object_clusters):
        """Add text labels to the visualization"""
        
        # Make a copy to avoid modifying original
        labeled_image = image.copy()
        
        # Font settings
        font = self.__font
        font_scale = self.__font_scale
        font_thickness = self.__font_thickness
        font_color = self.__font_color
        
        for obj_value, clusters in object_clusters.items():
            # Get object name from YOLO classes
            obj_name = self.__yolo_classes[obj_value - self.__object_offset]
            
            for i, cluster in enumerate(clusters):
                centroid = cluster['centroid']
                size = cluster['size']
                
                # Create label text
                if len(clusters) > 1:
                    # If multiple clusters of same type, add index
                    label = f"{obj_name} ({i+1})"
                else:
                    label = obj_name
                
                # Add size info if cluster is significant
                # if size > 10:
                #     label += f" ({size})"
                
                # Get text size for background rectangle
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, font, font_scale, font_thickness
                )
                
                # Calculate text position (slightly offset from centroid)
                text_x = centroid[0] - text_width // 2
                text_y = centroid[1] - 5
                
                # Ensure text stays within image bounds
                text_x = max(0, min(text_x, image.shape[1] - text_width))
                text_y = max(text_height, min(text_y, image.shape[0] - 5))
                
                # Draw background rectangle for better visibility
                # cv2.rectangle(
                #     labeled_image,
                #     (text_x - 2, text_y - text_height - 2),
                #     (text_x + text_width + 2, text_y + baseline + 2),
                #     (255, 255, 255),
                #     cv2.FILLED
                # )
                
                if self.__draw_text_labels:
                # Draw text
                    cv2.putText(
                        labeled_image,
                        label,
                        (text_x, text_y),
                        font,
                        font_scale,
                        font_color, 
                        font_thickness,
                        cv2.LINE_AA
                    )
                
                if self.__draw_centroid_circle:
                # Draw a small circle at centroid
                    cv2.circle(
                        labeled_image,
                        centroid,
                        3,
                        (0, 0, 255),  # Red circle
                        -1
                    )
                
                if self.__draw_bounding_boxes:
                # Optionally, draw bounding box around the cluster
                    if size > 20:  # Only for larger clusters
                        coords_y, coords_x = cluster['coords']
                        min_x, max_x = np.min(coords_x), np.max(coords_x)
                        min_y, max_y = np.min(coords_y), np.max(coords_y)
                        
                        cv2.rectangle(
                            labeled_image,
                            (min_x, min_y),
                            (max_x, max_y),
                            self.get_object_color(obj_value),
                            1
                        )

        return labeled_image
    #-----------------------------------------------------------------------------
    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MapVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()