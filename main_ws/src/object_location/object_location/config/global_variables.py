# Module for global variables and settings

USING_GAZEBO = False

OPEN_CELL_THRESHOLD = 50
A_STAR_HEAVY_WEIGHT = 5
A_STAR_LOOSE_THRESHOLD = 55

TURTLEBOT_RGB_TOPIC = '/oakd/rgb/preview/image_raw'
TURTLEBOT_DEPTH_TOPIC = '/oakd/stereo/image_raw'
GAZEBO_DEPTH_TOPIC = '/oakd/rgb/preview/depth'
ROBO_SYNC_TOPIC = '/sync/robot/state'
DETECTIONS_TOPIC = '/objects/detections'
LOCATIONS_TOPIC = '/objects/locations'
ROS_MAX_MSG = 10


#OpenCV
DEFAULT_IMAGE_ENCODING = 'passthrough'#'bgr8'  # OpenCV uses BGR format

# Default values:
DEFAULT_MAX = 10
DEFAULT_FEED_SHOW = False
DEFAULT_SHOULD_PUBLISH = True
DEFAULT_LINE_THICKNESS = 2
DEFAULT_FONT_SCALE = 0.5
DEFAULT_BGR_FONT_COLOR = (0,255,0)
DEFAULT_BGR_BOX_COLOR = (255,0,0)

#-------------------------------------------------------------------
# YOLO
YOLO_MODEL_LIST = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
YOLO_MODEL_PATH = YOLO_MODEL_LIST[1]  # Use yolov8n.pt for nano model
YOLO_CONFIDENCE_THRESHOLD = 0.8
YOLO_WANTED_LIST = ['bottle','cup','book']   # Items to look for
YOLO_REJECT_LIST = []

YOLO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
]

YOLO_IGNORE_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
]

YOLO_WANTED_CLASSES = [
    "backpack", "umbrella",
    "handbag", "bottle", "wine glass", "cup", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "book", "clock",
    "scissors",
]