# Type defs for Ros Configurations:

from enum import Enum

# Enum for various topics:
class TopicKey(Enum):
    RGB_CAMERA ='/oakd/rgb/preview/image_raw'
    DEPTH = '/oakd/stereo/image_raw'
    SYNC = '/sync/robot/state'
    DETECTIONS = '/objects/detections'
    OBJECT_BR = '/objects/locations'
    GAZEBO_DEPTH = '/oakd/rgb/preview/depth'
    GAZEBO_RGB_CAMERA = '/oakd/rgb/preview/image_raw'
    OCCUPANCY_GRID = '/map'
    OBJECT_OVERLAY = '/grid/overlay'
    GRID_REPUBLISH = '/grid/occupancy'
    NAVIGATION_GRID = '/grid/navigation'
    NAVIGATION_GOAL = '/navigate/goal'

from dataclasses import dataclass

# Data class for ros configurations
@dataclass(frozen=True)
class RosConfig:
    max_messages: int = 10
    rgb_topic: str
    depth_topic: str
    detections_topic: str = TopicKey.DETECTIONS
    object_br_topic: str = TopicKey.OBJECT_BR
    sync_topic: str = TopicKey.SYNC
    occupancy_grid_topic = TopicKey.OCCUPANCY_GRID
    grid_repub_topic = TopicKey.GRID_REPUBLISH
    navigation_grid_topic = TopicKey.NAVIGATION_GRID
    object_overlay_topic = TopicKey.OBJECT_OVERLAY
    navigation_goal_topic = TopicKey.NAVIGATION_GOAL
