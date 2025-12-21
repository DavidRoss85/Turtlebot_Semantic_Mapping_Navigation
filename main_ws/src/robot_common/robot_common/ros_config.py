# Type defs for Ros Configurations:

from enum import Enum

USING_GAZEBO = True

# Enum for various topics:
class TopicKey(str, Enum):
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
    CMD_VEL = '/cmd_vel'
    MISSION_STATE = '/mission/state'
    MISSION_STATUS = '/visual_servo/status'
    DYNAMIC_TRANSFORM = '/tf'
    STATIC_TRANSFORM = '/tf_static'

from dataclasses import dataclass

# Data class for ros configurations
@dataclass(frozen=True)
class RosConfig:
    rgb_topic: str
    depth_topic: str
    max_messages: int = 10
    sync_slop = 0.1
    detections_topic: str = TopicKey.DETECTIONS
    object_br_topic: str = TopicKey.OBJECT_BR
    sync_topic: str = TopicKey.SYNC
    occupancy_grid_topic: str = TopicKey.OCCUPANCY_GRID
    grid_repub_topic: str = TopicKey.GRID_REPUBLISH
    navigation_grid_topic: str = TopicKey.NAVIGATION_GRID
    object_overlay_topic: str = TopicKey.OBJECT_OVERLAY
    navigation_goal_topic: str = TopicKey.NAVIGATION_GOAL
    velocity_topic: str = TopicKey.CMD_VEL
    mission_state_topic: str = TopicKey.MISSION_STATE
    mission_status_topic: str = TopicKey.MISSION_STATUS
