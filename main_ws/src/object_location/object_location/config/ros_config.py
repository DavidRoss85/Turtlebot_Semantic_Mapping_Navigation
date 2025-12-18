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
    detections_topic: str = TopicKey.DETECTIONS.value
    object_br_topic: str = TopicKey.OBJECT_BR.value
    sync_topic: str = TopicKey.SYNC.value
    occupancy_grid_topic = TopicKey.OCCUPANCY_GRID.value
    grid_repub_topic = TopicKey.GRID_REPUBLISH.value
    navigation_grid_topic = TopicKey.NAVIGATION_GRID.value
    object_overlay_topic = TopicKey.OBJECT_OVERLAY.value
    navigation_goal_topic = TopicKey.NAVIGATION_GOAL.value
    velocity_topic = TopicKey.CMD_VEL.value
    mission_state_topic = TopicKey.MISSION_STATE.value
    mission_status_topic = TopicKey.MISSION_STATUS.value
