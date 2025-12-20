# Preset configurations for ROS2

from robot_common.ros_config import(
    TopicKey,
    RosConfig
)

# Simulation (Gazebo) preset:
SIM_CFG = RosConfig(
    rgb_topic=TopicKey.GAZEBO_RGB_CAMERA,
    depth_topic=TopicKey.GAZEBO_DEPTH,
)

# Real (Non-simulation) presets:
STD_CFG = RosConfig(
    rgb_topic=TopicKey.RGB_CAMERA,
    depth_topic=TopicKey.DEPTH,
)