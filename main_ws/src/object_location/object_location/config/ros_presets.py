# Preset configurations for ROS2

from object_location.config.ros_config import(
    TopicKey,
    RosConfig
)

# Simulation (Gazebo) preset:
SIM_CFG = RosConfig(
    rgb_topic=TopicKey.GAZEBO_RGB_CAMERA.value,
    depth_topic=TopicKey.GAZEBO_DEPTH.value,
)

# Real (Non-simulation) presets:
STD_CFG = RosConfig(
    rgb_topic=TopicKey.RGB_CAMERA.value,
    depth_topic=TopicKey.DEPTH.value,
)