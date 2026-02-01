""" Presets for robot configurations.  """

from robot_common.robot_model import RobotModel

#TurtleBot4 Preset
TB4_MODEL = RobotModel(
    radius_m=0.18,
    width_m=0.37,
    length_m=0.3,
    height_m=0.15,
    safety_margin_m=0.0
)