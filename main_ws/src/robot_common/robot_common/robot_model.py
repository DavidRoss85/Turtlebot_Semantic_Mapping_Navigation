

# robot_common/robot_model.py
from dataclasses import dataclass

@dataclass(frozen=True)
class RobotModel:
    radius_m: float = 0.18         # footprint radius
    width_m: float = 0.37          # robot width
    length_m: float  = 0.3
    height_m: float = 0.15
    safety_margin_m: float = 0.0
