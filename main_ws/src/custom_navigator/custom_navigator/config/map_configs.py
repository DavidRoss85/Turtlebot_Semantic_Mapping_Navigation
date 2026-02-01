# custom_navigator/config/map_configs.py

from dataclasses import dataclass

@dataclass(frozen=True)
class MapPolicy:
    occupancy_threshold: int = 50   # >= is considered blocked
    unknown_value: int = 50          # what unknown becomes after conversion
    inflation_scale: float = 1.0        # inflation radius = (robot_radius + safety_margin) * inflation_scale