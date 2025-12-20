
from enum import Enum

class MissionState(str,Enum):
    """Robot mission states"""
    IDLE = "IDLE"
    DETECTING = "DETECTING"
    APPROACHING = "APPROACHING"
    PICKING = "PICKING"
    DONE = "DONE"