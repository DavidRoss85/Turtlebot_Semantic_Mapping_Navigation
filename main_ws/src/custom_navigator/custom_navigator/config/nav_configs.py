from enum import Enum

class NavRequest(int, Enum):
    PAUSE = 0
    ALIGN = 1
    NAVIGATE = 2
    SEARCH = 3
    APPROACH = 4
    CANCEL = 5

class NavigationState(int, Enum):
    IDLE = 0
    ALIGNING = 1
    PLANNING = 2
    NAVIGATING = 3
    SEARCHING = 4
    APPROACHING = 5
    REACHED_GOAL = 6
    CANCELED = 7
    FAILED = 8
    PAUSED = 9
    EMERGENCY_STOP = 10


SERVER_STATES_TO_PROVIDE_FEEDBACK = {            
    NavigationState.ALIGNING,
    NavigationState.PLANNING,
    NavigationState.NAVIGATING,
    NavigationState.SEARCHING,
    NavigationState.APPROACHING,
    NavigationState.PAUSED
}  