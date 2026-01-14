from dataclasses import dataclass
from typing import Optional
import threading

from nav_msgs.msg import OccupancyGrid

@dataclass(frozen=True)
class NavGridSnapshot:
    msg: OccupancyGrid
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str

class NavMapCache:
    def __init__(self, node, topic_name: str):
        self._lock = threading.Lock()
        self._latest: Optional[NavGridSnapshot] = None
        self._sub = node.create_subscription(
            OccupancyGrid,
            topic_name,
            self._callback,
            10
        )

    def _callback(self, msg: OccupancyGrid):
        info = msg.info
        snap = NavGridSnapshot(
            msg=msg,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            frame_id=str(msg.header.frame_id),
        )
        with self._lock:
            self._latest = snap

    def has_map(self) -> bool:
        with self._lock:
            return self._latest is not None

    def latest(self) -> Optional[NavGridSnapshot]:
        with self._lock:
            return self._latest
