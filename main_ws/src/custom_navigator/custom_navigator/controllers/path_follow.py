import math
from typing import Optional
from geometry_msgs.msg import Twist

from robot_common.geometry import euclidean, wrap_to_pi, clamp

class PathFollower:
    def __init__(
        self,
        max_lin: float = 0.20,
        max_ang: float = 0.80,
        wp_tol: float = 0.10,
        goal_tol: float = 0.15,
        yaw_tol: float = 0.20,   # ~11 degrees
        k_lin: float = 0.8,
        k_ang: float = 1.5,
    ):
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.wp_tol = wp_tol
        self.goal_tol = goal_tol
        self.yaw_tol = yaw_tol
        self.k_lin = k_lin
        self.k_ang = k_ang

        self._plan = []
        self._i = 0
        self._active = False
        self._reached_goal = False

    #--------------------------------------------------------------------------------
    def start(self, world_plan: list):
        self._reached_goal = False
        self._plan = world_plan or []
        self._i = 0
        self._active = len(self._plan) > 0

    #--------------------------------------------------------------------------------
    def stop(self):
        self._active = False

    #--------------------------------------------------------------------------------
    def is_active(self) -> bool:
        return self._active

    #--------------------------------------------------------------------------------
    def has_reached_goal(self) -> bool:
        return self._reached_goal
    
    #--------------------------------------------------------------------------------
    def _target(self):
        if not self._active or self._i >= len(self._plan):
            return None
        return self._plan[self._i]  # [x, y]
    #--------------------------------------------------------------------------------
    def _advance_if_close(self, x: float, y: float):
        tgt = self._target()
        if tgt is None:
            self._active = False
            return

        d = math.hypot(tgt[0] - x, tgt[1] - y)

        # Advance through intermediate waypoints quickly
        if d <= self.wp_tol:
            self._i += 1
            if self._i >= len(self._plan):
                self._reached_goal = True
                self._active = False
    #--------------------------------------------------------------------------------
    def tick(self, x: float, y: float, yaw: float) -> Optional[Twist]:
        """
        Returns:
          Twist if still navigating, None if done (goal reached or no plan).
        """
        if not self._active:
            return None

        # Advance if we reached the current waypoint
        self._advance_if_close(x, y)

        # If we advanced past the end, we're done
        if not self._active:
            return None

        tgt = self._target()
        tx, ty = tgt[0], tgt[1]

        dx = tx - x
        dy = ty - y
        dist = euclidean(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - yaw)

        cmd = Twist()

        # Rotate-then-go:
        if abs(yaw_err) > self.yaw_tol:
            cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
            cmd.linear.x = 0.0
            return cmd

        # Drive forward with mild steering
        cmd.linear.x = clamp(self.k_lin * dist, 0.0, self.max_lin)
        cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)

        return cmd
    #--------------------------------------------------------------------------------
    def stop_twist(self) -> Twist:
        return Twist()
