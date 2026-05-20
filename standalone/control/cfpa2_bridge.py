"""CFPA2 → nav bridge — ported from cfpa2_to_nav2_bridge.py (rclpy stripped).

Translates a CFPA2 PointStamped waypoint into a (gx, gy, yaw) goal tuple for
the A* planner. Suppresses re-publishes of unchanged goals.

In standalone the "publish" side just writes to BUS topic /robot/goal_pose.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from ..core.bus import BUS
from ..core.ros_compat import PointStamped, PoseStamped, now_stamp

Goal2D = Tuple[float, float]  # (x, y)


class CFPA2Bridge:
    """Translate CFPA2 waypoints → planner goals."""

    def __init__(
        self,
        namespace: str = "robot",
        goal_change_min_m: float = 0.30,
        waypoint_topic_suffix: str = "/way_point_coord",
        goal_topic_suffix: str = "/goal_pose",
    ) -> None:
        self._ns = namespace
        self._change_min = goal_change_min_m
        self._last_goal: Optional[Goal2D] = None
        self._pose_x: Optional[float] = None
        self._pose_y: Optional[float] = None
        self._current_goal: Optional[Goal2D] = None

        wp_topic = f"/{namespace}{waypoint_topic_suffix}"
        goal_topic = f"/{namespace}{goal_topic_suffix}"
        odom_topic = f"/{namespace}/odom/nav"

        BUS.subscribe(wp_topic, self._on_waypoint)
        BUS.subscribe(odom_topic, self._on_odom)

        self._goal_topic = goal_topic

    def _on_odom(self, msg) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y

    def _on_waypoint(self, msg) -> None:
        gx = float(msg.point.x)
        gy = float(msg.point.y)

        if (self._last_goal is not None and
                math.hypot(gx - self._last_goal[0], gy - self._last_goal[1])
                < self._change_min):
            return

        yaw = 0.0
        if self._pose_x is not None:
            dx = gx - self._pose_x
            dy = gy - self._pose_y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        goal = PoseStamped.make(gx, gy, yaw)
        BUS.publish(self._goal_topic, goal)
        self._last_goal = (gx, gy)
        self._current_goal = (gx, gy)

    @property
    def current_goal(self) -> Optional[Goal2D]:
        return self._current_goal
