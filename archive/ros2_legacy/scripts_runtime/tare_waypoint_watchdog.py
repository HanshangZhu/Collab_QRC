#!/usr/bin/env python3
"""Reject TARE waypoints that land on obstacle geometry.

TARE picks frontier viewpoints from its rolling occupancy grid. In
constrained indoor scenes, a frontier can sit on or just inside a wall
voxel — FAR then can't route to it, localPlanner can't reach it, and the
stack wedges because TARE re-publishes the same bad goal forever.

This watchdog checks every fresh waypoint against the latest
``terrain_map`` (the same PointCloud2 that localPlanner reads, so the
geometric decision stays in the sensor-derived domain — no MuJoCo
cheating). If enough nearby points have ``intensity >=
obstacle_height_thre`` the waypoint is classified as in-obstacle and we
publish ``std_msgs/Empty`` on ``reset_waypoint``. TARE's existing
ResetWaypointCallback then drops the current lookahead and picks the
next-best viewpoint on its next planning tick.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Empty
from visualization_msgs.msg import Marker


class TareWaypointWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("tare_waypoint_watchdog")

        self.declare_parameter("terrain_map_topic", "terrain_map")
        self.declare_parameter("waypoint_topic", "way_point_coord")
        self.declare_parameter("reset_topic", "reset_waypoint")
        self.declare_parameter("obstacle_radius", 0.4)
        self.declare_parameter("obstacle_height_thre", 0.2)
        self.declare_parameter("min_obstacle_points", 3)
        self.declare_parameter("reset_cooldown_sec", 2.0)
        # 2D occupancy grid from octomap's projected_map. When the waypoint
        # cell (or any cell within occgrid_inflate_m of it) has value >=
        # occgrid_occupied_thre, treat as in-wall and reset. This is the
        # complement to the terrain-point check: terrain_map has no points
        # inside a wall volume (LiDAR can't see through), but octomap
        # inflates observed wall surfaces to occupied cells that cover the
        # wall's thickness.
        self.declare_parameter("occgrid_topic", "map")
        self.declare_parameter("occgrid_occupied_thre", 50)
        self.declare_parameter("occgrid_inflate_m", 0.25)
        # Stall detector. Tracks the minimum robot→waypoint distance
        # observed since the waypoint was last seen fresh; if that distance
        # hasn't improved in `stall_timeout_sec` and the waypoint hasn't
        # changed, treat as "robot can't get any closer" and reset.
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("stall_timeout_sec", 10.0)
        self.declare_parameter("stall_improve_epsilon_m", 0.05)
        self.declare_parameter("waypoint_change_epsilon_m", 0.20)
        # Visualization — publish a fat Marker sphere at the current goal
        # so RViz shows it clearly (TARE's raw PointStamped is a tiny dot).
        self.declare_parameter("marker_topic", "way_point_marker")
        self.declare_parameter("marker_frame", "map")
        # Robot pose marker — existing RViz configs (real autonomy.rviz)
        # have a "RobotPoseTriangle" display on /{ns}/robot_pose_marker
        # that's normally driven by astar_nav_node. Our nav_backend=far /
        # tare_real path doesn't run astar, so the topic has zero publishers
        # and the display stays blank. Republish the pose as a triangle
        # marker here so the display works regardless of planner.
        self.declare_parameter("robot_marker_topic", "robot_pose_marker")
        # Nogo boundary — the REAL blacklist. /reset_waypoint is only a
        # one-tick nudge in TARE's code; the next planning cycle re-picks
        # the same high-scoring viewpoint. By publishing a persistent
        # polygon around each stuck goal on /nogo_boundary, the
        # viewpoint_manager marks every viewpoint inside as invalid —
        # genuine blacklisting. The watchdog accumulates polygons and
        # re-publishes the whole set each time (TARE overwrites its
        # internal list on every message).
        self.declare_parameter("nogo_topic", "nogo_boundary")
        self.declare_parameter("nogo_square_half_m", 0.6)
        self.declare_parameter("nogo_max_regions", 40)
        # Do NOT blacklist waypoints that are within this radius of the
        # robot — those are almost always TARE's kExtendWayPoint collapsing
        # onto the robot after a previous reset, not real bad frontiers.
        # Blacklisting them would exclude the robot's own traversable area.
        self.declare_parameter("nogo_min_dist_from_robot_m", 0.5)
        # Stall detector treats "already there" (min_dist below this) as a
        # non-stall — robot is effectively at the goal, waiting for TARE
        # to advance to the next viewpoint. Matches pathFollower's
        # goalCloseDis.
        self.declare_parameter("stall_already_there_m", 0.4)

        p = self.get_parameter
        self._radius2 = float(p("obstacle_radius").value) ** 2
        self._h_thre = float(p("obstacle_height_thre").value)
        self._min_pts = int(p("min_obstacle_points").value)
        self._cooldown = float(p("reset_cooldown_sec").value)
        self._occ_thre = int(p("occgrid_occupied_thre").value)
        self._occ_inflate = float(p("occgrid_inflate_m").value)
        self._stall_timeout = float(p("stall_timeout_sec").value)
        self._stall_eps = float(p("stall_improve_epsilon_m").value)
        self._wp_change_eps = float(p("waypoint_change_epsilon_m").value)
        self._marker_frame = str(p("marker_frame").value)
        self._nogo_half = float(p("nogo_square_half_m").value)
        self._nogo_max = int(p("nogo_max_regions").value)
        self._nogo_min_dist = float(p("nogo_min_dist_from_robot_m").value)
        self._stall_near_m = float(p("stall_already_there_m").value)
        self._nogo_regions: list[tuple[float, float]] = []

        self._terrain_xy_h: np.ndarray | None = None
        # occgrid state: tuple (origin_x, origin_y, resolution, width, height, data_np)
        self._occgrid = None
        # robot pose latest (x, y) and yaw (rad, z-axis)
        self._pose: tuple[float, float] | None = None
        self._yaw: float = 0.0
        # stall-tracking state for the current goal
        self._cur_goal: tuple[float, float] | None = None
        self._goal_min_dist = float("inf")
        self._goal_last_improved_t = 0.0
        self._last_reset = -float("inf")
        self._n_checked = 0
        self._n_reset_terrain = 0
        self._n_reset_occgrid = 0
        self._n_reset_stall = 0
        self._n_reset_oob = 0  # out of occgrid bounds (unobserved territory)
        self._n_missing_terrain = 0
        self._max_obs_seen = 0

        self.create_subscription(
            PointCloud2, p("terrain_map_topic").value, self._terrain_cb, 5)
        self.create_subscription(
            PointStamped, p("waypoint_topic").value, self._waypoint_cb, 5)
        # octomap_server publishes /map with TRANSIENT_LOCAL + RELIABLE (latched).
        # Match that durability so we actually receive the stored grid.
        occ_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, p("occgrid_topic").value, self._occgrid_cb, occ_qos)
        self.create_subscription(
            Odometry, p("odom_topic").value, self._odom_cb, 10)
        self._reset_pub = self.create_publisher(
            Empty, p("reset_topic").value, 5)
        self._marker_pub = self.create_publisher(
            Marker, p("marker_topic").value, 5)
        self._robot_marker_pub = self.create_publisher(
            Marker, p("robot_marker_topic").value, 5)
        self._nogo_pub = self.create_publisher(
            PolygonStamped, p("nogo_topic").value, 5)
        # Stall check timer — independent of the waypoint callback rate so
        # we detect stall even when TARE goes silent on a bad goal.
        self.create_timer(1.0, self._stall_tick)

        self.get_logger().info(
            f"watchdog up: waypoint_topic={p('waypoint_topic').value}, "
            f"terrain_topic={p('terrain_map_topic').value}, "
            f"radius={p('obstacle_radius').value} m, "
            f"h_thre={self._h_thre} m, min_pts={self._min_pts}"
        )
        # Heartbeat — emits once every 5 s so silent sessions tell us whether
        # the watchdog is idle because waypoints are clean or because it never
        # sees terrain / waypoints at all.
        self.create_timer(5.0, self._heartbeat)

    def _terrain_cb(self, msg: PointCloud2) -> None:
        # read_points_numpy returns an Nx3 structured-flat array here.
        arr = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "intensity"), skip_nans=True)
        if arr.size == 0:
            return
        self._terrain_xy_h = arr

    def _odom_cb(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Extract yaw so we can orient the robot pose marker.
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._publish_robot_marker()

    def _publish_robot_marker(self) -> None:
        if self._pose is None:
            return
        m = Marker()
        m.header.frame_id = self._marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "robot_pose"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = float(self._pose[0])
        m.pose.position.y = float(self._pose[1])
        m.pose.position.z = 0.15
        # Quaternion from yaw (z-axis rotation).
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(self._yaw * 0.5)
        m.pose.orientation.w = math.cos(self._yaw * 0.5)
        m.scale.x = 0.6   # shaft length
        m.scale.y = 0.12  # shaft diameter
        m.scale.z = 0.12  # head diameter
        m.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.95)
        m.lifetime.sec = 0
        self._robot_marker_pub.publish(m)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_reset(self, reason: str, wp_x: float, wp_y: float, bucket: str) -> bool:
        now = self._now()
        if now - self._last_reset < self._cooldown:
            return False
        self.get_logger().warn(
            f"waypoint ({wp_x:.2f}, {wp_y:.2f}) — {reason} — "
            f"reset+blacklist (nogo regions={len(self._nogo_regions) + 1})"
        )
        self._reset_pub.publish(Empty())
        # Persistent blacklist: add a square around the bad goal to
        # /nogo_boundary so TARE's viewpoint_manager won't re-pick a
        # viewpoint near this location on the next planning tick.
        self._add_nogo_region(wp_x, wp_y)
        self._last_reset = now
        if bucket == "terrain":
            self._n_reset_terrain += 1
        elif bucket == "occgrid":
            self._n_reset_occgrid += 1
        elif bucket == "stall":
            self._n_reset_stall += 1
        elif bucket == "oob":
            self._n_reset_oob += 1
        # Restart the stall tracker so we don't re-fire immediately.
        self._cur_goal = None
        self._goal_min_dist = float("inf")
        return True

    def _add_nogo_region(self, x: float, y: float) -> None:
        """Blacklist a square centered at (x,y) and re-publish the full set.

        TARE's NogoBoundaryCallback overwrites its internal list on every
        message, so we have to re-emit the accumulated polygons each time.
        Multiple polygons ride in a single PolygonStamped separated by
        distinct `z` values on their points — see TARE's parser at
        sensor_coverage_planner_ground.cpp:591.
        """
        # Skip blacklisting points right on the robot — those are almost
        # always TARE's kExtendWayPoint collapsing onto the robot after a
        # reset nudge, not real bad frontiers. Blacklisting them would
        # shrink the robot's usable viewpoint sampling space around itself.
        if self._pose is not None:
            dr = math.hypot(self._pose[0] - x, self._pose[1] - y)
            if dr < self._nogo_min_dist:
                return
        # Dedupe: skip if we already have a region near this point.
        for rx, ry in self._nogo_regions:
            if math.hypot(rx - x, ry - y) < self._nogo_half:
                return
        self._nogo_regions.append((x, y))
        # Cap memory — drop oldest when over limit.
        if len(self._nogo_regions) > self._nogo_max:
            self._nogo_regions = self._nogo_regions[-self._nogo_max:]
        self._publish_nogo_regions()

    def _publish_nogo_regions(self) -> None:
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._marker_frame
        half = self._nogo_half
        for idx, (cx, cy) in enumerate(self._nogo_regions):
            # Counter-clockwise square; all 4 points share z=idx so the
            # TARE parser groups them into one polygon.
            z = float(idx)
            for dx, dy in [(-half, -half), (+half, -half),
                           (+half, +half), (-half, +half)]:
                p = Point32()
                p.x = float(cx + dx)
                p.y = float(cy + dy)
                p.z = z
                msg.polygon.points.append(p)
        self._nogo_pub.publish(msg)

    def _publish_goal_marker(self, x: float, y: float) -> None:
        m = Marker()
        m.header.frame_id = self._marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "tare_goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        m.color = ColorRGBA(r=1.0, g=0.2, b=1.0, a=0.9)
        m.lifetime.sec = 0  # forever (until replaced)
        self._marker_pub.publish(m)

    def _stall_tick(self) -> None:
        """Periodic check — fires even when TARE stopped publishing."""
        if self._cur_goal is None or self._pose is None:
            return
        gx, gy = self._cur_goal
        d = math.hypot(gx - self._pose[0], gy - self._pose[1])
        # "Already there": robot is inside goalCloseDis. Don't treat that
        # as a stall — pathFollower has effectively reached it, we're
        # waiting for TARE's next viewpoint pick. Reset the improvement
        # clock so if TARE is genuinely idle elsewhere we still notice
        # when it comes back with a new goal.
        if d <= self._stall_near_m:
            self._goal_last_improved_t = self._now()
            self._goal_min_dist = d
            return
        if d + self._stall_eps < self._goal_min_dist:
            self._goal_min_dist = d
            self._goal_last_improved_t = self._now()
            return
        if self._now() - self._goal_last_improved_t < self._stall_timeout:
            return
        reason = (f"stall: min_dist={self._goal_min_dist:.2f} m, "
                  f"stuck for {self._stall_timeout:.0f} s")
        self._publish_reset(reason, gx, gy, "stall")

    def _occgrid_cb(self, msg: OccupancyGrid) -> None:
        # Pack into a compact tuple; we only need 2D indexing.
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self._occgrid = (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.resolution,
            msg.info.width,
            msg.info.height,
            data,
        )

    def _waypoint_in_occupied(self, wx: float, wy: float) -> int:
        """Occupied-cell count within inflate radius of (wx, wy).
        Returns:
          -1  → no occgrid yet (unknown)
          -2  → waypoint is fully outside the occgrid bounds
                (i.e. in UNOBSERVED territory beyond the map extent)
          n≥0 → occupied cell count within radius (n>0 → in wall)
        """
        if self._occgrid is None:
            return -1
        ox, oy, res, w, h, data = self._occgrid
        if res <= 0.0:
            return -1
        half = max(1, int(math.ceil(self._occ_inflate / res)))
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        # Bail with -2 when the whole query region is outside the grid
        # (cx±half) — this is the "goal is in unexplored space beyond the
        # map" case and deserves its own blacklist reason.
        if cx + half < 0 or cy + half < 0 or cx - half >= w or cy - half >= h:
            return -2
        i0 = max(0, cx - half)
        i1 = min(w, cx + half + 1)
        j0 = max(0, cy - half)
        j1 = min(h, cy + half + 1)
        block = data[j0:j1, i0:i1]
        return int(np.count_nonzero(block >= self._occ_thre))

    def _waypoint_cb(self, msg: PointStamped) -> None:
        self._n_checked += 1
        wx, wy = msg.point.x, msg.point.y

        # Stall tracker: is this a new goal? (distance-based — TARE may
        # re-publish the same point jittered by cm.) If yes, reset the
        # min-distance tracker.
        if (self._cur_goal is None
                or math.hypot(self._cur_goal[0] - wx,
                              self._cur_goal[1] - wy) > self._wp_change_eps):
            self._cur_goal = (wx, wy)
            self._goal_min_dist = float("inf")
            self._goal_last_improved_t = self._now()

        # Loud RViz marker regardless of whether we reset.
        self._publish_goal_marker(wx, wy)

        # --- Check 1: terrain_map point cluster near the waypoint.
        # Catches waypoints *on* or *next to* an observed wall surface —
        # where the LiDAR has painted terrain_map points.
        terrain_n_obs = None
        terrain = self._terrain_xy_h
        if terrain is not None and terrain.shape[0] > 0:
            dx = terrain[:, 0] - wx
            dy = terrain[:, 1] - wy
            near = dx * dx + dy * dy <= self._radius2
            obstacle = terrain[:, 2] >= self._h_thre
            terrain_n_obs = int(np.count_nonzero(near & obstacle))
            if terrain_n_obs > self._max_obs_seen:
                self._max_obs_seen = terrain_n_obs
        else:
            self._n_missing_terrain += 1

        # --- Check 2: 2D occupancy grid (octomap's projected_map).
        # Catches waypoints *inside* a wall's volume — terrain_map has no
        # points there because LiDAR doesn't see through walls, but
        # octomap's 2D projection marks the entire wall footprint as
        # occupied. Fires on ANY occupied cell within the inflate radius.
        occ_n = self._waypoint_in_occupied(wx, wy)

        if terrain_n_obs is not None and terrain_n_obs >= self._min_pts:
            self._publish_reset(
                f"terrain: {terrain_n_obs} obstacle pts within "
                f"{self._radius2 ** 0.5:.2f} m", wx, wy, "terrain")
        elif occ_n is not None and occ_n > 0:
            self._publish_reset(
                f"occgrid: {occ_n} occupied cells within "
                f"{self._occ_inflate:.2f} m", wx, wy, "occgrid")
        elif occ_n == -2:
            # Waypoint lies entirely outside the observed occgrid — TARE
            # is chasing a frontier in unobserved space that localPlanner
            # can't reach directly. Blacklist so TARE picks something
            # within the current observed region instead.
            self._publish_reset(
                "oob: waypoint beyond occgrid extent (unobserved)",
                wx, wy, "oob")
        # else: clean; stall tracker may still fire from _stall_tick().

    def _heartbeat(self) -> None:
        occ_state = "ready" if self._occgrid is not None else "waiting"
        pose_state = f"{self._pose}" if self._pose else "waiting"
        cur_goal = f"{self._cur_goal}" if self._cur_goal else "none"
        min_d = (f"{self._goal_min_dist:.2f}"
                 if math.isfinite(self._goal_min_dist) else "inf")
        since_improve = (self._now() - self._goal_last_improved_t
                         if self._cur_goal else 0.0)
        self.get_logger().info(
            f"checked={self._n_checked}, "
            f"resets[terrain={self._n_reset_terrain}, "
            f"occ={self._n_reset_occgrid}, oob={self._n_reset_oob}, "
            f"stall={self._n_reset_stall}], "
            f"nogo_regions={len(self._nogo_regions)}, "
            f"no_terrain_yet={self._n_missing_terrain}, "
            f"peak_obs_pts_seen={self._max_obs_seen} "
            f"(threshold={self._min_pts}), occgrid={occ_state}, "
            f"pose={pose_state}, goal={cur_goal}, "
            f"min_dist={min_d} m, since_improve={since_improve:.1f} s"
        )


def main() -> None:
    rclpy.init()
    node = TareWaypointWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
