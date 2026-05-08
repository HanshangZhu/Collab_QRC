#!/usr/bin/env python3
"""
Dual-robot CLI status monitor.

Subscribes to all relevant topics for both robot_a and robot_b
and prints one structured status line per robot at a configurable rate.

Usage (standalone):
    ros2 run go2_gazebo_sim robot_status_monitor.py

Usage (CLI one-shot check, pipe-friendly):
    ros2 topic echo /robot_a/nav_status --once
    ros2 topic echo /robot_b/nav_status --once

Output format (one line per robot per second):
    [robot_a] t=42.3 pos=(1.20,-0.30,45.0°) v=0.30 goal=(3.20,-1.50) d=2.10 cmd=(0.30,0.12) mode=navigate steer=planner plan=4wp mf=1.20 blk=0.0 stop=0
    [robot_b] t=42.3 pos=(15.10,1.20,179.0°) v=0.00 goal=None d=- cmd=(0.00,0.00) mode=no_goal plan=0wp mf=- blk=- stop=0
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8, String
from visualization_msgs.msg import MarkerArray


class RobotInfo:
    """Holds latest state for one robot."""

    def __init__(self, ns: str):
        self.ns = ns
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.speed: Optional[float] = None

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None

        self.stop: int = 0

        self.nav_diag: dict = {}

        self.frontier_cluster_count: int = 0
        self.map_free: int = 0
        self.map_occ: int = 0
        self.map_total: int = 0
        self.path_total_m: float = 0.0
        self._last_path_x: Optional[float] = None
        self._last_path_y: Optional[float] = None


class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__("robot_status_monitor")

        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("report_rate", 0.1)
        self.declare_parameter("json_output", False)
        self.declare_parameter("meeting_debug_distance_m", 1.0)
        self.declare_parameter("meeting_debug_speed_threshold", 0.03)
        self.declare_parameter("meeting_debug_warn_period_sec", 5.0)

        namespaces = self.get_parameter("namespaces").value
        report_rate = float(self.get_parameter("report_rate").value)
        self.json_output = bool(self.get_parameter("json_output").value)
        self.meeting_debug_distance_m = max(0.1, float(self.get_parameter("meeting_debug_distance_m").value))
        self.meeting_debug_speed_threshold = max(
            0.0, float(self.get_parameter("meeting_debug_speed_threshold").value)
        )
        self.meeting_debug_warn_period_sec = max(
            0.5, float(self.get_parameter("meeting_debug_warn_period_sec").value)
        )
        self._last_meeting_warn_sec = 0.0

        self.robots: dict[str, RobotInfo] = {}

        for ns in namespaces:
            info = RobotInfo(ns)
            self.robots[ns] = info

            # Odometry
            self.create_subscription(
                Odometry,
                f"/{ns}/odom/ground_truth",
                lambda msg, n=ns: self._odom_cb(msg, n),
                10,
            )

            # Goal (way_point)
            self.create_subscription(
                PointStamped,
                f"/{ns}/way_point",
                lambda msg, n=ns: self._goal_cb(msg, n),
                10,
            )

            # External stop
            self.create_subscription(
                Int8,
                f"/{ns}/stop",
                lambda msg, n=ns: self._stop_cb(msg, n),
                10,
            )

            # Nav diagnostics (JSON from default_nav)
            self.create_subscription(
                String,
                f"/{ns}/nav_status",
                lambda msg, n=ns: self._nav_status_cb(msg, n),
                10,
            )

            # Frontier markers (cluster count)
            self.create_subscription(
                MarkerArray,
                f"/{ns}/frontier_markers",
                lambda msg, n=ns: self._frontier_cb(msg, n),
                10,
            )

            # Occupancy grid (coverage %)
            self.create_subscription(
                OccupancyGrid,
                f"/{ns}/map",
                lambda msg, n=ns: self._map_cb(msg, n),
                1,
            )

        self.report_timer = self.create_timer(1.0 / report_rate, self._report)
        self.get_logger().info(
            f"Status monitor started for {namespaces} at {report_rate}Hz"
        )

    # --- callbacks ---

    def _odom_cb(self, msg: Odometry, ns: str):
        r = self.robots[ns]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if r._last_path_x is not None and r._last_path_y is not None:
            r.path_total_m += math.hypot(x - r._last_path_x, y - r._last_path_y)
        r._last_path_x = x
        r._last_path_y = y
        r.x = x
        r.y = y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r.yaw = math.degrees(math.atan2(siny, cosy))
        r.speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

    def _goal_cb(self, msg: PointStamped, ns: str):
        r = self.robots[ns]
        r.goal_x = msg.point.x
        r.goal_y = msg.point.y

    def _stop_cb(self, msg: Int8, ns: str):
        self.robots[ns].stop = int(msg.data)

    def _nav_status_cb(self, msg: String, ns: str):
        try:
            self.robots[ns].nav_diag = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _frontier_cb(self, msg: MarkerArray, ns: str):
        self.robots[ns].frontier_cluster_count = len(msg.markers)

    def _map_cb(self, msg: OccupancyGrid, ns: str):
        r = self.robots[ns]
        r.map_total = len(msg.data)
        r.map_free = sum(1 for c in msg.data if c == 0)
        r.map_occ = sum(1 for c in msg.data if c == 100)

    # --- reporting ---

    def _report(self):
        for ns, r in self.robots.items():
            if self.json_output:
                self._report_json(ns, r)
            else:
                self._report_line(ns, r)
        self._report_pairwise_meeting_debug()

    def _report_line(self, ns: str, r: RobotInfo):
        d = r.nav_diag

        # Position
        if r.x is not None:
            pos = f"({r.x:.2f},{r.y:.2f},{r.yaw:.0f}°)"
        else:
            pos = "(?)"

        vel = f"{r.speed:.2f}" if r.speed is not None else "-"

        # Goal
        if r.goal_x is not None:
            goal = f"({r.goal_x:.2f},{r.goal_y:.2f})"
        else:
            goal = "None"

        dist = f"{d.get('dist_goal', '-')}"
        mode = d.get("mode", "?")
        steer = d.get("steer", "-")
        cmd = d.get("cmd", [0, 0])
        cmd_str = f"({cmd[0]:.2f},{cmd[1]:.2f})"

        mf = d.get("min_front", "-")
        if isinstance(mf, float) and mf > 50:
            mf = "inf"
        blk = d.get("blocked_sec", "-")
        plan_wps = d.get("plan_wps", 0)
        escape = d.get("escape")
        esc_str = ""
        if escape:
            esc_str = f" esc=({escape[0]:.1f},{escape[1]:.1f})"

        # Map coverage
        explored = r.map_free + r.map_occ
        if r.map_total > 0:
            known_pct = int(100 * explored / r.map_total)
        else:
            known_pct = 0

        line = (
            f"[{ns}] pos={pos} v={vel} goal={goal} d={dist} "
            f"cmd={cmd_str} mode={mode} steer={steer} "
            f"plan={plan_wps}wp mf={mf} blk={blk} stop={r.stop} "
            f"fronts={r.frontier_cluster_count} map={known_pct}% "
            f"disp={r.path_total_m:.2f}m"
            f"{esc_str}"
        )
        self.get_logger().info(line)

    def _report_pairwise_meeting_debug(self):
        items = list(self.robots.items())
        if len(items) < 2:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (now_sec - self._last_meeting_warn_sec) < self.meeting_debug_warn_period_sec:
            return

        for i in range(len(items)):
            ns_a, a = items[i]
            if a.x is None or a.y is None or a.speed is None:
                continue
            for j in range(i + 1, len(items)):
                ns_b, b = items[j]
                if b.x is None or b.y is None or b.speed is None:
                    continue

                pair_dist = math.hypot(a.x - b.x, a.y - b.y)
                if pair_dist > self.meeting_debug_distance_m:
                    continue

                a_cmd = a.nav_diag.get("cmd", [0.0, 0.0])
                b_cmd = b.nav_diag.get("cmd", [0.0, 0.0])
                a_cmd_small = abs(float(a_cmd[0])) < 0.02 and abs(float(a_cmd[1])) < 0.02
                b_cmd_small = abs(float(b_cmd[0])) < 0.02 and abs(float(b_cmd[1])) < 0.02
                a_stopped = abs(float(a.speed)) <= self.meeting_debug_speed_threshold
                b_stopped = abs(float(b.speed)) <= self.meeting_debug_speed_threshold
                if not (a_stopped and b_stopped and a_cmd_small and b_cmd_small):
                    continue

                self._last_meeting_warn_sec = now_sec
                self.get_logger().warn(
                    "MEETING_STALL suspect: "
                    f"{ns_a}<->{ns_b} pair_dist={pair_dist:.2f}m "
                    f"{ns_a}(mode={a.nav_diag.get('mode', '?')}, d={a.nav_diag.get('dist_goal', '-')}, "
                    f"zero={a.nav_diag.get('zero_reason', '-')}, goal_age={a.nav_diag.get('goal_age_sec', '-')}) "
                    f"{ns_b}(mode={b.nav_diag.get('mode', '?')}, d={b.nav_diag.get('dist_goal', '-')}, "
                    f"zero={b.nav_diag.get('zero_reason', '-')}, goal_age={b.nav_diag.get('goal_age_sec', '-')})"
                )
                return

    def _report_json(self, ns: str, r: RobotInfo):
        d = r.nav_diag.copy()
        d["ns"] = ns
        if r.x is not None:
            d["odom_pos"] = [round(r.x, 2), round(r.y, 2), round(r.yaw, 1)]
            d["odom_speed"] = round(r.speed, 3) if r.speed else 0.0
        if r.goal_x is not None:
            d["frontier_goal"] = [round(r.goal_x, 2), round(r.goal_y, 2)]
        d["frontier_clusters"] = r.frontier_cluster_count
        explored = r.map_free + r.map_occ
        d["map_cells"] = {"free": r.map_free, "occ": r.map_occ, "total": r.map_total}
        d["map_explored_pct"] = round(100 * explored / r.map_total, 1) if r.map_total > 0 else 0.0
        d["ext_stop"] = r.stop
        self.get_logger().info(json.dumps(d, separators=(",", ":")))


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
