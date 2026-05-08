#!/usr/bin/env python3
"""Unified exploration experiment logger + event stream + stop trigger.

Three responsibilities, single node:

1. **CSV time-series** of standard multi-robot exploration metrics:
   coverage %, trajectory length, velocity, goals, collision stops, frontier
   clusters, exploration efficiency, multi-robot overlap.

2. **Structured event log** to stdout + ``$ROS_LOG_SESSION_DIR/exploration_events.log``.
   One line per significant event with absolute timestamp + delta from the
   previous event. Sources:
     - ``/<ns>/exploration_status``     (CFPA2 state — searching/executing/no_reachable/no_frontiers/paused)
     - ``/<ns>/goal_pose``              (goal entering Nav2)
     - ``/<ns>/behavior_tree_log``      (planner / controller / recovery timing)
     - ``/<ns>/recovery_event``         (stuck_watchdog detected/backup events)
     - ``/cmd_vel``                     (time-to-first-motion after a goal)
   Plus a 30 s rolling summary line: plan p50/p95/max, plan success rate,
   goals completed / aborted, time-to-first-motion avg, coverage Δ.

3. **Exploration stop trigger** — publishes ``/<ns>/exploration_complete``
   (``std_msgs/String`` with reason) and stops the robot when:
     - ``exploration_status == "no_reachable"`` for ``consec_no_reachable_threshold`` ticks (primary), OR
     - coverage Δ% < ``coverage_stagnant_threshold_pct`` over ``coverage_stagnant_window_sec`` (safety net).
   Stop = cancel ``/<ns>/navigate_to_pose`` action + publish one zero ``/cmd_vel``.

Output: {output_dir}/exploration_{experiment_name}_{timestamp}.csv

Parameters:
    namespaces                        (list[str])  Robot namespaces          ["robot"]
    experiment_name                   (str)        Tag for output filename   "run"
    log_rate                          (float)      CSV write rate (Hz)       1.0
    output_dir                        (str)        Output directory          "/tmp"
    event_log_path                    (str)        Event-line file (empty → derive from ROS_LOG_SESSION_DIR or /tmp)
    consec_no_reachable_threshold     (int)        Ticks of no_reachable before stop  3
    coverage_stagnant_threshold_pct   (float)      Min Δ% over window before stop     0.5
    coverage_stagnant_window_sec      (float)      Coverage stagnation window         30.0
    summary_interval_sec              (float)      Rolling-summary cadence            30.0
    enable_stop_trigger               (bool)       Publish exploration_complete + cancel goal  true
"""

from __future__ import annotations

import bisect
import math
import os
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8, String
from visualization_msgs.msg import MarkerArray

try:
    from nav2_msgs.msg import BehaviorTreeLog  # noqa: F401
    _HAS_BT_LOG = True
except ImportError:
    _HAS_BT_LOG = False


class _RobotState:
    """Accumulates metrics for one robot."""

    def __init__(self, ns: str) -> None:
        self.ns = ns
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.speed: float = 0.0

        self.trajectory_m: float = 0.0
        self._prev_x: Optional[float] = None
        self._prev_y: Optional[float] = None

        self.map_free: int = 0
        self.map_occ: int = 0
        self.map_total: int = 0
        self.map_resolution: float = 0.05
        self.explored_cells: set[tuple[int, int]] = set()

        self.goals_received: int = 0
        self.goals_reached: int = 0

        self.collision_stops: int = 0
        self._prev_stop: int = 0

        self.frontier_clusters: int = 0
        self.nav_mode: str = "?"

        # Exploration state from CFPA2 — drives the stop-trigger logic.
        self.exploration_status: str = "?"
        self.consec_no_reachable: int = 0   # primary stop counter
        self.stop_triggered: bool = False

        # Coverage-stagnation ringbuffer: (t_wall, known_pct).
        self.coverage_window: deque = deque()

        # Goal/plan/motion tracking for event log + 30 s summary.
        self.last_goal_xy: Optional[tuple[float, float]] = None
        self.last_goal_t: float = 0.0
        self.awaiting_first_motion: bool = False
        self.first_motion_deltas: deque = deque(maxlen=64)  # last N
        # plan_start_bt_t: BT-log internal timestamp at IDLE→RUNNING.
        # plan_in_progress: dedupes double IDLE→RUNNING from BT XMLs that
        #   contain multiple ComputePathToPose nodes (main + recovery).
        self.plan_start_bt_t: Optional[float] = None
        self.plan_in_progress: bool = False
        self.plan_times_ms: deque = deque(maxlen=128)       # last N succeeded plans
        self.plan_outcomes: deque = deque(maxlen=128)       # ("succeeded"|"aborted"|"failed", t_wall)
        self.goals_completed: int = 0
        self.goals_aborted_planner: int = 0
        self.goals_aborted_controller: int = 0


class ExplorationMetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("exploration_metrics_logger")

        # Parameters
        self.declare_parameter("namespaces", ["robot"])
        self.declare_parameter("experiment_name", "run")
        self.declare_parameter("log_rate", 1.0)
        self.declare_parameter("output_dir", "/tmp")
        # Stop-trigger params (see module docstring for semantics).
        self.declare_parameter("event_log_path", "")
        self.declare_parameter("consec_no_reachable_threshold", 3)
        self.declare_parameter("coverage_stagnant_threshold_pct", 0.5)
        self.declare_parameter("coverage_stagnant_window_sec", 30.0)
        self.declare_parameter("summary_interval_sec", 30.0)
        self.declare_parameter("enable_stop_trigger", True)

        namespaces = self.get_parameter("namespaces").value
        experiment_name = str(self.get_parameter("experiment_name").value)
        log_rate = float(self.get_parameter("log_rate").value)
        output_dir = str(self.get_parameter("output_dir").value)

        self._consec_threshold = int(
            self.get_parameter("consec_no_reachable_threshold").value)
        self._cov_thr_pct = float(
            self.get_parameter("coverage_stagnant_threshold_pct").value)
        self._cov_window_sec = float(
            self.get_parameter("coverage_stagnant_window_sec").value)
        self._summary_interval_sec = float(
            self.get_parameter("summary_interval_sec").value)
        self._enable_stop_trigger = bool(
            self.get_parameter("enable_stop_trigger").value)

        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(output_dir, f"exploration_{experiment_name}_{ts}.csv")

        # Event log file path. Empty param → derive from ROS_LOG_SESSION_DIR
        # if set (real-robot launches set this), else next to the CSV.
        event_log_param = str(self.get_parameter("event_log_path").value).strip()
        if event_log_param:
            self._event_log_path = event_log_param
        else:
            session_dir = os.environ.get("ROS_LOG_SESSION_DIR", "").strip()
            base = session_dir if session_dir else output_dir
            os.makedirs(base, exist_ok=True)
            self._event_log_path = os.path.join(
                base, f"exploration_events_{experiment_name}_{ts}.log")
        # Open in line-buffered mode so `tail -f` works during a run.
        self._event_log_f = open(self._event_log_path, "a", buffering=1)

        # Event-stream wall-clock baseline: tracks the previous event's
        # wall time so each new line carries +ΔΔΔms since last event.
        self._prev_event_wall: Optional[float] = None

        self.robots: dict[str, _RobotState] = {}
        self.start_time: Optional[float] = None

        # One stop publisher + cancel-service client per namespace; one
        # Twist publisher to /cmd_vel for the global zero-cmd "stop pulse".
        self._stop_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._cancel_clients: dict[str, "rclpy.client.Client"] = {}
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        for ns in namespaces:
            rs = _RobotState(ns)
            self.robots[ns] = rs

            # ── Existing subscribers (CSV-feeding) ──
            self.create_subscription(
                Odometry, f"/{ns}/odom/ground_truth",
                lambda msg, n=ns: self._odom_cb(msg, n), 10,
            )
            self.create_subscription(
                OccupancyGrid, f"/{ns}/map",
                lambda msg, n=ns: self._map_cb(msg, n), 1,
            )
            self.create_subscription(
                String, f"/{ns}/nav_status",
                lambda msg, n=ns: self._nav_status_cb(msg, n), 10,
            )
            self.create_subscription(
                Int8, f"/{ns}/stop",
                lambda msg, n=ns: self._stop_cb(msg, n), 10,
            )
            self.create_subscription(
                MarkerArray, f"/{ns}/frontier_markers",
                lambda msg, n=ns: self._frontier_cb(msg, n), 10,
            )

            # ── New subscribers (event-stream + stop-trigger feeders) ──
            self.create_subscription(
                String, f"/{ns}/exploration_status",
                lambda msg, n=ns: self._exploration_status_cb(msg, n), 10,
            )
            self.create_subscription(
                PoseStamped, f"/{ns}/goal_pose",
                lambda msg, n=ns: self._goal_pose_cb(msg, n), 10,
            )
            self.create_subscription(
                String, f"/{ns}/recovery_event",
                lambda msg, n=ns: self._recovery_event_cb(msg, n), 10,
            )
            if _HAS_BT_LOG:
                self.create_subscription(
                    BehaviorTreeLog, f"/{ns}/behavior_tree_log",
                    lambda msg, n=ns: self._bt_log_cb(msg, n), 10,
                )

            # exploration_complete publisher (latched: subscribers that
            # arrive late — e.g. CFPA2 restart — still see the stop reason).
            stop_qos = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._stop_pubs[ns] = self.create_publisher(
                String, f"/{ns}/exploration_complete", stop_qos)

            # Service client to the action's CancelGoal service. Cleaner than
            # holding goal handles since we never own the goal — the bridge
            # / RViz / CFPA2 do. CancelGoal with empty goal_info cancels all
            # active goals for that action.
            self._cancel_clients[ns] = self.create_client(
                CancelGoal, f"/{ns}/navigate_to_pose/_action/cancel_goal")

        # cmd_vel monitor — used to detect "first motion after goal" deltas.
        self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, qos_profile_sensor_data,
        )

        # Build CSV header
        header_parts = ["t_wall", "t_sim"]
        for ns in namespaces:
            p = ns  # column prefix
            header_parts.extend([
                f"{p}_x", f"{p}_y", f"{p}_yaw_deg",
                f"{p}_velocity_mps", f"{p}_avg_velocity_mps",
                f"{p}_trajectory_m",
                f"{p}_coverage_pct", f"{p}_coverage_area_m2",
                f"{p}_goals_received", f"{p}_goals_reached",
                f"{p}_collision_stops",
                f"{p}_frontier_clusters",
                f"{p}_efficiency_m2_per_m",
                f"{p}_nav_mode",
            ])
        # Multi-robot overlap (only if > 1 robot)
        if len(namespaces) > 1:
            header_parts.append("overlap_pct")

        with open(self.csv_path, "w") as f:
            f.write(",".join(header_parts) + "\n")

        self.create_timer(1.0 / max(0.1, log_rate), self._log_row)
        # Stop-trigger evaluator @ 2 Hz — picks up exploration_status
        # repeats and the coverage ringbuffer.
        self.create_timer(0.5, self._stop_trigger_tick)
        # 30 s rolling summary (or whatever summary_interval_sec is set to).
        self.create_timer(self._summary_interval_sec, self._summary_tick)

        self.get_logger().info(f"Exploration logger started → {self.csv_path}")
        self.get_logger().info(f"  namespaces={namespaces}  rate={log_rate}Hz")
        self.get_logger().info(f"  event log → {self._event_log_path}")
        if not _HAS_BT_LOG:
            self.get_logger().warn(
                "nav2_msgs.msg.BehaviorTreeLog unavailable — plan-time metrics "
                "will not be captured. Install/source nav2_msgs to enable.")

    # ── Callbacks ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        rs = self.robots[ns]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if rs._prev_x is not None:
            rs.trajectory_m += math.hypot(x - rs._prev_x, y - rs._prev_y)
        rs._prev_x = x
        rs._prev_y = y

        rs.x = x
        rs.y = y
        q = msg.pose.pose.orientation
        rs.yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ))
        rs.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        rs = self.robots[ns]
        rs.map_total = len(msg.data)
        rs.map_resolution = msg.info.resolution
        rs.map_free = 0
        rs.map_occ = 0
        w = msg.info.width
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution

        explored = set()
        for i, v in enumerate(msg.data):
            if v >= 0:  # known cell (free=0 or occupied=100)
                rs.map_free += 1 if v == 0 else 0
                rs.map_occ += 1 if v == 100 else 0
                gx = i % w
                gy = i // w
                # Store world-discretised cell for overlap computation
                wx = int((ox + (gx + 0.5) * res) / res)
                wy = int((oy + (gy + 0.5) * res) / res)
                explored.add((wx, wy))
        rs.explored_cells = explored

    def _nav_status_cb(self, msg: String, ns: str) -> None:
        import json
        rs = self.robots[ns]
        try:
            d = json.loads(msg.data)
            new_mode = d.get("mode", "?")
            # Detect goal-reached transitions
            if new_mode == "reached" and rs.nav_mode != "reached":
                rs.goals_reached += 1
            # Detect new goal assignments
            if new_mode in ("navigate", "steer") and rs.nav_mode in ("no_goal", "reached", "?"):
                rs.goals_received += 1
            rs.nav_mode = new_mode
        except Exception:
            pass

    def _stop_cb(self, msg: Int8, ns: str) -> None:
        rs = self.robots[ns]
        val = int(msg.data)
        if val == 1 and rs._prev_stop == 0:
            rs.collision_stops += 1
        rs._prev_stop = val

    def _frontier_cb(self, msg: MarkerArray, ns: str) -> None:
        self.robots[ns].frontier_clusters = len(msg.markers)

    # ── Event stream + stop trigger ───────────────────────────────────

    def _emit_event(self, kind: str, payload: str = "") -> None:
        """Write one structured line to stdout + the event log file.

        Format: ``[HH:MM:SS.mmm +ΔΔΔms] KIND: payload``. The Δ is wall-clock
        time since the previous event (anywhere in the system) — this is the
        gold for spotting where time is going (e.g. plan_request → plan_returned).
        """
        now = time.time()
        if self._prev_event_wall is None:
            delta_ms_str = "+----ms"
        else:
            delta_ms = int(round((now - self._prev_event_wall) * 1000.0))
            delta_ms_str = f"+{delta_ms:>4d}ms"
        self._prev_event_wall = now

        ts_local = time.strftime("%H:%M:%S", time.localtime(now))
        ms = int((now - int(now)) * 1000)
        line = f"[{ts_local}.{ms:03d} {delta_ms_str}] {kind}: {payload}"
        # Stdout (so it ends up in console.log) + event-log file.
        print(line, flush=True)
        try:
            self._event_log_f.write(line + "\n")
        except Exception:
            pass

    def _exploration_status_cb(self, msg: String, ns: str) -> None:
        rs = self.robots[ns]
        new_status = msg.data.strip()
        if new_status == rs.exploration_status:
            # Track consecutive no_reachable ticks even when status repeats —
            # status_pub deduplicates, so this branch only fires on "tick
            # without state change", harmless.
            return
        prev = rs.exploration_status
        rs.exploration_status = new_status

        if new_status == "no_reachable":
            rs.consec_no_reachable += 1
        else:
            rs.consec_no_reachable = 0

        self._emit_event(
            "EXPLORATION_STATUS",
            f"ns={ns} {prev} → {new_status} (consec_no_reachable={rs.consec_no_reachable})")

    def _goal_pose_cb(self, msg: PoseStamped, ns: str) -> None:
        rs = self.robots[ns]
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        rs.last_goal_xy = (gx, gy)
        rs.last_goal_t = time.time()
        rs.awaiting_first_motion = True
        self._emit_event(
            "GOAL_SENT",
            f"ns={ns} target=({gx:+.2f}, {gy:+.2f}) frame={msg.header.frame_id}")

    def _cmd_vel_cb(self, msg: Twist) -> None:
        # First non-zero cmd_vel after a goal → "time to first motion".
        # This subscription is global (no namespace), so we attribute the
        # first-motion event to whichever robot most recently received a
        # goal AND is still awaiting first motion.
        if abs(msg.linear.x) < 1e-3 and abs(msg.linear.y) < 1e-3 and abs(msg.angular.z) < 1e-3:
            return
        for ns, rs in self.robots.items():
            if not rs.awaiting_first_motion or rs.last_goal_t <= 0:
                continue
            delta = time.time() - rs.last_goal_t
            rs.first_motion_deltas.append(delta)
            rs.awaiting_first_motion = False
            self._emit_event(
                "FIRST_CMD_VEL",
                f"ns={ns} t_to_motion={delta*1000:.0f}ms "
                f"lin=({msg.linear.x:+.2f}, {msg.linear.y:+.2f}) "
                f"ang={msg.angular.z:+.2f}")
            return

    def _recovery_event_cb(self, msg: String, ns: str) -> None:
        self._emit_event("RECOVERY", f"ns={ns} {msg.data}")

    def _bt_log_cb(self, msg, ns: str) -> None:
        """Capture plan timing from BehaviorTreeLog status transitions.

        Notes on the BT used by Nav2 here
        ---------------------------------
        ``nav_to_pose_with_consistent_replanning_*`` BT XMLs contain TWO
        ``ComputePathToPose`` nodes (main + recovery branch). They fire
        IDLE→RUNNING transitions in the same BT log message, so we dedupe
        with ``plan_in_progress``. We also use the BT log's per-event
        ``timestamp`` (Nav2-internal, not wall-clock) for the elapsed
        calc — when a plan completes within one BT publishing interval,
        ``time.time()`` between the two events is ~0 ms but the BT
        timestamp delta still reflects real planner duration.
        """
        rs = self.robots[ns]
        for ev in msg.event_log:
            node_name = ev.node_name
            prev = ev.previous_status
            curr = ev.current_status
            ev_t = ev.timestamp.sec + ev.timestamp.nanosec * 1e-9

            # Plan timing — collapse main+recovery duplicates via plan_in_progress.
            if node_name in ("ComputePathToPose", "ComputePathThroughPoses"):
                if prev == "IDLE" and curr == "RUNNING":
                    if not rs.plan_in_progress:
                        rs.plan_in_progress = True
                        rs.plan_start_bt_t = ev_t
                        self._emit_event(
                            "PLAN_REQUEST",
                            f"ns={ns} planner={node_name}")
                    # Else: already counting an earlier IDLE→RUNNING this cycle.
                elif prev == "RUNNING" and curr in ("SUCCESS", "FAILURE"):
                    if rs.plan_in_progress and rs.plan_start_bt_t is not None:
                        elapsed_ms = max(0.0, (ev_t - rs.plan_start_bt_t) * 1000.0)
                        if curr == "SUCCESS":
                            rs.plan_times_ms.append(elapsed_ms)
                            rs.plan_outcomes.append(("succeeded", time.time()))
                            self._emit_event(
                                "PLAN_RETURNED",
                                f"ns={ns} planner={node_name} t={elapsed_ms:.0f}ms ✓")
                        else:
                            rs.plan_outcomes.append(("failed", time.time()))
                            self._emit_event(
                                "PLAN_FAILED",
                                f"ns={ns} planner={node_name} t={elapsed_ms:.0f}ms ✗")
                            rs.goals_aborted_planner += 1
                        rs.plan_in_progress = False
                        rs.plan_start_bt_t = None

            # Controller (FollowPath) — track aborts for the failure breakdown.
            elif node_name == "FollowPath":
                if prev == "RUNNING" and curr == "FAILURE":
                    rs.goals_aborted_controller += 1
                    self._emit_event(
                        "CONTROLLER_FAILED",
                        f"ns={ns} (FollowPath FAILURE)")

            # Final goal outcome — most BT XMLs put NavigateToPose's terminal
            # status on the root or "navigate_to_pose" node. Capture both
            # "SUCCESS" and "FAILURE" terminal transitions on common BT roots.
            elif node_name in ("NavigateToPose", "navigate_to_pose", "NavigateRecovery"):
                if prev == "RUNNING" and curr == "SUCCESS":
                    rs.goals_completed += 1
                    self._emit_event("GOAL_COMPLETED", f"ns={ns}")
                elif prev == "RUNNING" and curr == "FAILURE":
                    self._emit_event("GOAL_ABORTED", f"ns={ns}")

    # ── Stop trigger ──────────────────────────────────────────────────

    def _stop_trigger_tick(self) -> None:
        """Evaluate stop conditions every 0.5 s.

        Two independent checks per robot:
          1. consec_no_reachable >= threshold → reason="no_reachable".
          2. coverage Δ% over window < threshold → reason="coverage_stagnant".
        Either trigger fires once per robot (stop_triggered latches).
        """
        if not self._enable_stop_trigger:
            return
        now = time.time()
        for ns, rs in self.robots.items():
            if rs.stop_triggered:
                continue

            # Update coverage ringbuffer.
            if rs.map_total > 0:
                known_pct = 100.0 * (rs.map_free + rs.map_occ) / rs.map_total
                rs.coverage_window.append((now, known_pct))
                cutoff = now - self._cov_window_sec
                while rs.coverage_window and rs.coverage_window[0][0] < cutoff:
                    rs.coverage_window.popleft()

            # Primary: no_reachable for N consecutive ticks.
            if rs.consec_no_reachable >= self._consec_threshold:
                self._trigger_stop(ns, "no_reachable")
                continue

            # Safety net: coverage stagnation. Need full window of data.
            if len(rs.coverage_window) >= 2:
                t_first, pct_first = rs.coverage_window[0]
                t_last, pct_last = rs.coverage_window[-1]
                if (t_last - t_first) >= self._cov_window_sec * 0.95:
                    delta_pct = pct_last - pct_first
                    if delta_pct < self._cov_thr_pct:
                        self._trigger_stop(
                            ns,
                            f"coverage_stagnant (Δ={delta_pct:.2f}% over {t_last - t_first:.0f}s)")
                        continue

    def _trigger_stop(self, ns: str, reason: str) -> None:
        """Publish exploration_complete + cancel Nav2 goal + zero cmd_vel."""
        rs = self.robots[ns]
        rs.stop_triggered = True

        self._emit_event("EXPLORATION_COMPLETE", f"ns={ns} reason={reason}")
        self.get_logger().warn(
            f"[exploration_complete] ns={ns} reason={reason} — "
            f"canceling Nav2 goal and stopping cmd_vel.")

        # Latched publish so late subscribers (CFPA2 reconnects, debugger)
        # also get the reason.
        msg = String()
        msg.data = reason
        self._stop_pubs[ns].publish(msg)

        # Cancel any in-flight NavigateToPose goal so Nav2 stops issuing
        # cmd_vel. CancelGoal with default (empty) goal_info cancels all
        # active goals for the action.
        try:
            client = self._cancel_clients[ns]
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.5):
                client.call_async(CancelGoal.Request())
            else:
                self.get_logger().warn(
                    f"[exploration_complete] ns={ns} cancel service not "
                    f"available; relying on zero-cmd_vel only.")
        except Exception as exc:
            self.get_logger().warn(
                f"[exploration_complete] ns={ns} cancel failed: {exc}")

        # Zero-cmd_vel pulse — bridge passes through to sport API and the
        # robot decelerates. Single shot is enough; supervisor / mux keeps
        # downstream nodes fed.
        zero = Twist()
        self._cmd_vel_pub.publish(zero)

    # ── 30 s summary ──────────────────────────────────────────────────

    @staticmethod
    def _percentile(sorted_vals: list[float], p: float) -> float:
        if not sorted_vals:
            return 0.0
        k = (len(sorted_vals) - 1) * p
        lo = int(math.floor(k))
        hi = int(math.ceil(k))
        if lo == hi:
            return sorted_vals[lo]
        return sorted_vals[lo] + (k - lo) * (sorted_vals[hi] - sorted_vals[lo])

    def _summary_tick(self) -> None:
        now = time.time()
        window = self._summary_interval_sec
        for ns, rs in self.robots.items():
            # Plan timing percentiles (over recent plans)
            plan_recent = [t for t in rs.plan_times_ms]
            plan_recent_sorted = sorted(plan_recent)
            p50 = self._percentile(plan_recent_sorted, 0.50)
            p95 = self._percentile(plan_recent_sorted, 0.95)
            pmax = max(plan_recent_sorted) if plan_recent_sorted else 0.0

            # Plan success rate over the window.
            success = sum(1 for o, t in rs.plan_outcomes if o == "succeeded" and (now - t) <= window)
            failed  = sum(1 for o, t in rs.plan_outcomes if o == "failed"    and (now - t) <= window)
            total   = success + failed
            rate_pct = (100.0 * success / total) if total > 0 else 0.0

            # First-motion average (from collected deltas).
            ttm_avg = (sum(rs.first_motion_deltas) / len(rs.first_motion_deltas)
                       * 1000.0) if rs.first_motion_deltas else 0.0

            # Coverage Δ over window.
            cov_first = rs.coverage_window[0][1] if rs.coverage_window else 0.0
            cov_last  = rs.coverage_window[-1][1] if rs.coverage_window else 0.0
            cov_delta = cov_last - cov_first

            self._emit_event(
                "SUMMARY",
                f"ns={ns} "
                f"plan_ms[p50/p95/max]={p50:.0f}/{p95:.0f}/{pmax:.0f} "
                f"plan_ok={success}/{total} ({rate_pct:.0f}%) "
                f"goals[done/abort_p/abort_c]={rs.goals_completed}/"
                f"{rs.goals_aborted_planner}/{rs.goals_aborted_controller} "
                f"ttm_avg={ttm_avg:.0f}ms "
                f"coverage={cov_last:.1f}% (Δ={cov_delta:+.2f}%/{int(window)}s) "
                f"status={rs.exploration_status}")

    # ── CSV logging ───────────────────────────────────────────────────

    def _log_row(self) -> None:
        now_wall = time.time()
        if self.start_time is None:
            self.start_time = now_wall
        t_wall = now_wall - self.start_time
        t_sim = self.get_clock().now().nanoseconds / 1e9

        parts: list[str] = [f"{t_wall:.2f}", f"{t_sim:.2f}"]

        for ns, rs in self.robots.items():
            explored = rs.map_free + rs.map_occ
            coverage_pct = (100.0 * explored / rs.map_total) if rs.map_total > 0 else 0.0
            coverage_area = explored * (rs.map_resolution ** 2)
            avg_vel = (rs.trajectory_m / t_wall) if t_wall > 1.0 else 0.0
            efficiency = (coverage_area / rs.trajectory_m) if rs.trajectory_m > 0.5 else 0.0

            parts.extend([
                f"{rs.x:.3f}" if rs.x is not None else "",
                f"{rs.y:.3f}" if rs.y is not None else "",
                f"{rs.yaw:.1f}" if rs.yaw is not None else "",
                f"{rs.speed:.3f}",
                f"{avg_vel:.3f}",
                f"{rs.trajectory_m:.3f}",
                f"{coverage_pct:.2f}",
                f"{coverage_area:.3f}",
                str(rs.goals_received),
                str(rs.goals_reached),
                str(rs.collision_stops),
                str(rs.frontier_clusters),
                f"{efficiency:.4f}",
                rs.nav_mode,
            ])

        # Multi-robot overlap
        if len(self.robots) > 1:
            all_sets = [rs.explored_cells for rs in self.robots.values() if rs.explored_cells]
            if len(all_sets) >= 2:
                union = set.union(*all_sets)
                intersection = set.intersection(*all_sets)
                overlap_pct = (100.0 * len(intersection) / len(union)) if union else 0.0
                parts.append(f"{overlap_pct:.2f}")
            else:
                parts.append("0.00")

        with open(self.csv_path, "a") as f:
            f.write(",".join(parts) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Final CSV at: {node.csv_path}")
        node.get_logger().info(f"Event log at: {node._event_log_path}")
        try:
            node._event_log_f.close()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
