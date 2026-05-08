#!/usr/bin/env python3
"""
Publish a synthetic /joy message to keep localPlanner and pathFollower
in autonomy mode.  Both nodes check joy->axes[2] <= -0.1 to enable
autonomyMode.  Without a physical joystick this never happens, so the
entire navigation stack sits idle.

This node waits for BOTH:
  1. startup_delay seconds (robot stand-up), AND
  2. at least one /way_point message (frontier goal ready)
before publishing /joy at 10 Hz with axes[2] = -1.0.
"""
import rclpy                          # ROS 2 Python client library
from rclpy.node import Node           # base Node class
from sensor_msgs.msg import Joy       # joystick message type
from geometry_msgs.msg import PointStamped  # waypoint message type
from std_msgs.msg import String       # supervisor_state (nominal | panic)


class AutonomyEnabler(Node):
    def __init__(self):
        super().__init__("autonomy_enabler")

        # --- parameters ------------------------------------------------
        self.declare_parameter("startup_delay", 10.0)   # seconds before first publish
        self.declare_parameter("rate", 10.0)             # Hz for /joy publishing
        self.declare_parameter("wait_for_waypoint", True)
        self.declare_parameter("supervisor_state_topic", "/robot/supervisor_state")
        self.startup_delay = float(self.get_parameter("startup_delay").value)
        self.rate = float(self.get_parameter("rate").value)
        self.wait_for_waypoint = bool(self.get_parameter("wait_for_waypoint").value)
        supervisor_state_topic = str(self.get_parameter("supervisor_state_topic").value)

        # --- state ------------------------------------------------------
        self.start_time = None                           # set on first timer callback
        self.enabled = False                             # set True after both conditions met
        self.goal_received = False                       # set True on first /way_point
        self.panic_active = False                        # True while supervisor_state == "panic"

        # --- subscriber (wait for frontier goal) -------------------------
        self.create_subscription(
            PointStamped, "/way_point", self.waypoint_cb, 10
        )
        self.create_subscription(String, supervisor_state_topic, self._state_cb, 10)

        # --- publisher --------------------------------------------------
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

        # --- timer ------------------------------------------------------
        self.timer = self.create_timer(1.0 / self.rate, self.publish_joy)

        self.get_logger().info(
            f"Autonomy enabler: will activate after {self.startup_delay:.1f}s "
            f"+ {'first /way_point' if self.wait_for_waypoint else 'startup delay only'}"
        )

    # ------------------------------------------------------------------
    def waypoint_cb(self, msg: PointStamped):
        """Record that at least one frontier goal has been published."""
        if not self.goal_received:
            self.goal_received = True
            self.get_logger().info(
                f"First /way_point received: ({msg.point.x:.2f}, {msg.point.y:.2f})"
            )

    def _state_cb(self, msg):
        """Supervisor panic latch — when active, disarm FAR via axes[2]=0.0."""
        self.panic_active = (str(msg.data).strip().lower() == "panic")

    # ------------------------------------------------------------------
    def publish_joy(self):
        # set start_time on first callback (sim clock is 0 at __init__)
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # wait for robot stand-up to finish
        if elapsed < self.startup_delay:
            return

        # Optional gate for pipelines that should not arm autonomy before goals are ready.
        if self.wait_for_waypoint and not self.goal_received:
            return

        if not self.enabled:
            self.enabled = True
            self.get_logger().info("Autonomy mode ENABLED via synthetic /joy")

        # build Joy message -----------------------------------------------
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # axes layout (matches typical gamepad):
        #   [0] left-stick-X   (manual yaw)
        #   [1] left-stick-Y
        #   [2] left-trigger   ** <= -0.1 → autonomy ON **
        #   [3] right-stick-X  (manual left)
        #   [4] right-stick-Y  (manual fwd)
        #   [5] right-trigger  ** <= -0.1 → manual ON — keep >-0.1 **
        #   [6] dpad-X
        #   [7] dpad-Y
        # During panic: zero the autonomy-enable axis so FAR / localPlanner
        # drop autonomyMode and stop commanding. joySpeed is also zeroed so
        # any residual path-follower output is scaled to zero.
        autonomy_axis = 0.0 if self.panic_active else -1.0
        forward_axis = 0.0 if self.panic_active else 1.0

        msg.axes = [
            0.0,              # 0  left-stick X
            0.0,              # 1  left-stick Y
            autonomy_axis,    # 2  left trigger → <= -0.1 enables autonomy; 0 disarms
            0.0,              # 3  right-stick X
            forward_axis,     # 4  right-stick Y → joySpeed
            0.0,              # 5  right trigger → NOT manual mode
            0.0,              # 6  dpad X
            0.0,              # 7  dpad Y
        ]
        msg.buttons = [0] * 11        # all-zero → panic node ignores this frame

        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyEnabler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
