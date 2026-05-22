#!/usr/bin/env python3
"""ROS 1 /robot/cmd_vel -> Unitree SDK2 SportClient bridge for onboard Go2 runs.

The Noetic onboard stack produces geometry_msgs/Twist on /<ns>/cmd_vel. The
real Go2 does not consume that ROS 1 topic directly, so this node is the final
actuation bridge: it subscribes to cmd_vel, clamps it to the configured safety
limits, and sends SportClient.Move(vx, vy, wz) through Unitree SDK2.
"""

from __future__ import annotations

import math
import os
import threading
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Twist


def _clamp(value: float, limit: float) -> float:
    if not math.isfinite(value):
        return 0.0
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


class CmdVelToSportBridge:
    def __init__(self) -> None:
        self.ns = rospy.get_param("~namespace", "robot").strip("/")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", f"/{self.ns}/cmd_vel")
        self.dds_interface = str(rospy.get_param("~dds_interface", "")).strip()
        self.max_vel = float(rospy.get_param("~max_vel", 0.15))
        self.max_vel_y = float(rospy.get_param("~max_vel_y", 0.0))
        self.max_vel_ang = float(rospy.get_param("~max_vel_ang", 0.5))
        self.deadband_v = float(rospy.get_param("~deadband_v", 0.01))
        self.deadband_w = float(rospy.get_param("~deadband_w", 0.02))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.stop_on_shutdown = bool(rospy.get_param("~stop_on_shutdown", True))

        self._lock = threading.Lock()
        self._latest_cmd: Optional[Tuple[float, float, float]] = None
        self._latest_stamp = rospy.Time(0)
        self._last_sent: Optional[Tuple[float, float, float]] = None

        self._init_unitree_sdk()

        self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.publish_rate)), self._timer_cb)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "cmd_vel_to_sport up: %s -> Unitree SportClient.Move "
            "(dds_interface=%s, max_v=%.3f, max_vy=%.3f, max_w=%.3f)",
            self.cmd_vel_topic,
            self.dds_interface or "auto",
            self.max_vel,
            self.max_vel_y,
            self.max_vel_ang,
        )

    def _init_unitree_sdk(self) -> None:
        sdk_path = "/home/unitree/unitree_sdk2_python"
        conda_site = "/home/unitree/miniconda3/envs/by/lib/python3.8/site-packages"
        for path in (sdk_path, conda_site):
            if os.path.isdir(path) and path not in os.sys.path:
                os.sys.path.insert(0, path)

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        if self.dds_interface:
            ChannelFactoryInitialize(0, self.dds_interface)
        else:
            ChannelFactoryInitialize(0)

        self.client = SportClient()
        self.client.SetTimeout(1.0)
        self.client.Init()

    def _cmd_cb(self, msg: Twist) -> None:
        vx = _clamp(msg.linear.x, self.max_vel)
        vy = _clamp(msg.linear.y, self.max_vel_y)
        wz = _clamp(msg.angular.z, self.max_vel_ang)

        if abs(vx) < self.deadband_v:
            vx = 0.0
        if abs(vy) < self.deadband_v:
            vy = 0.0
        if abs(wz) < self.deadband_w:
            wz = 0.0

        with self._lock:
            self._latest_cmd = (vx, vy, wz)
            self._latest_stamp = rospy.Time.now()

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        with self._lock:
            cmd = self._latest_cmd
            stamp = self._latest_stamp

        if cmd is None or (now - stamp).to_sec() > self.cmd_timeout:
            cmd = (0.0, 0.0, 0.0)

        try:
            self.client.Move(cmd[0], cmd[1], cmd[2])
            self._last_sent = cmd
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "SportClient.Move failed: %r", exc)

    def _on_shutdown(self) -> None:
        if not self.stop_on_shutdown:
            return
        try:
            self.client.Move(0.0, 0.0, 0.0)
            self.client.StopMove()
        except Exception:
            pass


def main() -> None:
    rospy.init_node("cmd_vel_to_sport_bridge")
    CmdVelToSportBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
