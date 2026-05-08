#!/usr/bin/env python3
"""Bridge /cmd_vel (Twist) → Unitree Go2 Sport API or Obstacle Avoidance API.

Subscribes to /cmd_vel geometry_msgs/Twist and publishes velocity commands.

Two modes:
  obstacle_avoidance=false (default):
    → /api/sport/request  api_id=1008 (Move)  {"x", "y", "z"}
  obstacle_avoidance=true:
    → /api/obstacles_avoid/request  api_id=1003  {"x", "y", "yaw", "mode": 0}
    Unitree's built-in obstacle avoidance: the robot will slow/stop/steer
    around obstacles detected by its own sensors.

To enable:  --ros-args -p obstacle_avoidance:=true
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request


class CmdVelToSportBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_sport_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_topic', '/api/sport/request')
        self.declare_parameter('obstacle_avoidance', False)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        sport_topic = str(self.get_parameter('sport_topic').value)
        self.obstacle_avoidance = bool(self.get_parameter('obstacle_avoidance').value)

        if self.obstacle_avoidance:
            # Override topic to obstacle avoidance endpoint
            self.api_id = 1003
            self.out_topic = '/api/obstacles_avoid/request'
            mode_label = 'obstacle_avoidance (api_id=1003)'
        else:
            self.api_id = 1008
            self.out_topic = sport_topic
            mode_label = 'sport Move (api_id=1008)'

        self.sport_pub = self.create_publisher(Request, self.out_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)

        self.get_logger().info(
            f'cmd_vel→sport bridge: {cmd_vel_topic} → {self.out_topic} [{mode_label}]')

    def _cmd_vel_cb(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = self.api_id

        if self.obstacle_avoidance:
            req.parameter = json.dumps({
                'x': msg.linear.x,
                'y': msg.linear.y,
                'yaw': msg.angular.z,
                'mode': 0,
            })
        else:
            req.parameter = json.dumps({
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.angular.z,
            })

        self.sport_pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSportBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
