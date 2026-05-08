#!/usr/bin/env python3
"""
Bridge node to convert TwistStamped to Twist for the quadruped controller.
The CMU autonomy stack's pathFollower publishes TwistStamped,
but the Champ quadruped controller expects Twist.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        
        # Subscribe to TwistStamped from pathFollower
        # Use relative names so ROS 2 remapping works (launch file remaps these)
        self.sub = self.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            self.twist_callback,
            10
        )
        
        # Publish Twist to the topic the controller expects.
        # Use relative name — launch file remaps to the correct output topic.
        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Resolve the actual topic names after remapping for the log message
        sub_topic = self.sub.topic_name
        pub_topic = self.pub.topic_name
        self.get_logger().info(f'Twist Bridge started: {sub_topic} -> {pub_topic}')
        
    def twist_callback(self, msg: TwistStamped):
        twist = Twist()
        twist.linear = msg.twist.linear
        twist.angular = msg.twist.angular
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
