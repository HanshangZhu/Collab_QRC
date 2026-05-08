#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSBridge(Node):
    def __init__(self):
        super().__init__('qos_bridge')
        
        # Input topic (Best Effort from Gazebo/laserscan_to_pointcloud)
        self.declare_parameter('input_topic', '/registered_scan')
        input_topic = self.get_parameter('input_topic').value
        
        # Output topic (Reliable for CMU stack)
        self.declare_parameter('output_topic', '/registered_scan_reliable')
        output_topic = self.get_parameter('output_topic').value

        # Best Effort QoS for Subscription
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Reliable QoS for Publisher
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos_best_effort
        )
        
        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_reliable
        )
        
        self.get_logger().info(f'QoS Bridge Started: {input_topic} (BestEffort) -> {output_topic} (Reliable)')

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
