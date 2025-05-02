#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_unstamped',
            self.twist_callback,
            10)

    def twist_callback(self, msg):
        stamped_msg = TwistStamped()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = "base_link"  # Set appropriate frame
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()