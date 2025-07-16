import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import random

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')
        self.publisher = self.create_publisher(Twist, 'mock_topic', 10)
        self.timer_period = 0.5  # seconds
        self.start_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Starting enhanced mock publisher...")

    def timer_callback(self):
        t = time.time() - self.start_time
        msg = Twist()

        # Simulate changing motion
        msg.linear.x = round(1.0 * math.sin(t) + random.uniform(-0.05, 0.05), 3)
        msg.angular.z = round(0.5 * math.cos(t) + random.uniform(-0.05, 0.05), 3)

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
