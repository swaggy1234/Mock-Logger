import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
import csv
from datetime import datetime

class MockSubscriber(Node):
    def __init__(self):
        super().__init__('mock_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'mock_topic',
            self.listener_callback,
            10
        )
        self.log_path = os.path.expanduser('~/mock_sub_log.csv')
        self.ensure_csv_header()

    def ensure_csv_header(self):
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'node', 'topic', 'linear_x', 'angular_z'])

    def listener_callback(self, msg):
        timestamp = datetime.now().isoformat()
        log_entry = [timestamp, self.get_name(), 'mock_topic', msg.linear.x, msg.angular.z]
        self.get_logger().info(f'{timestamp}: Received linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(log_entry)

def main(args=None):
    rclpy.init(args=args)
    node = MockSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

