import rclpy
import os
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')

        # Declare parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('linear_x', 0.5)
        self.declare_parameter('angular_z', 0.0)

        # Read initial values
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.linear_x = self.get_parameter('linear_x').get_parameter_value().double_value
        self.angular_z = self.get_parameter('angular_z').get_parameter_value().double_value

        self.get_logger().info(f'Starting with linear_x={self.linear_x}, angular_z={self.angular_z}, freq={self.frequency}Hz')

        # Create publisher
        self.publisher = self.create_publisher(Twist, 'mock_topic', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_message)

        # Parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

    def publish_message(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        # Log to file
        log_path = os.path.expanduser('~/mock_pub_log.txt')
        now = self.get_clock().now().to_msg()
        with open(log_path, 'a') as f:
            f.write(f'{now.sec}.{now.nanosec}: linear.x={msg.linear.x}, angular.z={msg.angular.z}\n')

    def param_callback(self, params):
        for param in params:
            if param.name == 'linear_x':
                self.linear_x = param.value
            elif param.name == 'angular_z':
                self.angular_z = param.value
            elif param.name == 'frequency':
                self.frequency = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.frequency, self.publish_message)

        self.get_logger().info(f'Updated params: linear_x={self.linear_x}, angular_z={self.angular_z}, frequency={self.frequency}')
        return rclpy.parameter.ParameterEventHandler.Result(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
