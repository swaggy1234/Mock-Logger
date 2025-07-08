from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='embodied_event_logger',
            executable='generate_mock_msgs',
            name='mock_publisher',
            output='screen',
            parameters=[
                {'linear_x': 0.5},
                {'angular_z': 1.0},
                {'frequency': 2.0},
            ]
        ),
        Node(
            package='embodied_event_logger',
            executable='subscriber_node',
            name='mock_subscriber',
            output='screen',
        ),
    ])
