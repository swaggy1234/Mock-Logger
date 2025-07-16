from setuptools import setup
import os
from glob import glob

package_name = 'embodied_event_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Event logger for different robot embodiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_mock_msgs = embodied_event_logger.generate_mock_msgs:main',
            'subscriber_node = embodied_event_logger.subscriber_node:main',
        ],
    },
)
