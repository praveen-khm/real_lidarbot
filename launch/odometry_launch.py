# Launches the custom written encoder and odometry nodes

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='real_lidarbot',
            executable='encoder_node.py',
            name='encoder_node'
        ),
        Node(
            package='real_lidarbot',
            executable='odometry_node.py',
            name='odometry_node'
        )
    ])
