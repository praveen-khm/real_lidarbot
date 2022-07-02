from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
            Node(
                package='real_lidarbot',
                executable='imu_node.py',
                name='imu_node',
                )
            ])
            
