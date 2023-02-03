#!/usr/bin/python3

'''
    This node susbcribes to the 2D Nav Goal ...
'''

from real_lidarbot.utils import euler_from_quaternion

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion

# Rviz click to 2D class
class RvizClick2DNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')

        # Create a subscription to topic /initialpose of type PoseWithCovarianceStamped
        self.initial_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, 
                'initialpose',
                self.pose_callback,
                1,
            )

        # Create a publisher to topic /initial_2d
        self.initial_pose_pub = self.create_publisher(PoseStamped, 'initial_2d', 1)
    
    def pose_callback(self, pose):
        ''' Takes /initialpose topic as input and publishes /initial_2d. ''' 
        convertedPose = PoseStamped()
        convertedPose.header.frame_id = 'map'
        convertedPose.header.stamp = pose.header.stamp
        convertedPose.pose.position.x = pose.pose.pose.position.x
        convertedPose.pose.position.y = pose.pose.pose.position.y
        convertedPose.pose.position.z = 0.0
        quat = Quaternion(x=0.0, y=0.0, z=pose.pose.pose.orientation.z, w=pose.pose.pose.orientation.w)
        euler_angles = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        convertedPose.pose.orientation.x = 0.0
        convertedPose.pose.orientation.y = 0.0
        convertedPose.pose.orientation.z = euler_angles[2]
        convertedPose.pose.orientation.w = 0.0
        self.initial_pose_pub.publish(convertedPose)


def main():
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create odometry node
        rviz_click_to_2d_node = RvizClick2DNode('rviz_click_to_2d_node')

        # Spin node for callback function
        rclpy.spin(rviz_click_to_2d_node)

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        rclpy.logging.get_logger("KeyboardInterrupt, destroying").info(rviz_click_to_2d_node.get_name())

        # Destroy node
        rviz_click_to_2d_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# TODO:
# Rename 'convertedPose'?

# Node Description

# Website: https://automaticaddison.com
#  *   ROS node that converts the user's desired initial pose and goal location
#  *   into a usable format.
#  * Subscribe:
#  *   initialpose : The initial position and orientation of the robot using 
#  *                 quaternions. (geometry_msgs/PoseWithCovarianceStamped)
#  *   move_base_simple/goal : Goal position and 
#  *                           orientation (geometry_msgs::PoseStamped)
#  * Publish: This node publishes to the following topics:   
#  *   goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
#  *   initial_2d : The initial position and orientation of the robot using 
#  *                Euler angles. (geometry_msgs/PoseStamped)