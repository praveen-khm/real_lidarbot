#!/bin/usr/python3

'''
    Node description
'''

from math import pi, cos, sin, floor
import time
from real_lidarbot.msg import Tick
import rclpy
from rclpy.node import Node

# Odometry class
class Odometry(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # 
        self.radius = 0.035                                           # Wheel radius
        self.wheel_base = 0.14                                        # Distance between wheel
        self.ticks_per_rev = 20                                       # Encoder ticks per revolution
        self.metres_per_tick = (2*pi*self.radius)/self.ticks_per_rev  # Wheel distance moved per tick
    
        # Or should I do ticks per metre like Addison?
        self.delta_right_tick = 0                                     # Change in right ticks
        self.delta_left_tick  = 0                                     # Change in left ticks

        # Initialize pose
        self.x = 0
        self.y = 0
        self.theta = 0
        self.delta_x = 0
        self.delta_y = 0
        self.delta_theta = 0


        # Create subscription to /ticks topic
        self.ticks_sub = self.create_subscription(
                Tick, 
                'ticks',
                self.ticks_callback,
                1
            )

        # Create publisher to /odom_data_quat and /odom_data_euler (confirm)

    def ticks_callback(self):
        '''

        '''

        # Initialize Tick message
        msg = Tick()        
        
        #
        self.delta_right_tick = msg.right_tick
        self.delta_left_tick = msg.left_tick

        # Calculate 
        right_whl_dist = metres_per_tick * self.delta_right_tick  # Distance traveled by right wheel
        left_whl_dist = metres_per_tick * self.delta_left_tick    # Distance traveled by left wheel
        centre_dist = (left_whl_dist + right_whl_dist) / 2        # Distance traveled by the robot
        
        self.delta_x = centre_dist * cos(theta)
        self.delta_y = centre_dist * sin(theta)
        self.delta_theta = (right_whl_dist - left_whl_dist) / self.wheel_base
        
        # New pose
        self.x += self.delta_x
        self.y += self.delta_y
        self.theta += self.delta_theta

        # Log data to console
        self.get_logger().info("x: %.2f" % (self.x) + 
                                "y: %.2f" % (self.y) +
                                "theta: %.2f" % (self.theta)
                                )

        # How about the time? 

        # Later publish these poses with the right message type to the right topic

        # Add a button to reset the odom values, so we'll need to subscribe to /joy


# Main function
def main():
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create odometry node
        odometry_node = Odometry('odometry_node')

        # Spin node for callback function
        rclpy.spin(odometry_node)

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        # Destroy node
        odometry_node.destroy_node()
        print('\nodometry_node destroyed')

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# NOTE:
# comments
