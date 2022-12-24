#!/usr/bin/python3

'''
    Node description
'''

from math import pi, cos, sin, floor
from real_lidarbot.msg import Tick
import rclpy
from rclpy.node import Node

# Odometry class
class Odometry(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # Instance variables 
        self.radius = 0.035                                           # Wheel radius
        self.wheel_base = 0.145                                        # Distance between wheel
        self.ticks_per_rev = 20                                       # Encoder ticks per revolution
        self.metres_per_tick = (2*pi*self.radius)/self.ticks_per_rev  # Wheel distance moved per tick
        self.delta_right_tick = 0                                     # Change in right ticks
        self.delta_left_tick  = 0                                     # Change in left ticks
        self.prev_right_tick = 0                                      # Previous right tick value
        self.prev_left_tick = 0                                       # Previous left tick value
        self.delta_x = 0                                              #
        self.delta_y = 0                                              #
        self.delta_theta = 0                                          #

        # Initialize pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # Create subscription to /ticks topic
        self.ticks_sub = self.create_subscription(
                Tick, 
                'ticks',
                self.ticks_callback,
                1
            )

        # Create publisher to /odom_data_quat and /odom_data_euler (confirm)

    def ticks_callback(self, msg):
        '''

        '''

        # Initialize Tick message
        #msg = Tick()        
        
        # Compute changes in wheel ticks
        self.delta_right_tick = msg.right_tick - self.prev_right_tick
        self.delta_left_tick = msg.left_tick - self.prev_left_tick

        # Update ticks for next ...
        self.prev_right_tick = msg.right_tick
        self.prev_left_tick = msg.left_tick

        # Calculate distances
        right_whl_dist = self.metres_per_tick * self.delta_right_tick  # Distance traveled by right wheel
        left_whl_dist = self.metres_per_tick * self.delta_left_tick    # Distance traveled by left wheel
        centre_dist = (left_whl_dist + right_whl_dist) / 2             # Distance traveled by the robot
        
        self.delta_x = centre_dist * cos(self.theta)
        self.delta_y = centre_dist * sin(self.theta)
        self.delta_theta = (right_whl_dist - left_whl_dist) / self.wheel_base
        
        # New pose
        self.x += self.delta_x
        self.y += self.delta_y
        self.theta += self.delta_theta
        theta_d = floor(self.theta * 180 / pi) # Convert theta to degrees

        # Log data to console
        self.get_logger().info("x: %.3f" % (self.x) + 
                                " y: %.3f" % (self.y) +
                                " theta: %.2f" % (theta_d)
                                )

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
# Comments
# Add a button to reset the pose(?)
