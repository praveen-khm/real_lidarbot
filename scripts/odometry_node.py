#!/usr/bin/python3

'''
    Node description
'''

from math import pi, cos, sin, floor
from real_lidarbot.msg import Tick
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# Odometry class
class OdometryNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # Instance variables 
        self.radius = 0.0335                                          # Wheel radius
        self.wheel_base = 0.134                                       # Distance between wheel
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

        # Create a subscription to /ticks topic of type Tick
        self.ticks_sub = self.create_subscription(
                Tick, 
                'ticks',
                self.ticks_callback,
                1
            )

        # Create a subscription to /joy topic of type Joy
        self.joy_sub = self.create_subscription(
                Joy,
                'joy',
                self.reset_pose_callback,
                1
            )
        
        # Create a publisher to /odom_data topic
        self.odom_pub = self.create_publisher(Odometry, 'odom_data', 2)

        # Create publisher to /odom_data_quat and /odom_data_euler (confirm)

    def reset_pose_callback(self, msg):
        '''
            The pose of the robot is reset after the 'X' button on the joy gamepad controller 
            is pressed.
            
            The 'X' button corresponds to buttons[3] on the joystick map, with a value of 1 when 
            pressed and 0 otherwise.
        '''
        if msg.buttons[3] == 1:
            self.x = 0
            self.y = 0
            self.theta = 0

    def ticks_callback(self, msg):
        '''

        '''

        # Compute changes in wheel ticks
        self.delta_right_tick = msg.right_tick - self.prev_right_tick
        self.delta_left_tick = msg.left_tick - self.prev_left_tick

        # Update ticks for next callback
        self.prev_right_tick = msg.right_tick
        self.prev_left_tick = msg.left_tick

        # Calculate distances
        right_wheel_dist = self.metres_per_tick * self.delta_right_tick # Distance traveled by right wheel
        left_wheel_dist = self.metres_per_tick * self.delta_left_tick   # Distance traveled by left wheel
        centre_dist = (left_wheel_dist + right_wheel_dist) / 2          # Distance traveled by robot
        
        self.delta_x = centre_dist * cos(self.theta)
        self.delta_y = centre_dist * sin(self.theta)
        self.delta_theta = (right_wheel_dist - left_wheel_dist) / self.wheel_base
        
        # New pose
        self.x += self.delta_x
        self.y += self.delta_y
        self.theta += self.delta_theta

        # Convert from radians to degrees
        theta_d = floor(self.theta * 180 / pi) 

        # Log data to console
        self.get_logger().info("x: %.3f" % (self.x) + 
                                " y: %.3f" % (self.y) +
                                " theta: %.2f" % (theta_d)
                                )

        # Publish pose values
        self.odom_data_publisher()

    def odom_data_publisher(self):
        '''
        '''

        # Initialize odometry message
        odom = Odometry()
        
        # Assign pose parameters to odom message
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # odom.pose.pose.orientation.z = self.theta

        # Publish odom message to /odom_data
        self.odom_pub.publish(odom)

# Main function
def main():
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create odometry node
        odometry_node = OdometryNode('odometry_node')

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
