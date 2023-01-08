#!/usr/bin/python3

'''
    Node description
'''

from real_lidarbot.motor import Motor
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # Motor and wheel parameters
        self.wheel_diameter = 0.067 # Diameter of the wheels in metres
        self.wheel_base = 0.134     # Distance between the centre of the wheels in metres
        self.left_max_rpm = 200.0   # Number of revolutions per minute of the left motor running at 100% power
        self.right_max_rpm = 195.0  # Number of revolutions per minute of the right motor running at 100% power
        self.speed = 0.0            # Initial linear speed in metres/sec
        self.spin = 0.0             # Initial angular speed in rads/sec
        
        # Initialize motor driver
        self.motor = Motor()     

        # Create a subscription to /joy topic of message type Joy
        self.joy_subscription = self.create_subscription(
                Joy, 
                'joy',
                self.joy_callback,
                5
            )
        
        # Create publishers for /right_motor_dir and /left_motor_dir topics of message type String
        self.right_motor_dir_pub = self.create_publisher(String, 'right_motor_dir', 1) 
        self.left_motor_dir_pub = self.create_publisher(String, 'left_motor_dir', 1) 

    def joy_callback(self, msg):
        '''        
        It translates buttons on the game controller into speed and spin values
        used to set the motor speeds of the robot.

        Using:

        Move right joystick left/right for corresponding left/right motion (rotation)
        Move left joystick forward/backward for corresponding forward/backward motion (translation)
        R2 for emergency stop
        
        Buttons                 Joystick map  Button movement/click status 
        Rstick left/right         axes[2]    +1 (left)    to -1 (right)
        Lstick forward/backward   axes[1]    +1 (forward) to -1 (backward)
        R2                        buttons[7]  1 pressed, 0 otherwise
        '''
        
        # Map left/right movement to self.spin, set to zero if below 0.10
        if abs(msg.axes[2]) > 0.10:
            self.spin = msg.axes[2]
        else:
            self.spin = 0.0

        # Map forward/backward movement to self.speed, set to zero if below 0.10
        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        # Set both self.speed and self.spin to zero when R2 button is pressed
        if msg.buttons[7] == 1:
            self.speed = 0.0
            self.spin = 0.0

        self.set_motor_speeds()
        
    def set_motor_speeds(self):
        '''
        Figures out the speed of each wheel based on spin: each wheel
        covers self.wheel_base meters in one radian, so the target speed
        for each wheel in meters per sec is spin (radians/sec) times
        wheel_base divided by wheel_diameter
        '''
        right_twist_mps = self.spin * self.wheel_base / self.wheel_diameter
        left_twist_mps = -1.0 * self.spin * self.wheel_base / self.wheel_diameter
        
        # Now add in linear motion
        right_mps = self.speed + right_twist_mps
        left_mps = self.speed + left_twist_mps

        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        right_target_rpm = (right_mps * 60.0) /  (math.pi * self.wheel_diameter)
        left_target_rpm = (left_mps * 60.0) / (math.pi * self.wheel_diameter)
        
        # Scale target motor speeds 
        right_motor_speed = (right_target_rpm / self.right_max_rpm) * 100.0
        left_motor_speed = (left_target_rpm / self.left_max_rpm) * 100.0

        # Clip speeds to +/- 50%
        right_motor_speed = max(min(right_motor_speed, 50.0), -50.0)
        left_motor_speed = max(min(left_motor_speed, 50.0), -50.0)
        
        # Initialize string messages
        right_motor_direction = String()
        left_motor_direction = String()
        
        # Set motor directions
        if right_motor_speed > 0:
            right_motor_direction.data = 'forward'
        else:
            right_motor_direction.data = 'backward' 

        if left_motor_speed > 0:
            left_motor_direction.data = 'forward' 
        else:
            left_motor_direction.data = 'backward'

        # Publish motor directions
        self.right_motor_dir_pub.publish(right_motor_direction)
        self.left_motor_dir_pub.publish(left_motor_direction)
        
        # Run motors with specified direction and speeds
        self.motor.MotorRun(0, left_motor_direction.data, abs(left_motor_speed))
        self.motor.MotorRun(1, right_motor_direction.data, abs(right_motor_speed))

# Main function
def main():
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create robot node
        robot = Robot('lidar_bot')

        # Wait for incoming commands
        rclpy.spin(robot)

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        rclpy.logging.get_logger("KeyboardInterrupt, destroying").info(robot.get_name())

        # Destroy node
        robot.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# NOTES: 
# Comments, comments
# Rename class and node
