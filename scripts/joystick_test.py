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
        
        self.wheel_diameter = 0.07  #
        self.wheel_base = 0.145     #
        self.left_max_rpm = 200.0   #
        self.right_max_rpm = 200.0  #
        self.motor = Motor()        # Initialize motor driver
        
        self.speed = 0.0            #
        self.spin = 0.0             #
        
        # Create subscription to /joy topic of message type Joy
        self.joy_subscription = self.create_subscription(
                Joy, 
                'joy',
                self.joy_callback,
                5
            )
        
        # Create publishers for /right_motor_dir and /left_motor_dir topics of message type String
        self.right_motor_dir_pub = self.create_publisher(String, 'right_motor_dir', 1) 
        self.left_motor_dir_pub = self.create_publisher(String, 'left_motor_dir', 1) 

    # Maximum speed in metres per second (mps) at maximum revolutions per minute (rpm)
    def max_speed(self):
        rpm = (self.left_max_rpm + self.right_max_rpm) / 2.0
        mps = rpm * math.pi * self.wheel_diameter / 60.0
        return mps

    # Rotation in radians per second at max rpm
    def max_twist(self):
        return self.max_speed() / self.wheel_diameter

    # Callback function for joy subscription
    def joy_callback(self, msg):
        '''        
        It translates buttons on the  game controller into speed and spin values
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

        #
        self.set_motor_speeds()
        
    #
    def set_motor_speeds(self):
        # First figure out the speed of each wheel based on spin: each wheel
        # covers self._wheel_base meters in one radian, so the target speed
        # for each wheel in meters per sec is spin (radians/sec) times
        # wheel_base divided by wheel_diameter

        right_twist_mps = self.spin * self.wheel_base / self.wheel_diameter
        left_twist_mps = -1.0 * self.spin * self.wheel_base / self.wheel_diameter
        
        # Now add in forward motion.
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps

        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        left_target_rpm = (left_mps * 60.0) / (math.pi * self.wheel_diameter)
        right_target_rpm = (right_mps * 60.0) /  (math.pi * self.wheel_diameter)
        
        left_percentage = (left_target_rpm / self.left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self.right_max_rpm) * 100.0

        # clip to +- 40%
        left_percentage = max(min(left_percentage, 40.0), -40.0)
        right_percentage = max(min(right_percentage, 40.0), -40.0)
        
        #
        index_r = String()
        index_l = String()

        if right_percentage > 0:
            index_r.data = 'forward'
        else:
            index_r.data = 'backward' 

        if left_percentage > 0:
            index_l.data = 'forward' 
        else:
            index_l.data = 'backward'

        # Publish motor directions
        self.right_motor_dir_pub.publish(index_r)
        self.left_motor_dir_pub.publish(index_l)
        
        #
        self.motor.MotorRun(0, index_l.data, abs(left_percentage))
        self.motor.MotorRun(1, index_r.data, abs(right_percentage))

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
        # Destroy node
        robot.destroy_node()
        print('\nlidar_bot joystick test node destroyed')

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# NOTES: 
# Comments, comments
