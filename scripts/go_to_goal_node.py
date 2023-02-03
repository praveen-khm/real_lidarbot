#!/usr/bin/python3

'''
    The go to goal node accepts coordinates and the proportionate number of coordinate pairs parameters to inform the robot
    on the positions it should move to.  
    
    The node subscribes to /odom and /joy topics, to calculate motor speeds and stop the motors, respectively. 
    It also publishes the right and left wheel directions on topics /right_motor_dir and /left_motor_dir correspondingly.
'''

from time import sleep, process_time
from math import atan2, sqrt, pi, cos, sin

from real_lidarbot.motor import Motor
from real_lidarbot.utils import euler_from_quaternion

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Go to goal class
class GoToGoal(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')

        # Declare ROS parameters
        self.declare_parameter('coordinates') 
        self.declare_parameter('coordinate_pairs') 

        self.coordinates = self.get_parameter('coordinates')
        self.coordinate_pairs = self.get_parameter('coordinate_pairs').value
        
        self.counter = 0                # Loop variable to keep track of the number of coordinate pairs

        # Instance variables
        self.wheel_diameter = 0.067     # Wheel diameter [metres]
        self.wheel_base = 0.134         # Distance between wheels [metres]
        self.left_max_rpm = 200.0       # Max revolutions per minute of the left motor
        self.right_max_rpm = 195.0      # Max revolutions per minute of the right motor
        self.linear_speed = 0.0         # Linear speed [metres/sec]
        self.angular_speed = 0.0        # Angular speed [rads/sec]
        self.I = 0.0                    # Angular integral error
        self.D = 0.0                    # Angular derivative error

        # Initialize motor driver
        self.motor = Motor() 

        # Create a subscription to /odom_data topic
        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                5 
            )
        
        # Create a subscription to /joy topic
        self.joy_sub = self.create_subscription(
                Joy,
                'joy',
                self.kill_switch_callback,
                1
            )

        # Create publishers for /right_motor_dir and /left_motor_dir topics of message type String
        self.right_motor_dir_pub = self.create_publisher(String, 'right_motor_dir', 1)
        self.left_motor_dir_pub = self.create_publisher(String, 'left_motor_dir', 1)

    def stop_motors(self):
        '''
        0 - for left motor
        1 - for right motor
        '''
        self.motor.MotorStop(0)
        self.motor.MotorStop(1)

    def kill_switch_callback(self, msg):
        '''
            This function abruptly stops the motors and raises a SystemExit exception, that is 
            handled towards the end of thie script, once the 'A' button on the joy game controller is 
            pressed. This is an emergency safety feature in case something goes wrong.

            'A' button corresponds to buttons[2] on the joystick map, with a value of 1 when pressed and
            0 otherwise.
        '''
        if msg.buttons[2] == 1:
            self.stop_motors()
            self.get_logger().info('Emergency Stop!')
            raise SystemExit

    def odom_callback(self, msg): 
        '''
            This function calculates the linear and angular speeds used to set the robot's motor speeds
            to move the robot from its current set of coordinates to the desired goal coordinates.
        '''
        global start_time
        quat = Quaternion()        

        x_goal = self.coordinates.value[self.counter*2]     # Goal x coordinate
        y_goal = self.coordinates.value[(self.counter*2)+1] # Goal y coordinate
        
        x_curr = msg.pose.pose.position.x                   # Current x coordinate
        y_curr = msg.pose.pose.position.y                   # Current y coordinate

        quat = msg.pose.pose.orientation                    # Current orientation in quaternion form

        # Obtain euler angles (in radians) from quaternion and assign as current theta angle
        euler_angles = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        theta_curr = euler_angles[2]
        
        # Euclidean distance between goal and current x,y coordinates
        euclid_dist = abs(sqrt(((x_goal - x_curr) ** 2) + ((y_goal - y_curr) ** 2)))
        K_p_linear = 1.4 # Proportial gain for linear speed

        # Calculate linear_speed
        self.linear_speed = K_p_linear * euclid_dist 

        # Angular PID controller gains
        K_p_angular = 0.45
        K_i = 0.0055
        K_d = 0.04

        # Angle from robot to goal
        theta_goal = atan2(y_goal - y_curr, x_goal - x_curr)

        # Error between the goal angle and robot's angle
        angle_error = theta_goal - theta_curr
        angle_error = atan2(sin(angle_error), cos(angle_error))
        
        stop_time = process_time() # Stop timer
        
        # Calculate integral and derivative angular error terms
        self.I = self.I + angle_error * (stop_time - start_time)
        self.D = (angle_error - self.D) / (stop_time - start_time)

        # Calculate angular_speed
        self.angular_speed = K_p_angular * angle_error + K_i * self.I + K_d * self.D
        
        # Assign derivative term for the next callback
        self.D = angle_error

        '''
            Move the robot until the euclidean distance between is less than 0.04 then robot moves to the 
            next set of coordinates (or waypoint) if there is one or stops the robot because the goal location 
            has been reached.
        '''
        if euclid_dist > 0.04: 
            self.set_motor_speeds()
        else:
            self.counter += 1
            # Check for new coordinates or waypoints
            if self.counter != self.coordinate_pairs:
                self.stop_motors()
                self.get_logger().info('Moving to the next waypoint ...')
                sleep(2)
            else:
                self.stop_motors()
                self.get_logger().info('Goal reached!')
                raise SystemExit
       
    def set_motor_speeds(self):
        '''
            Sets the motor speed of each wheel based on angular_speed: 
            
            Each wheel covers self.wheel_base meters in one radian, so the target speed for each wheel in meters per sec is 
            angular_speed * wheel_base / wheel_diameter.
        '''
        # Wheel angular motion
        right_twist_mps = self.angular_speed * self.wheel_base / self.wheel_diameter
        left_twist_mps = -1.0 * self.angular_speed * self.wheel_base / self.wheel_diameter
        
        # Now add in linear motion
        right_mps = self.linear_speed + right_twist_mps
        left_mps = self.linear_speed + left_twist_mps

        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        right_target_rpm = (right_mps * 60.0) / (pi * self.wheel_diameter)
        left_target_rpm = (left_mps * 60.0) / (pi * self.wheel_diameter)
        
        # Scale target motor speeds 
        right_motor_speed = (right_target_rpm / self.right_max_rpm) * 100.0
        left_motor_speed = (left_target_rpm / self.left_max_rpm) * 100.0
        
        # Clip speeds to +/- 65%
        right_motor_speed = max(min(right_motor_speed, 65.0), -65.0)
        left_motor_speed = max(min(left_motor_speed, 65.0), -65.0)
        
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

        # Create go to goal node
        go_to_goal_node = GoToGoal('go_to_goal_node')

        # Start timer
        global start_time
        start_time = process_time()

        # Spin node for callback function 
        rclpy.spin(go_to_goal_node)

    except SystemExit:
        rclpy.logging.get_logger("Destroying").info(go_to_goal_node.get_name())
        
        # Destroy node
        go_to_goal_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        rclpy.logging.get_logger("KeyboardInterrupt, destroying").info(go_to_goal_node.get_name())

        # Destroy node
        go_to_goal_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()