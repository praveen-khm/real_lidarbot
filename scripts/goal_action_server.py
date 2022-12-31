#!/usr/bin/python3

'''
    Node description
'''

from math import atan2, sqrt, pi 
from real_lidarbot.msg import Tick
from real_lidarbot.motor import Motor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Goal action server class
class GoalActionServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')

        # Declare ROS parameters
        self.declare_parameter('distance')
        self.desired_distance = self.get_parameter('distance')
        
        #
        self.wheel_diameter = 0.067
        self.wheel_base = 0.134
        self.left_max_rpm = 200.0
        self.right_max_rpm = 200.0

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Initialize motor driver
        self.motor = Motor() 

        # Create a subscription to /odom_data topic
        self.odom_sub = self.create_subscription(
                Odometry,
                'odom_data',
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
        This function abruptly stops the motors and shuts down the ROS python 
        client when  once the 'A' button on the joy game controller is pressed. This is an 
        emergency safety feature in case something goes wrong.

        'A' button corresponds to buttons[2] on the joystick map, with a value of 1 when pressed and
        0 otherwise.
        '''
        if msg.buttons[2] == 1:
            self.stop_motors()
            print('Emergency stop!')

            # Shutdown ROS python client
            rclpy.shutdown()

    def odom_callback(self, msg): # rename callback(?)
        '''
        '''
        
        x_goal = self.desired_distance.value[0]
        y_goal = self.desired_distance.value[1]

        #print(len(self.desired_distance))
        x_curr = msg.pose.pose.position.x           # Current x coordinate
        y_curr = msg.pose.pose.position.y           # Current y coordinate
        theta_curr = msg.pose.pose.orientation.z    #
        
        K_linear = 0.8 # 0.95
        euclid_dist = abs(sqrt(((x_goal - x_curr) ** 2) + ((y_goal - y_curr) ** 2)))
        self.linear_speed = K_linear * euclid_dist 

        K_angular = 0.4
        desired_angle = atan2(y_goal - y_curr, x_goal - x_curr) # rename var later?        
        self.angular_speed = K_angular * (desired_angle - theta_curr)

        if euclid_dist > 0.01:
            self.set_motor_speeds()
        else:
            print(euclid_dist)
            self.stop_motors()

        # Multiple waypoints

        #current_distance = msg.pose.pose.position.x

        # 
        #if (abs(current_distance - self.desired_distance.value)) >  0.030 :
        #    self.motor.MotorRun(0, self.wheel_direction.value, self.speed.value)
        #    self.motor.MotorRun(1, self.wheel_direction.value, 0.97*self.speed.value) #
        #else:
        #    self.stop_motors()

    def set_motor_speeds(self):
        '''
        '''

        right_twist_mps = self.angular_speed * self.wheel_base / self.wheel_diameter
        left_twist_mps = -1.0 * self.angular_speed * self.wheel_base / self.wheel_diameter
        
        # Now add in linear motion
        right_mps = self.linear_speed + right_twist_mps
        left_mps = self.linear_speed + left_twist_mps

        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        right_target_rpm = (right_mps * 60.0) /  (pi * self.wheel_diameter)
        left_target_rpm = (left_mps * 60.0) / (pi * self.wheel_diameter)
        
        # Scale target motor speeds 
        right_motor_speed = (right_target_rpm / self.right_max_rpm) * 100.0
        left_motor_speed = (left_target_rpm / self.left_max_rpm) * 100.0
        
        # Clip speeds to +/- 60%
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

        # Create goal action server node
        goal_act_server_node = GoalActionServer('goal_act_server_node')

        # Spin node for callback function 
        rclpy.spin(goal_act_server_node)

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        # Destroy node
        goal_act_server_node.destroy_node()
        print('\ngoal_act_server_node destroyed with Ctrl+C')

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# NOTES:
# Comments
