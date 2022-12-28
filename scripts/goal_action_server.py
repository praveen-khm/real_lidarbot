#!/usr/bin/python3

'''
    Node description
'''

from real_lidarbot.msg import Tick
from real_lidarbot.motor import Motor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# Goal action server class
class GoalActionServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')

        # Declare ROS parameters
        self.declare_parameter('distance')
        self.declare_parameter('direction')
        self.declare_parameter('speed')

        # 
        self.desired_distance = self.get_parameter('distance')
        self.wheel_direction = self.get_parameter('direction')
        self.speed = self.get_parameter('speed')

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

    def stop_motors(self):
        '''
        0 - for left motor
        1 - for right motor
        '''
        self.motor.MotorStop(0)
        self.motor.MotorStop(1)
    
    def odom_callback(self, msg): # rename callback(?)
        '''
        '''
        current_distance = msg.pose.pose.position.x

        # 
        if (abs(current_distance - self.desired_distance.value)) >  0.030 :
            self.motor.MotorRun(0, self.wheel_direction.value, self.speed.value)
            self.motor.MotorRun(1, self.wheel_direction.value, 0.97*self.speed.value) #
        else:
            self.stop_motors()

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
