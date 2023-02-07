#!/usr/bin/python3

'''
    This joystick pad node subscribes to the joystick button commands, received on the /joy topic, and publishes
    them as Twist messages on the /cmd_vel topic. 

    In this project application, the simulated two wheeled drive lidar robot subscribes to the /cmd_vel topic, 
    therefore, the robot can be controlled by the buttons on the joystick pad.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# JoystickPad class inheriting from the Node class
class JoystickPad(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.get_logger().info(self.get_name() + " is initialized")

        self.speed = 0.0 # Translation attribute
        self.spin = 0.0  # Rotation attribute

        # Create subscription to /joy topic of message type Joy
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            5
        )
        self.joy_subscription # Added to prevent unused variable warning

        # Create publisher for /cmd_vel topic of message type Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        '''
        This method is called once a Joy message is received on the /joy topic.
        It translates buttons on a generic game controller into speed and spin values
        to be published to the /cmd_vel topic.

        Using:

        Move right joystick left/right for corresponding left/right motion (rotation)
        Move left joystick forward/backward for corresponding forward/backward motion (translation)
        R2 for emergency stop 

        Rstick left/right         axes[2]    +1 (left)    to -1 (right)
        Lstick forward/backward   axes[1]    +1 (forward) to -1 (backward)
        R2                        buttons[7]  1 pressed, 0 otherwise
        '''

        # Map left/right movement to self.spin, set to zero if below 0.10
        if abs(msg.axes[2]) > 0.10:
            self.spin = msg.axes[2]
        else:
            self.spin = 0.0

        # Map forward/backward movement to self.speed; set to zero if below 0.10
        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        # Set both self.speed and self.spin to zero when R2 button is pressed
        if msg.buttons[7] == 1:
            self.speed = 0.0
            self.spin = 0.0

        # Publish cmd_vel values
        self.command_vel_pub()    
    
    def command_vel_pub(self):
        # Initialize twist message
        twist = Twist() 

        twist.linear.x = self.speed
        twist.angular.z = self.spin

        # Publish twist messages to /cmd_vel
        self.cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    try: 
        # Initialize ROS python client
        rclpy.init()

        # Create joystick_pad node
        joystick_pad = JoystickPad() 

        # Wait for incoming commands 
        rclpy.spin(joystick_pad)

    # On Ctrl+C executed in the terminal
    except KeyboardInterrupt:
        # Destroy node
        joystick_pad.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

# TODO: 
# Format code inline with other nodes