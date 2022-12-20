#!/usr/bin/python3

'''
    This encoder node subscribes to topics /right_motor_dir and /left_motor_dir in order to 
    publish to topics /right_ticks and /left_ticks for the individual motor pulses/ticks.
'''

import signal
import sys
import time
import RPi.GPIO as GPIO
from real_lidarbot.msg import Tick
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GPIO Pins
LEFT_WHL_ENCODER = 25
RIGHT_WHL_ENCODER = 24

# Pulse counters
left_wheel_pulse_count = 0
right_wheel_pulse_count = 0

# Default wheel directions
left_wheel_direction = 'backward'
right_wheel_direction = 'backward'

# Set mode for GPIO pins
GPIO.setmode(GPIO.BCM)

# Setup GPIO pins with internal pull up resistors activated
GPIO.setup(LEFT_WHL_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_WHL_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Left wheel callback function
def left_wheel_pulse(channel):
    global left_wheel_pulse_count, left_wheel_direction

    if left_wheel_direction == 'forward':
        left_wheel_pulse_count += 1
    else:
        left_wheel_pulse_count -= 1
    print('Left: ', left_wheel_pulse_count)

# Right wheel callback function
def right_wheel_pulse(channel):
    global right_wheel_pulse_count, right_wheel_direction

    if right_wheel_direction == 'forward':
        right_wheel_pulse_count += 1
    else:
        right_wheel_pulse_count -= 1
    print('Right: ', right_wheel_pulse_count)

# Initialize encoder interrupts for falling signal states
GPIO.add_event_detect(LEFT_WHL_ENCODER, GPIO.FALLING, callback=left_wheel_pulse)
GPIO.add_event_detect(RIGHT_WHL_ENCODER, GPIO.FALLING, callback=right_wheel_pulse)

# Encoder class
class Encoder(Node):
    def __init__(self, name):

       super().__init__(name)
       self.get_logger().info(self.get_name() + ' is initialized')

       # any class attributes?
       
       # Create subscription to /right_motor_dir 
       self.right_dir_sub = self.create_subscription(
               String,
               'right_motor_dir',
               self.right_dir_callback,
               1 # Play with this as well as the queue variable for the right_motor_dir topic
            )

       # Create subscription to /left_motor_dir topic
       self.left_dir_sub = self.create_subscription(
               String,
               'left_motor_dir',
               self.left_dir_callback,
               1
            )

       #self.right_dir_sub # Added to prevent unused variable warning (remove this?)
       #self.left_dir_sub # Added to prevent unused variable warning

       # Create publisher for /right_ticks and /left_wheel topics of type Tick
       self.right_tick_pub = self.create_publisher(Tick, 'right_ticks', 1)
       self.left_tick_pub = self.create_publisher(Tick, 'left_ticks', 1)
       timer_period = 0.5 # seconds
       self.timer = self.create_timer(timer_period, self.timer_callback)

    def right_dir_callback(self, msg):
        global right_wheel_direction
        right_wheel_direction = msg.data

    def left_dir_callback(self, msg):
        global left_wheel_direction
        left_wheel_direction = msg.data

    def timer_callback(self):
        global right_wheel_pulse_count, left_wheel_pulse_count
        
        # Initialize tick messages
        right = Tick()
        left = Tick()
        
        # Assign wheel pulse counts to tick messages
        right.tick = right_wheel_pulse_count
        left.tick = left_wheel_pulse_count
        
        # Publish tick messages to /right_ticks and /left_ticks topics
        self.right_tick_pub.publish(right)
        self.left_tick_pub.publish(left)


# Handles CTRL+C to shutdown the program
def signal_handler(sig, frame):
    # Clean up GPIO pins
    GPIO.cleanup()

    # Destroy node
    print('\nencoder_node destroyed')
    encoder_node.destroy_node()
    
    # Shutdown ROS python client
    rclpy.shutdown()
    
    # Exit python interpreter
    sys.exit(0)


if __name__ == '__main__':
    #Initialize signal module
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize ROS python client
    rclpy.init()

    # Create encoder node
    encoder_node = Encoder('encoder_node')
    
    # Spins node to call callback function    
    rclpy.spin(encoder_node)

    # Pause program indefinitely until the next interrupt
    signal.pause()



# Change the summary of the program on top
# Change message type to int16 if there's a need for it like Addison did.
