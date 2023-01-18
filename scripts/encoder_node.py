#!/usr/bin/python3

'''
    The encoder node subscribes to topics /right_motor_dir and /left_motor_dir in order to 
    publish to the /ticks topic which contains message fields for right and left motor pulses/ticks.
'''

import signal
import sys
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
left_wheel_direction = 'forward'
right_wheel_direction = 'forward'

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

       # Create subscription to /right_motor_dir topic
       self.right_dir_sub = self.create_subscription(
               String,
               'right_motor_dir',
               self.right_dir_callback,
               1 
            )

       # Create subscription to /left_motor_dir topic
       self.left_dir_sub = self.create_subscription(
               String,
               'left_motor_dir',
               self.left_dir_callback,
               1
            )

       # Create a publisher for the /ticks topic, which holds both left and right ticks, of type Tick
       self.ticks_pub = self.create_publisher(Tick, 'ticks', 1)
       timer_period = 0.05 # seconds
       self.timer = self.create_timer(timer_period, self.timer_callback)

    def right_dir_callback(self, msg):
        '''Assign the right wheel direction obtained from the /right_motor_dir topic. '''
        global right_wheel_direction
        right_wheel_direction = msg.data

    def left_dir_callback(self, msg):
        ''' Assign the left wheel direction obtained from the /left_motor_dir topic. '''
        global left_wheel_direction
        left_wheel_direction = msg.data

    def timer_callback(self):
        '''
            This function executes every 0.05 seconds to assign the wheel pulse counts to a tick 
            message and publishes this message to the /ticks topic.
        '''
        global right_wheel_pulse_count, left_wheel_pulse_count
        
        # Initialize tick message
        ticks = Tick()
        
        # Assign wheel pulse counts to tick message
        ticks.right_tick = right_wheel_pulse_count
        ticks.left_tick = left_wheel_pulse_count

        # Publish tick message to /ticks topic
        self.ticks_pub.publish(ticks)

# Handles CTRL+C to shutdown the program
def signal_handler(sig, frame):
    # Clean up GPIO pins
    GPIO.cleanup()

    rclpy.logging.get_logger("KeyboardInterrupt, destroying").info(encoder_node.get_name())

    # Destroy node
    encoder_node.destroy_node()
    
    # Shutdown ROS python client
    rclpy.shutdown()
    
    # Exit python interpreter
    sys.exit(0)

if __name__ == '__main__':
    # Initialize signal module
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize ROS python client
    rclpy.init()

    # Create encoder node
    encoder_node = Encoder('encoder_node')
    
    # Spins node for callback function    
    rclpy.spin(encoder_node)

    # Pause program indefinitely until the next interrupt
    signal.pause()