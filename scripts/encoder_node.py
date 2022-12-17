#!/usr/bin/python3

'''
    This encoder node publishes the right and left wheel encoder pulses/ticks to topics /sdfasd and /asdfasd respectively.
'''

import signal
import sys
import time
import RPi.GPIO as GPIO
from real_lidarbot.msg import Tick
from real_lidarbot.PCA9685 import PCA9685
import rclpy
from rclpy.node import Node

# GPIO Pins
LEFT_WHL_ENCODER = 25
RIGHT_WHL_ENCODER = 24

# Pulse counters
left_wheel_pulse_count = 0
right_wheel_pulse_count = 0

# Default wheel directions
left_wheel_direction = 'backward'
right_wheel_direction = 'backward'

direction = ['forward','backward']  # 'list of wheel direction options'

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
    #print(left_wheel_direction)

# Right wheel callback function
def right_wheel_pulse(channel):
    global right_wheel_pulse_count, right_wheel_direction

    if right_wheel_direction == 'forward':
        right_wheel_pulse_count += 1
    else:
        right_wheel_pulse_count -= 1
    print('Right: ', right_wheel_pulse_count)
    #print(right_wheel_direction)

# Handles CTRL+C to shutdown the program
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

# Initialize encoder interrupts for falling signal states
GPIO.add_event_detect(LEFT_WHL_ENCODER, GPIO.FALLING, callback=left_wheel_pulse)
GPIO.add_event_detect(RIGHT_WHL_ENCODER, GPIO.FALLING, callback=right_wheel_pulse)

# Setup Motor Driver...
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    #
    def MotorRun(self, motor, index, speed):
        global right_wheel_direction, left_wheel_direction
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            left_wheel_direction = index

            if(index == direction[0]):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            right_wheel_direction = index

            if(index == direction[0]):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    #
    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)

#
class Encoder(Node):
    def __init__(self):
       super().__init__('encoder_node')
       self.get_logger().info(self.get_name() + ' is initialized')

       # any class attributes?

       # Create publisher for /right_ticks and /left_wheel topics of type Tick
       self.right_pub = self.create_publisher(Tick, 'right_ticks', 1)
       self.left_pub = self.create_publisher(Tick, 'left_ticks', 1)
       timer_period = 0.5 # seconds
       self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global right_wheel_pulse_count, left_wheel_pulse_count
        
        #
        right = Tick()
        left = Tick()
        
        #
        right.tick = right_wheel_pulse_count
        left.tick = left_wheel_pulse_count
        
        #
        self.right_pub.publish(right)
        self.left_pub.publish(left)

    # Create a subscriber to motor driver or /joy topic

if __name__ == '__main__':
    try:
        # Initialize signal module
        signal.signal(signal.SIGINT, signal_handler)

        # 'Initialize MotorDriver instance' 
        Motor = MotorDriver()

        # Initialize ROS python client
        rclpy.init()

        # Create encoder node
        encoder_node = Encoder()
        
        # Motor tests
        print("Motor driver tests:")
        print("forward 2 s")
        Motor.MotorRun(0, 'forward', 60)
        Motor.MotorRun(1, 'forward', 60)
        time.sleep(2)

        print("backward 2 s")
        Motor.MotorRun(0, 'backward', 60)
        Motor.MotorRun(1, 'backward', 60)
        time.sleep(2)

        print('Move left forwards and right backwards')
        Motor.MotorRun(0, 'forward', 60)
        Motor.MotorRun(1, 'backward', 60)
        time.sleep(2)

        print("stop")
        Motor.MotorStop(0)
        Motor.MotorStop(1)

        #
        rclpy.spin(encoder_node)

        # Pause program indefinitely until the next interrupt
        signal.pause()

    except IOError as e:
        print(e)

    except KeyboardInterrupt:
        # Destroy node
        encoder_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

# Way to minimize the use of global variables?
# Rename variables (if needed) later
# Finalize on name of the node and file
# Change the summary of the program on top
# PWMA and PWMB control speed of motors, AIN1 and AIN2, BIN1 and BIN2 control rotate direction of motors.
# A1 and A2, B1 and B2 are connect to positive/negative poles of two motors separately.
# Change message type to int16 if there's a need for it like Addison did.
