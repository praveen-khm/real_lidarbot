#!/usr/bin/python3

# Program to count the number of wheel encoder ticks/pulses

import signal
import sys
import RPi.GPIO as GPIO

# GPIO Pins
LEFT_WHL_ENCODER = 25
RIGHT_WHL_ENCODER = 24

# Pulse counters
left_wheel_pulse_count = 0
right_wheel_pulse_count = 0

# Left wheel callback function
def left_wheel_pulse(channel):
    global left_wheel_pulse_count
    left_wheel_pulse_count += 1
    print('Left pulse: ', left_wheel_pulse_count)

# Right wheel callback function
def right_wheel_pulse(channel):
    global right_wheel_pulse_count
    right_wheel_pulse_count += 1
    print('Right pulse:', right_wheel_pulse_count)

# Handles CTRL+C to shutdown the program 
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)


if __name__ == '__main__':

    # Set mode for GPIO pins
    GPIO.setmode(GPIO.BCM)

    # Setup GPIO pins with internal pull up resistors activated
    GPIO.setup(LEFT_WHL_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_WHL_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Initialize encoder interrupts for falling signal states
    GPIO.add_event_detect(LEFT_WHL_ENCODER, GPIO.FALLING, callback=left_wheel_pulse)
    GPIO.add_event_detect(RIGHT_WHL_ENCODER, GPIO.FALLING, callback=right_wheel_pulse)

    # Initialize signal module
    signal.signal(signal.SIGINT, signal_handler)

    # Pause program indefinitely until the next interrupt
    signal.pause()
