#!/usr/bin/python3

'''
    Script to test the motors.
'''

from real_lidarbot.motor import Motor
import time

if __name__ == '__main__':
    # Initialize motor driver
    motor = Motor()

    print("Testing motors:\n")

    print("Forward for 2 seconds")
    motor.MotorRun(0, 'forward', 50)
    motor.MotorRun(1, 'forward', 50)
    time.sleep(2)

    print("Backward for 2 seconds")
    motor.MotorRun(0, 'backward', 50)
    motor.MotorRun(1, 'backward', 50)
    time.sleep(2)

    print("Stop motors!")
    motor.MotorStop(0)
    motor.MotorStop(1)