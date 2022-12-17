#!/usr/bin/python3

from real_lidarbot.PCA9685 import PCA9685
import time

#
direction = ['forward','backward']

#
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class Motor():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
    
    #
    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return

        # Left motor
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == direction[0]):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        # Right motor
        else:
            pwm.setDutycycle(self.PWMB, speed)
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
