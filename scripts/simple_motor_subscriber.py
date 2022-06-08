#!/usr/bin/python3

#--- Import packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from real_lidarbot import PCA9685

#--- Motor driver setup
pwm = PCA9685.PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

#--- Motor driver class
class MotorDriver():
    def __init__(self):
        #
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    # Method to run a motor
    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == 'forward'):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == 'forward'):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    # Method to stop one motor
    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)
        pass


#--- Subscriber class
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'wheel',
                self.listener_callback,
                10)
        self.subscription # To prevent unused variable warning
        self.motor = MotorDriver() # Initialize motor driver

    #
    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward':
            print('Moving forward...')
            self.motor.MotorRun(0, 'forward', 60)
            self.motor.MotorRun(1, 'forward', 60)
        elif command == 'backward':
            print('Moving backward...')
            self.motor.MotorRun(0, 'backward', 50)
            self.motor.MotorRun(1, 'backward', 50)
        elif command == 'left':
            print('Turning left...')
            self.motor.MotorRun(0, 'forward', 10)
            self.motor.MotorRun(1, 'forward', 60)
        elif command == 'right':
            print('Turning right...')
            self.motor.MotorRun(0, 'forward', 60)
            self.motor.MotorRun(1, 'forward', 10)
        elif command == 'stop':
            print('Stopping...')
            self.motor.MotorStop(0)
            self.motor.MotorStop(1)
        else:
            print('Unknown command, stopping motors instead')
            self.motor.MotorStop(0)
            self.motor.MotorStop(1)

#---
def main(args=None):
    # Initialize python client library
    rclpy.init(args=args)

    # Create subscriber node
    minimal_subscriber = MinimalSubscriber()

    # Wait for incoming commands
    rclpy.spin(minimal_subscriber)

    # Stop robot action and destroy node after an interrupt has been detected
    minimal_subscriber.motor.MotorStop(0)
    minimal_subscriber.motor.MotorStop(1)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# Check PCA9685 document for proper def/explanations for the initialized variables 

# Can we destroy the node better that first shutdowns the wheels; simple elegant solution

# Comments, comments!
