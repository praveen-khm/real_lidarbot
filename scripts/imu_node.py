#!/usr/bin/python3

'''
    This imu node publishes accelerometer and gyroscope data, obtained from the MPU6050 accelerometer/gyroscope 
    module, to the 'imu/data' topic. 
'''

import rclpy
from rclpy.node import Node
from real_lidarbot.mpu6050 import mpu6050
from sensor_msgs.msg import Imu

# IMU class inheriting from the Node class
class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.get_logger().info(self.get_name() + " is initialized")
        
        # Initialize MPU6050 object
        self.mpu = mpu6050(0x68)

        # Create publisher for 'imu/data' topic of type Imu
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)

        timer_period = 0.05 # seconds

        # Create a timer with a callback to be executed every 0.05 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''
        This method is called every 0.05 seconds to access the accelerometer and gyroscope/angular velocity 
        data from MPU6050 and publish the data to the 'imu/data' topic. 

        Acceleromter data unit - m/s^2
        Gyroscope data unit   - rad/sec

        '''

        # Initialize Imu message
        msg = Imu()

        accel_data = self.mpu.get_accel_data() # Obtain accelerometer data from MPU6050
        gyro_data = self.mpu.get_gyro_data()   # Obtain gyroscope data from MPU6050

        # Access acceleration data
        msg.linear_acceleration.x = accel_data['x']
        msg.linear_acceleration.y = accel_data['y']
        msg.linear_acceleration.z = accel_data['z']
        
        # Access gyroscope data
        msg.angular_velocity.x = gyro_data['x']
        msg.angular_velocity.y = gyro_data['y']
        msg.angular_velocity.z = gyro_data['z']
        
        msg.header.frame_id = 'imu_link'       # Set transform frame with which this message is associated

        # Publish imu message to 'imu/data' topic
        self.imu_publisher.publish(msg)

        # Log data to the console for debugging purposes
        self.get_logger().info("Accel x: " + str(msg.linear_acceleration.x) + 
                                " y: " + str(msg.linear_acceleration.y) +
                                " z: " + str(msg.linear_acceleration.z) +
                                " Gyro x: " + str(msg.angular_velocity.x) +
                                " y: " + str(msg.angular_velocity.y) +
                                " z: " + str( msg.angular_velocity.z)
                                )

if __name__ == '__main__':
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create imu_node node
        imu_node = ImuNode()

        # Wait for incoming 'messages/callbacks'
        rclpy.spin(imu_node)

    # On Ctrl+C executed in the terminal
    except KeyboardInterrupt:
        # Destroy node
        imu_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()


# TO-DO:
# Adjust timer period later
# Disable data logging to console?
