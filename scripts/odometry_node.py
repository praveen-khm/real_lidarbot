#!/usr/bin/python3

'''
    The odometry node subscribes to the /ticks and /joy topics, to calculate the robot's pose and reset the
    pose, respectively.

    The node also publishes the robot's pose to the /odom_data topic.
'''

from math import pi, cos, sin, degrees
from real_lidarbot.msg import Tick
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# Odometry class
class OdometryNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # Instance variables 
        self.radius = 0.0335                                          # Wheel radius [metres]
        self.wheel_base = 0.134                                       # Distance between wheels [metres]
        self.ticks_per_rev = 20                                       # Encoder ticks per revolution
        self.metres_per_tick = (2*pi*self.radius)/self.ticks_per_rev  # Wheel distance moved per tick
        self.delta_right_tick = 0                                     # Change in right ticks
        self.delta_left_tick  = 0                                     # Change in left ticks
        self.prev_right_tick = 0                                      # Previous right tick value
        self.prev_left_tick = 0                                       # Previous left tick value

        # Pose information
        self.x = 0                                                    # Robot x coordinate [metres]
        self.y = 0                                                    # Robot y coordinate [metres]
        self.theta = 0                                                # Robot angle [radians]
        self.delta_x = 0                                              # Change in x coordinate
        self.delta_y = 0                                              # Change in y coordinate
        self.delta_theta = 0                                          # Change in angle

        # Create a subscription to /ticks topic of type Tick
        self.ticks_sub = self.create_subscription(
                Tick, 
                'ticks',
                self.ticks_callback,
                1
            )

        # Create a subscription to /joy topic of type Joy
        self.joy_sub = self.create_subscription(
                Joy,
                'joy',
                self.reset_pose_callback,
                1
            )
        
        # Create a publisher to /odom_data topic
        self.odom_pub = self.create_publisher(Odometry, 'odom_data', 2)

    def reset_pose_callback(self, msg):
        '''
            The pose of the robot is reset after the 'X' button on the joy gamepad controller is pressed. 
            This helps to prevent restarting the odometry_node every time the odometry data is used to tune 
            controller gains for the go_to_goal_node for instance.
            
            The 'X' button corresponds to buttons[3] on the joystick map, with a value of 1 when 
            pressed and 0 otherwise.
        '''
        if msg.buttons[3] == 1:
            self.x = 0
            self.y = 0
            self.theta = 0

    def ticks_callback(self, msg):
        ''' This function uses the wheel tick data to calculate the robot's new pose and publishes it. '''

        # Compute changes in wheel ticks
        self.delta_right_tick = msg.right_tick - self.prev_right_tick
        self.delta_left_tick = msg.left_tick - self.prev_left_tick

        # Update ticks for next callback
        self.prev_right_tick = msg.right_tick
        self.prev_left_tick = msg.left_tick

        # Calculate distances
        right_wheel_dist = self.metres_per_tick * self.delta_right_tick # Distance traveled by right wheel
        left_wheel_dist = self.metres_per_tick * self.delta_left_tick   # Distance traveled by left wheel
        centre_dist = (left_wheel_dist + right_wheel_dist) / 2          # Distance traveled by robot
        
        # Calculate pose changes
        self.delta_x = centre_dist * cos(self.theta)
        self.delta_y = centre_dist * sin(self.theta)
        self.delta_theta = (right_wheel_dist - left_wheel_dist) / self.wheel_base
        
        # New pose
        self.x += self.delta_x
        self.y += self.delta_y
        self.theta += self.delta_theta

        # Convert from radians to degrees
        theta_d = degrees(self.theta)

        # Log data to console
        self.get_logger().info("x: %.3f" % (self.x) + 
                                " y: %.3f" % (self.y) +
                                " theta: %.2f" % (theta_d)
                                )

        # Publish pose values
        self.odom_data_publisher()

    def odom_data_publisher(self):
        ''' Publish pose data to /odom_data topic. '''

        # Initialize odometry message
        odom = Odometry()
        
        # Assign pose data to odom message
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = self.theta

        # Publish odom message to /odom_data
        self.odom_pub.publish(odom)

# Main function
def main():
    try:
        # Initialize ROS python client
        rclpy.init()

        # Create odometry node
        odometry_node = OdometryNode('odometry_node')

        # Spin node for callback function
        rclpy.spin(odometry_node)

    # On executing Ctrl+C in the terminal
    except KeyboardInterrupt:
        rclpy.logging.get_logger("KeyboardInterrupt, destroying").info(odometry_node.get_name())

        # Destroy node
        odometry_node.destroy_node()

        # Shutdown ROS python client
        rclpy.shutdown()

if __name__ == '__main__':
    main()