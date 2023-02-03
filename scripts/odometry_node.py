#!/usr/bin/python3

'''
    The odometry node subscribes to the /ticks and /joy topics, to calculate the robot's pose from the motor ticks/pulses
    and reset the pose, respectively.

    The node also publishes the robot's pose to the /odom topic.
'''

from time import process_time
from math import pi, cos, sin, degrees

from real_lidarbot.msg import Tick
from real_lidarbot.utils import quaternion_from_euler

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped

from tf2_ros import TransformBroadcaster

# Odometry class
class OdometryNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(self.get_name() + ' is initialized')
        
        # Instance variables 
        self.radius = 0.0335                                                # Wheel radius [metres]
        self.wheel_base = 0.134                                             # Distance between wheels [metres]
        self.ticks_per_rev = 20                                             # Encoder ticks per revolution
        self.metres_per_tick = (2*pi*self.radius)/self.ticks_per_rev        # Wheel distance moved per tick
        self.delta_right_tick = 0                                           # Change in right ticks
        self.delta_left_tick  = 0                                           # Change in left ticks
        self.prev_right_tick = 0                                            # Previous right tick value
        self.prev_left_tick = 0                                             # Previous left tick value
        self.robot_dist = 0                                                 # Distance traveled by robot [metres]

        self.start_time = process_time()                                    # Get current time [seconds]

        # Pose information
        self.x = 0                                                          # Robot x coordinate [metres]
        self.y = 0                                                          # Robot y coordinate [metres]
        self.theta = 0                                                      # Robot angle [radians]
        self.delta_x = 0                                                    # Change in x coordinate
        self.delta_y = 0                                                    # Change in y coordinate
        self.delta_theta = 0                                                # Change in angle

        # Message initializations
        self.tf_broadcaster = TransformBroadcaster(self)                    
        self.tf_stamped = TransformStamped()
        self.odom = Odometry()
        self.joint_state = JointState()

        # Create a subscription to /ticks topic of type Tick
        self.ticks_sub = self.create_subscription(
                Tick, 
                'ticks',
                self.ticks_callback,
                10
            )

        # Create a subscription to /joy topic of type Joy
        self.joy_sub = self.create_subscription(
                Joy,
                'joy',
                self.reset_pose_callback,
                1
            )

        # Create a subscription to /initial_2d topic of type PoseStamped
        self.initial_2d_sub = self.create_subscription(
                PoseStamped,
                'initial_2d',
                self.set_initial_2d_callback,
                1
            )
        
        # Create a publisher to /odom topic
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Create a publisher to /joint_states topic 
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def set_initial_2d_callback(self, rvizClick):
        ''' '''
        self.x = rvizClick.pose.position.x
        self.y = rvizClick.pose.position.y
        self.theta = rvizClick.pose.orientation.z

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
        right_wheel_dist = self.metres_per_tick * self.delta_right_tick     # Distance traveled by right wheel
        left_wheel_dist = self.metres_per_tick * self.delta_left_tick       # Distance traveled by left wheel
        self.robot_dist = (left_wheel_dist + right_wheel_dist) / 2              
        
        # Calculate pose changes
        self.delta_x = self.robot_dist* cos(self.theta)
        self.delta_y = self.robot_dist * sin(self.theta)
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

    def odom_data_publisher(self): # Change function name and description
        ''' Publish pose data to /odom topic. '''

        # Fill out TransformStamped message fields
        now = self.get_clock().now()
        self.tf_stamped.header.stamp = now.to_msg()
        self.tf_stamped.header.frame_id = 'odom'
        self.tf_stamped.child_frame_id = 'base_link'
        self.tf_stamped.transform.translation.z = 0.0
        self.tf_stamped.transform.translation.x = self.x
        self.tf_stamped.transform.translation.y = self.y

        # Create quaternion from theta value and assign to tf_stamped rotation variable
        q = quaternion_from_euler(0, 0, self.theta)
        quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 
        self.tf_stamped.transform.rotation = quat

        # Fill out Odometry message field
        self.odom.header.stamp = now.to_msg()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.z = 0.0
        self.odom.twist.twist.linear.y = 0.0 
        self.odom.twist.twist.linear.z = 0.0 
        self.odom.twist.twist.angular.x = 0.0 
        self.odom.twist.twist.angular.y = 0.0 
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.orientation = quat

        # Calculate velocities
        stop_time = process_time() 
        self.odom.twist.twist.linear.x = self.robot_dist / (stop_time - self.start_time)
        self.odom.twist.twist.angular.z = self.delta_theta / (stop_time - self.start_time)

        # Set start_time for next callback
        self.start_time = stop_time

        # Publish odom message to /odom
        self.odom_pub.publish(self.odom)

        # Fill out JointState message fields
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state.velocity = [self.odom.twist.twist.angular.z, self.odom.twist.twist.angular.z] # angular or linear?
        # self.joint_state.position = [self.odom.pose.pose.position, self.odom.pose.pose.position]

        # Send the joint state and transformation
        self.joint_pub.publish(self.joint_state)
        self.tf_broadcaster.sendTransform(self.tf_stamped)


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

# TODO:
# Review comments and descriptions