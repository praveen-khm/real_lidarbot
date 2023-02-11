import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Process URDF file
    pkg_path = FindPackageShare(package='real_lidarbot').find('real_lidarbot')
    urdf_model_path = os.path.join(pkg_path, 'models/lidarbot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) time if true')
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='False',
        description='Use ros2_control if true'
    )
    
    # Launch config variables
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Robot state publisher parameter values
    # robot_description_config = Command([urdf_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # robot_description_config = Command(['xacro', urdf_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # robot_description_config = Command(['xacro', urdf_file])
    # params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Start robot state publisher
    # start_robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params])
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[urdf_model_path])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_use_ros2_control_cmd)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    
    return ld