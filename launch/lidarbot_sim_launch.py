# Launch simulation of lidarbot in Rviz and Gazebo
# File adapted from https://automaticaddison.com

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Set the path to different files and folders
    pkg_share = FindPackageShare(package='real_lidarbot').find('real_lidarbot')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'models/lidarbot.urdf.xacro')
    world_filename = 'obstacles.world'
    world_path = os.path.join(pkg_share, 'worlds', world_filename)

    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joystick = LaunchConfiguration('use_joystick')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_joystick_cmd = DeclareLaunchArgument(
        name='use_joystick',
        default_value='False',
        description='Whether to run joystick node')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model to load')
    
    # Specify the actions
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])
    
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # Start gazebo with the launch file provided by the gazebo_ros package
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]))
    
    # Run spawner node from gazebo_ros package to spawn lidarbot in gazebo
    start_spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                    '-entity', 'lidarbot'],
                    output='screen')

    # Launch the inbuilt ros2 joy node
    start_joy_node_cmd = Node(
        condition=IfCondition(use_joystick),
        package='joy',
        executable='joy_node',
        name='joy_node')

    # Launch joystick_pad_node 
    start_joystick_cmd =  Node(
        condition=IfCondition(use_joystick),
        package='twd_lidar_robot',
        executable='joystick_pad_node.py',
        name='joystick_pad_node')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_joystick_cmd)
    # ld.add_action(declare_world_cmd)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawn_entity_cmd)
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_joystick_cmd)
    
    return ld