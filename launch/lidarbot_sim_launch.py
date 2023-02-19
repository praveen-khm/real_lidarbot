# Launch simulation of lidarbot in Rviz and Gazebo
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Set the path to different files and folders
    pkg_share = FindPackageShare(package='real_lidarbot').find('real_lidarbot')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/view_lidarbot.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'models/lidarbot.urdf.xacro')
    world_filename = 'obstacles.world'
    world_path = os.path.join(pkg_share, 'worlds', world_filename)

    # Launch configuration variables specific to simulation
    # headless = LaunchConfiguration('headless')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
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
    
    # declare_simulator_cmd = DeclareLaunchArgument(
    #     name='headless',
    #     default_value='False',
    #     description='Whether to execute gzclient')
    
    declare_joystick_cmd = DeclareLaunchArgument(
        name='use_joystick',
        default_value='True',
        description='Whether to run joystick node')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator', 
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model to load')
    
    # Specify the actions

    # Start Gazebo server
    # start_gazebo_server_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    #     condition=IfCondition(use_simulator),
    #     launch_arguments={'world': world}.items())
    
    # # Start Gazebo client    
    # start_gazebo_client_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    #     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'robot_state_publisher_launch.py')]), 
        launch_arguments={'use_sim_time': use_sim_time, 'urdf_model': urdf_model}.items())

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    
    # Launch Gazebo 
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())   

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        condition=IfCondition(use_simulator),
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lidarbot'])
    
    # Launch the inbuilt ros2 joy node
    start_joy_node_cmd = Node(
        condition=IfCondition(use_joystick),
        package='joy',
        executable='joy_node',
        name='joy_node')

    # Launch joystick_pad_node 
    start_joystick_cmd =  Node(
        condition=IfCondition(use_joystick),
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_simulator_cmd) 
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_joystick_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_joystick_cmd)
    
    return ld