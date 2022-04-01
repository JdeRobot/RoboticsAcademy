# Author: Addison Sears-Collins
# Date: September 19, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com

# Updated by: Carlos Caminero (Carlosalpha1)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

  # Set the path to the Turtlebot2 ROS package
  pkg_turtlebot2 = FindPackageShare(package='turtlebot2').find('turtlebot2')

  # Set Turtlebot2 Arguments
  x_turtlebot2_position = '0'
  y_turtlebot2_position = '10'
  declare_x_position_cmd = DeclareLaunchArgument(
    '-x', default_value=x_turtlebot2_position,
    description="Position on the axis x of Turtlebot2"
  )
  declare_y_position_cmd = DeclareLaunchArgument(
    '-y', default_value=y_turtlebot2_position,
    description="Position on the axis y of Turtlebot2"
  )

  # Set the path to this package.
  pkg_share = FindPackageShare(package='hospital_world').find('hospital_world')
 
  # Set the path to the world file
  world_file_name = 'hospital_follow_person.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
  # Set the path to the SDF model files.
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] += gazebo_models_path
 
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    
  # Specify the actions
   
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
  
  start_turtlebot2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot2, 'launch', 'spawn_model.launch.py')),
    launch_arguments = {'-x': x_turtlebot2_position, '-y': y_turtlebot2_position}.items()
  )
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_x_position_cmd)
  ld.add_action(declare_y_position_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_turtlebot2_cmd)
 
  return ld