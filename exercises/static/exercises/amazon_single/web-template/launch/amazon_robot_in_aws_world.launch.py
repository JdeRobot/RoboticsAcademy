#!/usr/bin/env python3
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified by:
# Shreyas Gokhale <shreyas6gokhale@gmail.com>

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Use Simulation time from gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Brinup Package locations for nav2
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # Our bringup location
    amazon_gazebo_package_dir = get_package_share_directory('amazon_robot_gazebo')
    amazon_gazebo_package_launch_dir= os.path.join(amazon_gazebo_package_dir, 'launch')
    
    amazon_description_dir = get_package_share_directory('amazon_robot_description')
    this_launch_dir = os.path.dirname(os.path.realpath(__file__))

    amazon_bringup_package_dir = get_package_share_directory('amazon_robot_bringup')


    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    # We will not spawn the robot
    spawn_robot = True

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    # Declare the launch arguments
    # Useful with multi robot
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    # Map
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amazon_bringup_package_dir, 'maps', 'aws_warehouse' ,'map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Navigation 2 parameters
    # Change accordingly
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(amazon_bringup_package_dir, 'params', 'nav2_params_with_control.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Navigation 2 default behaviour tree. Might need to change in the future
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(amazon_bringup_package_dir, 'rviz', 'amazon_robot_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    # Our own gazebo world from CustomRobots
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=os.path.join(aws_dir, 'worlds', 'no_roof_small_warehouse.world'),
        description='Full path to world model file to load')


    # Default Nav2 actions
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s' , 'libgazebo_ros_factory.so', '-s' , 'libgazebo_ros_force_system.so' , world],
        cwd=[this_launch_dir], output='screen')


    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[this_launch_dir], output='screen')

    # Our own robot urdf from CustomRobot
    urdf = os.path.join(amazon_description_dir, 'urdf', 'amazon_robot_xacro_generated.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amazon_bringup_package_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())
    
    if spawn_robot is True:
        spawn_robot_cmd = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(amazon_bringup_package_dir, 'launch',
                                                            'spawn_tb3_launch.py')),
                    launch_arguments={
                                    'x_pose': '0',
                                    'y_pose': '0',
                                    'z_pose': '0',
                                    'robot_name': 'amazon_robot'
                                    }.items())


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    if spawn_robot is True:
        ld.add_action(spawn_robot_cmd)


    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld