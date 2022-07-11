import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
	
	pkg_kobuki_node = FindPackageShare(package='kobuki_node').find('kobuki_node')
	pkg_laser_node = FindPackageShare(package='rplidar_ros').find('rplidar_ros')
	
	# camera node
	v4l2_camera_node = ExecuteProcess(
	    cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node', '--ros-args', '-p', 'video_device:="/dev/video0"'], output='screen')
	
	# Kobuki Base launch file
	kobuki_launch = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(os.path.join(pkg_kobuki_node, 'launch', 'kobuki_node-launch.py'))
	)
	
	# RPLIDAR launch file
	rplidar_launch = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(os.path.join(pkg_laser_node, 'launch', 'rplidar.launch.py'))
	)
	
	ld = LaunchDescription()
	
	ld.add_action(v4l2_camera_node)
	ld.add_action(kobuki_launch)
	ld.add_action(rplidar_launch)
	
	return ld
