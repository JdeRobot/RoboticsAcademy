from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():

    kobuki_launch_file_path = os.path.join(get_package_share_directory('kobuki_launch'), 'launch')

    kobuki_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_launch_file_path, 'kobuki_base.launch.py')
        )
    )

    camera_node = Node(
        package='usb_cam',
        namespace='camera_node',
        executable='usb_cam_node_exe',
        name='camera'
    )

    rplidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(kobuki_launcher)
    ld.add_action(rplidar_node)
    ld.add_action(camera_node)

    return ld