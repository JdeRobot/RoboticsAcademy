from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    camera_node = Node(
    package='usb_cam',
    namespace='camera_node',
    executable='usb_cam_node_exe',
    name='camera'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(camera_node)

    return ld