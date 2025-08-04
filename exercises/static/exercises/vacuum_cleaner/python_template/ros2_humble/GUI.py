import json
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactsState

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from map import Map

# Try to import HAL functions (for manual mode)
try:
    from HAL import getPose3d as hal_getPose3d
    from HAL import getLaserData as hal_getLaserData  
    from HAL import getBumperData as hal_getBumperData
    HAL_AVAILABLE = True
except ImportError:
    HAL_AVAILABLE = False

class ROS2DataNode(Node):
    """ROS2 node to handle data subscriptions for auto mode"""
    def __init__(self):
        super().__init__('map_data_subscriber')
        
        # Data storage
        self.latest_pose = None
        self.latest_laser = None
        self.latest_bumper_state = 0
        self.contact_states = [ContactsState() for _ in range(3)]
        self.data_lock = threading.Lock()
        
        # Subscribers (will be created if topics exist)
        self.odom_sub = None
        self.laser_sub = None
        self.bumper_subs = []
        
    def setup_subscriptions(self, available_topics):
        """Set up subscriptions based on available topics"""
        if '/odom' in available_topics:
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            
        if '/roombaROS/laser/scan' in available_topics:
            self.laser_sub = self.create_subscription(
                LaserScan,
                '/roombaROS/laser/scan',
                self.laser_callback,
                10
            )
            
        # Set up bumper subscriptions
        bumper_topics = [
            '/roombaROS/events/right_bumper',
            '/roombaROS/events/center_bumper', 
            '/roombaROS/events/left_bumper'
        ]
        
        for i, topic in enumerate(bumper_topics):
            if topic in available_topics:
                sub = self.create_subscription(
                    ContactsState,
                    topic,
                    lambda msg, idx=i: self.bumper_callback(msg, idx),
                    10
                )
                self.bumper_subs.append(sub)
    
    def odom_callback(self, msg):
        with self.data_lock:
            self.latest_pose = msg.pose.pose
            
    def laser_callback(self, msg):
        with self.data_lock:
            self.latest_laser = msg
            
    def bumper_callback(self, msg, bumper_index):
        with self.data_lock:
            self.contact_states[bumper_index] = msg
            self.update_bumper_state()
            
    def update_bumper_state(self):
        self.latest_bumper_state = 0
        for contact in self.contact_states:
            if len(contact.states) > 0:
                self.latest_bumper_state = 1
                break
                
    def get_pose3d(self):
        """Convert ROS2 odometry to HAL-compatible pose"""
        with self.data_lock:
            if self.latest_pose is None:
                class DefaultPose:
                    x = 0.0
                    y = 0.0
                    yaw = 0.0
                return DefaultPose()
            
            import math
            pose = self.latest_pose
            
            # Convert quaternion to yaw
            siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + 
                           pose.orientation.x * pose.orientation.y)
            cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + 
                               pose.orientation.z * pose.orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            class Pose3D:
                def __init__(self, x, y, yaw):
                    self.x = x
                    self.y = y
                    self.yaw = yaw
            
            return Pose3D(pose.position.x, pose.position.y, yaw)

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {'map': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
            
        self.ros2_data_node = None
        self.auto_mode = False
        self._setup_auto_mode()
        
        # Create map with appropriate pose getter
        if self.auto_mode:
            self.map = Map(self.ros2_data_node.get_pose3d)
            self.get_logger_info = lambda msg: self.ros2_data_node.get_logger().info(msg)
        else:
            if HAL_AVAILABLE:
                self.map = Map(hal_getPose3d)
                self.get_logger_info = lambda msg: print(f"GUI: {msg}")
            else:
                # Fallback to dummy functions
                self.map = Map(self._dummy_pose_getter)
                self.get_logger_info = lambda msg: print(f"GUI (No HAL): {msg}")
        
        # Start ROS2 executor if in auto mode
        if self.auto_mode:
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.ros2_data_node)
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()

        self.start()

    def _setup_auto_mode(self):
        """Check for ROS2 topics and set up auto mode if available"""
        try:
            # Create temporary node to check topics
            temp_node = rclpy.create_node('topic_checker_temp')
            topic_names_and_types = temp_node.get_topic_names_and_types()
            topic_names = [topic_name for topic_name, _ in topic_names_and_types]
            
            # Check if key ROS2 topics exist
            required_topics = ['/odom']
            optional_topics = ['/roombaROS/laser/scan']
            
            has_required = any(topic in topic_names for topic in required_topics)
            
            if has_required:
                self.ros2_data_node = ROS2DataNode()
                self.ros2_data_node.setup_subscriptions(topic_names)
                self.auto_mode = True
                print("GUI: Auto mode enabled - ROS2 topics detected")
            else:
                print("GUI: Manual mode - No ROS2 topics found, using HAL")
                
            temp_node.destroy_node()
            
        except Exception as e:
            print(f"GUI: Failed to check ROS2 topics, falling back to manual mode: {e}")
            self.auto_mode = False

    def _dummy_pose_getter(self):
        """Dummy pose getter when neither ROS2 nor HAL is available"""
        class DummyPose:
            x = 0.0
            y = 0.0
            yaw = 0.0
        return DummyPose()

    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def reset_gui(self):
        self.map.reset()

    def get_map_mode(self):
        """Return information about current map mode"""
        return {
            'auto_mode': self.auto_mode,
            'topics_subscribed': ['/odom', '/roombaROS/laser/scan'] if self.auto_mode else None,
            'manual_mode_available': HAL_AVAILABLE,
            'hal_available': HAL_AVAILABLE
        }

# Global variables for GUI management
_gui = None
_gui_lock = threading.Lock()

def get_gui():
    global _gui
    with _gui_lock:
        if _gui is None:
            host = "ws://127.0.0.1:2303"
            _gui = GUI(host)
            start_console()
    return _gui

def get_map_mode():
    """Get information about current map mode"""
    gui = get_gui()
    if gui is not None:
        return gui.get_map_mode()
    return {
        'auto_mode': False, 
        'topics_subscribed': None, 
        'manual_mode_available': HAL_AVAILABLE,
        'hal_available': HAL_AVAILABLE
    }

# Use the singleton pattern
gui = get_gui()