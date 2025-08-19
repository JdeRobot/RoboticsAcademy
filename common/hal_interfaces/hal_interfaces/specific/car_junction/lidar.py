from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI
import rclpy
import numpy as np
from sensor_msgs_py import point_cloud2  

if not rclpy.ok():
    rclpy.init()

### AUXILIARY FUNCTIONS
class LidarData:
    def __init__(self):
        self.points = []  # List of (x,y,z) coordinates in meters
        self.intensities = []  # List of intensity values
        self.timestamps = []  # Individual timestamps if available
        self.timeStamp = 0  # Main timestamp (seconds)
        self.min_range = 0  # Minimum valid range (meters)
        self.max_range = 0  # Maximum valid range (meters)
        self.field_of_view = (0, 0)  # (horizontal_fov, vertical_fov) in radians
        self.is_dense = True  # Whether the cloud contains NaN/Inf points

    def __str__(self):
        s = "LiDARData: {\n"
        s += f"   timeStamp: {self.timeStamp}\n"
        s += f"   min_range: {self.min_range}\n"
        s += f"   max_range: {self.max_range}\n"
        s += f"   field_of_view: {self.field_of_view}\n"
        s += f"   num_points: {len(self.points)}\n"
        s += f"   is_dense: {self.is_dense}\n"
        s += "}"
        return s

def pointCloud2LidarData(cloud):
    
    lidar = LidarData()
    
    # Read all points from the cloud
    lidar.points = list(point_cloud2.read_points(
        cloud, 
        field_names=("x", "y", "z"), 
        skip_nans=False
    ))
    try:
        lidar.intensities = list(point_cloud2.read_points(
            cloud, 
            field_names=("intensity",), 
            skip_nans=False
        ))
    except:
        pass
    
    lidar.timeStamp = cloud.header.stamp.sec + (cloud.header.stamp.nanosec * 1e-9)
    
    lidar.field_of_view = (2*PI/3, PI/18)  

    lidar.min_range = 0.1  # minimum range
    lidar.max_range = 15.0  # maximum range
    
    lidar.is_dense = not any(
        any(not np.isfinite(coord) for coord in point)
        for point in lidar.points
    )
    
    return lidar

### LiDAR INTERFACE ###
class LidarNode(Node):
    def __init__(self, topic):
        super().__init__("lidar_node")
        self.sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, 
            topic, 
            self.pointcloud_callback, 
            10
        )
        self.last_cloud_ = None

    def pointcloud_callback(self, cloud):
        self.last_cloud_ = cloud

    def getLidarData(self):
        if self.last_cloud_ is None:
            return LidarData()
        return pointCloud2LidarData(self.last_cloud_)

    def get_point_cloud(self):
        return self.last_cloud_