from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI
import rclpy
import numpy as np
from sensor_msgs_py import point_cloud2
from threading import Lock
from builtin_interfaces.msg import Time


class LidarData:
    def __init__(self):
        self.points = []
        self.intensities = []
        self.timeStamp = 0.0
        self.min_range = 0.1
        self.max_range = 15.0
        self.field_of_view = (2 * PI / 3, PI / 18)
        self.is_dense = True

    def __str__(self):
        return (
            f"LiDARData:\n"
            f"  timestamp: {self.timeStamp}\n"
            f"  points: {len(self.points)}\n"
            f"  range: [{self.min_range}, {self.max_range}]\n"
            f"  FOV: {self.field_of_view}\n"
            f"  is_dense: {self.is_dense}"
        )


def pointCloud2LidarData(cloud):
    lidar = LidarData()
    if not cloud or cloud.width * cloud.height == 0:
        return lidar

    # Read XYZ points
    lidar.points = list(
        point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=False)
    )

    # Read intensities if available
    if "intensity" in [f.name for f in cloud.fields]:
        intensities = point_cloud2.read_points(
            cloud, field_names=("intensity",), skip_nans=False
        )
        lidar.intensities = [i[0] for i in intensities]

    # Timestamp (ROS 2 Time -> seconds)
    lidar.timeStamp = (
        Time(sec=cloud.header.stamp.sec, nanosec=cloud.header.stamp.nanosec).nanoseconds
        / 1e9
    )

    # Validate point cloud
    lidar.is_dense = all(
        all(np.isfinite(coord) for coord in point) for point in lidar.points
    )

    return lidar


class LidarNode(Node):
    def __init__(self, topic):
        super().__init__("lidar_node")
        self._lock = Lock()
        self.last_cloud_ = None
        self.sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, topic, self.pointcloud_callback, 10
        )

    def pointcloud_callback(self, cloud):
        with self._lock:
            self.last_cloud_ = cloud

    def getLidarData(self):
        with self._lock:
            return pointCloud2LidarData(self.last_cloud_)

    def get_point_cloud(self):
        with self._lock:
            return self.last_cloud_
