from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI
import rclpy
import numpy as np
from sensor_msgs_py import point_cloud2
from threading import Lock


class LidarData:
    """3D LiDAR reading with XYZ points, intensities and sensor limits."""
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
    """
    Convert a ROS PointCloud2 message to a LidarData object.

    Args:
        cloud: sensor_msgs/PointCloud2 message, or None.

    Returns:
        LidarData with the XYZ points (NaNs kept), intensities if the cloud
        has that field, and is_dense set to False if any coordinate is not
        finite. An empty LidarData is returned for a missing or empty cloud.
    """
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
    lidar.timeStamp = cloud.header.stamp.sec + (cloud.header.stamp.nanosec * 1e-9)

    # Validate point cloud
    lidar.is_dense = all(
        all(np.isfinite(coord) for coord in point) for point in lidar.points
    )

    return lidar


class LidarNode(Node):
    """
    ROS2 node that subscribes to a PointCloud2 topic and stores the latest cloud.

    Access to the stored cloud is guarded by a lock because the callback and
    the user code run on different threads.
    """
    def __init__(self, topic):
        super().__init__("lidar_node")
        self._lock = Lock()
        self.last_cloud_ = None
        self.sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, topic, self.pointcloud_callback, 10
        )

    def pointcloud_callback(self, cloud):
        """Store the latest PointCloud2 message received from the topic."""
        with self._lock:
            self.last_cloud_ = cloud

    def getLidarData(self):
        """
        Return the latest LiDAR reading.

        Returns:
            LidarData built from the last cloud, empty if none was received.
        """
        with self._lock:
            return pointCloud2LidarData(self.last_cloud_)

    def get_point_cloud(self):
        """
        Return the raw latest point cloud.

        Returns:
            The last sensor_msgs/PointCloud2 message, or None if none was received.
        """
        with self._lock:
            return self.last_cloud_
