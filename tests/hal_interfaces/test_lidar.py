import unittest
from unittest.mock import MagicMock, patch
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from hal_interfaces.specific.car_junction.lidar import (
    LidarData as CarJunctionLidarData,
    pointCloud2LidarData as carJunctionPointCloud2LidarData,
    LidarNode as CarJunctionLidarNode,
)
from hal_interfaces.general.lidar import (
    LidarData as GeneralLidarData,
    pointCloud2LidarData as generalPointCloud2LidarData,
    LidarNode as GeneralLidarNode,
)


def create_mock_pointcloud(sec=10, nanosec=500000000):
    cloud = PointCloud2()
    cloud.width = 2
    cloud.height = 1
    cloud.fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 24
    cloud.data = bytes(24)
    cloud.header.stamp.sec = sec
    cloud.header.stamp.nanosec = nanosec
    return cloud


class TestCarJunctionLidar(unittest.TestCase):
    def test_empty_cloud(self):
        cloud = PointCloud2()
        cloud.width = 0
        cloud.height = 0
        data = carJunctionPointCloud2LidarData(cloud)
        self.assertIsInstance(data, CarJunctionLidarData)
        self.assertEqual(len(data.points), 0)

    def test_pointcloud_timestamp_conversion(self):
        cloud = create_mock_pointcloud(sec=10, nanosec=500000000)
        data = carJunctionPointCloud2LidarData(cloud)
        self.assertIsInstance(data, CarJunctionLidarData)
        self.assertAlmostEqual(data.timeStamp, 10.5, places=5)
        self.assertEqual(len(data.points), 2)


class TestGeneralLidar(unittest.TestCase):
    def test_general_pointcloud_timestamp_conversion(self):
        cloud = create_mock_pointcloud(sec=5, nanosec=250000000)
        data = generalPointCloud2LidarData(cloud)
        self.assertIsInstance(data, GeneralLidarData)
        self.assertAlmostEqual(data.timeStamp, 5.25, places=5)
        self.assertEqual(len(data.points), 2)


if __name__ == "__main__":
    unittest.main()
