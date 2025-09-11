import numpy as np


class MockROSHeader:
    def __init__(self, stamp_sec=0, stamp_nanosec=0, frame_id=""):
        self.stamp = MockROSStamp(stamp_sec, stamp_nanosec)
        self.frame_id = frame_id


class MockROSStamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class MockROSImage:
    def __init__(self, width=640, height=480, encoding="bgr8", data=None):
        self.header = MockROSHeader()
        self.height = height
        self.width = width
        self.encoding = encoding
        if data is None:
            self.data = bytes([0] * (height * width * 3))
        else:
            self.data = data


class MockROSLaserScan:
    def __init__(
        self,
        angle_min=-1.57,
        angle_max=1.57,
        angle_increment=0.01,
        range_min=0.1,
        range_max=10.0,
        ranges=None,
    ):
        self.header = MockROSHeader()
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        if ranges is None:
            num_points = int((angle_max - angle_min) / angle_increment) + 1
            self.ranges = [5.0] * num_points
        else:
            self.ranges = ranges


class MockROSTwist:
    def __init__(self):
        self.linear = MockROSVector3()
        self.angular = MockROSVector3()


class MockROSVector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
