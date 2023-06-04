import numpy as np
import mmap
from posix_ipc import Semaphore, O_CREX, ExistentialError, O_CREAT, SharedMemory, unlink_shared_memory
from ctypes import sizeof, c_float
import struct

# Auxiliary class to store position data (x, y, yaw)
class Pose():
    def __init__(self):
        self.x = 0.0    # X coord [meters]
        self.y = 0.0    # Y coord [meters]
        self.yaw = 0.0  #Yaw angle[rads]

    def __str__(self):
        s = "Pose3D: {\n   x: " + str(self.x) + "\n   y: " + str(self.y) + "\n   Yaw: " + str(self.yaw)  + "\n}"
        return s 
    
class SharedPose3D:
    def __init__(self, name):
        # Initialize variables for memory regions and buffers and Semaphore
        self.shm_buf = None
        self.shm_region = None
        self.pose_lock = None

        self.shm_name = name
        self.pose_lock_name = name

        self.pose = Pose()

        # Initialize shared memory buffer
        try:
            self.shm_region = SharedMemory(self.shm_name)
            self.shm_buf = mmap.mmap(self.shm_region.fd, sizeof(c_float)*3)
            self.shm_region.close_fd()
        except ExistentialError:
            self.shm_region = SharedMemory(self.shm_name, O_CREAT, size=sizeof(c_float)*3)
            self.shm_buf = mmap.mmap(self.shm_region.fd, self.shm_region.size)
            self.shm_region.close_fd()

        # Initialize or retreive Semaphore
        try:
            self.pose_lock = Semaphore(self.pose_lock_name, O_CREX)
        except ExistentialError:
            pose_lock = Semaphore(self.pose_lock_name, O_CREAT)
            pose_lock.unlink()
            self.pose_lock = Semaphore(self.pose_lock_name, O_CREX)

        self.pose_lock.release()

    # Get the shared value
    def get(self):
        # Retreive the data from buffer
        self.pose_lock.acquire()
        pose_tuple = struct.unpack('3f', self.shm_buf)
        self.pose_lock.release()

        pose = Pose()
        pose.x = pose_tuple[0]
        pose.y = pose_tuple[1]
        pose.yaw = pose_tuple[2]

        return pose

    # Add the shared value
    def add(self, pose):
        # Send the data to shared regions
        self.pose_lock.acquire()
        self.shm_buf[:] = struct.pack('3f', pose.x, pose.y, pose.yaw)
        self.pose_lock.release()

    # Destructor function to unlink and disconnect
    def close(self):
        self.pose_lock.acquire()
        self.shm_buf.close()

        try:
            unlink_shared_memory(self.shm_name)
        except ExistentialError:
            pass

        self.pose_lock.release()
        self.pose_lock.close()
