import numpy as np
import mmap
from posix_ipc import Semaphore, O_CREX, ExistentialError, O_CREAT, SharedMemory, unlink_shared_memory
from ctypes import sizeof, c_float
import struct
import array

class SharedLaserData:
    def __init__(self, name):
        # Initialize variables for memory regions and buffers and Semaphore
        self.shm_buf = None; self.shm_region = None
        self.laserdata_lock = None

        self.shm_name = name; 
        self.laserdata_lock_name = name

        # Initialize shared memory buffer
        try:
            self.shm_region = SharedMemory(self.shm_name)
            self.shm_buf = mmap.mmap(self.shm_region.fd, sizeof(c_float)*360)
            self.shm_region.close_fd()
        except ExistentialError:
            self.shm_region = SharedMemory(self.shm_name, O_CREAT, size=sizeof(c_float)*360)
            self.shm_buf = mmap.mmap(self.shm_region.fd, self.shm_region.size)
            self.shm_region.close_fd()

        # Initialize or retreive Semaphore
        try:
            self.laserdata_lock = Semaphore(self.laserdata_lock_name, O_CREX)
        except ExistentialError:
            laserdata_lock = Semaphore(self.laserdata_lock_name, O_CREAT)
            laserdata_lock.unlink()
            self.laserdata_lock = Semaphore(self.laserdata_lock_name, O_CREX)

        self.laserdata_lock.release()

    # Get the shared value
    def get(self):
        # Retreive the data from buffer
        self.laserdata_lock.acquire()
        laserdata_tuple = struct.unpack('360f', self.shm_buf)
        laserdata = array.array('f', laserdata_tuple)
        self.laserdata_lock.release()

        return laserdata

    # Add the shared value
    def add(self, laserdata):
        # Send the data to shared regions
        self.laserdata_lock.acquire()
        self.shm_buf[:] = laserdata
        self.laserdata_lock.release()

    # Destructor function to unlink and disconnect
    def close(self):
        self.laserdata_lock.acquire()
        self.shm_buf.close()

        try:
            unlink_shared_memory(self.shm_name)
        except ExistentialError:
            pass

        self.laserdata_lock.release()
        self.laserdata_lock.close()
