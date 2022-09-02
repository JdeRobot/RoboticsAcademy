import numpy as np
import mmap
from posix_ipc import Semaphore, O_CREX, ExistentialError, O_CREAT, SharedMemory, unlink_shared_memory
from ctypes import sizeof, memmove, addressof, create_string_buffer, c_float
import struct

class SharedValue:
    def __init__(self, name):
        # Initialize varaibles for memory regions and buffers and Semaphore
        self.shm_buf = None; self.shm_region = None
        self.value_lock = None

        self.shm_name = name; self.value_lock_name = name

        # Initialize or retreive Semaphore
        try:
            self.value_lock = Semaphore(self.value_lock_name, O_CREX)
        except ExistentialError:
            value_lock = Semaphore(self.value_lock_name, O_CREAT)
            value_lock.unlink()
            self.value_lock = Semaphore(self.value_lock_name, O_CREX)

        self.value_lock.release()

    # Get the shared value
    def get(self, type_name= "value"):
        # Retreive the data from buffer
        if type_name=="value":
            try:
                self.shm_region = SharedMemory(self.shm_name)
                self.shm_buf = mmap.mmap(self.shm_region.fd, sizeof(c_float))
                self.shm_region.close_fd()
            except ExistentialError:
                self.shm_region = SharedMemory(self.shm_name, O_CREAT, size=sizeof(c_float))
                self.shm_buf = mmap.mmap(self.shm_region.fd, self.shm_region.size)
                self.shm_region.close_fd()
            self.value_lock.acquire()
            value = struct.unpack('f', self.shm_buf)[0]
            self.value_lock.release()

            return value
        elif  type_name=="list":
            self.shm_region = SharedMemory(self.shm_name)
            self.shm_buf = mmap.mmap(self.shm_region.fd, sizeof(c_float))
            self.shm_region.close_fd()
            self.value_lock.acquire()
            array_val = np.ndarray(shape=(3,),
                                dtype='float32', buffer=self.shm_buf)
            self.value_lock.release()

            return array_val

        else:
            print("missing argument for return type")

     

    # Add the shared value
    def add(self, value, type_name= "value"):
        # Send the data to shared regions
        if type_name=="value":
            try:
                self.shm_region = SharedMemory(self.shm_name)
                self.shm_buf = mmap.mmap(self.shm_region.fd, sizeof(c_float))
                self.shm_region.close_fd()
            except ExistentialError:
                self.shm_region = SharedMemory(self.shm_name, O_CREAT, size=sizeof(c_float))
                self.shm_buf = mmap.mmap(self.shm_region.fd, self.shm_region.size)
                self.shm_region.close_fd()

            self.value_lock.acquire()
            self.shm_buf[:] = struct.pack('f', value)
            self.value_lock.release()
        elif  type_name=="list":
            byte_size = value.nbytes
            self.shm_region = SharedMemory(self.shm_name, O_CREAT, size=byte_size)
            self.shm_buf = mmap.mmap(self.shm_region.fd, byte_size)
            self.shm_region.close_fd()
            self.value_lock.acquire()
            self.shm_buf[:] = value.tobytes()
            self.value_lock.release()

    # Destructor function to unlink and disconnect
    def close(self):
        self.value_lock.acquire()
        self.shm_buf.close()

        try:
            unlink_shared_memory(self.shm_name)
        except ExistentialError:
            pass

        self.value_lock.release()
        self.value_lock.close()
