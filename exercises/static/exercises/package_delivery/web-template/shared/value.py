import numpy as np
import mmap
from posix_ipc import Semaphore, O_CREX, ExistentialError, O_CREAT, SharedMemory, unlink_shared_memory
from ctypes import sizeof, memmove, addressof, create_string_buffer, c_float
import struct

class SharedValue:
    def __init__(self, name, n_elem = 1):
        # Initialize varaibles for memory regions and buffers and Semaphore
        self.n_elem = n_elem
        self.shm_buf = [None]*self.n_elem; self.shm_region = [None]*self.n_elem
        self.value_lock = [None]*self.n_elem

        self.shm_name = name; self.value_lock_name = name

        # Initialize or retreive Semaphore
        for i in range(self.n_elem):
            try:
                self.value_lock[i] = Semaphore(self.value_lock_name+str(i), O_CREX)
            except ExistentialError:
                value_lock = Semaphore(self.value_lock_name+str(i), O_CREAT)
                value_lock.unlink()
                self.value_lock[i] = Semaphore(self.value_lock_name+str(i), O_CREX)

            self.value_lock[i].release()

    # Get the shared value
    def get(self):
        # Retreive the data from buffer
        
        value = [None]*self.n_elem
        for i in range(self.n_elem):
            try:
                self.shm_region[i] = SharedMemory(self.shm_name+str(i))
                self.shm_buf[i] = mmap.mmap(self.shm_region[i].fd, sizeof(c_float))
                self.shm_region[i].close_fd()
            except ExistentialError:
                self.shm_region[i] = SharedMemory(self.shm_name+str(i), O_CREAT, size=sizeof(c_float))
                self.shm_buf[i] = mmap.mmap(self.shm_region[i].fd, self.shm_region[i].size)
                self.shm_region[i].close_fd()
            self.value_lock[i].acquire()
            value[i] = struct.unpack('f', self.shm_buf[i])[0]
            self.value_lock[i].release()

        if self.n_elem <=1:
            return value[0]
        else: 
            return value

        
     

    # Add the shared value
    def add(self, value):
        # Send the data to shared regions
       
        for i in range(self.n_elem):
            try:
                self.shm_region[i] = SharedMemory(self.shm_name+str(i))
                self.shm_buf[i] = mmap.mmap(self.shm_region[i].fd, sizeof(c_float))
                self.shm_region[i].close_fd()
            except ExistentialError:
                self.shm_region[i] = SharedMemory(self.shm_name+str(i), O_CREAT, size=sizeof(c_float))
                self.shm_buf[i] = mmap.mmap(self.shm_region[i].fd, self.shm_region[i].size)
                self.shm_region[i].close_fd()

            self.value_lock[i].acquire()
            if self.n_elem <=1:
                self.shm_buf[i][:] = struct.pack('f', value)
            else:
                self.shm_buf[i][:] = struct.pack('f', value[i])
            self.value_lock[i].release()
        

    # Destructor function to unlink and disconnect
    def close(self):
        for i in range(self.n_elem):
            self.value_lock[i].acquire()
            self.shm_buf[i].close()

            try:
                unlink_shared_memory(self.shm_name+str(i))
            except ExistentialError:
                pass

            self.value_lock[i].release()
            self.value_lock[i].close()
