from multiprocessing import shared_memory
import numpy as np
import threading

class SharedNumpy:
    def __init__(self, name, width, height):
        self.name = name
        self.data_type = np.uint8
        self.shape = (width,height)

        self.shared_mem_add = None
        self.shared_mem_get = None

        self.lock = threading.Lock()
    
    def add(self, numpy):
        self.lock.acquire()
        self.shared_mem_add = shared_memory.SharedMemory(name=self.name, create=True, size=numpy.nbytes) # create shared memory buffer
        shared_numpy = np.ndarray(self.shape, dtype=self.data_type, buffer=self.shared_mem_add.buf) # create shared-memory np.array
        shared_numpy[:] = numpy[:] # copy content
        self.lock.release()

    def get(self):
        self.lock.acquire()
        self.shared_mem_get = shared_memory.SharedMemory(name=self.name,create=False) # get existing shared memory buffer
        shared_numpy = np.ndarray(self.shape, dtype=self.data_type, buffer=self.shared_mem_get.buf)
        self.lock.release()

        return shared_numpy

    def close(self):
        try:
            self.shared_mem_add.close()
            self.shared_mem_add.unlink()
        except:
            pass

        try:
            self.shared_mem_get.close()
            self.shared_mem_add.unlink()
        except:
            pass



