                                                                                         
#Set up path for the benchamrking folder                                           
                                                                                         
import sys
import os


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


currentPath = os.path.dirname(os.path.realpath(__file__))

# Add benchmarking folder to PYTHONPATH
libPath = os.path.join(currentPath, 'benchmarking')
add_path(libPath)
