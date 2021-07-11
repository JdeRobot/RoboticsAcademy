# RealSense recipe
Recipe to install the drivers of the RGBD Intel RealSense D435 camera.

## 0) Previously:
The official library, written in C ++, is here (step 1): https://github.com/IntelRealSense/librealsense

There is a "deprecated" coating for Python (https://pypi.org/project/pyrealsense) that only supports librealsense v1.12.1. It does not support, therefore, the last SDK 2.0 (2.16.5).

And another one that supports it, pyrealsense2 (https://pypi.org/project/pyrealsense2) (https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python)

## 1) Install library "librealsense":
a) Download the last (2.16.5) "source code (tar.gz)" from here: https://github.com/IntelRealSense/librealsense/releases
b) We extract its content: tar -xvzf librealsense-2.16.5.tar.gz
c) cmake .
-> We get the bug "PkgConfig package is missing!"
----> Solution: sudo apt-get install pkg-config
-> We get the bug "Failed to find libusb-1.0"
----> Solution: sudo aptitude install libusb-1.0-0 libusb-1.0-0-dev
-> We get the failure of having to set the environment variable "GLFW_INCLUDE_DIR" because the "build" is prepared for Microsoft Visual Studio (MVS) and not for Linux.
----> Solution: cmake . -DBUILD_GRAPHICAL_EXAMPLES = Off
d) make [It takes about half an hour]
e) sudo make install

Once installed, we can test the examples offered, both graphic mode (those that start with prefix rs_, realsense) and non-graphic (prefix cpp_).
In Ubuntu there is no renderer to graphically show what was received by the camera.
Anyway, we can use the web renderer in those examples that have _web termination.

Example:
- Launch in terminal: rs_pt_tutorial_1_web
- View in browser: http://localhost:8000/view.html
(The code would be in ".../librealsense-2.16.5/build/realsense_samples-0.6.2/samples/pt_tutorial_1_web/cpp/main.cpp").

## 2) Install python wrapper "pyRealSense":
a) We need to have done step 1 and have Cython installed; if not, we install it: pip install --upgrade cython

b) Install pyrealsense: pip install pyrealsense
-> It can give us a problem: I can not find the library "rs.h".
----> We test "sudo apt-get install libncurses5-dev" but it's not a solution

[...]

c) We install the wrapper "pyrealsense2" (up-to-date version), which we will install with Python3.
-> (We could also get it from here: https://pypi.org/project/pyrealsense2/)
-> sudo aptitude install pip3
-> pip3 install pyrealsense2

d) We tested this example: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
-> python3 opencv_viewer_example.py
----> We may not have, and we need to install: numpy, cv
------> pip3 install numpy
------> pip3 install opencv-python

The folder with the Python wrapper, and examples of use, etc., are on the official GitHub, specifically here: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python

## Credits
* *Julio Vega (@jmvega)*
