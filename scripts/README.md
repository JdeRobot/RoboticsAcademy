# JdeRobot Academy: Instructions to start the docker image

Build your own docker image to start simulations

## Structure of Image

The image contains the following:

- [ROS Noetic](http://wiki.ros.org/noetic)
- [Gazebo-11](http://gazebosim.org/) that comes with ROS Noetic
- [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy) is present in `\`
- [CustomRobots](https://github.com/JdeRobot/CustomRobots) is present in `\opt\jderobot\`
- [Gzweb](https://github.com/osrf/gzweb) is present in `\`
- Python3.8 with the library [websocket_server](https://pypi.org/project/websocket-server/)
- Python3.8 with the library [websockets](https://pypi.org/project/websockets/) and [asyncio](https://pypi.org/project/asyncio/)
- [Xvfb](https://www.x.org/releases/X11R7.6/doc/man/man1/Xvfb.1.xhtml) as the XServer for loading images from Gazebo camera

The exercise runs using the CustomRobots directory as a dependency. The Gazebo simulation's visual view is provided by Gzweb.


## Installation

First you need to build the image. Then, you need to run a container.

```
git clone https://github.com/JdeRobot/RoboticsAcademy.git -b noetic
cd scripts
./build.sh <tag>
docker run -it --name=container_name -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:<tag> ./start.sh
```

## Manager Script
The `manager.py` script runs exercises by following the steps:

- Exporting environment variables: `GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH`
- Launching the Gazebo simulation: `roslaunch`
- Running the main exercise script: `host.py`
