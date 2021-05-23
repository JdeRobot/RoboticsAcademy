# JdeRobot Academy: Instructions to start the docker image

Build your own docker image to start simulations

## Adding new instructions to Dockerfile

The idea is to keep, `Dockerfile.base` to install the non changing elements and `Dockerfile` to install the elements that keep changing.

1. If the dependencies are a framework for an exercise, like ROS, PX4 or MoveIt! add the dependencies
to `Dockerfile.base` after the PX4 installation. Comments mark the spot.

2. If the dependencies are not ROS related but required for almost all the exercises, add them to `Dockerfile.base`. Comments mark the spot. If they are `apt-get` related, add them to the `apt-get` list, otherwise add them add the end.

3. `Dockerfile` remains mostly untouched for most of the cases. It clones the `CustomRobot` and `RoboticsAcademy` repositories, which are quite dynamic.

## Installation

First you need to build the image. Then, you need to run a container.

```
git clone https://github.com/JdeRobot/RoboticsAcademy.git
cd scripts
./build.sh <tag>
docker run -it --name=container_name -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:<tag> ./start.sh
```

## Structure of Image

The image contains the following:

- [ROS Melodic](http://wiki.ros.org/melodic)
- [Gazebo-9](http://gazebosim.org/) that comes with ROS Melodic
- [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy) is present in `\`
- [CustomRobots](https://github.com/JdeRobot/CustomRobots) is present in `\opt\jderobot\`
- [Gzweb](https://github.com/osrf/gzweb) is present in `\`
- Python2.7 with the library [websocket_server](https://pypi.org/project/websocket-server/)
- Python3.8 with the library [websockets](https://pypi.org/project/websockets/) and [asyncio](https://pypi.org/project/asyncio/)
- [Xvfb](https://www.x.org/releases/X11R7.6/doc/man/man1/Xvfb.1.xhtml) as the XServer for loading images from Gazebo camera

The exercise runs using the CustomRobots directory as a dependency. The Gazebo simulation's visual view is provided by Gzweb.


## Manager Script
The `manager.py` script runs exercises by following the steps:

- Exporting environment variables: `GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH`
- Launching the Gazebo simulation: `roslaunch`
- Running the main exercise script: `host.py`
