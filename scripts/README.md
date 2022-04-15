# JdeRobot Academy: Instructions to start the docker image

This document gathers all the information about the RoboticsAcademy Docker Image (RADI). 

## RADI structure

RADI is built from **Ubuntu 20.04**.

Main components of the RADI are:

- [**ROS Noetic**](http://wiki.ros.org/noetic)
- [**Gazebo-11**](http://gazebosim.org/)
- **Python 3.8**, main libraries:
    - [websocket_server](https://pypi.org/project/websocket-server/)
    - [websockets](https://pypi.org/project/websockets/)
    - [asyncio](https://pypi.org/project/asyncio/)
- **VirtualGL**, GPU acceleration
- **TurboVNC**
- **noVNC**, VNC client web application 
- [**Xvfb**](https://www.x.org/releases/X11R7.6/doc/man/man1/Xvfb.1.xhtml), XServer for loading images from Gazebo camera
- [**PX4**](https://github.com/PX4/PX4-Autopilot), drones simulation. Dependencies:
    - Gradle
    - foonathan_memory
    - Fast-DDS
    - Fast-RTPS-Gen
- [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy) source code
- JdeRobot [CustomRobots](https://github.com/JdeRobot/CustomRobots)
- JdeRobobot [Drones](https://github.com/JdeRobot/drones)


RADI is split in two dockerfiles that creates two images while building, RADI-base image and RADI. Base image contains most of the dependencies, while second image gets source files and sets RADI the configuration.

## How to build your own RADI?
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git -b noetic
cd scripts
./build.sh <tag>
```

[`build.sh`](build.sh) script contains both images building commands.

## How to run the RADI?
```
docker run -it --rm -p <ports> jderobot/robotics-academy
```

This will execute [entrypoint.sh](entrypoint.sh) script. The script takes care of setting up the environment, launching the webserver and launching the manager.

About the entrypoint script:

```txt
entrypoint.sh [-h] [--debug] [--logs] [--no-server]

optional arguments:
    -h          show this help message and exit
    --debug     run bash inside RADI
    --logs      record logs and run RADI
    --no-server run RADI without webserver
```

### RADI entrypoint options
Run RADI as bash:
```
docker run -it --rm -p <ports> jderobot/robotics-academy --debug
```

Record launching logs:
```
docker run -it --rm -p <ports> jderobot/robotics-academy --debug
```

Run RADI without webserver:
```
docker run -it --rm -p <ports> jderobot/robotics-academy --no-server
```

## Important files

### Manager Script
The [`manager.py`](manager.py) script runs exercises by following the steps:

- Launching the Gazebo simulation: `roslaunch` instruction
- Running the main exercise script: `exercise.py`

Some exercise might also have some other specific launching steps.

### Instuctions JSON
The [`instructions.json`](instructions.json) gather the different needed commands for launching each exercise.

### Enviroment file
The [.env](.env) sets up the environment for launching. It is executed on the entrypoint script. Setted variables are:
- `GAZEBO_RESOURCE_PATH`: launch and world files path.
- `GAZEBO_PLUGIN_PATH`: plugin files path.
- `GAZEBO_MODEL_PATH`: model files path.
- `LD_LIBRARY_PATH`: shared/dynamic libraries path.
- `ROS_PACKAGE_PATH`: ros packages path.
- `STDR_LAUNCH`: stdr launch files path.
- `STDR_RESOURCES`: stdr resources (robots and sensors) path.