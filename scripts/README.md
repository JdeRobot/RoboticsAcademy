# JdeRobot Academy: Instructions to start the docker image

This document gathers all the information about the RoboticsAcademy Docker Image (RADI). 

The RADI is split in **two dockerfiles**, to build the **RADI** you have to first build **RADI-base** image. Base image contains most of the dependencies, while second image adds source files and sets RADI the configuration. This is done to speed up the development, RADI could be built many times but the base, which contains most of the dependencies is only built once.

Notice that only RADI image is available on [RoboticsAcademy docker hub](https://hub.docker.com/r/jderobot/robotics-academy/tags), since RADI-base is only used during development.

## How to build your own RADI?
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git -b master
cd scripts
./build.sh <tag>
```

[`build.sh`](build.sh) script contains both images building commands. `<tag>` will be new RADI's name. To only build RADI without base image run `docker build --no-cache=true -t jderobot/robotics-academy:<tag> .`

## How to run the RADI?
```
docker run -it --rm -p <ports> jderobot/robotics-academy
```

This will execute [entrypoint.sh](entrypoint.sh) script. The script takes care of setting up the environment, launching the webserver and launching the manager. `<ports>` tag has to be replaced with the port list you want to open with the host machine. You can see the port list available [here](#radi-port-list). 

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
docker run -it --rm -p <ports> jderobot/robotics-academy --logs
```

Run RADI without webserver:
```
docker run -it --rm -p <ports> jderobot/robotics-academy --no-server
```

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

## Important files

### Classic Manager Script
The [`manager.py`](manager.py) script runs exercises by following the steps:

- Launching the Gazebo simulation: `roslaunch` instruction.
- Running the main exercise script, `exercise.py`, which will be also launch `hal.py` and `gui.py`.

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

## RADI port list

- **8000**: Django server.
- **7163**: RoboticsAcademy Manager (RAM) websocket.
- **8765**: classic Manager websocket (deprecated in RADI v3.3).
- **1905**: classic HAL (*exercise.py*) websocket.
- **1904**: HAL guest websocket (experimental use).
- **2303**: classic GUI (*gui.py*) websocket.
- **2304**: GUI guest websocket (experimental use).
- **6080**: Gazebo noVNC client.
- **1108**: Console noVNC client.
- **1831**: WebRTC client (**deprecated**).

### Other port useful for development
- **11311**: ROS master URI.