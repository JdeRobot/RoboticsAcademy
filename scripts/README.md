# JdeRobot Academy: Instructions to start the docker image

Build your own docker image to start simulations


## Installation

First you need to build the image. Then, you need to run a container.

```
#git clone https://github.com/JdeRobot/RoboticsAcademy.git
#cd scripts
#docker build -f image-name .
#docker run -it --name=container_name -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 jderobot/robotics-academy:manual python3.8 manager.py
```
