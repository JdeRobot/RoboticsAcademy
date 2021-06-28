# Global Navigation Exercise using WebTemplate

## How to launch the exercise?

- Clone the Robotics Academy repository on your local machine

```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
```

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

```bash
docker pull jderobot/robotics-academy:2.4.3
```

- Start a new docker container of the image and keep it running in the background

```bash
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.3 ./start.sh
```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected. 

- And the exercise is ready to be used after the alert.

For more informations on how to execute this exercise, please refer [Global Navigation page of JdeRobot Academy](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/)

## Dependencies

[taxi_holo_ROS and cityLarge models of Gazebo](https://github.com/JdeRobot/CustomRobots/tree/melodic-devel/Taxi_navigator) from CustomRobots repository

### Launch Files

`launch` folder contains the various launch files and world files, for running the Gazebo simulation of the exercise

### Asset Files

`assets` folder contains the Javascript code for the webpage part of the web templates

### Python Files

`exercise.py` is the main Python file to run the exercise
