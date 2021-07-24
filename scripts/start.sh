#!/bin/bash
rm -rf instructions.json
cp /RoboticsAcademy/scripts/instructions.json /instructions.json
sudo apt-get update
sudo apt-get install ros-melodic-navigation -y
sudo apt-get install ros-melodic-eband-local-planner
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &
python3.8 manager.py
