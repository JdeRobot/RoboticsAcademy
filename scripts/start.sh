#!/bin/bash

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &
python3.8 manager-3.1.py $1