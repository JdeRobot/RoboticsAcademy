#!/bin/bash
rm -rf /instructions.json /manager.py
cp /RoboticsAcademy/scripts/instructions.json /instructions.json
cp /RoboticsAcademy/scripts/manager.py /manager.py
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &
python3.8 manager.py
