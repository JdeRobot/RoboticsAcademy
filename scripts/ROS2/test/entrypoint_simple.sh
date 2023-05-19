#!/bin/bash

root="cd /"
ros_setup="source /.env && source ~/.bashrc && source /home/ws/install/setup.bash"
runserver="python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000"
runram="python3 /manager.py 0.0.0.0 7163"

eval ${ros_setup} && echo 'environment set'
$root && $runram
