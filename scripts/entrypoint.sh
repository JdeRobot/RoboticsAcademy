#!/bin/bash

usage="$(basename "$0") [-h] [--logs] [--no-server]\n\n

optional arguments:\n
\t  -h  show this help message and exit\n
\t  --logs record logs and run RADI\n
\t  --no-server run RADI without webserver"

log=false
webserver=true

while [[ "$1" =~ ^- && ! "$1" == "--" ]]; do case $1 in
  -h | --help )
    echo -e $usage
    exit
    ;;
  -l | --logs )
    shift; log=true
    ;;
  -ns | --no-server )
    webserver=false
    ;;
esac; shift; done
if [[ "$1" == '--' ]]; then shift; fi

# TODO usar .sh y no .bash
# /bin/sh -c ""
ros_setup="source /opt/ros/noetic/setup.bash ; source /catkin_ws/devel/setup.bash"
if [ $webserver == true ]; then
    runserver="python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &"
else
    runserver=""
fi
runmanager="python3.8 manager.py"

# TEST LOGS
if [ $log == true ]; then
    DATE_TIME=$(date +%F-%H-%M) # FORMAT year-month-date-hours-mins
    mkdir -p /root/.roboticsacademy/log/$DATE_TIME/
    script -q -c "{ $ros_setup & $runserver & $runmanager; }" /root/.roboticsacademy/log/$DATE_TIME/manager.log
    cp $(readlink -f /root/.ros/log/latest)/* /root/.roboticsacademy/log/$DATE_TIME
else
    { $ros_setup & $runserver & $runmanager; }
fi
