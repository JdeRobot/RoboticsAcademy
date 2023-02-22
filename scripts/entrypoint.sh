#!/bin/bash

usage="$(basename "$0") [-h] [--debug] [--logs] [--no-server]\n\n

optional arguments:\n
\t  -h  show this help message and exit\n
\t  --debug run bash inside RADI\n
\t  --logs record logs and run RADI\n
\t  --no-server run RADI without webserver"

debug=false
log=false
webserver=true

while [[ "$1" =~ ^- && ! "$1" == "--" ]]; do case $1 in
  -h | --help )
    echo -e $usage
    exit
    ;;
  -d | --debug )
    shift; debug=true
    ;;
  -l | --logs )
    shift; log=true
    ;;
  -ns | --no-server )
    webserver=false
    ;;
esac; shift; done
if [[ "$1" == '--' ]]; then shift; fi


ros_setup=" source /opt/ros/noetic/setup.bash ; source /catkin_ws/devel/setup.bash ; source /.env ; "
if [ $webserver == true ]; then
    runserver="python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000"
else
    runserver=""
fi
runmanager="python3.8 manager.py"
runram="python3.8 RoboticsAcademy/src/manager/manager.py 0.0.0.0 7163"

# TEST LOGS
if [ $log == true ]; then
    DATE_TIME=$(date +%F-%H-%M) # FORMAT year-month-date-hours-mins
    mkdir -p /root/.roboticsacademy/log/$DATE_TIME/
    eval ${ros_setup}
    script -q -c "$runserver & $runmanager ;" /root/.roboticsacademy/log/$DATE_TIME/manager.log
    cp -r /root/.ros/log/* /root/.roboticsacademy/log/$DATE_TIME
else
    eval ${ros_setup}
    if [ $debug == true ]; then
      { bash ; }
    else
      { $runserver & $runmanager & $runram ; }
    fi
fi
