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

if [ $webserver == true ]; then
    runserver="python3 /RoboticsAcademy/manage.py runserver 0.0.0.0:8000"
else
    runserver=""
fi
ros_setup=" source /.env && source ~/.bashrc && source /home/ws/install/setup.bash; "
runmanager="python3 RoboticsAcademy/manager/manager.py"
runram="python3 RoboticsAcademy/src/manager/manager/manager.py 0.0.0.0 7163"
root="cd /"

# TEST LOGS
if [ $log == true ]; then
    DATE_TIME=$(date +%F-%H-%M) # FORMAT year-month-date-hours-mins
    mkdir -p /root/.roboticsacademy/log/$DATE_TIME/
    eval ${ros_setup}
    script -q -c "$root & $runserver & $runram ;" /root/.roboticsacademy/log/$DATE_TIME/manager.log
    cp -r /root/.ros/log/* /root/.roboticsacademy/log/$DATE_TIME
else
    eval ${ros_setup}
    if [ $debug == true ]; then
      { bash ; }
    else
      { $root & $runserver & $runram ; }
    fi
fi
