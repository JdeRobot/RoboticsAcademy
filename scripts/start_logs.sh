#!/bin/bash

DATE_TIME=$(date +%F-%H-%M) # FORMAT year-month-date-hours-mins
mkdir -p /root/.roboticsacademy/log/$DATE_TIME/
script -q -c "./start.sh" /root/.roboticsacademy/log/$DATE_TIME/manager.log
cp $(readlink -f /root/.ros/log/latest)/* /root/.roboticsacademy/log/$DATE_TIME