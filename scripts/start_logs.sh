#!/bin/bash

mkdir -p /root/.roboticsacademy/log
export ROS_LOG_DIR=/root/.roboticsacademy/log
script -q -c "./start.sh log" /root/.roboticsacademy/log/manager.log