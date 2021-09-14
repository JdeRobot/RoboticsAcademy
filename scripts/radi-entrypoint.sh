#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
roscd cob_scan_unifier/src/
chmod +x scan_unifier_node.cpp
cd /