#!/usr/bin/env bash
source /opt/ros/$ROS_DISTRO/local_setup.bash
source /usr/share/gazebo-11/setup.sh
export TURTLEBOT3_MODEL=waffle

# Convenience variable used inside GAZEBO_MODEL_PATH
export AMAZON_ROBOT_PATH=/opt/jderobot/CustomRobots/amazon_robot

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${AMAZON_ROBOT_PATH}/amazon_robot_gazebo/models:${AMAZON_ROBOT_PATH}/aws-robomaker-small-warehouse-world/models


# Picked these from Noetic Dockerfile. Keep whatever's relevant after testing
# GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11:/RoboticsAcademy/exercises/follow_line/web-template/launch
# GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/PX4-Autopilot/build/px4_sitl_default/build_gazebo
# GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models:/opt/jderobot/CustomRobots/f1:/opt/jderobot/CustomRobots/roomba_robot:/opt/jderobot/CustomRobots/3d_reconstruction:/opt/jderobot/CustomRobots/Taxi_navigator:/PX4-Autopilot/Tools/sitl_gazebo/models:/opt/ros/$ROS_DISTRO/share/drone_assets/models:/drones/drone_assets/models
# LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/PX4-Autopilot/build/px4_sitl_default/build_gazebo