#!/usr/bin/env bash
source /opt/ros/$ROS_DISTRO/local_setup.bash
source /usr/share/gazebo-11/setup.sh
export TURTLEBOT3_MODEL=waffle

# For Amazon Warehouse exercise
source /opt/jderobot/CustomRobots/install/setup.sh

# For Simulated Follow Person exercise
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/jderobot/CustomRobots/amazon_hospital/models:/opt/jderobot/CustomRobots/amazon_hospital/fuel_models:/opt/jderobot/CustomRobots/amazon_hospital/hospital_world/models

# Convenience variable used to simplify GAZEBO_MODEL_PATH
export AMAZON_ROBOT_PATH=/opt/jderobot/CustomRobots/amazon_robot
# Make all Gazebo models accessible
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${AMAZON_ROBOT_PATH}/amazon_robot_gazebo/models:${AMAZON_ROBOT_PATH}/aws-robomaker-small-warehouse-world/models
