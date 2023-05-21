#!/bin/bash

. install/setup.bash
rviz2 &
ros2 launch turtlebot2 spawn_model.launch.py &
ros2 launch gazebo_ros gazebo.launch.py