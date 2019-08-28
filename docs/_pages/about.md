---
permalink: /about/
title: "About Robotics Academy"

youtubeId: ID7qaEcIu4k
---


JdeRobot-Academy is an open source collection of exercises to learn robotics in a practical way. Gazebo simulator is the main tool required, as ROS. The students program their solutions in Python language. Each exercise is composed of :

1. Gazebo configuration files,
2. A ROS node that hosts the student's code,
3. A file with instructions, hints, etc..
4. The student solution itself.

1, 2, and 3 are already provided, the student has to develop her code on a separate file which already has a template. The student may use there an existing simple Python API to access to sensor readings and actuator commands_ (HAL API) and she may use an existing simple Python API for Graphical User Interface and debugging_ (GUI API). To develop her solution the student has to edit that template file and add her code, using her favorite text editor.

For execution the student launches Gazebo with certain configuration file (specifying the robot and the simulated scenario for that exercise) and launches the ROS node hosting her code. On that code lies the intelligence of the robot to solve the exercise. For instance, check the recent solution of one degree student here for the local navigation exercise:

{% include youtubePlayer.html id=page.youtubeId %}


There are exercises about drone programming, about computer vision, about mobile robots, about autonomous cars, etc.. In the JdeRobotFoundation we are improving the quality of the existing exercises and creating a few exercises more. We are also working in a webserver to code and [run the exercises from the web browser](https://www.youtube.com/watch?v=bTwt6W8vCGQ) but that is a ongoing project yet.

