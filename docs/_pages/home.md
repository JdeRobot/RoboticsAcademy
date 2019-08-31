---
layout: splash
permalink: /
header:
  overlay_color: "#5e616c"
  overlay_image: /assets/images/cover/test_header_shear_3.png
  actions:
    - label: "<i class='fas fa-download'></i> Install now"
      url: "/installation/"
excerpt: 
  Learn Robotics and Computer Vision
feature_row:
  - image_path: /assets/images/cover/cover_column_1.png
    alt: "Exercises"
    title: "Exercises"
    excerpt: "Take a look at the list of exercises available, under development or future."
    url: "/docs/configuration/"
    btn_class: "btn--primary"
    btn_label: "Learn more"
  - image_path: /assets/images/cover/cover_column_2.png
    alt: "fully responsive"
    title: "Contribute"
    excerpt: "Contribute with new exercises. For more information go to the following site."
    url: "/docs/layouts/"
    btn_class: "btn--primary"
    btn_label: "Learn more"
  - image_path: /assets/images/cover/cover_column_3.png
    alt: "100% free"
    title: "More information"
    excerpt: "Find out more about the requirements and objectives of the project."
    url: "/about/"
    btn_class: "btn--primary"
    btn_label: "Learn more"   
youTube_id: ID7qaEcIu4k
---

{% include feature_row %}


## What is Robotics-Academy?

Robotics-Academy is an **open source** collection of exercises to learn robotics in a practical way. [Gazebo simulator](http://gazebosim.org) is the main tool required, as [ROS](https://www.ros.org). The students program their solutions in **Python** language. Each exercise is composed of :

1. Gazebo configuration files,
2. A ROS node that hosts the student's code,
3. A file with instructions, hints, etc..
4. The student solution itself.

1, 2, and 3 are already provided, the student has to develop her code on a separate file which already has a template. The student may use there an existing simple Python API to access to sensor readings and actuator commands_ (HAL API) and she may use an existing simple Python API for Graphical User Interface and debugging_ (GUI API). To develop her solution the student has to edit that template file and add her code, using her favorite text editor.

For execution the student launches Gazebo with certain configuration file (specifying the robot and the simulated scenario for that exercise) and launches the ROS node hosting her code. On that code lies the intelligence of the robot to solve the exercise. For instance, check the recent solution of one degree student here for the local navigation exercise:


{% include video id="ID7qaEcIu4k" provider="youtube" %}

There are exercises about drone programming, about computer vision, about mobile robots, about autonomous cars, etc.. In the JdeRobotFoundation we are improving the quality of the existing exercises and creating a few exercises more. We are also working in a webserver to code and [run the exercises from the web browser](https://www.youtube.com/watch?v=bTwt6W8vCGQ) but that is a ongoing project yet.