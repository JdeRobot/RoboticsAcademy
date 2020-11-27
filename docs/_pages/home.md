---
layout: splash
permalink: /
header:
  overlay_color: "#5e616c"
  overlay_image: /assets/images/cover/test_header_shear_3.png
#  actions:
#    - label: "<i class='fas fa-download'></i> Install now"
#      url: "/installation/"
excerpt: 
  Learn Robotics and Computer Vision
feature_row:
  - image_path: /assets/images/cover/cover_column_1.png
    alt: "Exercises"
    title: "Exercises"
    excerpt: "Learn and play by performing the RoboticsAcademy's many exercises"
    url: "/exercises/"
    btn_class: "btn--primary"
    btn_label: "Learn more"

  - image_path: /assets/images/cover/cover_column_2.png
    alt: "fully responsive"
    title: "Installation"
    excerpt: "Installation instructions for RoboticsAcademy native release in Linux"
    url: "/installation"
    btn_class: "btn--primary"
    btn_label: "Learn more"

  - image_path: /assets/images/cover/cover_column_3.png
    alt: "100% free"
    title: "Web release (beta)"
    excerpt: "Use RoboticsAcademy from your browser, no installation required"
    url: "/webrelease"
    btn_class: "btn--primary"
    btn_label: "Learn more"


youTube_id: ID7qaEcIu4k
---


Robotics-Academy is an **open source** collection of exercises to learn robotics in a practical way.
There are exercises about drone programming, about computer vision, about mobile robots, about autonomous cars, etc. 
It is mainly based on [Gazebo simulator](http://gazebosim.org) and [ROS](https://www.ros.org). The students program their solutions in **Python** language.

Each exercise is composed of (a) Gazebo configuration files, (b) a ROS node that is the template to host student's code and (c) theory contents. The student inserts her code in the template file and uses the provided simple API to access to _sensor readings_ and _actuator commands_ (HAL API) and the provided simple API for Graphical User Interface and debugging (GUI API).

For execution the student launches Gazebo with certain configuration file (specifying the robot and the simulated scenario for that exercise) and launches the ROS node hosting her code.

{% include feature_row %}

{% include video id="ID7qaEcIu4k" provider="youtube" %}

## Sponsors

<figure class="third">
    <a href="https://www.urjc.es/" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/logoURJC.jpg" style="width:400px;"></a>
    <a href="https://github.com/RoboticsLabURJC" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/peloto.png" style="width:400px;"></a>
    <a href="https://jderobot.github.io/" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/logo.png" style="width:400px;"></a>
</figure>
