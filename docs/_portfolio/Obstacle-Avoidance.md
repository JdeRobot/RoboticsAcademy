---
title: "Local navigation of a Formula1 with VFF"
excerpt: "The objective of this practice is to implement the logic of the VFF navigation algorithm."
header:
  image: /images/obstacle_avoidance.png
  teaser: /images/obstacle_avoidance.png
gallery:
  - url: /images/obstacle_avoidance.png
    image_path: images/obstacle_avoidance.png
    alt: "placeholder image 1"
  - url: /images/f1_laser.png
    image_path: images/f1_laser.png
    alt: "placeholder image 2"
youtubeId: 5SVkvfKPi_s
---

The objective of this practice is to implement the logic of the VFF navigation algorithm.

Navigation using VFF (Virtual Force Field), consists of:

* Each object in the environment generates a repulsive force towards the robot.

* Destiny generates an attractive force in the robot.

This makes it possible for the robot to go towards the target, distancing itself of the obstacles, so that their address is the vector sum of all the forces.

{% include gallery caption="Obstacle Avoidance." %}

The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

* Go as quickly as possible

* Choose the safest way

* Obstacles in movement

* Robustness in situations of indecision (zero vector sum)

{% include youtubePlayer.html id=page.youtubeId %}

