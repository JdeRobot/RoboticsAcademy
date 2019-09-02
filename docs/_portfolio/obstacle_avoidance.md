---
title: "Local navigation with VFF"
excerpt: "Logic of the VFF navigation algorithm using a F1."

header:
  image: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance.png
  teaser: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance_teaser.png

gallery:
    image_path: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance.png
    alt: "Obstacle Avoidance"
    image_path: /assets/images/exercises/obstacle_avoidance/f1_laser.png
    alt: "F1 laser"

youtubeId: 5SVkvfKPi_s
---

The objective of this practice is to implement the logic of the VFF navigation algorithm.

Navigation using VFF (Virtual Force Field), consists of:

* Each object in the environment generates a repulsive force towards the robot.

* Destiny generates an attractive force in the robot.

This makes it possible for the robot to go towards the target, distancing itself of the obstacles, so that their address is the vector sum of all the forces.

<img src="/assets/images/exercises/obstacle_avoidance/obstacle_avoidance.png" width="100%" height="60%">
{% include gallery caption="Obstacle Avoidance." %}

The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

* Go as quickly as possible

* Choose the safest way

* Obstacles in movement

* Robustness in situations of indecision (zero vector sum)

{% include youtubePlayer.html id=page.youtubeId %}

