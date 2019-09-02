---
title: "Global navigation of a TeleTaxi"
excerpt: "Logic of a Gradient Path Planning (GPP) algorithm."

header:
  image: /assets/images/exercises/global_navigation/global_navigation.png
  teaser: /assets/images/exercises/global_navigation/global_navigation_teaser.png

gallery:
    image_path: /assets/images/exercises/global_navigation/taxi.png
    alt: "Taxi"

    image_path: /assets/images/exercises/global_navigation/global_navigation.png
    alt: "Map"

youtubeId1: zUtK6seVL5g
youtubeId2: q6G6BHqljP4
youtubeId3: itTbU4uLwfE
youtubeId4: zcS4X-ZO68U
---

The objective of this practice is to implement the logic of a Gradient Path Planning (GPP) algorithm. Global navigation through GPP, consists of:

<img src="/assets/images/exercises/global_navigation/taxi.png" width="100%" height="60%">
{% include gallery caption="Taxi." %}

* Selected a destination, the GPP algorithm is responsible for finding the shortest path to it, avoiding, in the case of this practice, everything that is not road.

* Once the path has been selected, the logic necessary to follow this path and reach the objective must be implemented in the robot.

With this, it is possible for the robot to go to the marked destination autonomously and following the shortest path.

<img src="/assets/images/exercises/global_navigation/global_navigation.png" width="100%" height="60%">
{% include gallery caption="Global navigation of a TeleTaxi with GPP." %}

The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

* Reach the goal.

* Optimize the way to find the shortest path.

* Arrive as quickly as possible to the destination.

{% include youtubePlayer.html id=page.youtubeId1 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId2 %}

<br/>

Global Navigation teletaxi with OMPL:

{% include youtubePlayer.html id=page.youtubeId3 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId4 %}

