---
title: "Bump and go"
excerpt: "The objective of this practice is to program a basic behaviour of bump-spin using a finite state machine."
header:
  image: /images/bump_go.png
  teaser: /images/bump_go.png
gallery:
  - url: /images/bump_go.png
    image_path: images/bump_go.png
    alt: "placeholder image 1"
youtubeId1: HRjXf2GzK70
youtubeId2: Yon6yKY5oqk
youtubeId3: zA7jN7ZR2sk
---

The intention of this excersise is to program a basic behaviour of bump-spin using a finite state machine. For that, we will use JdeRobot visualStates tool, that allows you to create your own states machine in an intuitive way.

There is a Kobuki robot inside a labyrinth or scenario. The robot will go front until it gets close to an obstacle. The it will go back, turn a random angle and go front again repeating the process. This exercise aims to show the power of automata when building robot behavior.

{% include gallery caption="Bump and Go." %}

{% include youtubePlayer.html id=page.youtubeId1 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId2 %}

<br/>


Using the [JdeRobot tool VisualStates](https://jderobot.org/index.php/Tools#VisualStates) the solution works like this. The tool's detailed manual can be found [here](https://jderobot.org/VisualStates). 

{% include youtubePlayer.html id=page.youtubeId3 %}

