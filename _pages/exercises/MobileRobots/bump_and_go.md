---
permalink: "/exercises/MobileRobots/bump_and_go/"
title: "Bump and Go"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Bump and Go"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/bump_and_go/bump_and_go_teaser.png
    image_path: /assets/images/exercises/bump_and_go/bump_and_go_teaser.png
    alt: "Bump and Go"
    title: "Bump and Go"

  - url: /assets/images/exercises/bump_and_go/bump_and_go.png
    image_path: /assets/images/exercises/bump_and_go/bump_and_go.png
    alt: "Bump and Go"
    title: "Bump and Go"

youtubeId1: HRjXf2GzK70
youtubeId2: Yon6yKY5oqk
youtubeId3: zA7jN7ZR2sk
---

## Goal

The intention of this excersise is to program a basic behavior of bump-spin using a Finite State Machine. For that, we will use the VisualStates tool, that allows you to create your own states machine in an intuitive way.

There is a Kobuki robot inside a labyrinth or scenario. The robot will go front until it gets close to an obstacle. The it will go back, turn a random angle and go front again repeating the process. This exercise aims to show the power of automata when building robot behavior.

{% include gallery caption="Gallery" %}

{% include youtubePlayer.html id=page.youtubeId1 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId2 %}

<br/>


Using the JdeRobot [VisualStates tool](https://jderobot.github.io/VisualStates) the solution works like this. The tool's detailed manual, installation instructions, etc. can be found [here](https://jderobot.github.io/VisualStates). 

{% include youtubePlayer.html id=page.youtubeId3 %}

