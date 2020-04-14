---
permalink: /exercises/ComputerVision/3d_reconstruction
title: "3D Reconstruction"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC 3D Reconstruction"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/3d_reconstruction/3d_reconstruction.png
    image_path: /assets/images/exercises/3d_reconstruction/3d_reconstruction.png
    alt: "3D Reconstruction"
    title: "3D Reconstruction"


youtubeId1: 11pxsE__DPw
youtubeId2: r_xrY5-whmc
---

In this practice, the intention is to program the necessary logic to allow kobuki robot to generate a 3D reconstruction of the scene that it is receiving throughout its left and right cameras.

{% include gallery caption="Scene to reconstruct" %}

## Installation
Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

## How to run your solution?
Navigate to the 3d_reconstruction directory

```bash
cd exercises/3d_reconstruction
```

Launch Gazebo with the kobuki_3d_reconstruction world through the command 

```bash
roslaunch ./launch/3d_reconstruction_ros.launch
```

Then you have to execute the academic application, which will incorporate your code:

```bash
python2 ./3d_reconstruction.py 3d_reconstruction_conf.yml
```

## How to perform the exercise?
To carry out the exercise, you have to edit the file `MyAlgorithm.py` and insert in it your code, which reconstructs 3d points from the two stero views.

### Where to insert the code?
In the `MyAlgorithm.py` file,

```python
def algorithm(self):
	#GETTING THE IMAGES
	# imageR = self.getImage('right')
	# imageL = self.getImage('left')

	#EXAMPLE OF HOW TO PLOT A RED COLORED POINT
	# position = [1.0, 0.0, 0.0]	X, Y, Z
	# color = [1.0, 0.0, 0.0]	R, G, B
	# self.point.plotPoint(position, color) 
```

### Application Programming Interface

* `self.getImage('left')` - to get the left image
* `self.getImage('right')` - to get the right image
* `self.point.PlotPoint(position, color)` - to plot the point in the 3d tool

## Theory
Coming Soon

## Hints
Coming Soon

## Template for Exercise
{% include youtubePlayer.html id=page.youtubeId1 %}

## Reference Solution
{% include youtubePlayer.html id=page.youtubeId2 %}

