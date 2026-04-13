---
permalink: /exercises/ComputerVision/montecarlo_visual_loc
title: "Montecarlo Visual Loc"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Montecarlo Visual Loc"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/montecarlo_visual_loc/montecarlo_visual_loc_teaser.png
    image_path: /assets/images/exercises/montecarlo_visual_loc/montecarlo_visual_loc_teaser.png
    alt: "Montecarlo Visual Loc"
    title: "Montecarlo Visual Loc"
    
model:
  - url: /assets/images/exercises/montecarlo_visual_loc/probab_model.png
    image_path: /assets/images/exercises/montecarlo_visual_loc/probab_model.png
    alt: "Probabilistic location model"
    title: "Probabilistic location model"
    
diagram:
  - url: /assets/images/exercises/montecarlo_visual_loc/paticle_filter_diagram.png
    image_path: /assets/images/exercises/montecarlo_visual_loc/paticle_filter_diagram.png
    alt: "Diagram of the particle filter algorithm"
    title: "Diagram of the particle filter algorithm"

evolution:
  - url: /assets/images/exercises/montecarlo_visual_loc/particle_filter_evolution.png
    image_path: /assets/images/exercises/montecarlo_visual_loc/particle_filter_evolution.png
    alt: "Evolution of particles"
    title: "Evolution of particles"

youtubeId1: xUpTw0_jt5s
youtubeId2: n-Fk4VCfvL8

---

## Goal

The objective of this exercise is to develop a visual localisation algorithm based on the particle filter.

{% include gallery caption="Gallery" %}


## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to get the image.
* `HAL.setV()` - to set the linear speed.
* `HAL.setW()` - to set the angular velocity.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate).
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate).
* `HAL.getPose3d().yaw` - to get the orientation of the robot.
* `HAL.getOdom().x` - to get the approximated X coordinate of the robot (with noise).
* `HAL.getOdom().y` - to get the approximated XY coordinate of the robot (with noise).
* `HAL.getOdom().yaw` - to get the approximated orientation position of the robot (with noise).
* `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).
* `WebGUI.showImage()` - allows you to view a debug image or with relevant information.
* `WebGUI.showParticles(particles)` - shows the particles on the map. Accepts a list of particles as an argument. Each particle must be a list with [position_x, position_y, angle_in_radians, weight]. The values must be in gazebo world coordinate system.
* `WebGUI.showPosition(x, y, yaw)` - shows the estimated user position in the map view in blue. Accepts a list with [position_x, position_y, angle_in_radians]. The values must be in gazebo world coordinate system. The map view will also show the real position of the robot in red, so you can compare how good your algorithm is.
* `WebGUI.mapToPose(x, y, yaw)`- converts a map pixel to gazebo world coordinate system position.
* `WebGUI.poseToMap(x, y, yaw)`- converts a gazebo world coordinate system position to a map pixel.
* `WebGUI.getMap(url)` - Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A), each value ranging from 0 to 1. The image is 1012x1012.
* `WebGUI.getBGRMap(url)` - Returns a numpy array with the image data in a 3 dimensional array (B, G, R), each value ranging from 0 to 255. The image is 1012x1012.

The instruction to get the map is:

```python
array = WebGUI.getMap('/resources/exercises/montecarlo_visual_loc/images/mapgrannyannie.png')
```

The instruction to get the image with the roof textures is:

```python
array = WebGUI.getColorMap('/resources/exercises/montecarlo_visual_loc/images/color_mapgrannyannie.png')
```
### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

- `/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`

- `/odom` - Subscribe to this topic to receive the robot ground-truth odometry. Message type: `nav_msgs/msg/Odometry`

- `/odom_noisy` - Subscribe to this topic to receive the noisy odometry. Message type: `nav_msgs/msg/Odometry`

- `/roombaROS/laser/scan` - Subscribe to this topic to receive laser data. Message type: `sensor_msgs/msg/LaserScan`

- `/camera/image_raw` - Subscribe to this topic to receive the camera image. Message type: `sensor_msgs/msg/Image`

- `/webgui/estimated_pose` - Publish to this topic to display the estimated robot pose in the WebGUI. Message type: `geometry_msgs/msg/PoseStamped`  
  QoS: `TRANSIENT_LOCAL`, depth `1`

- `/webgui/particles` - Publish to this topic to display the particle set in the WebGUI. Message type: `geometry_msgs/msg/PoseArray`  
  QoS: `TRANSIENT_LOCAL`, depth `1`

- `/webgui/image_debug` - Publish to this topic to display a debug image in the WebGUI. Message type: `sensor_msgs/msg/Image`

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

## Theory

Probabilistic localisation seeks to estimate the position of the robot and the model of the surrounding environment:

* A. Probabilistic motion model. Due to various sources of noise (bumps, friction, imperfections of the robot, etc.) it is very difficult to predict the movement of the robot accurately. Therefore, it would be more convenient to describe this movement by means of a probability function, which will move with the robot's movements [1].
* B. Probabilistic model of sensory observation. It is related to the sensor measurements at each instant of time. This model is built by taking observations at known positions in the environment and calculating the probability that the robot is in each of these positions [2].
* C. Probability fusion. This consists on accumulating the information obtained each instant, something that can be done using Bayes' theorem. This fusion achieves a result in which in each observation some modes of the probability function go up and others go down, so that as the number of iterations advances, the probability will be concentrated in only one of the modes, which will indicate the position of the robot [3].

The following figure shows an example of probabilistic localisation. In the first phase, the robot does not know its initial state, the initial probability distribution is uniform. In the second phase, the robot is looking at a door, the sensory observation model determines that there are three zones or modes with equal probability of being the zone where the robot is. In the third phase, the robot is moving forward so the probabilistic motion model is applied, the probability distribution should move the same distance that the robot has moved, but as estimating the motion is difficult, what is done is to smooth it. In the last phase, the robot detects another door and this observation is merged with the accumulated information. This causes the probability to concentrate on a single possible area where it can be found, and thus ends the global localisation process.

{% include gallery id="model" caption="Probabilistic location model" %}

### Monte Carlo

* Monte Carlo localisation is based on a collection of particles or samples. Particle filters allow the localisation problem to be solved by representing the a posteriori probability function, which estimates the most likely positions of the robot. The a posteriori probability distribution is sampled, in a way where each sample is called a particle [4].
* Each particle represents a state (position) at time t and has an associated weight. With each movement of the robot, they perform a correction and decrease the accumulated error. After a number of iterations, the particles are grouped in the zones with the highest probability, until they converge to a single zone, which corresponds to the robot's position.
* When the program starts, the robot does not know where it is. However, the actual samples are evenly distributed, and the importance weights are all equal. After a long time, the samples nearing the current position have a higher probability, and those further away have a lower pobability. The basic algorithm is as follows:
  1. Initialise the set of samples. Their locations are evenly distributed and have the same weight.
  2. Repeat for each sample until: a) Move the robot a fixed distance and read the sensor. b) For each particle, update the location. c) Assign the importance weights of each particle to the probability of that sensor, and read that new location.
  3. Create a collection of samples, by sampling with replacements from the current set of samples, based on their importance weights.
  4. Let the group become the current round of samples.

{% include gallery id="diagram" caption="Diagram of the particle filter algorithm" %}

* The following figure shows an example of the operation of the particulate filter. At the initial instant the particles are uniformly distributed in the environment. As new observations are obtained, the particles accumulate in probability zones until they converge to the probability zone [5].

{% include gallery id="evolution" caption="Probabilistic location model" %}

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId2 %}

## Contributors

* Contributors: [Jose María Cañas](https://github.com/jmplaza), [David Valladares](https://github.com/dvalladaresv)
* Maintained by [David Valladares](https://github.com/dvalladaresv)

## References

1. [http://www.natalnet.br/lars2013/WGWR-WUWR/122602.pdf](http://www.natalnet.br/lars2013/WGWR-WUWR/122602.pdf)
2. [https://robotica.unileon.es/vmo/pubs/robocity2009.pdf](https://robotica.unileon.es/vmo/pubs/robocity2009.pdf)
3. [https://core.ac.uk/download/pdf/60433799.pdf](https://core.ac.uk/download/pdf/60433799.pdf)
4. [http://intranet.ceautomatica.es/old/actividades/jornadas/XXIX/pdf/315.pdf](http://intranet.ceautomatica.es/old/actividades/jornadas/XXIX/pdf/315.pdf)
5. [https://www.researchgate.net/publication/283623730_Calculo_de_incertidumbre_en_un_filtro_de_particulas_para_mejorar_la_localizacion_en_robots_moviles](https://www.researchgate.net/publication/283623730_Calculo_de_incertidumbre_en_un_filtro_de_particulas_para_mejorar_la_localizacion_en_robots_moviles)
