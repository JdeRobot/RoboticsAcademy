---
permalink: "/exercises/MobileRobots/montecarlo_laser_loc/"
title: "MonteCarlo laser-based robot localization"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Montecarlo Laser Loc"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png
    image_path: /assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png
    alt: "Montecarlo Laser Loc"
    title: "Montecarlo Laser Loc"

model:
  - url: /assets/images/exercises/laser_loc/probab_model.png
    image_path: /assets/images/exercises/laser_loc/probab_model.png
    alt: "Probabilistic location model"
    title: "Probabilistic location model"

diagram:
  - url: /assets/images/exercises/laser_loc/paticle_filter_diagram.png
    image_path: /assets/images/exercises/laser_loc/paticle_filter_diagram.png
    alt: "Diagram of the particle filter algorithm"
    title: "Diagram of the particle filter algorithm"

evolution:
  - url: /assets/images/exercises/laser_loc/particle_filter_evolution.png
    image_path: /assets/images/exercises/laser_loc/particle_filter_evolution.png
    alt: "Evolution of particles"
    title: "Evolution of particles"

youtubeId: y7rBPpV2NdI
---

## Goal

The aim of this practical is to develop a localization algorithm based on the particle filter using the robot's laser.

{% include gallery caption="MonteCarlo Laser Location." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Robot API

* `import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.setW()` - to set the angular velocity
* `HAL.setV()` - to set the linear velocity

* `HAL.getPose3d().x` - to get the X coordinate of the robot
* `HAL.getPose3d().y` - to get the Y coordinate of the robot
* `HAL.getPose3d().yaw` - to get the orientation of the robot

* `HAL.getBumperData().state` - To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* `HAL.getBumperData().bumper` - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.
* `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).

* `GUI.showParticles(particles)` - shows the particles on the map. Accepts a list of particles as an argument. Each particle must be a list with [position_x, position_y, angle_in_radians]. The values must be in gazebo world coordinate system.
* `GUI.showPosition(x, y, yaw)` - shows the estimated user position in the map view in blue. Accepts a list with [position_x, position_y, angle_in_radians]. The values must be in gazebo world coordinate system. The map view will also show the real position of the robot in red, so you can compare how good your algorithm is.
* `GUI.mapToPose(x, y, yaw)`- converts a map pixel to gazebo world coordinate system position.
* `GUI.poseToMap(x, y, yaw)`- converts a gazebo world coordinate system position to a map pixel.
* `GUI.getMap(url)` - Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A). The image is 1012x1012. The instruction to get the map is

```python
array = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/montecarlo_laser_loc/resources/mapgrannyannie.png')
```

### Types conversion

#### Laser

```python
import math
import numpy as np

def parse_laser_data(laser_data):
    """ Parses the LaserData object and returns a tuple with two lists:
        1. List of  polar coordinates, with (distance, angle) tuples,
           where the angle is zero at the front of the robot and increases to the left.
        2. List of cartesian (x, y) coordinates, following the ref. system noted below.

        Note: The list of laser values MUST NOT BE EMPTY.
    """
    laser_polar = []  # Laser data in polar coordinates (dist, angle)
    laser_xy = []  # Laser data in cartesian coordinates (x, y)
    for i in range(180):
        # i contains the index of the laser ray, which starts at the robot's right
        # The laser has a resolution of 1 ray / degree
        #
        #                (i=90)
        #                 ^
        #                 |x
        #             y   |
        # (i=180)    <----R      (i=0)

        # Extract the distance at index i
        dist = laser_data.values[i]
        # The final angle is centered (zeroed) at the front of the robot.
        angle = math.radians(i - 90)
        laser_polar += [(dist, angle)]
        # Compute x, y coordinates from distance and angle
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        laser_xy += [(x, y)]
    return laser_polar, laser_xy

# Usage
laser_data = HAL.getLaserData()
if len(laser_data.values) > 0:
    laser_polar, laser_xy = parse_laser_data(laser_data)
```

## Theory

Probabilistic localisation seeks to estimate the position of the robot and the model of the surrounding environment:

* A. Probabilistic motion model. Due to various sources of noise (bumps, friction, imperfections of the robot, etc.) it is very difficult to predict the movement of the robot accurately. Therefore, it would be more convenient to describe this movement by means of a probability function, which will move with the robot's movements [1].
* B. Probabilistic model of sensory observation.  It is related to the sensor measurements at each instant of time. This model is built by taking observations at known positions in the environment and calculating the probability that the robot is in each of these positions [2].
* C. Probability fusion.  This consists of accumulating the information obtained at each time instant, something that can be done using Bayes' theorem. This fusion achieves that in each observation some modes of the probability function go up and others go down, so that as the number of iterations advances, the probability will be concentrated in only one of the modes, which will indicate the position of the robot [3].

The following figure shows an example of probabilistic localisation. In the first phase, the robot does not know its initial state, the initial probability distribution is uniform. In the second phase, the robot is looking at a door, the sensory observation model determines that there are three zones or modes with equal probability of being the zone where the robot is.  In the third phase, the robot is moving forward so the probabilistic motion model is applied, the probability distribution should move the same distance that the robot has moved, but as estimating the motion is difficult, what is done is to smooth it.  In the last phase, the robot detects another door and this observation is merged with the accumulated information. This causes the probability to concentrate on a single possible area where it can be found, and thus ends the global localisation process.

{% include gallery id="model" caption="Probabilistic location model" %}

## Montecarlo

* Monte Carlo localisation is based on a collection of particles or samples. Particle filters allow the localisation problem to be solved by representing the a posteriori probability function, which estimates the most likely positions of the robot. The a posteriori probability distribution is sampled, where each sample is called a particle [4].
* Each particle represents a state (position) at time t and has an associated weight. At each movement of the robot, they perform a correction and decrease the accumulated error. After a number of iterations, the particles are grouped in the zones with the highest probability, until they converge to a single zone, which corresponds to the robot's position.
* When the programme starts, the robot does not know where it is. However, the actual samples are evenly distributed, and the importance weights are all equal. evenly distributed, and the importance weights are all equal. After a long time, the samples near the current the current position are more likely, and those further away are less likely. The basic algorithm is as follows:
  1. Initialise the set of samples. Their locations are evenly distributed and have the same weights.
  2. Repeat for each sample until: a) Move the robot a fixed distance and read the sensor. b) For each particle, update the location. c) Assign the importance weights of each particle to the probability of that sensor, and read that new location.
  3. Create a collection of samples, by sampling with replacement from the current set of samples, based on their importance weights.
  4. Let the group become the current round of samples.

{% include gallery id="diagram" caption="Diagram of the particle filter algorithm" %}

* The following figure shows an example of the operation of the particulate filter. At the initial instant the particles are uniformly distributed in the environment. As new observations are obtained, the particles accumulate in probability zones until they converge to the probability zone [5].

{% include gallery id="evolution" caption="Probabilistic location model" %}

## Contributors

* Contributors: [Jose María Cañas](https://github.com/jmplaza), [Vladislav Kravchenko](https://github.com/vladkrav)
* Maintained by [David Valladares](https://github.com/dvalladaresv)

## References

1. [http://www.natalnet.br/lars2013/WGWR-WUWR/122602.pdf](http://www.natalnet.br/lars2013/WGWR-WUWR/122602.pdf)
2. [https://robotica.unileon.es/vmo/pubs/robocity2009.pdf](https://robotica.unileon.es/vmo/pubs/robocity2009.pdf)
3. [https://core.ac.uk/download/pdf/60433799.pdf](https://core.ac.uk/download/pdf/60433799.pdf)
4. [http://intranet.ceautomatica.es/old/actividades/jornadas/XXIX/pdf/315.pdf](http://intranet.ceautomatica.es/old/actividades/jornadas/XXIX/pdf/315.pdf)
5. [https://www.researchgate.net/publication/283623730_Calculo_de_incertidumbre_en_un_filtro_de_particulas_para_mejorar_la_localizacion_en_robots_moviles](https://www.researchgate.net/publication/283623730_Calculo_de_incertidumbre_en_un_filtro_de_particulas_para_mejorar_la_localizacion_en_robots_moviles)
