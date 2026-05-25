---
permalink: /exercises/AutonomousCars/dynamic_window_approach
title: "Local navigation with DWA"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Follow Line"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

youtubeId1: 0fsE49EijDc
---

## Goal

The objective of this exercise is to implement the Dynamic Window Approach (DWA) for local navigation.

The robot must:
- Generate a set of admissible linear and angular velocities
- Simulate trajectories for each velocity pair
- Evaluate them based on:
  - Distance to obstacles
  - Alignment with the target
  - Forward velocity
- Select the optimal velocity command

This allows the robot to reach the target while avoiding obstacles in a smooth and dynamically feasible way.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

### HAL-based Implementation

#### Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL.getPose3d().x` - to get the position of the robot (x coordinate)
- `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate)
- `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
- `HAL.getLaserData()` - to obtain laser sensor data
  It is composed of 180 pairs of values: (0-180º distance in meters)
- `HAL.setV()` - to set the linear speed
- `HAL.setW()` - to set the angular velocity
- `WebGUI.getNextTarget()` - to obtain the next target object on the scenario.
- `WebGUI.setTargetx` - sets the x coordinate of the target on the WebGUI.
- `WebGUI.setTargety` - sets the y coordinate of the target on the WebGUI.

To access the target 'x' and 'y' coordinates use (target is the object obtained from WebGUI.getNextTarget):

- `target.getPose().x` - to obtain the x position of the target
- `target.getPose().y` - to obtain the y position of the target

**Own API**

To simplify the exercise, the implementation of control points is offered.
To use it, only two actions must be carried out:

1. Obtain the following point:

   `currentTarget = WebGUI.getNextTarget()`

2. Mark it as visited when necessary:

   `currentTarget.setReached(True)`

As well as the destination that we have assigned:

```python
# Current target
target = [1.0, 1.0]
WebGUI.showLocalTarget(target)

```

<!---
**API**

* `pose3d.getPose3d().x` - to get the position of the robot (x coordinate)
* `pose3d.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `pose3d.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `laser.getLaserData()` - to obtain laser sensor data
  It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `setV()` - to set and send the linear speed
* `setW()` - to set and send the angular velocity

**Own API**

To simplify, the implementation of control points is offered.
To use it, only two actions must be carried out:
1. Obtain the following point:
   `self.currentTarget = self.getNextTarget()`
2. Mark it as visited when necessary:
   `self.currentTarget.setReached(True)` --->

### Conversion of types

**Laser**

The following function parses laser data taking into account 1) laser only has 180º coverage and 2) the measure read at 90º corresponds to the 'front' of the robot.

You must apply the conversions needed to transform that laser data to a vector of the polar coordinates and a vector in the relavite coodinate system of the robot.

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

**Coordinate system**

We have 2 different coordinate systems in this exercise.

- **Absolute coordinate system**: Its origin (0,0) is located in the finish line of the circuit (exactly where the F1 starts the lap).
- **Relative coordinate system**: It is the coordinate system solidary to the robot (F1). Positive values of X means 'forward', and positive values of Y means 'left'.

You can use the following code to convert absolute coordinates to relative ones (solidary to the F1).

```python
def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel
```

## Theory

This exercise requires the implementation of a local navigation algorithm called the **Dynamic Window Approach (DWA)**.

### Navigation

Robot Navigation involves all the tasks required to move a robot from point A to point B **autonomously** without collisions.

The main subproblems are:

- **Localisation**: The robot needs to know where it is.
- **Collision Avoidance**: Detect and avoid obstacles.
- **Mapping**: Represent the environment.
- **Planning**: Compute a path to the goal.
- **Exploration**: Discover unknown areas.

Navigation is typically divided into:

- **Global Navigation** → computes a path using a map.
- **Local Navigation** → reacts to the environment in real time.

---

### Local Navigation

Local navigation adapts the robot motion based on sensor data and robot constraints.

Unlike methods based on artificial forces, the **Dynamic Window Approach (DWA)** operates directly in the **velocity space**, selecting the best motion command at each iteration.

---

### Dynamic Window Approach

The Dynamic Window Approach selects the optimal control command by evaluating possible velocities of the robot.

Instead of computing forces, the robot:

- Evaluates possible velocities (v, w)
- Simulates their resulting trajectories
- Selects the best one according to a cost function

---

### Algorithm Steps

At each control cycle:

1. **Obtain Dynamic Window**

   The valid velocity ranges are provided by the system:

   ```python
   v_min, v_max, w_min, w_max = getDynamicWindowLimits()
   ```

   These limits already take into account:
   - Current robot velocity
   - Maximum acceleration
   - Velocity constraints

   **Note:** You are not required to implement this step.

---

2. **Sample Velocities**

   Generate candidate velocity pairs (v, w) within the dynamic window.

---

3. **Simulate Trajectories**

   For each candidate velocity, predict the robot motion over a short time horizon.

   The motion model is:

   ```python
   x = x + v * cos(theta) * dt  
   y = y + v * sin(theta) * dt  
   theta = theta + w * dt
   ```

---

4. **Evaluate Trajectories**

   Each trajectory is scored based on:

   - **Heading** → alignment with the goal  
   - **Clearance** → distance to obstacles  
   - **Velocity** → forward speed  

---

5. **Select Best Command**

   The velocity pair (v, w) with the highest score is applied to the robot.

---

### Cost Function

Each trajectory is evaluated using a weighted sum:

```python
score = α * heading + β * clearance + γ * velocity
```

Where:

- **heading**: how well the robot is oriented towards the goal  
- **clearance**: distance to the closest obstacle  
- **velocity**: forward speed  

The weights (α, β, γ) determine the robot behavior.

---

### Debugging and Visualization

To help understand the behavior of the Dynamic Window Approach, the WebGUI provides visualization tools.

#### Dynamic Window

You can visualize the sampled velocity space:

```python
showDynamicWindow(dynamic_window)
```

Where:
- `dynamic_window` contains the sampled velocity pairs (v, w)

This allows you to see which velocities are being considered at each iteration.

---

#### Best Velocity

You can also visualize the selected control command:

```python
showBestVelocity(best_vw)
```

Where:
- `best_vw` is the selected pair (v, w)

This helps to understand how the algorithm chooses the optimal motion.

---

#### Notes

- Use these functions for debugging and tuning
- They are especially useful to verify:
  - Velocity sampling
  - Cost function behavior
  - Stability of the controller

### Advantages of DWA

- Considers robot dynamics  
- Produces smooth and realistic motion  
- Avoids oscillations present in force-based methods  
- Ensures feasible velocity commands  

## Hints

Visualizing the dynamic window can help detect issues such as:
- Poor sampling resolution
- Incorrect cost weighting
- Unsafe trajectory selection

When displaying the dynamic window in the WebGUI, it may be useful to **normalize the scores only for visualization purposes**.

- Do **not** normalize values when computing the real cost function.
- Use normalization only when sending data to the GUI.

This improves readability of the dynamic window representation, making it easier to compare candidate velocities visually.

---

For example, you can scale values to the range [0, 1]:

```python
normalized_value = (value - min_value) / (max_value - min_value)
```

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Eduardo Perdices](eperdices@gsyc.es), [Francisco Pérez](https://github.com/fqez), Victor Arribas, [Julio Vega](julio.vega@urjc.es), [Jose María Cañas](https://gsyc.urjc.es/jmplaza/), [Nacho Arranz](https://github.com/igarag), [Javier Izquierdo](https://github.com/javizqh).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).
