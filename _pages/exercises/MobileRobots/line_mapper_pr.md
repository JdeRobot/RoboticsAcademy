---
permalink: /exercises/MobileRobots/line_mapper_pr
title: "Laser Line Mapping"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Laser Line Mapping"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/line_mapper/line_mapper_teaser.png
    image_path: /assets/images/exercises/line_mapper/line_mapper_teaser.png
    alt: "Laser Line Mapping"
    title: "Laser Line Mapping"



segment_map:
  - url: /assets/images/exercises/line_mapper/segment_map.png
    image_path: /assets/images/exercises/line_mapper/segment_map.png
    alt: "Resulting segment map"
    title: "Resulting segment map"

youtubeId1: 1c5vxZyYtSQ
---

## Goal

The goal of this exercise is to implement a laser-based line segment mapping algorithm that builds a geometric map of a warehouse environment using a mobile robot equipped with a 2D laser scanner.

The robot must be able to:
* Process raw laser scan data and project it into world coordinates
* Extract line segments from the laser point cloud
* Maintain a consistent global map by merging new detections with existing segments
* Visualize the resulting segment map in real time on the WebGUI

{% include gallery caption="Laser Line Mapping in a warehouse." %}

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

### Python

* `import HAL` - to import the HAL library class. This class contains the functions that receive information from the sensors or work with the actuators.
* `import WebGUI as GUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to visualize the segment map.
* `HAL.getPose3d().x` - to get the X position of the robot in world coordinates (meters).
* `HAL.getPose3d().y` - to get the Y position of the robot in world coordinates (meters).
* `HAL.getPose3d().yaw` - to get the orientation of the robot in radians.
* `HAL.setV(v)` - to set the linear velocity of the robot (m/s).
* `HAL.setW(w)` - to set the angular velocity of the robot (rad/s).
* `HAL.getLaserData()` - returns the laser scan data object with the following fields:
  * `.values` - list of range measurements (meters)
  * `.minAngle` - minimum scan angle (radians)
  * `.maxAngle` - maximum scan angle (radians)
  * `.maxRange` - maximum valid range (meters)
* `GUI.addLine(color, p1, p2)` - draws a line segment on the map canvas between two world-coordinate points. `color` is an RGB tuple e.g. `(0, 200, 255)`. `p1` and `p2` are world-coordinate tuples `(x, y)` in meters.
* `GUI.clearSegments()` - clears all segments previously drawn on the map canvas.
* `GUI.getMap()` - returns the warehouse map as a NumPy array (RGB).
* `GUI.worldToMap(x, y)` - converts world coordinates (meters) to map pixel coordinates `(col, row)`.
* `GUI.mapToWorld(col, row)` - converts map pixel coordinates `(col, row)` to world coordinates `(x, y)` in meters.

## Odometry Noise Variants

This exercise provides three universe variants to test the robustness of the mapping algorithm under different odometry noise conditions:

* **Rover 4wd Warehouse** — no odometry noise. The robot pose is perfectly accurate. The resulting segment map should closely match the real geometry of the warehouse.
* **Rover 4wd Warehouse Low Noise** — low odometry noise. Small drift accumulates over time. The segment map will show minor misalignments between segments observed from different robot positions.
* **Rover 4wd Warehouse High Noise** — high odometry noise. Significant drift accumulates. The segment map will show visible inconsistencies as the robot moves further from its starting position.

Comparing the maps produced across the three variants illustrates the impact of odometry error on geometric mapping and motivates the use of localization or loop closure techniques in real systems.

## Theory

### Laser Scan Projection

The raw laser data consists of range measurements at known angles relative to the robot's laser frame. To build a world-frame map, each range measurement must be transformed into world coordinates using the robot's current pose.

Each laser beam is described by a distance and an angle relative to the robot. The angle of each beam is computed from the scan parameters:

```
angle_increment = (maxAngle - minAngle) / num_beams
angle_i = minAngle + i * angle_increment
```

The Cartesian coordinates of the beam endpoint in the laser frame are:

```
lx = dist * cos(angle_i)
ly = dist * sin(angle_i)
```

These local coordinates are then rotated into the robot body frame and projected into world coordinates using the robot pose `(x, y, yaw)`.

The laser sensor is typically mounted at an offset from the robot's geometric center. This offset must be added to the robot position before projecting the scan points into the world frame.

### Line Extraction from Point Clouds

Once the laser scan is projected into world coordinates, the resulting point cloud can be processed to extract line segments that represent the flat surfaces and walls in the environment.

A typical pipeline consists of three stages:

**1. Line fitting**

A line model is fitted to a subset of points. The goal is to find the line that best explains the maximum number of nearby points. RANSAC (Random Sample Consensus) is a classical approach: it randomly samples pairs of points, fits a line through them, and counts how many other points lie within a distance threshold. The iteration with the most inliers is kept.

**2. Segment extraction**

The inlier points are projected onto the fitted line direction and sorted. Gaps larger than a threshold break the sorted sequence into individual segments. Segments that are too short or too long are discarded.

**3. Map update (merge or add)**

Each extracted segment is compared against the existing global map. If a compatible segment already exists — similar orientation, close distance, overlapping position — the two are merged into an extended segment. Otherwise the new segment is added to the map.

{% include gallery id="segment_map" caption="Example line map built over the warehouse." %}

### Segment Compatibility

Two segments are considered compatible if they satisfy a set of geometric criteria simultaneously:

* Their orientations differ by less than an angular threshold.
* The midpoint of the new segment lies close to the infinite line supporting the existing segment.
* The endpoints of both segments are spatially close enough to suggest they describe the same physical surface.

### Segment Lifetime

In a noisy environment, false detections will appear and disappear. A robust mapper should filter them out. Two common strategies are:
line mapper RA
* **Hit counting:** each segment accumulates a counter each time a compatible observation confirms it. Only segments with enough confirmations are rendered in the GUI. Segments that stop receiving confirmations are eventually removed.

* **Negative evidence:** if the laser beam passes through the location of a segment without detecting an obstacle, this counts as evidence against it. Segments that accumulate enough such misses are deleted.

### Effect of Odometry Noise

All coordinate transformations in this exercise rely on the robot's pose estimate provided by `HAL.getPose3d()`. Any error in this estimate directly degrades the quality of the map. As the robot moves further from its starting position, small errors in each pose estimate accumulate, causing segments observed from different locations to appear misaligned.

The three universe variants (no noise, low noise, high noise) allow the user to observe this degradation experimentally and understand the fundamental relationship between localization accuracy and map quality.

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

- Contributors: [Carlos Iglesias](https://github.com/ciglesias1995)
- Maintained by [Carlos Iglesias](https://github.com/ciglesias1995)