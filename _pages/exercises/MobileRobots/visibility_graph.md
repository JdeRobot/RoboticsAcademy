---
permalink: /exercises/MobileRobots/visibility_graph
title: "Visibility Graph Navigation"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visibility Graph Navigation"
toc_icon: "cog"


gallery:
  - url: /assets/images/exercises/visibility_graph/visibility_graph_teaser.png
    image_path: /assets/images/exercises/visibility_graph/visibility_graph_teaser.png
    alt: "Visibility Graph Navigation"

visibility_graph_warehouse:
  - url: /assets/images/exercises/visibility_graph/visibility_graph_warehouse.png
    image_path: /assets/images/exercises/visibility_graph/visibility_graph_warehouse.png
    alt: "Visibility Graph Warehouse"

youtubeId1: ozeVvHxN5ys
---

## Goal

The goal of this exercise is to implement a Visibility Graph path planning algorithm that allows a robot to navigate autonomously through a warehouse environment from its current position to a target destination.

The robot must be able to:
* Load and process a 2D occupancy map of the environment
* Build a visibility graph connecting obstacle vertices that have line-of-sight to each other
* Find the shortest collision-free path from start to goal using Dijkstra's algorithm
* Follow the computed path by navigating through each waypoint in sequence

{% include gallery caption="Visibility Graph Navigation." %}

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

### Python

* `import HAL` - to import the HAL library class. This class contains the functions that receive information from the sensors or work with the actuators.
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to interact with the map visualization and coordinate conversions.
* `HAL.getPose3d().x` - to get the ground-truth X position of the robot in world coordinates.
* `HAL.getPose3d().y` - to get the ground-truth Y position of the robot in world coordinates.
* `HAL.getPose3d().yaw` - to get the ground-truth orientation of the robot in radians.
* `HAL.setV(v)` - to set the linear velocity of the robot (m/s).
* `HAL.setW(w)` - to set the angular velocity of the robot (rad/s).
* `WebGUI.getMap()` - returns the warehouse map as a NumPy array (RGB, 603×1002 pixels).
* `WebGUI.showNumpy(image)` - displays a NumPy image array on the map canvas. Accepts RGB or grayscale arrays; resizes automatically to 603×1002 if needed.
* `WebGUI.worldToMap(x, y)` - converts world coordinates (meters) to map pixel coordinates `(col, row)`.
* `WebGUI.mapToWorld(col, row)` - converts map pixel coordinates `(col, row)` to world coordinates `(x, y)` in meters.
* `WebGUI.line(color, p1, p2)` - draws a line on the map overlay between two world-coordinate points. `color` is an RGB tuple, e.g. `(0, 255, 0)`. `p1` and `p2` are world-coordinate tuples `(x, y)`.
* `WebGUI.clearLines()` - clears all lines previously drawn on the map overlay.

## Theory

### Path Planning

Path planning is the problem of finding a collision-free route for a robot between a start position and a goal position in an environment with obstacles. Classical approaches represent the environment as a graph and apply graph-search algorithms to find the optimal path.

### Visibility Graph

A visibility graph is a graph-based path planning structure built from the geometry of the obstacles in the environment. The key idea is that the shortest path between two points in a 2D environment with polygonal obstacles always passes through the vertices of those obstacles — never through a point on the interior of an obstacle boundary.

{% include gallery id="visibility_graph_warehouse" caption="Example visibility graph built over the warehouse map." %}

**Construction:**

1. **Obstacle extraction:** The occupancy map is thresholded and the obstacle contours are extracted. To ensure safe navigation, the obstacles are inflated (dilated) by a margin before extracting their vertices.
2. **Vertex extraction:** The corner points of each obstacle contour are collected as candidate graph nodes. The start and goal positions are also added as nodes.
3. **Edge generation:** For every pair of nodes, a line-of-sight check is performed. If the straight segment between two nodes does not intersect any obstacle, an edge is added to the graph.
4. **Shortest path:** Dijkstra's algorithm is applied on the visibility graph to find the minimum-cost path from start to goal.

**Line-of-sight check:**

A segment between two nodes is considered free if no point along it falls inside an obstacle. This check is performed in pixel space over the dilated obstacle mask.

**Dijkstra's algorithm:**

Dijkstra finds the shortest path in a weighted graph. Edge weights are Euclidean distances between nodes in pixel space. The algorithm maintains a priority queue and expands the lowest-cost node first, guaranteeing optimality.

### Obstacle Inflation

Before extracting vertices, the obstacle mask is morphologically dilated using a square kernel. This creates a safety margin so that the robot's body does not clip obstacle edges when following the planned path. The inflation radius should be chosen according to the robot's physical size.

### Illustrations

{% include gallery id="Occupancy_grid" caption="Example visibility graph built over the warehouse map." %}

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

- Contributors: [Carlos Iglesias](https://github.com/ciglesias1995)
- Maintained by [Carlos Iglesias](https://github.com/ciglesias1995)
