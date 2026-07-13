---
permalink: /exercises/ComputerVision/visual_odom
title: "Visual 3D Odometry"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Odometry"
toc_icon: "cog"

correlation:
  - url: /assets/images/exercises/visual_odometry/correlation.png
    image_path: /assets/images/exercises/visual_odometry/correlation.png
    alt: "Feature correlation between frames"
    title: "Feature correlation between frames"

trajectories:
  - url: /assets/images/exercises/visual_odometry/trajectories.png
    image_path: /assets/images/exercises/visual_odometry/trajectories.png
    alt: "Ground truth and estimated trajectories"
    title: "Ground truth and estimated trajectories"

spacethree:
  - url: /assets/images/exercises/visual_odometry/3dspace.png
    image_path: /assets/images/exercises/visual_odometry/3dspace.png
    alt: "3D space trajectory visualization"
    title: "3D space trajectory visualization"

youtubeId: B95ZKyl_bIk
---

## Goal

The objective of this exercise is to implement a **Visual Odometry system using monocular images**, capable of working with two different input sources:

- A **user-selected video file**
- A **ROS2 KITTI dataset in rosbag format**

The system estimates the motion of a camera by tracking visual features over time and reconstructing the trajectory in a 3D world representation.

The performance of the algorithm is visualized in real time in a **3D viewer representing the robot workspace**, along with optional Ground Truth when available.

{% include gallery id="spacethree" caption="Reconstructed camera trajectory in 3D space." %}

## How to run

The exercise supports two input modes.

### 1. ROS 2 Bag Mode (Dataset)

In this mode, the system reads data from a ROS 2 bag file. The rosbag must contain the following topics:

- `/kitti/camera/gray/left/image_raw` - the monocular image stream. Message type: `sensor_msgs/msg/Image`.
- `/kitti/gt/pose` - the ground truth camera position, used to draw the reference trajectory. Message type: `geometry_msgs/msg/PoseStamped`.

The rosbag must be compatible with ROS 2 Humble, otherwise playback may fail.

Place your dataset inside `/RoboticsAcademy/exercises/visual_odom/frontend/resources/`, for example:

```
/RoboticsAcademy/exercises/visual_odom/frontend/resources/your_rosbag_name/
    metadata.yaml
    your_rosbag_name.db3
```

Then, inside the Unibotics / Docker environment, play it with:

```
ros2 bag play /RoboticsAcademy/exercises/visual_odom/frontend/resources/your_rosbag_name
```

Make sure the ROS 2 topics are being published correctly before starting the exercise.

### 2. Video Mode (User Input)

In this mode, the user selects a video file from their local machine. The video is streamed frame by frame to the algorithm, no rosbag or ROS topics are involved, and ground truth is not available. This mode is useful for debugging, testing algorithms quickly and offline development.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `WebGUI.getImage()` - to get the current camera frame (numpy array). It can be None.
- `WebGUI.getGT()` - to get the ground-truth position `[x, y, z]`. It is None when no ground truth is available (video mode).
- `WebGUI.showImage(image)` - allows you to view a debug image or one with relevant information.
- `WebGUI.showEstimatedPoint([x, y, z])` - allows you to view your estimated camera position in the 3D viewer.

#### C++

- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `WebGUI::get_image();` - to get the current camera frame as a `cv::Mat`. It may be empty; check with `image.empty()`.
- `WebGUI::get_gt();` - returns the ground-truth position as a `std::vector<double>` `{x, y, z}`. It is empty when no ground truth is available (video mode).
- `WebGUI::show_image(image);` - allows you to view a debug image (`cv::Mat`) or one with relevant information.
- `WebGUI::show_estimated_point(point);` - displays the user-estimated camera position in the 3D viewer. The input must be a `std::vector<double>` containing `{x, y, z}`. Returns `void`.

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "WebGUI.hpp"
#include "Frequency.hpp"

void exercise() {
    Frequency freq = Frequency();
    // Enter sequential code!

    while (true)
    {
        // Enter iterative code!
        freq.tick();


    }
}
```

### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication.

- `/visual_odom/image_raw` - Subscribe to this topic to receive the input camera frame (BGR8), whether it comes from the video or the rosbag. WebGUI republishes it on this single topic so a ROS 2-direct node does not need to care about the source. Message type: `sensor_msgs/msg/Image`. QoS: default profile, depth `10`, `RELIABLE`.

- `/kitti/gt/pose` - Subscribe to this topic to receive the ground-truth pose (available only in rosbag mode, published directly by the rosbag). Message type: `geometry_msgs/msg/PoseStamped`. QoS: `BEST_EFFORT`, depth `10`. The KITTI rosbag publishes with `BEST_EFFORT` reliability, so a `RELIABLE` subscriber would silently receive nothing.

For WebGUI debugging:

- `/webgui/estimated_point` - Publish to this topic to display the estimated camera position in the 3D viewer. Message type: `geometry_msgs/msg/PointStamped`. QoS: default profile, depth `10`.

- `/webgui/image_debug` - Publish to this topic to display a debug image in the WebGUI. Message type: `sensor_msgs/msg/Image`, encoding `bgr8`. QoS: default profile, depth `10`.

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

#### C++

In order to use direct ros controls you must include the following lines:

```cpp
#ifndef USER_NODE
#define USER_NODE

#include "rclcpp/rclcpp.hpp"

class UserNode : public rclcpp::Node {
  // Your class
};

#endif
```

You must define `USER_NODE` and a `UserNode` node class.

To have frequency control you may use a timer and a control function as follows:

```cpp
  UserNode() : Node("user_node")
  {
    // More subscribers and publishers
    timer_ = create_wall_timer(100ms, std::bind(&UserNode::control_cycle, this));
  };

// More Code

  void control_cycle(){
    // Your function
  };
```

## Theory

Visual Odometry (VO) is the process of estimating the motion of a camera over time by analyzing the changes between consecutive images.

In this exercise, the input is a monocular camera stream, either the KITTI dataset or a video, and the goal is to reconstruct the trajectory of the camera in 3D.

### Key idea

We estimate motion by tracking how image features move between frames. Instead of using wheel encoders or IMU data, VO relies only on the pixel motion in the image, the camera calibration and geometric constraints.

{% include gallery id="correlation" caption="Feature correspondences between consecutive frames used for motion estimation." %}

### Camera model and intrinsic matrix

To convert pixel motion into real geometric motion, we need the intrinsic camera matrix:

```
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
```

`fx` and `fy` are the focal length, which sets the scale of the projection, and `cx` and `cy` are the principal point, the optical center of the image. In this exercise we use the KITTI calibration:

```python
K = np.array([
    [718.8560, 0.0, 607.1928],
    [0.0, 718.8560, 185.2157],
    [0.0, 0.0, 1.0]
])
```

Without this matrix we cannot recover real motion from pixel coordinates.

### Feature detection

The first step is to extract salient points in the image that are easy to track over time, using Shi-Tomasi corner detection (`cv2.goodFeaturesToTrack`) or, as an optional improvement, FAST features. These points typically represent corners of objects, textured regions and other high-gradient areas.

### Feature tracking with optical flow

Once features are detected, we track them between consecutive frames using Lucas-Kanade optical flow:

```python
cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts)
```

This gives us `p1`, the points in the previous frame, and `p2`, the corresponding points in the current frame, which together describe the 2D motion of pixels over time.

### Outlier rejection

Not all tracked points are valid: some carry large motion caused by noise, some are incorrect matches and some are simply unstable tracks. On top of discarding these, we use RANSAC inside the essential matrix estimation itself:

```python
cv2.findEssentialMat(p1, p2, K, method=cv2.RANSAC)
```

This removes geometric outliers automatically.

### Essential matrix estimation

The essential matrix `E` encodes the relative rotation and translation between two camera frames. It is computed as:

```python
E = cv2.findEssentialMat(p1, p2, K)
```

and satisfies the epipolar constraint `p2^T * E * p1 = 0`, which is the key geometric relationship used to recover the camera motion.

### Pose recovery

From the essential matrix, we recover the rotation `R` and the translation direction `t`:

```python
retval, R, t, _ = cv2.recoverPose(E, p1, p2, K)
```

The translation is recovered only up to scale: its direction is correct, but its magnitude is unknown.

### Scale problem

Monocular VO cannot directly recover real scale. In this exercise the scale can be approximated heuristically, normalized per frame, or accumulated with a constant factor, for example:

```python
t = t / np.linalg.norm(t)
```

### Trajectory integration

The final camera position is obtained by accumulating motion frame by frame:

```python
t_global = t_global + R_global @ t
R_global = R_global @ R
```

### Output

The system outputs the estimated camera trajectory in 3D, together with a live visualization of the feature tracking and an overlay of the motion in the GUI:

```python
WebGUI.showEstimatedPoint([x, y, z])
```

{% include gallery id="trajectories" caption="Example of ground-truth and estimated trajectories." %}

### Summary

The full pipeline is feature detection, followed by optical flow tracking, outlier rejection with RANSAC, essential matrix estimation, pose recovery and, finally, trajectory integration.

The whole system works because 2D pixel motion, combined with the calibration matrix `K`, allows the recovery of 3D camera motion. This is the core principle of monocular visual odometry.

## Hint

Simple hints provided to help you solve the exercise. Please note that the **following hint is only a suggestive approach. Any other algorithm to solve the exercise is acceptable.**

1) Get the RGB images either from the ROS 2 bag topic `/kitti/camera/gray/left/image_raw` or from the video input selected by the user.

2) Detect feature points in the first frame using FAST or the Shi-Tomasi corner detector.

3) Track the detected features in the next frame using the Lucas-Kanade optical flow algorithm.

4) Filter incorrect correspondences using the optical flow status output and a RANSAC consistency check.

5) Estimate the relative motion between consecutive frames using the essential matrix (`cv2.findEssentialMat`) and pose recovery (`cv2.recoverPose`).

6) Integrate the estimated motion over time to reconstruct the trajectory of the camera.

7) Show the estimated position in the 3D viewer using the provided GUI function.

## Demonstration video

{% include youtubePlayer.html id=page.youtubeId %}

## Contributors

- Contributors: Jose Miguel Jiménez, [Jose María Cañas](https://github.com/jmplaza).
