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

The exercise supports two input modes:

---

### 1. ROS2 Bag Mode (Dataset)

In this mode, the system reads data from a **ROS2 bag file**.

#### Dataset requirements

The rosbag must contain:

```
/kitti/camera/gray/left/image_raw
```

- Ground truth:
```
/kitti/oxts/gps/fix
```

---

#### Where to place the rosbag

Place your dataset inside:

```
/RoboticsAcademy/exercises/visual_odom/frontend/resources/
```

Example:

```
/RoboticsAcademy/exercises/visual_odom/frontend/resources/your_rosbag_name/
    ├── metadata.yaml
    ├── *.db3
```

---

#### IMPORTANT (ROS2 compatibility)

The rosbag must be compatible with **ROS2 Humble**.

If not, playback may fail.

---

#### How to run rosbag

Inside the Unibotics / Docker environment:

```
ros2 bag play /RoboticsAcademy/exercises/visual_odom/frontend/resources/your_rosbag_name
```

Make sure ROS2 topics are being published correctly before starting the exercise.

---

### 2. Video Mode (User Input)

In this mode, the user selects a **video file from their local machine**.

- The video is streamed frame-by-frame to the algorithm
- No rosbag is required
- No ROS topics are used
- Ground truth is not mandatory

This mode is useful for:
- debugging
- testing algorithms quickly
- offline development

---

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

- `/visual_odom/image_raw` - Subscribe to this topic to receive the input camera frame (BGR8), whether it comes from the video or the rosbag. Message type: `sensor_msgs/msg/Image`

- `/kitti/gt/pose` - Subscribe to this topic to receive the ground-truth pose (available only in rosbag mode). Message type: `geometry_msgs/msg/PoseStamped`

For WebGUI debugging:

- `/webgui/estimated_point` - Publish to this topic to display the estimated camera position in the 3D viewer. Message type: `geometry_msgs/msg/PointStamped`

- `/webgui/image_debug` - Publish to this topic to display a debug image in the WebGUI. Message type: `sensor_msgs/msg/Image`

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

Visual Odometry (VO) is the process of **estimating the motion of a camera over time by analyzing the changes between consecutive images**.

In this exercise, the input is a **monocular camera stream (KITTI dataset or video)**, and the goal is to reconstruct the trajectory of the camera in 3D.

---

### Key idea

We estimate motion by tracking how **image features move between frames**.

Instead of using wheel encoders or IMU data, VO relies only on:

- Pixel motion in the image
- Camera calibration
- Geometric constraints

{% include gallery id="correlation" caption="Feature correspondences between consecutive frames used for motion estimation." %}

---

## 1. Camera model and intrinsic matrix (K)

To convert pixel motion into real geometric motion, we need the **intrinsic camera matrix**:

...
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
...

This matrix encodes:

- fx, fy → focal length (scale of projection)
- cx, cy → principal point (optical center)

In this exercise we use the KITTI calibration:

...
K = np.array([
    [718.8560, 0.0, 607.1928],
    [0.0, 718.8560, 185.2157],
    [0.0, 0.0, 1.0]
])
...

Without K, we cannot recover real motion from pixel coordinates.

---

## 2. Feature detection

The first step is to extract **salient points in the image** that are easy to track over time.

We use:

- Shi-Tomasi corner detection (cv2.goodFeaturesToTrack)
- or FAST features (optional improvement)

These points represent:

- corners of objects
- textured regions
- high-gradient areas

---

## 3. Feature tracking (Optical Flow)

Once features are detected, we track them between consecutive frames using:

...
cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts)
...

This gives us:

- p1 → points in previous frame
- p2 → corresponding points in current frame

This step gives the **2D motion of pixels over time**.

---

## 4. Outlier rejection

Not all tracked points are valid.

We remove:

- points with large motion (noise)
- incorrect matches
- unstable tracks

Additionally, we use **RANSAC** inside:

...
cv2.findEssentialMat(p1, p2, K, method=cv2.RANSAC)
...

This removes geometric outliers automatically.

---

## 5. Essential matrix estimation

The Essential Matrix (E) encodes the **relative rotation and translation between two camera frames**.

It is computed as:

...
E = cv2.findEssentialMat(p1, p2, K)
...

It satisfies:

...
p2^T * E * p1 = 0
...

This is the key geometric constraint of epipolar geometry.

---

## 6. Pose recovery

From the Essential Matrix, we recover:

- Rotation (R)
- Translation direction (t)

...
retval, R, t, _ = cv2.recoverPose(E, p1, p2, K)
...

Important:

- Translation is recovered only up to scale
- Direction is correct, magnitude is unknown

---

## 7. Scale problem

Monocular VO cannot directly recover real scale.

In this exercise:

- scale is approximated heuristically
- or normalized per frame
- or accumulated with a constant factor

...
t = t / ||t||
...

---

## 8. Trajectory integration

The final camera position is obtained by accumulating motion:

...
t_global = t_global + R_global @ t
R_global = R_global @ R
...

This builds the full trajectory frame by frame.

---

## 9. Output

The system outputs:

- Estimated camera trajectory (3D)
- Live visualization of feature tracking
- Overlay of motion in GUI

...
WebGUI.showEstimatedPoint([x, y, z])
...

{% include gallery id="trajectories" caption="Example of ground-truth and estimated trajectories." %}

---

## Summary

The full pipeline is:

Feature detection  
→ Optical flow tracking  
→ Outlier rejection (RANSAC)  
→ Essential matrix estimation  
→ Pose recovery  
→ Trajectory integration  

---

## Key insight

The entire system works because:

- 2D pixel motion + calibration (K)
→ allows recovery of 3D camera motion

This is the core principle of monocular visual odometry.

## Hint

Simple hints provided to help you solve the exercise. Please note that the **following hint is only a suggestive approach. Any other algorithm to solve the exercise is acceptable.**

1) Get the RGB images either from:
   - ROS2 bag topic `/kitti/camera/gray/left/image_raw`
   - or video input selected by the user

2) Detect feature points in the first frame using FAST or Shi-Tomasi corner detector.

3) Track the detected features in the next frame using Lucas-Kanade Optical Flow Algorithm.

4) Filter incorrect correspondences using:
   - Optical flow status output
   - RANSAC consistency check

5) Estimate the relative motion between consecutive frames using:
   - Essential Matrix (`cv2.findEssentialMat`)
   - Pose recovery (`cv2.recoverPose`)

6) Integrate the estimated motion over time to reconstruct the trajectory of the camera.

7) Show the estimated position in the 3D viewer using the provided GUI function.

---

## Demonstration video:

{% include youtubePlayer.html id=page.youtubeId %}

## Contributors

- Contributors: , [Jose María Cañas](https://github.com/jmplaza)

- Maintained by . 
