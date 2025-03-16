---
permalink: /exercises/ComputerVision/marker_visual_loc
title: "Marker Based Visual Loc"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Marker Based Visual Loc"
toc_icon: "cog"

layout: archive
classes: wide

gallery:
  - url: /assets/images/exercises/marker_based_visual_loc/marker_teaser.png
    image_path: /assets/images/exercises/marker_based_visual_loc/marker_teaser.png
    alt: "Marker Based Visual Loc"
    title: "Marker Based Visual Loc"

matrix:
  - url: /assets/images/exercises/marker_based_visual_loc/camera-intrinsic.png
    image_path: /assets/images/exercises/marker_based_visual_loc/camera-intrinsic.png
    alt: "Camera Matrix"
    title: "Camera Matrix"

exampletag:
  - url: /assets/images/exercises/marker_based_visual_loc/example_tag.png
    image_path: /assets/images/exercises/marker_based_visual_loc/example_tag.png
    alt: "April Tag"
    title: "April Tag"

youtubeId1: PtdqHpICMy0
---


## Goal

The goal of this exercise is to estimate the position and orientation (pose) of a robot in a 2D space by detecting and analyzing visual markers, specifically AprilTags. This process involves using computer vision to identify the tags in the robot's environment and mathematical methods to derive its relative pose with respect to the detected tags.

The green robot represents the real position.
The blue robot represents the position from the odometry (with noise).
The red robot represents the user estimated position.

{% include gallery caption="Gallery" %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Robot API

* `from HAL import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `from GUI import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to get the image
* `GUI.showImage()` - allows you to view a debug image or with relevant information
* `GUI.showEstimatedPose((x, y, yaw))` - allows you to view your estimate pose in the map
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity

* `HAL.getOdom().x` - to get the approximated X coordinate of the robot (with noise)
* `HAL.getOdom().y` - to get the approximated XY coordinate of the robot (with noise)
* `HAL.getOdom().yaw` - to get the approximated orientation position of the robot (with noise)

### Load tags position

```python
import yaml
from pathlib import Path

conf = yaml.safe_load(
    Path("/resources/exercises/marker_visual_loc/apriltags_poses.yaml").read_text()
)

tags = conf["tags"]
```

The content of the yaml file has this structure:
```yaml
tags:
  tag_0:
    position: [1.75, -3.535, 0.8, -1.57079]
```
Being the position [X, Y, Z, Yaw]
## Theory

### Visual Markers: AprilTags

AprilTags are fiducial markers similar to QR codes but designed for robust detection and pose estimation. They consist of a binary grid with a unique pattern that encodes an identifier. They are ideal for robotics applications due to:

* Their high detection reliability.
* Robustness to noise, partial occlusion, and perspective distortions.
* Easy decoding to obtain a unique ID and retrieve the tag's known position in the environment.

Example code for apriltags detection:

```python
import GUI
import HAL
import pyapriltags
import cv2

detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")

while True:
    print("[INFO] loading image...")
    image = HAL.getImage()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    print("[INFO] detecting AprilTags...")
    results = detector.detect(gray)
    print("[INFO] {} total AprilTags detected".format(len(results)))

    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(
            image,
            tagFamily,
            (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
        print("[INFO] tag family: {}".format(tagFamily))
    GUI.showImage(image)

```

### Camera Model and Calibration

Before detecting AprilTags, the camera used to observe the environment must be calibrated. Camera calibration provides intrinsic parameters like focal length and optical center, which are critical for accurate pose estimation. A camera model is typically represented using the pinhole camera model with distortion corrections.

The intrinsic parameters are used in:

Projection Matrix (P): Converts 3D world coordinates to 2D image coordinates.

{% include gallery id="matrix" caption="Matrix" %}

You can use the following code to define the default parameters:

```python
        size = image.shape
        focal_length = size[1]
        center = (size[1] / 2, size[0] / 2)

        matrix_camera = np.array(
            [[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]],
            dtype="double",
        )

        dist_coeffs = np.zeros((4, 1))
```

### AprilTag Detection

The detection process includes:

* Preprocessing: Filtering the image and identifying regions likely to contain tags.
* Decoding: Analyzing the binary pattern to extract the tag's unique ID.
* Pose Estimation: Computing the 6-DoF pose of the tag with respect to the camera.

{% include gallery id="exampletag" caption="Exampletag" %}

The apriltags are set at 0.8 meters in height and have a size of 0.3 x 0.3 meters (the black part is 0.24 x 0.24 meters).

### PnP (Perspective-n-Point)

Given the 3D coordinates of the tag and their corresponding 2D image points (without z), the robot's pose can be estimated using algorithms like Direct Linear Transformation (DLT) or iterative methods (e.g., Levenberg-Marquardt).

Here is the documentation of PnP in [OpenCV](https://docs.opencv.org/3.4/d5/d1f/calib3d_solvePnP.html).

### Rotation and Translation matrix

The rotation and translation matrixes can be extracted from the [sdf files](https://github.com/JdeRobot/RoboticsInfrastructure/blob/humble-devel/CustomRobots/turtlebot3/models/turtlebot3_waffle/model.sdf) that describe the turtlebot 3 waffle robot.

### More Resources

You may find interesting the following resources:

* [Accuracy analysis of marker-based 3D visual localization](https://gsyc.urjc.es/jmplaza/papers/jornadasautomatica2016-pnp.pdf)

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

* Contributors: [Jose María Cañas](https://github.com/jmplaza), [David Duro](https://github.com/dduro2020), [Javier Izquierdo](https://github.com/javizqh).

## References

[https://pyimagesearch.com/2020/11/02/apriltag-with-python/](https://pyimagesearch.com/2020/11/02/apriltag-with-python/)
[https://en.wikipedia.org/wiki/Perspective-n-Point](https://en.wikipedia.org/wiki/Perspective-n-Point)
[https://april.eecs.umich.edu/software/apriltag](https://april.eecs.umich.edu/software/apriltag)
[https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_pose/apriltag-pose.html](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_pose/apriltag-pose.html)
