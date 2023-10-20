---
permalink: /exercises/ComputerVision/visual_odometry_3D
title: "Visual Odometry 3D"

sidebar:
 nav: "docs"

toc: true
toc_label: "TOC Visual Odometry 3D"
toc_icon: "cog"

gallery:
 - url: /assets/images/exercises/visual_odometry_3D/visual_odometry_3D_teaser.png
   image_path: /assets/images/exercises/visual_odometry_3D/visual_odometry_3D_teaser.png
   alt: "Visual Odometry 3D"
   title: "Visual Odometry 3D"

flow:
 - url: /assets/images/exercises/visual_odometry/flow.PNG
   image_path: /assets/images/exercises/visual_odometry/flow.PNG
   alt: "flowchart"
   title: "Flowchart"

epipolar:
 - url: /assets/images/exercises/3d_reconstruction/epipolar.png
   image_path: /assets/images/exercises/3d_reconstruction/epipolar.png
   alt: "Epipolar Geometry"
   title: "Epipolar Geometry"

youtubeId1: nQrEuxnBdTM
---

## Goal

The objective of this exercise is to implement the logic of a 3D visual odometry algorithm. The accuracy of the users' algorithm will be shown on the GUI of the exercise.

{% include gallery caption="Visual Odometry 3D"%}

## Instructions

This is the preferred way for running the exercise.

### Installing and Launching

1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.
2. Pull the current distribution of Robotics Academy Docker Image:
 ```bash
 docker pull jderobot/robotics-academy:latest
 ```
3. Download the KITTI dataset. It will take much time (22GB).
 ```bash
   # Download KITTI dataset
   cd ~ || exit 1
   mkdir -p datasets/kitti && cd datasets/kitti || exit
   wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip
   wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip
   unzip data_odometry_gray.zip >/dev/null 2>/dev/null
   unzip data_odometry_poses.zip >/dev/null 2>/dev/null
   rm -rf ./*.zip
 ```

- Make sure the `datasets` folder presents the following tree (**you must see 21 sequences**):
  ```bash
  tree -I "img|textures/*|node_*|venv|*__*|*png|*jpg|tree*|LICENSE|README*"
  ```
  ```bash
├── kitti
│   └── dataset
│       ├── poses
│       │   ├── 00.txt
│       │   ├── 01.txt
│       │   ├── 02.txt
│       │   ├── 03.txt
│       │   ├── 04.txt
│       │   ├── 05.txt
│       │   ├── 06.txt
│       │   ├── 07.txt
│       │   ├── 08.txt
│       │   ├── 09.txt
│       │   └── 10.txt
│       └── sequences
│           ├── 00
│           │   ├── calib.txt
│           │   ├── image_0
│           │   ├── image_1
│           │   └── times.txt
│           ├── 01
│           │   ├── calib.txt
│           │   ├── image_0
│           │   ├── image_1
│           │   └── times.txt
│           ├── 02
│           │   ├── calib.txt
│           │   ├── image_0
│           │   ├── image_1
│           │   └── times.txt
  ``````


- In order to obtain optimal performance, Docker should be using multiple CPU cores. In the case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

### How to perform the exercises?

- Start a new docker container of the image and keep it running in the background:
```bash
 docker run -it --rm --name RADI -v ~/datasets/:/datasets -p 7164:7164 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 8000:8000 jderobot/robotics-academy
 ```

- On the local machine navigate to `127.0.0.1:7164/` in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display `"Connected"`. Click on the `"Launch"` button and wait for some time until an alert appears with the message `Connection Established` and the button displays "Ready"`. Click on the `Load in Robot` to make the exercise start.

- The exercise can be used after the alert.

### Enable GPU Acceleration

- Follow the advanced launching instructions from [here](https://jderobot.github.io/RoboticsAcademy/user_guide/#enable-gpu-acceleration).

**Where to insert the code?**

In the launched webpage, type your code in the text editor,

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
   # Enter iterative code!
```

### Using the Interface

- **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the position of the robot).

- **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency which is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

- **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer.

- **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the print() command in the Editor.

## API

- `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
- `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL.advance()` - in the next iteration you will see the new frame.
- `HAL.get_image('left')` - return the left image.
- `HAL.get_image('right')` - return the right image.
- `HAL.get_current_groundtruth_position()` - return the `x,y,z` real position.
- `HAL.get_camera_model()` - return the Pinhole camera model used.
- `HAL.set_estimated_position(x: float, y: float, z: float)` - set your estimated position.
- `HAL.set_estimated_euler_angles(roll: float, pitch: float, yaw: float)` -  set your estimated orientation.
- `GUI.show_image(image)` - allows you to view the image you are working on.      

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:

# Enter iterative code!   
   img = HAL.get_image("left")
   GUI.show_image(img)
  
   HAL.advance()
```

## Theory

VO involves the real-time estimation of the incremental 3D motion of the camera. In other words, it calculates the three-dimensional rotation and translation of the camera based on consecutive images. It's an incremental technique because it relies on the previous position to compute the new one.

In these types of algorithms, techniques for extracting key points, descriptor calculations, and matching algorithms are commonly employed. The typical process is as follows: once the matched points are calculated, the fundamental or essential matrix is computed and then decomposed using SVD to obtain the rotation and translation matrix (RT). Once these rotation and translation matrices are known, an algorithm can be used to determine the camera's position in space.

Within visual odometry, we can differentiate between various types:

1. Monocular and Stereo: Visual odometry can be classified into monocular visual odometry (using a single camera) and stereo visual odometry (utilizing two cameras in a stereo configuration) based on the camera setup.

2. Direct and Feature-based: Traditional visual odometry primarily relies on the "feature-based method," which involves extracting specific image features and tracking them across a sequence of images. Recent advancements in this field have introduced an alternative approach known as the "direct method," which directly uses the intensity of all pixels in the image sequence as visual input. Hybrid methods that combine both approaches are also available.

3. Inertial Visual Odometry: When an inertial measurement unit (IMU) is integrated into the system, it is commonly referred to as "inertial visual odometry" (VIO).

The estimation of a mobile robot's position can be applied in three different ways: feature-based, appearance-based, and hybrid approaches. The choice of which algorithm to implement depends on your specific application and requirements.

Here is a flowchart explaining the step-by-step process for implementing a visual odometry algorithm.

{% include gallery id="flow"%}

More detailed deliberation on visual odometry is provided here.

1. [http://www.cs.toronto.edu/~urtasun/courses/CSC2541/03_odometry.pdf](http://www.cs.toronto.edu/~urtasun/courses/CSC2541/03_odometry.pdf)

### Algorithms
To estimate the Rt matrices solution of the epipolar geometry problem, you can use one of the following algorithms:
- Eight points
- Seven points
- RANSAC
- LMEDS

### Epipolar Geometry

When two cameras view a 3D scene from two different positions, there are a number of geometric relations between the 3D points and their projections onto the 2D images that lead to constraints between the image points. The study of these properties and constraints is called Epipolar Geometries. The image and explanation below may generalize the idea:

{% include gallery id="epipolar" caption="Epipolar Geometry" %}

Suppose a point `X` in 3d space is imaged in two views, at `x` in the first and `x'` in the second. Back Projection the rays to their camera centers through the image planes, we obtain a plane surface, denoted by π.

Supposing now that we know only `x`, we may ask how the corresponding point `x'` is constrained. The plane π is determined by the baseline(line connecting the camera centers) and the ray defined by `x`. From above, we know that the ray corresponding to the *unknown* point `x'` lies in π, hence the point `x'` lies on the line of intersection `l'` of π with the second image plane. This line is called the epipolar line corresponding to `x`. This relation helps us to reduce the search space of the point in the right image, from a plane to a line. Some important definitions to note are:

- The **epipole** is the point of intersection of the line joining the camera centers (the baseline) with the image plane.
- An **epipolar plane** is a plane containing the baseline.
- An **epipolar line** is the intersection of the epipolar plane with the image plane.

### Triangulation

Triangulation in simple terms is just calculating where the 3d point is going to lie using the two determined points in the image planes.

In reality, the position of the image points cannot be measured exactly. For general cameras, there may be geometric or physical distortions. Therefore, a lot of mathematics goes behind minimizing that error and calculating the most accurate 3d point projection. Refer to this [link](https://courses.cs.washington.edu/courses/cse455/09wi/Lects/lect16.pdf) for a simple model.

## Hints

Simple hints provided to help you solve the Visual Odometry 3D exercise. The **OpenCV library** is used extensively for this exercise.

### Setup

Using the exercise API, we can easily retrieve the images. Also, after getting the images, it is a good idea to perform **the flowchart**.

### Calculating Optic Flow

OpenCV already has built-in correlation filters which can be called through `calcOpticalFlowPyrLK`.

### Estimate the Position and Orientation

To calculate $R$ and $t$ OpenCV provides the following methods: `findEssentialMat` and `recoverPose`. **You will need to correct the scale**. That is why we provide you the ground-truth position.

Keep in mind the difference between simple 3D coordinates and homogeneous 4D coordinates. Check out this [video](https://www.youtube.com/watch?v=JSLG8n_IY9s) for details. Simply dividing the complete 4d vector by its 4th coordinate gives us the 3d coordinates.

### Navigation Equations
You can implement the following incremental equations:

- $$\mathbf{t}_{t+1}^{abs} = \mathbf{t}_{t}^{abs} + \mathbf{R}_{t}^{abs} \cdot \mathbf{t}_{t,t+1}$$
- $$\mathbf{R}_{t+1}^{abs} = \mathbf{R}_{t}^{abs} \cdot \mathbf{R}_{t,t+1}$$

## Contributors

- Contributors: [Pablo Asensio Martínez](https://github.com/PabloAsensio), [Jose María Cañas](https://github.com/jmplaza).
## Videos
That is an example of what you should expect to perform the exercise.

{% include youtubePlayer.html id=page.youtubeId1 %}

## References

1. [https://www.researchgate.net/publication/330183516_3D_Computer_Vision_Stereo_and_3D_Reconstruction_from_Disparity](https://www.researchgate.net/publication/330183516_3D_Computer_Vision_Stereo_and_3D_Reconstruction_from_Disparity)
