---
permalink: /exercises/AutonomousCars/end_to_end_visual_control/
title: "End to End Visual Control"

sidebar:
    nav: "docs"

toc: true
toc_label: "TOC End to End Visual Control"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
    - url: /assets/images/exercises/follow_line/formula1_circuit.png
      image_path: /assets/images/exercises/follow_line/formula1_circuit.png
      alt: "Racing circuit."
      title: "Racing circuit."
    - url: /assets/images/exercises/follow_line/formula1.png
      image_path: /assets/images/exercises/follow_line/formula1.png
      alt: "First Person."
      title: "First Person."
    - url: /assets/images/exercises/follow_line/formula1_2.png
      image_path: /assets/images/exercises/follow_line/formula1_2.png
      alt: "Model."
      title: "Model."

gifs:
    - url: /assets/images/exercises/follow_line/oscillations.gif
      image_path: /assets/images/exercises/follow_line/oscillations.gif
      alt: "examples"
      title: "examples"
    - url: /assets/images/exercises/follow_line/slowresponse.gif
      image_path: /assets/images/exercises/follow_line/slowresponse.gif
      alt: "examples"
      title: "examples"

pid:
    - url: /assets/images/exercises/follow_line/ControlSystems.jpg
      image_path: assets/images/exercises/follow_line/ControlSystems.jpg
      alt: "Control Systems"
      title: "Control Systems"

    - url: /assets/images/exercises/follow_line/TypesofControlSystems.jpg
      image_path: /assets/images/exercises/follow_line/TypesofControlSystems.jpg
      alt: "Types of Control Systems"
      title: "Types of Control Systems"

    - url: /assets/images/exercises/follow_line/PID.png
      image_path: /assets/images/exercises/follow_line/PID.png
      alt: "PID"
      title: "PID"

youtubeId1: eNuSQN9egpA
youtubeId2: gHZVESBcgKE
youtubeId3: XzgfaQ20atY
---

## Goal

The goal of this exercise is to implement a PID reactive control capable of following the line painted on the racing circuit.

{% include gallery caption="Gallery" %}

The students will program a Formula1 car in a race circuit to follow the red line in the middle of the road.

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is done, how to launch a RoboticsBackend and how to access the exercises.

## Frequency API

-   `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
-   `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-native implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

-   `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
-   `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
-   `HAL.getImage()` - to get the image (BGR8).
-   `HAL.setV(velocity)` - to set the linear speed.
-   `HAL.setW(velocity)` - to set the angular velocity.
-   `WebGUI.showImage(image)` - allows you to view a debug image or with relevant information.

### ROS 2-native Implementation

`from WebGUI import gui` - to enable the Web GUI for visualizing camera images.

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

#### ROS 2 Topics

Use standard ROS 2 topics for direct communication with the simulation.

-   `/cam_f1_left/image_raw ` - Subscribe to this topic to receive camera images (BGR8). Message type: `sensor_msgs/msg/Image`
-   `/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`

#### Frequency Control

Use standard ROS 2 mechanisms to manage loop timing:

-   `rclpy.spin()` - Event-driven execution using callbacks.
-   `rclpy.spin_once()` - Single-step processing, often with custom timers.
-   `rclpy.Rate()` - Loop-based frequency control.

#### Image Debugging

-   Publish processed images to the topic: `/webgui_image`
    Used for sending debug or processed visuals to the frontend.
-   The GUI automatically subscribes to `/webgui_image`
    Images published to this topic are displayed in the GUI interface.

<!-- TODO: DEVELOP DEEP LEARNING MODEL -->

## Develop a Deep Learning Model

### 1. Dataset overview

For students who want to develop deep learning models for the End-to-End Visual Control exercise, we provide **two datasets**:

#### i) Simple Circuit Dataset

<p style="text-align:justify"> 
This dataset is specifically designed for training and testing models on a single, <strong>simple circuit</strong>. It is ideal for beginners or for initial experiments to understand how the model reacts to basic driving scenarios. The simple circuit is easier to complete, allowing users to quickly train and evaluate their models without facing complex turns or intersections.
</p>

#### ii) Combine Circuit Dataset

<p style="text-align:justify">
This dataset includes data from all <strong>four circuits</strong> available in the exercise. It is intended for advanced model development, enabling students to train models that generalize across all four circuits and handle various driving conditions, including <code class="language-plaintext highlighter-rouge">sharp left and right turns</code>. The combined dataset captures a wide range of driving scenarios, including sharp turns, straight paths, and varying circuit complexities. We provide an <strong>adjustment dataset</strong> designed to support users in managing diverse driving scenarios, facilitating more experimentation.
</p>

### 2. Datasets Downloads

<p style="text-align:justify">The datasets for the End-to-End Visual Control exercise are hosted on Hugging Face under the JdeRobot organization. Students can access them using the <code class="language-plaintext highlighter-rouge">load_dataset()</code> method and directly apply them for training and testing their models. Although multiple download options are available, this guide highlights two recommended approaches for retrieving the datasets to a local machine.</p>

-   Simple Circuit Dataset: [JdeRobot/Follow-Line-Simple-Circuit-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset)
-   Combined Circuit Dataset: [JdeRobot/Follow-Line-Combine-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset)

#### Method 01: Use the git lfs command [Recommended: Low]

Visit the [git-lfs](https://git-lfs.com/) website and install `git-lfs` on your local machine.

##### Simple Circuit Dataset

```bash
git clone https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset
```

##### Combine Circuit Dataset

```bash
git clone https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset
```

#### Method 02: Use the git lfs command [Recommended: High]

## Theory

## Hints

Simple hints provided to help you solve the follow_line exercise.

### References to ROS 2 Concepts

Understanding these ROS 2 concepts will help you implement the exercise natively. Refer to these links for more details:

1. ROS 2 Publisher & Subscriber – [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
2. ROS 2 Spin & Spin Once – [https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html](https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html)
 <!-- 3. ROS 2 Rate - add content for rate -->

### Illustrations

{% include gallery id="gifs" caption="Unstable Oscillations (left) - Slow Response (right)" %}

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

_This solution is an illustration for the Web Templates_

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId3 %}

## Contributors

<!-- TODO: -->

-   Contributors: [Md. Shariar Kabir](https://github.com/codezerro),[Jose María Cañas](https://github.com/jmplaza),[David Pascual](https://github.com/dpascualhe)
-   Maintained by [Md. Shariar Kabir](https://github.com/codezerro),[Jose María Cañas](https://github.com/jmplaza),[David Pascual](https://github.com/dpascualhe).

## References

1. [https://huggingface.co/JdeRobot](https://huggingface.co/JdeRobot)
2. [JdeRobot/Follow-Line-Simple-Circuit-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset)
3. [JdeRobot/Follow-Line-Combine-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset)

```

```
