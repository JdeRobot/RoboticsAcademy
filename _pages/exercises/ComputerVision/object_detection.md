---
permalink: /exercises/ComputerVision/object_detection
title: ""

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Object Detection"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

teaser:
  - url: /assets/images/exercises/object_detection/object_detection_teaser.jpeg
    image_path: /assets/images/exercises/object_detection/object_detection_teaser.jpeg
    alt: "Visual Object Detection"
    title: "Visual Object Detection"

instruction:
  - url: /assets/images/exercises/object_detection/object-detection-exercise-instruction-img-1.png
    image_path: /assets/images/exercises/object_detection/object-detection-exercise-instruction-img-1.png
    alt: "Exercise instruction"
    title: "Exercise instruction"

youtubeId1: I0GhS3ePMHM
---

<!-- title -->

# Deep learning-based Visual Object Detection Exercise

<!-- cover photo -->

{% include gallery id="teaser" caption="" %}

<!-- descriptions -->
<p style="text-align:justify;">The primary goal of the object detection exercise is to identify individuals in the video feed from a webcam and to draw a rectangular boundary around each person. This exercise supports live webcam video stream inference, allowing users to observe real-time object detection performance using their own trained models.</p>
<p style="text-align:justify;">Users are expected to upload a deep learning-based object detection model in the <a href="https://onnx.ai/" target="_blank" ><strong>ONNX (Open Neural Network Exchange)</strong></a>  format. Users are encouraged to build and train their own object detection models using libraries such as <a href="https://pytorch.org/" target="_blank" ><strong>PyTorch</strong></a>  or <a href="https://www.tensorflow.org/" target="_blank" ><strong>TensorFlow</strong></a>. After training, the model must be exported to the ONNX format to ensure compatibility with the exercise environment.</p>

<p style="text-align:justify;">After training, the model must be exported to the ONNX format to ensure compatibility with the exercise environment, and you must use the editor to write Python code that processes input from a live video feed, which is captured using your browser's webcam.</p>

<!-- instruction image -->

{% include gallery id="instruction" caption="exercise instruction" %}

<!-- Note Guide -->
<!-- <br/> -->

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Exercise API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

- `WebGUI.getImage()` - to get the image. It can be None.

```python
while True:
    image = WebGUI.getImage()
    if image is not None:
      # rest of the code.
```

- `WebGUI.showImage(image)` - allows you to view a debug image or one with relevant information.

<!-- Model Path -->

## File Path for Uploaded Model

To obtain the path of an uploaded **ONNX** model, use the `model_path_func` helper function.

This function returns the full file path to a model located in the workspace directory. The model filename must be passed as an argument.

```python
from model import model_path_func

model_path = model_path_func("my_model.onnx")
```

### ROS 2-direct Implementation

#### ROS 2 Topics

Use standard ROS 2 topics for direct communication.

- `/input/image_raw` - Subscribe to this topic to receive input images (BGR8). Message type: `sensor_msgs/msg/Image`
- `/webgui_image` - Publish to this topic to send the processed image to the GUI. Message type: `sensor_msgs/msg/Image`

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

## Exercise Instructions

- The uploaded ONNX format model should adhere to the input/output specifications, please keep that in mind while building your model.
- The user can train their model in any framework of their choice and export it to the ONNX format. Refer to this [**article**](https://docs.unity3d.com/Packages/com.unity.barracuda@1.0/manual/Exporting.html) to know more about exporting your model to the ONNX format.

### Model Input Specification

<p style="text-align:justify;">
`input_shape` - The application code pre processes the input frame of shape (H, W, C) <bold>TO</bold>> (1, 300, 300, 3) i.e (batch_size, H, W, C). This is a typical input shape for a `Conv2D` layer, so it is mandatory for your custom built model to have its first layer as `Conv2D`.
</p>

### Model Output Specification

Given 1 frame per batch, the model must return 4 tensor arrays in the following order:

`detection_boxes`: a list of bounding boxes. Each list item describes a box with top, left, bottom, right relative to the image size.

`detection_classes`: Array of detected classes. The class label must be `1` for humans.

`detection_scores`: the score for each detection with values between 0 and 1 representing probability that a class was detected.

`num_detections`: the number of detections.

**Note**: Make sure to keep the class label for Humans while training your model as 1. Any object detected by your model with any other class label other than 1, will not be accounted for.

## Guide to Fine Tuning pre-existing models

<p style="text-align:justify;">Expecting the user to build the model from scratch would be an overkill, we have compliled and provided the revelevant guide for Fine Tuning pre exisiting models in TensorFlow and Pytorch. This includes everything from making the process of collecting data, preprocessing it and fine tuning with it on a pre-existing model architecture. Since the process of exporting models to ONNX format is different for different frameworks, we have also added so under the respective guide. We strongly suggest the user to go through the guide.
</p>
### Pytorch

We have documented a guide for the PyTorch implementation. Please refer to it below for the detailed information.

- [**SSDMobilenet_PyTorch_FineTune**](https://github.com/TheRoboticsClub/gsoc2021-Shashwat_Dalakoti/blob/main/Fine_Tuning/PyTorch/SSDMobilenet_pytorch_FineTune.ipynb)

### TensorFlow

- [**Training a TensorFlow MobileNet Object Detection Model with a Custom Dataset**](https://blog.roboflow.com/training-a-tensorflow-object-detection-model-with-a-custom-dataset/)

This guide walks you through using the TensorFlow object detection API to train a MobileNet Single Shot Detector (v2) to your own dataset. Here’s the complete Jupyter notebook guide for the above article:

- [**Roboflow-tensorflow-object-detection-mobilenet-colab.ipynb**](https://colab.research.google.com/drive/1wTMIrJhYsQdq_u7ROOkf0Lu_fsX5Mu8a)

## Exercise Features

- **Live Inference** - Perform live inference on the input feed from the web-cam.
- **Upload own model** - You can upload your own object detection model.
  <!-- * **Video Inference** - Perform inference on an uploaded video. -->
  <!-- * **Model Benchmarking** - Evaluate the uploaded model by benchmarking against a ground truth dataset(Oxford Town Centre dataset).  -->
  <!-- * **Model Visualization** - Visualize and analyse the uploaded model to get a visual summary of the model, which will make it easier to identify trends and patterns, understand connections, and interact with your data. -->

## Using the interface

<!-- * **Dropdown**: Use the dropdown menu to choose a specific mode. The required control buttons will pop-up accordingly.

* **Control Buttons**: The control buttons enable the control of the interface.
  - **Live/Video/Benchmark buttons** - Send the uploaded model for inference to the core application.
  - **Stop button**: Stops the inference process.
  - **Visualizer button**: Opens the model visualizer. -->

- **Browse and Upload buttons**: These are used to browse and upload the model and video. The control buttons for the specific mode will only activate once all the required files have been uploaded.

<!-- * **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the model inference and benchmarking code. A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency. -->

- **Debug Level**: This decides the debugging level of the application. A debug level of 1 implies no debugging at all. A debug level greater than or equal to 2 enables all the WebGUI functions working properly.

- **Pseudo Console**: This shows the error messages and a few intermediate outputs along the inference.

## Tutorial Video

{% include youtubePlayer.html id=page.youtubeId1 %}

<!-- contributors and maintainers -->

## Contributors

- Contributors: [David Pascual](https://github.com/dpascualhe), [Md. Shariar Kabir](https://github.com/codezerro) ,[Shashwat Dalakoti](https://github.com/shashwat623)
- Maintained by [David Pascual](https://github.com/dpascualhe), [Md. Shariar Kabir](https://github.com/codezerro)

<!-- Reference -->

## References

1. [https://onnx.ai/](https://onnx.ai/)
2. [https://pytorch.org/](https://pytorch.org/)
3. [https://www.tensorflow.org/](https://www.tensorflow.org/)
4. [https://debuggercafe.com/image-augmentation-using-pytorch-and-albumentations/](https://debuggercafe.com/image-augmentation-using-pytorch-and-albumentations/)
