---
permalink: /exercises/ComputerVision/human_detection
title: ""

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Human Detection"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/human_detection/human_detection_teaser.jpeg
    image_path: /assets/images/exercises/human_detection/human_detection_teaser.jpeg
    alt: "Human Detection"
    title: "Human Detection"
    
youtubeId1: xZQ9x8J-shY

---

# Human Detection Exercise using Deep Learning

A Human Detection Exercise to identify the presence of humans and identification of the rectangular boundary around them. 
The user is expected to upload a Deep Learning model which fits the required input and output specifications for inference. The input model is supposed to be in the ONNX format. For more information refer to the "Exercise Instructions" section below.

{% include gallery caption="Detection Example" %}


## Launch Instructions

- There are two ways to run the exercise using web-template:

  - Run the exercise with docker container
   - Run it without container

### Run with docker container

- First you need to build the image. Then, you need to run a container.
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git checkout noetic
cd scripts
docker build -f Dockerfile-noetic -t image-name .
docker run -it --name=container_name -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 image-name ./start.sh
```  
- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.
- Click the connect button and wait for some time until an alert appears with the message Connection Established and button displays connected.
- The exercise can be used after the alert.
- It is necessary to map the port where the camera is located to the docker container.

### Run without docker container

The following dependencies should be pre-installed:
- Python 3 or later
- Python dependencies
     - OpenCV
     - onnxruntime
     - WebsocketServer

- Clone the Robotics Academy repository to your local machine, switch to the noetic branch and head over to the Human_Detection exercise.
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git checkout noetic
```
      
- Determine your machine dns server IP address which is generally in the form of **127.0.0.xx for Linux machine** by running this command

```bash
cat /etc/resolv.conf
```

- Inside `assets/websocket_address.js` file, change the **variable websocket_address** to the IP address found with the above command

- Start the host application along with the same IP address which is used for connection.

```bash
python exercise.py 127.0.0.xx
```

- Open the browser template from `exercice.html`

- The page should says **[open]Connection established!**.Means it is working as expected.

**__NOTE:__**  If you get **socket.error: [Errno 99] Cannot assign requested address** error,you need to check and pass the correct IP address.


## Exercise Instructions

- The uploaded ONNX format model should adhere to the input/output specifications, please keep that in mind while building your model. 
- The user can train their model in any framework of their choice and export it to the ONNX format. Refer to this [**article**](https://docs.unity3d.com/Packages/com.unity.barracuda@1.0/manual/Exporting.html) to know more about exporting your model to the ONNX format.

### Model Input Specification

`input_shape` - The application code pre processes the input frame of shape (H, W, C) **TO** (1, 300, 300, 3) i.e (batch_size, H, W, C). This is a typical input shape for a `Conv2D` layer, so it is mandatory for your custom built model to have its first layer as `Conv2D`.

### Model Output Specification

Given 1 frame per batch, the model must return 4 tensor arrays in the following order:

`detection_boxes`: a list of bounding boxes. Each list item describes a box with top, left, bottom, right relative to the image size.

`detection_classes`: Array of detected classes. The class label must be **1** for humans. 

`detection_scores`: the score for each detection with values between 0 and 1 representing probability that a class was detected.

`num_detections`: the number of detections.

**Note**: Make sure to keep the class label for Humans while training your model as 1. Any object detected by your model with any other class label other than 1, will not be accounted for.

## Demo Model

A demo model has been provided inside the `Demo_Model` folder to test and play around with the application.

## Guide to Fine Tuning pre-existing models

Expecting the user to build the model from scratch would be an overkill, we have compliled and provided the revelevant guide for Fine Tuning pre exisiting models in TensorFlow and Pytorch. This includes everything from making the process of collecting data, preprocessing it and fine tuning with it on a pre-existing model architecture. Since the process of exporting models to ONNX format is different for different frameworks, we have also added so under the respective guide. We strongly suggest the user to go through the guide.

### Pytorch 

We have documented a guide for the PyTorch implementation. Please refer to it below for the detailed information.

* [**SSDMobilenet_PyTorch_FineTune**](https://github.com/TheRoboticsClub/gsoc2021-Shashwat_Dalakoti/blob/main/Fine_Tuning/PyTorch/SSDMobilenet_pytorch_FineTune.ipynb)

### TensorFlow

* [**Training a TensorFlow MobileNet Object Detection Model with a Custom Dataset**](https://blog.roboflow.com/training-a-tensorflow-object-detection-model-with-a-custom-dataset/)

This guide walks you through using the TensorFlow object detection API to train a MobileNet Single Shot Detector (v2) to your own dataset. Here’s the complete Jupyter notebook guide for the above article:

* [**Roboflow-tensorflow-object-detection-mobilenet-colab.ipynb**](https://colab.research.google.com/drive/1wTMIrJhYsQdq_u7ROOkf0Lu_fsX5Mu8a)


## Using the interface

* **Browse Button**: The browse button is used to browse and upload the model from the local machine.

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the uploaded model for inference to the core application. Stop button stops the inference process.

* **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the model inference and benchmarking code. A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency.

* **Debug Level**: This decides the debugging level of the application. A debug level of 1 implies no debugging at all. A debug level greater than or equal to 2 enables all the GUI functions working properly.

* **Psuedo Console**: This shows the error messages and a few intermediate outputs along the inference and benchmarking process.

## Demonstrative Video with Web Template

{% include youtubePlayer.html id=page.youtubeId1 %}


## Note
This exercise is still a work under progress. Certain features like benchmarking/evaluating the uploaded model, GUI features for uploaded model visualization and inference source flexibility are still being developed and tested. We will keep updating the page and the code base concurrently.
