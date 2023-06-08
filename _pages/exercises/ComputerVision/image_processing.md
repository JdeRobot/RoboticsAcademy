---
permalink: /exercises/ComputerVision/image_processing
title: "Image Processing"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Image Processing"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/image_processing/image_processing_teaser.png
    image_path: /assets/images/exercises/image_processing/image_processing_teaser.png
    alt: "Image Processing
    title: "Image Processing"

---

## Goal

This exercise is an introduction to the image processing world. In this practice the objective is to complete a series of mini-challenges. The difficulty of the mini-challenges will increase progressively, without leaving the premise of an introduction exercise.

{% include gallery caption="Tracking example" %}


## Instructions
This is the preferred way for running the exercise.

### Installation 
- Clone the Robotics Academy repository on your local machine

```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
```

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

```bash
docker pull jderobot/robotics-academy:latest
```

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background.

```bash
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy
```   
- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

- The exercise can be used after the alert.


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

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Image. Stop button stops the code that is currently running on the Image. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the image of the camera).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student is provided with `console.print()` similar to `print()` command in the Python Interpreter. 

## Robot API

* `from HAL import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `from GUI import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage(frame)` - to get the frame corresponding to the i position
* `GUI.showImage()` - allows you to view a debug image or with relevant information

## Mini-challenges

The mini-challengues to complete are the following ones (trying them ir order is recomended for an initial level): 

**Challenges lists**
* Reproduce the video in the visualization window.
* Convert each frame to black & white
* Apply a smoothing filter and show the blurred image. 
* Find the borders of the objects in the image
* Paint the contours of objects in the image
* Find all the squares
* Filter the blue objects and find all the squares of that color
* Follow only the blue circle until it disappears from the frame.


## Theory
Image processing consist in various algorithms or tecnhniques used to manipulate images. The objectives can be diverse, from getting information (such as in the medic field) or to improve the quality of said images.

### Smoothing filter
These filters are used to reduce the noise of an image and improving it's quality, to reduce the details in an image so, for examplen, it's more difficult to mistake the borders of an object or as a way to pre-process an image. These are the smoothing filter most used, but this can be done in other ways too like for example setting a matrix with a determined amount of rows and columns, depending of the level of smoothering that the user desired to apply. The biggest the size of the matrix is, the more blur there will be.
* `cv.GaussianBlur(img, (rows, col), borderType)` - 
* `cv.medianBlur(img, matriz size)` - Calculates the new value of the central pixel doing the media of all the values in matrix.
* `cv.bilateralFilter(img,x,y)` - Similar to the GaussianBlur, but takes into account that the nearby pixels have a similar value to the others. It keeps the edges sharp.

### Edge detection
In edge detection treatment, the regions where there are big changes in intesity. Normally, a big change indicates a border or limit between different objects.

Edge detection is really important in image processing.

### Hough Circles 
The Hough Circles function is used to detect circular shapes in an image. The function takes an input image and several parameters as input. The key parameters include the minimum and maximum radius of circles to be detected, the minimum distance between the centers of detected circles, and a sensitivity parameter that determines the threshold for circle detection.

Internally, the function scans the image and accumulates votes for possible circle centers and radius. For each pixel in the image, it examines all possible combinations of center and radius that could form a circle passing through that pixel. It increments a counter at the corresponding parameter coordinates in the Hough parameter space for each valid combination.

After accumulating votes, the function identifies the most significant circles by searching for local the most voted coordinates. The function returns the detected circles as a list of parameters, in the form of (x, y, radius). These parameters can be used to draw circles on the image or perform further analysis.
* `HoughCircles(inputImage, storeResultsIn, HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);`

## Hints

[Color space conversion](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

[Simple thresholding, Adaptive thresholding, Otsu’s thresholding](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html)

[Smoothing Images](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)

[Contour Features](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)

[Hough Circles](https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html)



## Contributors

- Contributors: [David Valladares](https://github.com/dvalladaresv),  [Jose María Cañas](https://github.com/jmplaza),  [Felicidad Abad](https://github.com/felicidadaqm)
- Maintained by [Felicidad Abad](https://github.com/felicidadaqm)



## References

1. [https://es.wikipedia.org/wiki/Procesamiento_digital_de_im%C3%A1genes](https://es.wikipedia.org/wiki/Procesamiento_digital_de_im%C3%A1genes)
2. [https://www.delftstack.com/es/howto/python/opencv-blur/](https://www.delftstack.com/es/howto/python/opencv-blur/) 
3. [https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html] (https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html)
4. [https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html] (https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html)
5. [https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html](https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html)
6. [https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html] (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)
7. [https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html] (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)
