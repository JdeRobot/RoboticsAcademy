---
permalink: /exercises/ComputerVision/image_processing
title: "Image Processing"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Image Processing"
toc_icon: "cog"

layout: archive
classes: wide

gallery:
  - url: /assets/images/exercises/image_processing/image_processing_teaser.png
    image_path: /assets/images/exercises/image_processing/image_processing_teaser.png
    alt: "Image Processing"
    title: "Image Processing"
---


## Goal

This exercise is an introduction to the image processing world. In this practice the objective is to complete a series of mini-challenges. The difficulty of the mini-challenges will increase progressively, without leaving the premise of an introduction exercise.

{% include gallery caption="Tracking example" %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

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
3. [https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html)
4. [https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html](https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html)
5. [https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html](https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html)
6. [https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)
7. [https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)
