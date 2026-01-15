---
permalink: /exercises/ComputerVision/basic_computer_vision
title: "Basic Computer Vision"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Basic Computer Vision"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/basic_computer_vision/basic_computer_vision_teaser.png
    image_path: /assets/images/exercises/basic_computer_vision/basic_computer_vision_teaser.png
    alt: "Basic Computer Vision"
    title: "Basic Computer Vision"

colorspace:
  - url: /assets/images/exercises/basic_computer_vision/colorspace.png
    image_path: /assets/images/exercises/basic_computer_vision/colorspace.png
    alt: "color space"
    title: "color space"

youtubeId1: mE3Ww0qo5N4
youtubeId2: JgbPyjJuBVE
youtubeId3: ZNDWv_oZRpE
youtubeId4: pNV3SJCVXhk
youtubeId5: 2U6K_dgpjNA
youtubeId6: 31HoUDv8_zI
youtubeId7: otzet0RazOI
youtubeId8: 3jDu4iyVPqc
youtubeId9: 4u19cW4ecL4
youtubeId10: XtTHAN9zPr8
---

## Goal

Here the intention is to develop some basic exercises about computer vision. You will have to get in contact with the *OpenCV* (*Python*) library.

{% include gallery caption="Tracking example" %}


## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Exercise API

This exercise now supports ROS 2-native implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `WebGUI.getImage()` - to get the image (numpy array). It can be None.
* `WebGUI.showImage(image)` - allows you to view a debug image or one with relevant information.

### ROS 2-native Implementation

#### ROS 2 Topics

Use standard ROS 2 topics for direct communication.

* `/input/image_raw` - Subscribe to this topic to receive input images (BGR8). Message type: `sensor_msgs/msg/Image`
* `/webgui_image` - Publish to this topic to send the processed image to the GUI. Message type: `sensor_msgs/msg/Image`

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

* `rclpy.spin()` - Event-driven execution using callbacks.
* `rclpy.spin_once()` - Single-step processing, often with custom timers.
* `rclpy.Rate()` - Loop-based frequency control.

## Theory
In this exercise different computer vision functionalities are proposed for their implementation:
* Change to grayscale
* Morphological processing
* Implement a color filter
* Edge filters (for example canny or laplace)
* Convolutions (smoothing and enhancing)
* Optical flow
* Corner detector
* Hough transform

### Grayscale
It is proposed to transform the image obtained from the camera into a grayscale image.

{% include youtubePlayer.html id=page.youtubeId1 %}

### Morphological Processing
It is proposed to apply different morphological filters to the image, such as dilation, erosion, etc. Morphological filters are a set of image processing techniques primarily used for binary or high-contrast images. These operations are based on the shape or structure of the objects within an image and are designed to transform the shapes of objects in a way that depends on their structure. Morphological filters are widely used in tasks such as noise removal, filling holes, edge detection, and image segmentation.

Common Morphological Operations:
* Erosion: Erosion reduces the size of the objects in an image. For each pixel in the image, it checks if all the pixels under the mask (structuring element) are 1 (white). If they are, the central pixel remains white; if not, it turns black. In practice, this operation "shrinks" the white regions in the image.

* Dilation: Dilation increases the size of the objects in the image. For each pixel in the image, if at least one of the pixels under the mask is 1, the central pixel becomes white. In practice, this operation "expands" the white regions in the image.

* Opening: The opening operation is a combination of erosion followed by dilation. It removes small objects and fine details from the image.

* Closing: The morphological closing filter is a combination of two basic morphological operations: dilation followed by erosion. It is used to remove small holes or gaps inside objects, while preserving the overall shape and size of the objects.

{% include youtubePlayer.html id=page.youtubeId5 %}

### Implement a Color Filter
This involves creating a color filter, that is, filtering an object by its color. For example, you can show the camera an object of a certain color and have that object appear boxed in the image. To do this, you'll need to threshold the image based on color. Perhaps a morphological filter could be applied to obtain a more precise representation of the object.

{% include youtubePlayer.html id=page.youtubeId6 %}

### Edge Filters
An edge filter must be applied and displayed in the output image. This can be achieved using a Canny or Laplace filter. The Canny Edge Detector is an edge detection operator used in image processing to detect a wide range of edges in images. It is widely used for feature extraction and object detection due to its ability to identify sharp changes in intensity within an image, which correspond to the boundaries of objects. The Laplace Filter, also known as the Laplacian of Gaussian (LoG) filter, is a second-order derivative edge detection technique in image processing. Unlike the Sobel or Canny edge detectors, which detect edges by evaluating the first derivative (the rate of change), the Laplace filter detects edges by calculating the second derivative (the rate of change of the rate of change). This can help identify areas where the intensity changes sharply in the image.

{% include youtubePlayer.html id=page.youtubeId2 %}

### Convolutions
The proposal is to apply a convolution to the image obtained from the camera. This convolution can be used to generate a smoothed image or to enhance the image.

{% include youtubePlayer.html id=page.youtubeId3 %}

### Optical Flow
Optical flow must be calculated from the camera image, thus depicting the motion of the image. Optical flow is a technique used in computer vision and image processing to estimate the motion of objects between two consecutive frames in a video or image sequence. It calculates the apparent motion of pixel intensities, which helps in understanding the movement of objects or the camera itself.

OpenCV provides functions to calculate optical flow. Check this [tutorial](https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html)! 

**Some tips:**
* Pixel intensities and the frame rate (FPS) can influence the accuracy of the flow estimation.

{% include youtubePlayer.html id=page.youtubeId9 %}

For a more advanced implementation, you can select good feature points and track only those specific pixels, as shown in the following example:

{% include youtubePlayer.html id=page.youtubeId7 %}

### Corner Detector
A corner detector needs to be implemented. A Harris corner detector could be used. The Harris corner detector is a popular algorithm used in image processing to identify corner points in an image.

{% include youtubePlayer.html id=page.youtubeId4 %}

### Hough Transform
The Hough transform must be applied to the image. The Hough Transform is a technique used in image processing and computer vision to detect simple shapes, such as lines, circles, and other parametric curves, in an image.

{% include youtubePlayer.html id=page.youtubeId8 %}

## Hints

[Color space conversion](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

[Simple thresholding, Adaptive thresholding, Otsu’s thresholding](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html)

[Smoothing Images](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)

[Contour Features](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId10 %}

## Contributors

- Contributors: [Jose María Cañas](https://github.com/jmplaza), [Vanessa Fernández](https://github.com/vmartinezf), [Jessica Fernández](https://github.com/jessiffmm), [Lucía Lishan Chen Huang](https://github.com/lu164), [Ashish Ramesh](https://github.com/AshishRamesh).
- Maintained by [Pankhuri Vanjani](https://github.com/pankhurivanjani) and [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).



## References

1. [https://www.geeksforgeeks.org/color-spaces-in-opencv-python/](https://www.geeksforgeeks.org/color-spaces-in-opencv-python/) 
2. [https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/](https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/) 
3. [https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/](https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/)
4. [https://opencv-python-tutroals.readthedocs.io/en/latest/index.html](https://opencv-python-tutroals.readthedocs.io/en/latest/index.html)
5. [https://www.geeksforgeeks.org/python-visualizing-image-in-different-color-spaces/?ref=rp](https://www.geeksforgeeks.org/python-visualizing-image-in-different-color-spaces/?ref=rp)
6. [https://realpython.com/python-opencv-color-spaces/](https://realpython.com/python-opencv-color-spaces/)
