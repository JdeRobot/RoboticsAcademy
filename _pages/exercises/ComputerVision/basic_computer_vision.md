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

youtubeId1: gzALLE2jlRI
youtubeId2: Fv9s99IEIvc

---

## Goal

In this practice the intention is to develop some basic exercises about computer vision. You will have to get in contact with *OpenCV* (*Python*) library.

{% include gallery caption="Tracking example" %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Exercise API

* `GUI.getImage()` - to get the image. It can be None
* `GUI.showImage()` - allows you to view a debug image or with relevant information

## Theory
In this exercise it is proposed to implement different computer vision functionalities:
* Change to grayscale
* Morphological processing
* Implement a color filter
* Edge filters (for example canny or laplace)
* Convolutions (smoothing and enhancing)
* Optical flow
* Corner detector
* Hough transform

### Grayscale
It is proposed to transform the image obtained from the camera into grayscale.

### Morphological Processing
It is proposed to apply different morphological filters to the image, such as dilation, erosion, etc. Morphological filters are a set of image processing techniques primarily used for binary or high-contrast images. These operations are based on the shape or structure of the objects within an image and are designed to transform the shapes of objects in a way that depends on their structure. Morphological filters are widely used in tasks such as noise removal, filling holes, edge detection, and image segmentation.

Common Morphological Operations:
* Erosion: Erosion reduces the size of the objects in an image. For each pixel in the image, it checks if all the pixels under the mask (structuring element) are 1 (white). If they are, the central pixel remains white; if not, it turns black. In practice, this operation "shrinks" the white regions in the image.

* Dilation: Dilation increases the size of the objects in the image. For each pixel in the image, if at least one of the pixels under the mask is 1, the central pixel becomes white. In practice, this operation "expands" the white regions in the image.

* Opening: The opening operation is a combination of erosion followed by dilation. It removes small objects and fine details from the image.

### Implement a Color Filter
This involves creating a color filter, that is, filtering an object by its color. For example, you can show an object of a certain color and have that object appear boxed in the image. To do this, you'll need to threshold the image based on color. Perhaps a morphological filter could be applied to obtain a more precise representation of the object.

### Edge Filters
An edge filter must be applied and displayed in the output image. This can be achieved using a Canny or Laplace filter. The Canny Edge Detector is an edge detection operator used in image processing to detect a wide range of edges in images. It is widely used for feature extraction and object detection due to its ability to identify sharp changes in intensity within an image, which correspond to the boundaries of objects. The Laplace Filter, also known as the Laplacian of Gaussian (LoG) filter, is a second-order derivative edge detection technique in image processing. Unlike the Sobel or Canny edge detectors, which detect edges by evaluating the first derivative (the rate of change), the Laplace filter detects edges by calculating the second derivative (the rate of change of the rate of change). This can help identify areas where the intensity changes sharply in the image.

### Convolutions
The proposal is to apply a convolution to the image obtained from the camera. This convolution can be used to generate a smoothed image or to enhance the image.

### Optical Flow
Optical flow must be calculated from the camera image, thus depicting the motion of the image. Optical flow is a technique used in computer vision and image processing to estimate the motion of objects between two consecutive frames in a video or image sequence. It calculates the apparent motion of pixel intensities, which helps in understanding the movement of objects or the camera itself.

### Corner Detector
A corner detector needs to be implemented. A Harris corner detector could be used. The Harris corner detector is a popular algorithm used in image processing to identify corner points in an image.

### Hough Transform
The Hough transform must be applied to the image. The Hough Transform is a technique used in image processing and computer vision to detect simple shapes, such as lines, circles, and other parametric curves, in an image.


## Hints

[Color space conversion](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

[Simple thresholding, Adaptive thresholding, Otsu’s thresholding](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html)

[Smoothing Images](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)

[Contour Features](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)



## Contributors

- Contributors: [Jose María Cañas](https://github.com/jmplaza), [Vanessa Fernández](https://github.com/vmartinezf), [Jessica Fernández](https://github.com/jessiffmm), [Lucía Lishan Chen Huang](https://github.com/lu164)
- Maintained by [Pankhuri Vanjani](https://github.com/pankhurivanjani) and [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).



## References

1. [https://www.geeksforgeeks.org/color-spaces-in-opencv-python/](https://www.geeksforgeeks.org/color-spaces-in-opencv-python/) 
2. [https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/](https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/) 
3. [https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/](https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/)
4. [https://opencv-python-tutroals.readthedocs.io/en/latest/index.html](https://opencv-python-tutroals.readthedocs.io/en/latest/index.html)
5. [https://www.geeksforgeeks.org/python-visualizing-image-in-different-color-spaces/?ref=rp](https://www.geeksforgeeks.org/python-visualizing-image-in-different-color-spaces/?ref=rp)
6. [https://realpython.com/python-opencv-color-spaces/](https://realpython.com/python-opencv-color-spaces/)
7. [https://medium.com/@jijupax/connect-the-webcam-to-docker-on-mac-or-windows-51d894c44468](https://medium.com/@jijupax/connect-the-webcam-to-docker-on-mac-or-windows-51d894c44468)
