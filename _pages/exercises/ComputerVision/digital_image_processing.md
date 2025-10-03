---
permalink: /exercises/ComputerVision/digital_image_processing
title: "Digital Image Processing"

sidebar:
  nav: "docs"

toc: true
toc_label: "Digital Image Processing"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/digital_image_processing/digital_image_processing.png
    image_path: /assets/images/exercises/digital_image_processing/digital_image_processing.png
    alt: "Digital Image Processing"
    title: "Digital Image Processing"

youtubeId1: bUAcQUONyjk
youtubeId2: bUAcQUONyjk
youtubeId3: bUAcQUONyjk
youtubeId4: bUAcQUONyjk
youtubeId5: bUAcQUONyjk
youtubeId6: bUAcQUONyjk
youtubeId7: bUAcQUONyjk
youtubeId8: bUAcQUONyjk
---

## Goal

This series of exercises introduces Digital Image Processing concepts through interactive, hands-on tasks. The goal is to build a robust intuition in vision processing, mathematical reasoning, and algorithmic implementation within a robotics context. Each exercise presents a clear visual goal and provides boilerplate code, challenging you to implement the core function required to achieve the visual result. This methodology ensures a direct connection between theory and application.

{% include gallery caption="Image Operations Example" %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is done, how to launch a RoboticsBackend and how to access the exercises.

## Exercise API

* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains functions to view debugging information.
* `WebGUI.getSpecificImage(url)` - to get a specific image from a given URL. It can be `None`.
* `WebGUI.showImage(image)` - allows you to view a single debug image.
* `WebGUI.showTwoImages(image1, image2)` - allows you to view two images side-by-side for comparison.

## Theory

In this exercise, different computer vision functionalities are proposed for implementation:

* Histogram Equalization
* Selective Contrast and Brightness Adjustment
* Median Filtering
* Bit-Plane Decomposition
* Mean Squared Error (MSE)
* Image Rotation and Padding
* Convolution

### Exercise 1: Histogram Equalization

**Objective:** To understand how histogram equalization can dramatically improve the contrast of a low-light or washed-out image.

**Task:** You are presented with a low-contrast image. Your task is to implement the histogram equalization function. Upon successful implementation, the output panel will display the high-contrast, equalized version of the image, providing instant visual feedback.

{% include youtubePlayer.html id=page.youtubeId1 %}

### Exercise 2: Selective Contrast and Brightness Adjustment

**Objective:** To understand how pixel-wise transformations can selectively enhance or suppress regions based on brightness levels, introducing the concept of contrast stretching.

**Task:** You are provided with an image and must implement a function that selectively applies a linear transformation to pixel values that exceed a defined brightness threshold. Specifically, the transformation $g(x)=\alpha\cdot f(x)+\beta$ should be applied only when $f(x)>T$, where T is the brightness threshold. Pixel values below the threshold should remain unchanged.

{% include youtubePlayer.html id=page.youtubeId2 %}

### Exercise 3: Median Filtering for Image Denoising

**Objective:** To understand how a median filter works as a non-linear denoising technique, particularly effective against salt-and-pepper noise, while preserving important image details like edges.

**Task:** You will be shown an image corrupted by noise. Your task is to implement a function that applies a median filter, which iterates over the image, computes the median value within each kernel neighborhood, and reconstructs the denoised output image.

{% include youtubePlayer.html id=page.youtubeId3 %}

### Exercise 4: Bit-Plane Decomposition

**Objective:** To understand how an 8-bit grayscale pixel value is composed of 8 individual binary planes and to appreciate how different bits contribute to the overall image information.

**Task:** Given a grayscale image, you must implement a function that performs bitwise operations on the image matrix to isolate and return a selected binary bit-plane image (from 0 to 7). The result will show coarse structure for high-order bits and fine detail or noise for low-order bits.

{% include youtubePlayer.html id=page.youtubeId4 %}

### Exercise 5: Mean Squared Error (MSE) for Image Comparison

**Objective:** To learn a fundamental method for quantitatively measuring the difference or "error" between two images.

**Task:** You are presented with a reference image and a distorted version. You are tasked with implementing the Mean Squared Error formula:
$$MSE=\frac{1}{m\times n}\sum_{i=0}^{m-1}\sum_{j=0}^{n-1}[I(i,j)-K(i,j)]^{2} \text{}$$
The function will take both images as input, and the calculated MSE value will be displayed.

{% include youtubePlayer.html id=page.youtubeId5 %}

### Exercise 6: Image Rotation, Interpolation, and Padding

**Objective:** To understand how image rotation is performed at the pixel level using geometric transformations and how interpolation and padding methods affect the visual quality of the output.

**Task:** You will implement an image rotation function that applies a rotation matrix to pixel coordinates based on a specified angle. It must perform interpolation (e.g., nearest-neighbor, bilinear) to estimate pixel values at the new positions and support different padding strategies (zero, mirror, reflect, wrap-around) for out-of-bounds pixels.

{% include youtubePlayer.html id=page.youtubeId6 %}

### Exercise 7: Convolution Operation and Visualization

**Objective:** To understand convolution as a fundamental image filtering operation, grasp its mathematical basis using kernel matrices, and observe its visual effects.

**Task:** You will implement a basic 2D convolution function. You will be provided with several predefined kernels (e.g., for blurring, sharpening, edge detection). Upon applying a kernel, you will see the original image alongside the filtered output, allowing you to compare the effect of each kernel.

{% include youtubePlayer.html id=page.youtubeId7 %}

## Hints

[Color space conversion](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

[Histogram Equalization](https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html)

[Smoothing and Filtering Images](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)

[Image Geometric Transformations](https://docs.opencv.org/4.x/da/d6e/tutorial_py_geometric_transformations.html)

## Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId8 %}

## Contributors

* Contributor: [Amirmohammad Erfan](https://github.com/amirmerfan)

## References

1. Erfan, Amirmohammad. "Proposal for New Computer Vision Exercises in Robotics-Academy." June 18, 2025.
