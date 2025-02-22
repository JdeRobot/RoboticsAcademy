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


## Robot API

* `from HAL import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `HAL.getImage()` - to get the image
* `HAL.showImage()` - allows you to view a debug image or with relevant information


## Theory
In this exercise it is proposed to implement different computer vision functionalities:
* Change to grayscale
* Implement a color filter
* Edge filters (for example canny or laplace)
* Convolutions (smoothing and enhancing)
* Optical flow
* Corner detector
* Hudge transform
* Morphological processing

### Color Space
Color spaces are a way to represent the color channels present in the image that gives the image that particular hue. There are several different color spaces and each has its own significance.
Some of the popular color spaces are RGB (Red, Green, Blue), CMYK (Cyan, Magenta, Yellow, Black), HSV (Hue, Saturation, Value), etc. In the figure below, a)RGB Color Space and b) HSV color space can be visualized.

{% include gallery id="colorspace" caption="RGB and HSV Color Spaces" %}


**BGR color space**: OpenCV’s default color space is RGB. However, it actually stores color in the BGR format. It is an additive color model where the different intensities of Blue, Green and Red give different shades of color. It turns out that this will not work effectively since the RGB values are highly sensitive to illumination making them not great for color detection.

**HSV color space**: HSV(H : Hue represents dominant wavelength S : Saturation represents shades of color V : Value represents Intensity) stores color information in a cylindrical representation of RGB color points. In HSV, each "tint" of colour is assigned a particular number (the Hue). The "amount" of colour is assigned another number (the Saturation) and the brightness of the colour is assigned another number (the Intensity or Value. It attempts to depict the colors as perceived by the human eye. Hue value varies from 0-179, Saturation value varies from 0-255 and Value value varies from 0-255. 

- Hue : This channel encodes color color information. Hue can be thought of an angle where 0 degree corresponds to the red color, 120 degrees corresponds to the green color, and 240 degrees corresponds to the blue color.
- Saturation : This channel encodes the intensity/purity of color. For example, pink is less saturated than red.
- Value : This channel encodes the brightness of color. Shading and gloss components of an image appear in this channel.

It is mostly used for color segmentation purpose and for identifying contrast in images. These color spaces are frequently used in color selection tools in software and for web design. HSV is widely used for building color filters due to its good invariability to illumination.

**CMYK color space**: Unlike, RGB it is a subtractive color space. The CMYK(cyan, magenta, yellow, and key (black)) model works by partially or entirely masking colors on a lighter, usually white, background. The ink reduces the light that would otherwise be reflected. Such a model is called subtractive because inks “subtract” the colors red, green and blue from white light. White light minus red leaves cyan, white light minus green leaves magenta, and white light minus blue leaves yellow.

In reality, color is a continuous phenomenon, meaning that there are an infinite number of colors. Color spaces, however, represent color through discrete structures (a fixed number of whole number integer values), which is acceptable since the human eye and perception are also limited. Color spaces are fully able to represent all the colors we are able to distinguish between.


## Hints

[Color space conversion](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

[Simple thresholding, Adaptive thresholding, Otsu’s thresholding](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html)

[Smoothing Images](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html)

[Contour Features](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html)



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