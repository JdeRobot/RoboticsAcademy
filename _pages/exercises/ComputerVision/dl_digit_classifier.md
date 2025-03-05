---
permalink: /exercises/ComputerVision/dl_digit_classifier
title: ""

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Digit Classifier"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

mnist:
  - url: /assets/images/exercises/dl_digit_classifier/mnist.png
    image_path: /assets/images/exercises/dl_digit_classifier/mnist.png
    alt: "MNIST samples"
    title: "MNIST samples"
    
cnn:
  - url: /assets/images/exercises/dl_digit_classifier/cnn.png
    image_path: /assets/images/exercises/dl_digit_classifier/cnn.png
    alt: "Example of a Convolutional Neural Network"
    title: "Example of a Convolutional Neural Network"

youtubeId1: 80K0Fd9GFkU

---

# Digit Classification Exercise using Deep Learning

In this exercise, we will train our own deep learning model to solve the widely known task of digit classification. In order to do so, the trained model has to match the input and output specifications described in this documentation. The input model must be provided in ONNX format, which we will talk about in the following sections.

{% include youtubePlayer.html id=page.youtubeId1 %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Exercise Instructions
- The uploaded model should adhere to the following input/output specifications, please keep that in mind while building your model.
  - The model must accept as input grayscale images with size 28x28 pixels. Input shape: 
    ``[batch_size, num_channels, height, width] = [1, 1, 28, 28]``
  - The output must be size 10 array with the probabilities for each class. Output shape: 
    ``[batch_size, num_classes] = [1, 10]``
- The user can train their model in any framework of their choice and export it to the ONNX format [[1]](https://onnx.ai/). Refer to this [**article**](https://docs.unity3d.com/Packages/com.unity.barracuda@1.0/manual/Exporting.html) to know more about how to export your model. For instance, if you are working with PyTorch [[2]](https://pytorch.org/):
```python
import torch
model = ...
dummy_input = torch.randn(1, 1, 28, 28)
torch.onnx.export(
  model, dummy_input, "mnist_cnn.onnx", verbose=True, export_params=True, input_names=['input'], output_names=['output']
)
```

## Theory
Digit classification is a classic toy example for validating machine and deep learning models. More specifically, the MNIST database of handwritten digits [[3]](http://yann.lecun.com/exdb/mnist/) is one of the most popular benchmarks in the literature and is widely used in tutorials as a starting point for machine learning practitioners. For solving this exercise, it is highly recommended training your model using this database.

{% include gallery id="mnist" caption="Samples from the MNIST database" %}

Image classification can be achieved using classic machine learning algorithms like SVMs or Decision Trees [[4]](https://towardsdatascience.com/machine-learning-classifiers-a5cc4e1b0623). However, these algorithms cannot compete in performance with Convolutional Neural Networks (CNNs). CNNs are a particular class of deep neural network which takes advantage of the spatial relationship between variables that are close to each other, allowing for translation invariance. In that way, they are specially suitable for processing grid-like data, such as pixels in an image or time-steps in an audio signal. CNNs are formed by subsequent convolutional layers, each of them composed of convolutional filters. The number of layers and filters has a great impact on the performance on the model, and the optimal configuration depends mostly on the particular task and the available computational resources. Other basic building blocks in CNNs are fully connected layers, activation functions and regularization strategies, such as dropout or pooling. If you are not familiarized with these concepts, here is [a nice article to warm up](https://towardsdatascience.com/simple-introduction-to-convolutional-neural-networks-cdf8d3077bac).

{% include gallery id="cnn" caption="Example of a Convolutional Neural Network" %}

For solving the particular task of digit classification, we don't need complex architectures. Here is an example of how you can build a CNN and train a model using MNIST database with Pytorch: [Basic MNIST Example](https://github.com/pytorch/examples/tree/master/mnist). If you want to further improve the accuracy of your model, try increasing the number of layers and play around with different regularization strategies, such as data augmentation [[5]]((https://debuggercafe.com/image-augmentation-using-pytorch-and-albumentations/)).

## Contributors
- Contributors: [David Pascual](https://github.com/dpascualhe), [Shashwat Dalakoti](https://github.com/shashwat623)
- Maintained by [David Pascual](https://github.com/dpascualhe)

## References

1. [https://onnx.ai/](https://onnx.ai/)
2. [https://pytorch.org/](https://pytorch.org/)
3. [http://yann.lecun.com/exdb/mnist/](http://yann.lecun.com/exdb/mnist/)
4. [https://towardsdatascience.com/machine-learning-classifiers-a5cc4e1b0623](https://towardsdatascience.com/machine-learning-classifiers-a5cc4e1b0623)
5. [https://debuggercafe.com/image-augmentation-using-pytorch-and-albumentations/](https://debuggercafe.com/image-augmentation-using-pytorch-and-albumentations/)
