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
    - url: /assets/images/exercises/end_to_end_visual_control/formula1.png
      image_path: /assets/images/exercises/end_to_end_visual_control/formula1.png
      alt: "First Person."
      title: "First Person."
    - url: /assets/images/exercises/end_to_end_visual_control/formula1_3.png
      image_path: /assets/images/exercises/end_to_end_visual_control/formula1_3.png
      alt: "Model."
      title: "Model."

gpu_verify:
    - url: /assets/images/exercises/end_to_end_visual_control/nvidia-smi-output.png
      image_path: /assets/images/exercises/end_to_end_visual_control/nvidia-smi-output.png
      alt: "nvidia-smi output"
      title: "nvidia-smi output"

gifs:
    - url: /assets/images/exercises/end_to_end_visual_control/oscillations.gif
      image_path: /assets/images/exercises/end_to_end_visual_control/oscillations.gif
      alt: "examples"
      title: "examples"
    - url: /assets/images/exercises/end_to_end_visual_control/slowresponse.gif
      image_path: /assets/images/exercises/end_to_end_visual_control/slowresponse.gif
      alt: "examples"
      title: "examples"

youtubeId1: 5HaMNChV6vI
---

## Goal

<p style="text-align:justify">
The end-to-end visual control exercise demonstrates end-to-end visual control of an autonomous vehicle using deep learning. The exercise provides datasets for students to develop and train their own deep learning models. The deep learning model takes raw images from the vehicle camera as input and predicts vehicle commands, including linear speed (v) and angular velocity (w), to navigate the autonomous vehicle through different circuits. Using the web interface, users can upload their models, test them in real-time inference, and observe how vision-based AI enables autonomous vehicle navigation within simulated environments created in Gazebo.
</p>

{% include gallery caption="Gallery" %}

The students will develop a deep learning model that helps the autonomous vehicle complete the circuits.

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is done, how to launch a RoboticsBackend and how to access the exercises.

<!-- TODO: DEVELOP DEEP LEARNING MODEL -->

## 🤖 Develop a Deep Learning Model

### 1. 💾 Dataset overview

For students who want to develop deep learning models for the End-to-End Visual Control exercise, we provide **two datasets**:

#### i) Simple Circuit Dataset

<p style="text-align:justify"> 
This dataset is specifically designed for training and testing models on a single, <strong>simple circuit</strong>. It is ideal for beginners or for initial experiments to understand how the model reacts to basic driving scenarios. The simple circuit is easier to complete, allowing users to quickly train and evaluate their models without facing complex turns or intersections.
</p>

#### ii) Combine Circuit Dataset

<p style="text-align:justify">
This dataset includes data from all <strong>four circuits</strong> available in the exercise. It is intended for advanced model development, enabling students to train models that generalize across all four circuits and handle various driving conditions, including <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">sharp left and right turns</code>. The combined dataset captures a wide range of driving scenarios, including sharp turns, straight paths, and varying circuit complexities. We provide an <strong>adjustment dataset</strong> designed to support users in managing diverse driving scenarios, facilitating more experimentation.
</p>

### 2. ☁️ Datasets Downloads

<p style="text-align:justify">The datasets for the End-to-End Visual Control exercise are hosted on Huggingface under the JdeRobot organization. Students can access them using the <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">load_dataset()</code> method and directly apply them for training and testing their models. Although multiple download options are available, this guide highlights two recommended approaches for retrieving the datasets to a local machine.</p>

-   [JdeRobot/Follow-Line-Simple-Circuit-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset)
-   [JdeRobot/Follow-Line-Combine-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset)

#### Method 01: Use the git lfs command [Recommended: Low]

Visit the [git-lfs](https://git-lfs.com/) website and install `git-lfs` on your local machine.

```bash
# Simple Circuit Dataset
git clone https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset

# Combine Circuit Dataset
git clone https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset
```

#### Method 02: Huggingface Hub API [Recommended: High]

<p style="text-align:justify">
The Huggingface <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">huggingface_hub</code> library provides a <strong>Python API for</strong> interacting with the Huggingface Hub. The primary client class for this is HfApi, which lets you programmatically manage repositories, upload and download files, and access model metadata. The Hub also offers a free Inference API for running models directly on Huggingface.</p>

<p style="text-align:justify">
First, create and activate a <a href="https://docs.python.org/3/tutorial/venv.html" style="text-decoration:underline;text-underline-offset:4px;text-decoration-style:dotted;"  target="_blank" rel="noopener noreferrer">Python Environment<strong>⤴️</strong></a> on your local machine and install the Huggingface Hub pip package. Next, obtain a Huggingface <a href="https://huggingface.co/settings/tokens" style="text-decoration:underline;text-underline-offset:4px;text-decoration-style:dotted;"  target="_blank" rel="noopener noreferrer">ACCESS TOKEN<strong>⤴️</strong></a>  from Huggingface
 and use it to download the datasets with code like:
</p>

##### Huggingface Hub Package

```bash
pip install huggingface_hub
```

##### Datasets Downloads code

```python
from huggingface_hub import snapshot_download

# Download Simple Circuit Dataset to local folder
snapshot_download(repo_id="JdeRobot/Follow-Line-Simple-Circuit-Dataset",
    repo_type="dataset",resume_download=True,max_workers=16,
    token="HF_ACCESS_TOKEN",local_dir=output_dir
)

# Download Combine Dataset to local folder
snapshot_download(repo_id="JdeRobot/Follow-Line-Combine-Dataset",
    repo_type="dataset",resume_download=True,max_workers=16,
    token="HF_ACCESS_TOKEN",local_dir=output_dir
)
```

### 3. 📂 Datasets Folder Structure

#### Simple Circuit Dataset

<p style="text-align:justify">
The dataset is divided into <strong>training</strong> and <strong>testing</strong> parts. The training images are split into seven folders named <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">train_images_part_01</code> to <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">train_images_part_07</code>, and their corresponding labels are provided in the <strong>train.csv</strong> file. For evaluation, the dataset includes a <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">test_images</code> folder that contains all the test images, with their labels stored separately in the <strong>test.csv</strong> file. 
</p>

#### Combine Circuit Dataset

<p style="text-align:justify">
The dataset is organized into several folders and CSV files. The main training images are divided into six parts, stored in the folders <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">images_part_01</code> to <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">images_part_06</code>. Each of these images is linked to labels provided in the <strong>train.csv</strong> file, which contains the vehicle commands. In addition to the main dataset, there is an <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">adjustment_images</code> folder that includes extra images intended for adjusting the sharp corner. The labels for these images are stored separately in the <strong>adjustment_data.csv</strong> file.
</p>

### 4. Model Training and Evaluation Pipeline

#### i) Data Preprocessing

<p style="text-align:justify">
Data preprocessing ensures that all inputs to the model are in a clean, consistent, and usable format. 
The <strong>Combine Circuit Dataset</strong> is already balanced.
</p>

<p style="text-align:justify">
However, the <strong>Simple Circuit Dataset</strong> is <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">imbalanced</code>. Training on such <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">imbalanced</code> data can cause the model to favor majority classes and perform poorly.
To address this, you should first <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">categorize</code> the samples into their respective classes and then apply data balancing techniques, such as:

  <ul>
    <li><strong>Undersampling:</strong> Reducing the number of majority-class samples.</li>
    <li><strong>Oversampling:</strong> Increasing the number of minority-class samples, sometimes by duplicating or augmenting them.</li>
  </ul>
</p>

<p style="text-align:justify">
In the preprocessing stage, images are first loaded from the dataset folders and paired with their corresponding labels from <strong>CSV files</strong> that contain control values such as <strong>linear</strong> and <strong>angular velocity</strong>. To ensure consistency, all images are <strong>resized</strong> to a fixed dimension and <strong>normalized</strong> by scaling pixel values to a 
standard range, which improves training stability. The <strong>labels</strong> are extracted directly from the CSV files and linked to the appropriate images. To enhance <strong>robustness</strong> and <strong>generalization</strong>, <strong>data augmentation techniques</strong> 
such as <strong>flipping</strong>, <strong>rotation</strong>, and <strong>brightness adjustment</strong> are applied, 
creating diverse variations of the training data.
</p>

#### ii) Dataset Splitting

<p style="text-align:justify">
The <strong>Simple Circuit dataset</strong> is divided into two sets: <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">training</code> and <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">validation</code>. The training set, which comprises the majority of the available data, is utilized to enable the model to learn underlying patterns and acquire the target task. The validation set is the data used during training to fine-tune hyper-parameters, prevent over-fitting, and monitor the model’s performance on unseen data.
</p>

<p style="text-align:justify;">
The <strong>Combine Circuit Dataset</strong> consists of <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">training</code> data and <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">adjustment data</code> (used for sharp turns). If needed, these can be combined to create a larger dataset. You can then split the combined dataset into <strong>training</strong> and <strong>validation</strong> sets using an <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">80%-20% ratio</code> with the <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">train_test_split</code> function <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">(from sklearn.model_selection)</code>.
</p>
<!-- <br> -->
<p style="margin-bottom:4px; font-size:16px">Example:</p>

```python
from sklearn.model_selection import train_test_split

# Combine training and adjustment data
all_data = train_data + adjustment_data

# Split into 80% train, 20% validation
train_set, val_set = train_test_split(all_data, test_size=0.2, random_state=42)
```

#### iii) Model Architecture and Training:

<p style="text-align:justify">
For this exercise, a deep learning model, such as a Convolutional Neural Network (CNN) like <strong><a href="https://developer.nvidia.com/blog/deep-learning-self-driving-cars/" target="_blank">PilotNet</a></strong> or <strong><a href="https://pytorch.org/vision/stable/models.html" target="_blank">ResNet</a></strong>, can be chosen to perform End-to-End Visual Control. The model consists of multiple convolutional layers that automatically extract meaningful features from raw camera images, followed by fully connected layers that map these features to the predicted outputs, such as linear and angular velocities for the autonomous vehicle. As a starting point, users can also explore <a href="https://pytorch.org/vision/stable/models.html" target="_blank">pretrained models provided by torchvision</a> for experimentation and benchmarking.
</p>
<p style="text-align:justify">
During training, the model learns by comparing its predictions with the correct outputs (ground truth) from the dataset and trying to reduce the difference. This process uses <strong>backpropagation</strong>, which calculates how the model’s internal parameters (weights) should be adjusted to improve accuracy. <strong>Optimization</strong> algorithms like <strong>Adam</strong> are then applied to update these weights step by step. To make the model more reliable and avoid <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">overfitting, techniques such as normalization, dropout, and adjustment of the learning rate </code> may also be applied.
</p>

#### iv) Validation, Testing, and Evaluation:

<p style="text-align:justify">
While training, testing dataset is used to monitor the model’s performance on unseen data with the help of the <strong>Loss Validation</strong> graph. This helps ensure that the model is not just memorizing the training data (overfitting). Techniques such as <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">early stopping</code> (ending training when validation performance stops improving) or <code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">learning rate</code> scheduling (gradually reducing the learning rate) may be applied to improve results.
</p>
<p style="text-align:justify">
Once training is complete, the model is evaluated on a <strong>testing dataset</strong>. To judge performance, different metrics could be used, such as <strong>Mean Squared Error (MSE)</strong>, <strong>Mean Absolute Error (MAE)</strong>, and the <strong>R-squared</strong> score for regression accuracy for classification tasks.
</p>

#### v) Export the Model

<p style="text-align:justify">
After training your deep learning model, you must export it to the <a href="https://onnx.ai/" style="text-decoration:underline;text-underline-offset:4px;text-decoration-style:dotted;" target="_blank" rel="noopener noreferrer"><strong>ONNX (Open Neural Network Exchange)</strong><strong>⤴️</strong></a> format. This is the only model format currently supported by the <strong>RoboticsAcademy</strong> web interface. ONNX is an open standard that facilitates interoperability among various deep learning frameworks, including <strong>PyTorch</strong> and <strong>TensorFlow</strong>.
<br>
<code class="language-plaintext highlighter-rouge" style="color:#FF0000;background:#ccc; font-size:14px">NB: RoboticsAcademy currently supports only the ONNX (Open Neural Network Exchange) format.</code>

</p>

#### vi) Testing Criteria

<p style="text-align:justify">
While monitoring <strong>validation loss</strong> and checking <strong>evaluation metrics</strong> helps understand how well your model has learned, these measures do not guarantee that the autonomous vehicle can successfully complete a full circuit. To ensure real-world performance, you must also run the model in real-time inference within the <strong>End-to-End Visual Control</strong> exercise in the <strong>RoboticsAcademy</strong> environment and observe how it behaves while driving through the circuit.
</p>

| Dataset Name    | Test Circuit         | Result                                             |
| --------------- | -------------------- | -------------------------------------------------- |
| Simple Circuit  | Circuit Circuit only | A full lap must be completed on the Simple Circuit |
| Combine Circuit | All Four Circuits    | A full lap must be completed within four circuits. |

<!-- TODO: EXERCISE API's AND EXAMPLE CODE -->

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

<!-- TODO: USER CODE -->

## Run the Exercise

### Enable GPU Acceleration

<p style="text-align:justify">
Deep learning models perform much faster when executed on a <strong>GPU</strong> compared to a <strong>CPU</strong>. To take advantage of GPU acceleration in this exercise, you need to ensure that your system has a compatible <a href="https://www.nvidia.com/en-us/drivers/" style="text-decoration:underline;text-underline-offset:4px;text-decoration-style:dotted;" target="_blank" rel="noopener noreferrer">NVIDIA GPU and the required drivers installed<strong>⤴️</strong></a>.
</p>

<p style="text-align:justify;font-size:16px">
RoboticsAcademy currently supports GPU acceleration on <strong>NVIDIA GPUs</strong> only. To take advantage of GPU support when running the <strong>RoboticsAcademy Docker image (RADI)</strong>, you must use the provided <a href="https://jderobot.github.io/RoboticsAcademy/user_guide/#2-how-to-launch-a-robotics-academy-container" style="text-decoration: none;" target="_blank" rel="noopener noreferrer"><code class="language-plaintext highlighter-rouge" style="color:#222831;background:#bdbdbd">execution script </code><strong>⤴️</strong></a>.
</p>

#### Verify GPU Availability

<p style="text-align:justify; margin-bottom:6px; font-size:14px" >
Run the following command to check if your GPU is accessible.
</p>

```bash
nvidia-smi
```

{% include gallery id="gpu_verify" caption="The output should look like this" %}

<!-- Model Path -->

#### File Path for Uploaded Model

The `model_path` holds the file path to the uploaded <strong>ONNX</strong> model.

```python
from model import model_path
```

#### Import GPU Configuration

<p style="text-align:justify">
<a href="https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html" target="_blank" rel="noopener noreferrer">ONNX Runtime supports running models on NVIDIA GPUs</a> through the <strong>CUDA Execution Provider</strong>. This allows you to significantly speed up inference compared to CPU-only execution. import ONNX Runtime and preload the necessary CUDA/cuDNN libraries before creating a session:

</p>

```python
import onnxruntime
from model import model_path

# Preload CUDA/cuDNN DLLs
onnxruntime.preload_dlls()

# Create an inference session that uses the GPU
session = onnxruntime.InferenceSession(
    model_path,
    providers=["CUDAExecutionProvider"] # CUDA as Execution provider
)
```

#### Debug

```python
# To confirm that ONNX Runtime is using the GPU:
print("Execution Provider:", session.get_providers())

# Expected output should include:
['CUDAExecutionProvider', 'CPUExecutionProvider', ...]
```

## Example Code

<!-- Load ONNX session -->

Recommended to load the ONNX model session

```python
# Import the required package
from model import model_path
import onnxruntime
import sys

# preload dlls
onnxruntime.preload_dlls()

# Load ONNX model
try:
    ort_session = onnxruntime.InferenceSession(model_path,providers=["CUDAExecutionProvider"])
except Exception as e:
    print("ERROR: Model couldn't be loaded")
    print(str(e))
    sys.exit(1)
```

## Exercise Instructions

-   The uploaded ONNX format model should adhere to the input/output specifications, please keep that in mind while building your model.
-   The user can train their model in any framework of their choice and export it to the ONNX format. Refer to this [**article**](https://docs.unity3d.com/Packages/com.unity.barracuda@1.0/manual/Exporting.html) to know more about exporting your model to the ONNX format.

<!-- ## Hints

Simple hints provided to help you solve the follow_line exercise. -->

### References to ROS 2 Concepts

Understanding these ROS 2 concepts will help you implement the exercise natively. Refer to these links for more details:

1. ROS 2 Publisher & Subscriber – [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
2. ROS 2 Spin & Spin Once – [https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html](https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html)
 <!-- 3. ROS 2 Rate - add content for rate -->

<!-- ### Illustrations -->

<!-- {% include gallery id="gifs" caption="Unstable Oscillations (left) - Slow Response (right)" %} -->

## Videos

<!-- _This solution is an illustration for the Web Templates_ -->

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId1 %}

<!-- {% include youtubePlayer.html id=page.youtubeId3 %} -->

## Contributors

-   Contributors: [Md. Shariar Kabir](https://github.com/codezerro),[Jose María Cañas](https://github.com/jmplaza),[David Pascual](https://github.com/dpascualhe), [L. Roberto Morales](https://github.com/lr-morales)
-   Maintained by [Md. Shariar Kabir](https://github.com/codezerro),[Jose María Cañas](https://github.com/jmplaza),[David Pascual](https://github.com/dpascualhe),[L. Roberto Morales](https://github.com/lr-morales).

## References

<!-- TODO: -->

1. [https://huggingface.co/JdeRobot](https://huggingface.co/JdeRobot)
2. [JdeRobot/Follow-Line-Simple-Circuit-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Simple-Circuit-Dataset)
3. [JdeRobot/Follow-Line-Combine-Dataset](https://huggingface.co/datasets/JdeRobot/Follow-Line-Combine-Dataset)
4. [ONNX (Open Neural Network Exchange)](https://onnx.ai/)
5. [Nvidia GPU Drivers](https://www.nvidia.com/en-us/drivers/)
6. [CUDA Execution Provider](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html)
