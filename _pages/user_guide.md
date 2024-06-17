---
permalink: /user_guide/
title: "User Guide"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

**Robotics Academy** supports Linux (Ubuntu 18.04, 20.04, 22.04 and other distributions), MacOS and Windows.


<a name="installation"></a>
# 1. Installation

The installation of ROS, Gazebo, etc. has been greatly simplified, as all the required dependencies are already pre-installed in the Robotics-Academy Docker Image (RADI).

## Minimum System Requirements

* **CPU**: A 4-cored processor.
* **RAM**: 2 gb RAM.
* **Memory**: 20 gb of disk space.

## Linux Users

1. Download [Docker](https://docs.docker.com/get-docker/) **(minimum version of docker-py: 5.0.3)**.

2. Pull the current distribution of Robotics Academy Docker Image **(currently version 4.5.11)**:

```bash
docker pull jderobot/robotics-academy:4.5.11
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

## Windows Users

Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

1. Install WSL2. Here's a link to the [tutorial](https://learn.microsoft.com/en-us/windows/wsl/install) **(minimum version of docker hub: 4.30.0)**.

2. Install Docker Desktop. Docker made an app for Windows users to adapt the user experience. You can download it from this [link](https://www.docker.com/products/docker-desktop/).

3. Enable Docker Desktop WSL integration: In order to wsl2 to recognise Docker, you need to enable it. For that, go to Docker Desktop -> Settings -> Resources -> WSL integration. Click on the check box and the slider.

    ![WSL integration](/RoboticsAcademy/assets/images/user_guide/wsl-integration-docker.png)

4. Pull the current distribution of Robotics Academy Docker Image **(currently version 4.5.11)**:

```bash
docker pull jderobot/robotics-academy:4.5.11
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

## MacOs (PLACEHOLDER!)

* Remember to add minium docker version to run a RADI.

<a name="launch"></a>
# 2. How to launch a RADI container?

* Start a new docker container of the image and keep it running in the background:

```bash
docker run --rm -it -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

## Enable GPU Acceleration
ROS and Gazebo can be accelerated within RoboticsAcademy thanks to VirtualGL if a GPU is available.

### Linux


- **Intel:** For Linux machines and Intel GPUs, acceleration can be achieved by simply setting the ```--device``` argument when running the Docker container:
```bash
docker run --rm -it --device /dev/dri -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

- **NVIDIA:** For NVIDIA GPUs, acceleration can be achieved by [installing the nvidia-container-runtime package](https://docs.docker.com/config/containers/resource_constraints/#gpu), and then running the command above, but adding the ```--gpus all``` flag:
```bash
docker run --rm -it --gpus all --device /dev/dri -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

- **MULTIPLE GPUs:** If the PC has several GPUs, we can choose which one will be used by setting the ```DRI_NAME``` environment variable (e.g. ```card0``` or ```card1```)
```bash
docker run --rm -it --gpus all --device /dev/dri -e DRI_NAME=card1 -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```
You can check the names associated with your graphic cards by running:
```bash
 drm-info -j | jq 'with_entries(.value |= .driver.desc)'
```
You should get something like:
```bash
{
  "/dev/dri/card1": "NVIDIA DRM driver",
  "/dev/dri/card0": "Intel Graphics"
}
```

### Windows
For Windows machines, GPU acceleration to Docker can be implemented with WSL2 as per instructions given [here](https://docs.docker.com/desktop/gpu/#using-nvidia-gpus-with-wsl2).

<a name="perform-exercise"></a>
# 3. How to use and perform the exercises?

1. On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

    ![Robotics Academy Main Page](/RoboticsAcademy/assets/images/user_guide/ra-main-page.png)

2. Click on the exercise you want to work with.

3. Wait until the exercise is fully loaded. You know it is fully loaded when the `Robotics Backend`, `World` and `Visualization state` boxes are green.

    ![Exercise-page](/RoboticsAcademy/assets/images/user_guide/exercise-page.jpeg)

4. Let's take a look to the exercise page:
    1. **World Button**: Some exercises have multiple universes to test your code. Clicking on this button, a list of the universes appears and you can choose whichever you want.

    2. **Documentation and Forum Button**: These buttons links you to the documentation of the exercise and the forum respectively. On the documentation, you can find important info such as implemented methods, the objective of the exercise, etc... the Forum lets you ask question about different subjects.

    3. **Load and Save Buttons**: If you're in the middle of an exercise but don't have much time left, you can save the code in an external file on your computer and load it too. 

    4. **Start, Restart Buttons and RTF**:
        1. Start button: To send your code into the docker container, you must use the start button. This will load you're entire code and execute it. If the simulation is running, it will be replaced with the `pause` button, which pauses the simulation.
        2. Restart button: The restart button stops the simulation and returns it to its initial state.
        3. RTF (Real Time Factor): These two numbers will let you know the fps of the simulation and the speed on which the exercise is being executed.

    5. **Code editor**: Here you can write your code to solve the exercise. When you're done, it's sent to the container with the `start` button.

    6. **VNC displays**: These VNC displays gives you info about the simulation. The one at the top usually has GUI info for the exercise. Below that is the Gazebo universe display and on the left you have a console where you can print info about the exercise.

5. On the exercise you'll select a world on which you want to resolve the exercise. Then you'll write the code solution in the text editor and launch it with the start button. You can pause the simulation whenever you want and check if it is executing effectively with RTF visor. At the end you can save your code or load it to resume it whenever you want.

# 4. Reference execution performance data
In this section, various data tables will be provided showing the performance cost of each RADI exercise on different PCs, both running without GPU acceleration and with GPU acceleration. The values collected will include the % of CPU usage, the % of GPU usage (if is running with GPU acceleration), the RTF of Gazebo and the FPS of Gazebo.

## No GPU Acceleration

### Linux

SO: Ubuntu 22.04 LTS, RAM: 8 GB, CPU: Intel Core i7-7700HQ (4 cores)

*(Average Percentages up to 100%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|
| **Follow Line**                | 0.44         | 7            | 98%         |
| **Vacuum Cleaner**             | 0.95         | 10           | 70%         |
| **Autoparking**                | 0.82         | 3            | 78%         |
| **Follow Person**              | 0.98         | 3            | 99%         |
| **Localized Vacuum Cleaner**   | 0.97         | 12           | 85%         |
| **Global Navigation**          | 0.38         | 7            | 97%         |
| **Rescue People**              | 0.92         | 6            | 95%         |
| **Obstacle Avoidance**         | 0.44         | 4            | 94%         |
| **3D Reconstruction**          | 0.70         | 6            | 91%         |
| **Amazon Warehouse**           | 0.70         | 2            | 99%         |
| **Montecarlo Laser Localized** | 0.95         | 11           | 88%         |

### Windows

SO: Windows 11, RAM: 16 GB, CPU: Intel Core i5-9400F (6 cores)

*(Average Percentages up to 100%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|
| **Follow Line**                | 0.13         | 3            | 79%         |
| **Vacuum Cleaner**             | 0.89         | 7            | 74%         |
| **Autoparking**                | 0.80         | 3            | 77%         |
| **Follow Person**              | 0.68         | 4            | 79%         |
| **Localized Vacuum Cleaner**   | 0.80         | 7            | 77%         |
| **Global Navigation**          | 0.73         | 7            | 78%         |
| **Rescue People**              | 0.59         | 5            | 79%         |
| **Obstacle Avoidance**         | 0.29         | 3            | 80%         |
| **3D Reconstruction**          | 0.56         | 3            | 79%         |
| **Amazon Warehouse**           | 0.56         | 2            | 79%         |
| **Montecarlo Laser Localized** | 0.80         | 9            | 76%         |

---

SO: Windows 11, RAM: 16 GB, CPU: Intel Core i5-13600KF (14 cores)

*(Average Percentages up to 100%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|
| **Follow Line**                | 0.89         | 30           | 78%         |
| **Vacuum Cleaner**             | 1.00         | 19           | 51%         |
| **Autoparking**                | 0.98         | 8            | 52%         |
| **Follow Person**              | 0.99         | 18           | 85%         |
| **Localized Vacuum Cleaner**   | 0.99         | 37           | 60%         |
| **Global Navigation**          | 0.88         | 40           | 80%         |
| **Rescue People**              | 0.88         | 30           | 72%         |
| **Obstacle Avoidance**         | 0.91         | 12           | 70%         |
| **3D Reconstruction**          | 0.97         | 11           | 65%         |
| **Amazon Warehouse**           | 0.92         | 7            | 90%         |
| **Montecarlo Laser Localized** | 0.98         | 38           | 70%         |

### MacOS

SO: MacOS Sonoma 14.5, RAM: 8 GB, CPU: Apple M1 (8 cores)

*(Average Percentages up to 800%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|
| **Follow Line**                | 0.57         | 20           | 620%        |
| **Vacuum Cleaner**             | 0.50         | 15           | 400%        |
| **Autoparking**                | 0.47         | 5            | 530%        |
| **Follow Person**              | 0.78         | 13           | 650%        |
| **Localized Vacuum Cleaner**   | 0.45         | 20           | 500%        |
| **Global Navigation**          | 0.63         | 15           | 550%        |
| **Rescue People**              | -            | -            | -           |
| **Obstacle Avoidance**         | 0.7          | 8            | 550%        |
| **3D Reconstruction**          | 0.46         | 4            | 550%        |
| **Amazon Warehouse**           | 0.5          | 6            | 620%        |
| **Montecarlo Laser Localized** | 0.46         | 18           | 500%        |

## Intel GPU Acceleration

### Linux

No data yet.

### Windows

No data yet.

### MacOS

No data yet.

## Nvidia GPU Acceleration

### Linux

No data yet.

### Windows

SO: Windows 11, RAM: 16 GB, CPU: Intel Core i5-9400F (6 cores), GPU: NVIDIA GeForce GTX 1660

*(Average Percentages up to 100%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage | % GPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|:-----------:|
| **Follow Line**                | 0.42         | 14           | 73%         | 5%          |
| **Vacuum Cleaner**             | 0.95         | 10           | 55%         | 5%          |
| **Autoparking**                | 0.85         | 5            | 65%         | 8%          |
| **Follow Person**              | 0.95         | 22           | 67%         | 3%          |
| **Localized Vacuum Cleaner**   | 0.93         | 15           | 68%         | 5%          |
| **Global Navigation**          | 0.73         | 9            | 68%         | 7%          |
| **Rescue People**              | 0.49         | 8            | 77%         | 10%         |
| **Obstacle Avoidance**         | 0.25         | 3            | 75%         | 4%          |
| **3D Reconstruction**          | 0.68         | 9            | 77%         | 8%          |
| **Amazon Warehouse**           | 0.67         | 2            | 78%         | 4%          |
| **Montecarlo Laser Localized** | 0.85         | 12           | 73%         | 18%         |

---

SO: Windows 11, RAM: 16 GB, CPU: Intel Core i5-13600KF (14 cores), GPU: NVIDIA GeForce RTX 4060

*(Average Percentages up to 100%)*

|                                | Gazebo's RTF | Gazebo's FPS | % CPU usage | % GPU usage |
|:------------------------------:|:------------:|:------------:|:-----------:|:-----------:|
| **Follow Line**                | 0.88         | 41           | 75%         | 3%          |
| **Vacuum Cleaner**             | 1.00         | 22           | 43%         | 4%          |
| **Autoparking**                | 0.98         | 15           | 55%         | 5%          |
| **Follow Person**              | 1.00         | 10           | 70%         | 10%         |
| **Localized Vacuum Cleaner**   | 0.99         | 45           | 56%         | 6%          |
| **Global Navigation**          | 0.88         | 60           | 70%         | 8%          |
| **Rescue People**              | 1.00         | 62           | 60%         | 9%          |
| **Obstacle Avoidance**         | 0.90         | 16           | 73%         | 4%          |
| **3D Reconstruction**          | 0.98         | 10           | 72%         | 8%          |
| **Amazon Warehouse**           | 0.94         | 7            | 87%         | 4%          |
| **Montecarlo Laser Localized** | 0.99         | 44           | 52%         | 12%         |

### MacOS

No data yet.

# 5. Troubleshooting

