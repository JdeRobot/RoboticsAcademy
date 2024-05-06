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

## Linux Users

1. Download [Docker](https://docs.docker.com/get-docker/).

2. Pull the current distribution of Robotics Academy Docker Image **(currently version 4.5.11)**:

```bash
docker pull jderobot/robotics-academy:4.5.11
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

## Windows Users

Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

1. Install WSL2. Here's a link to the [tutorial](https://learn.microsoft.com/en-us/windows/wsl/install).

2. Install Docker Desktop. Docker made an app for Windows users to adapt the user experience. You can download it from this [link](https://www.docker.com/products/docker-desktop/).

3. Enable Docker Desktop WSL integration: In order to wsl2 to recognise Docker, you need to enable it. For that, go to Docker Desktop -> Settings -> Resources -> WSL integration. Click on the check box and the slider.

![WSL integration](/assets/images/user_guide/wsl-integration-docker.png)

4. Pull the current distribution of Robotics Academy Docker Image **(currently version 4.5.11)**:

```bash
docker pull jderobot/robotics-academy:4.5.11
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

## MacOs (PLACEHOLDER!)

<a name="launch"></a>
# 2. How to launch a RADI container?

1. Start a new docker container of the image and keep it running in the background:

```bash
docker run --rm -it -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

# Enable GPU Acceleration
ROS and Gazebo can be accelerated within RoboticsAcademy thanks to VirtualGL if a GPU is available.

## Linux


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

## Windows
For Windows machines, GPU acceleration to Docker can be implemented with WSL2 as per instructions given [here](https://docs.docker.com/desktop/gpu/#using-nvidia-gpus-with-wsl2).

<a name="perform-exercise"></a>
# 3. How to use and perform the exercises?

1. On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

![Robotics Academy Main Page](/assets/images/user_guide/ra-main-page.png)

2. Click on the exercise you want to work with.

3. Wait until the exercise is fully loaded. You know it is fully loaded when the `Robotics Backend`, `World` and `Visualization state` boxes are green.

![Exercise-page](/assets/images/user_guide/exercise-page.png)

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

5. On the exercise you'll select a world on which you want to resolve the exercise. Then you'll write the code solution and launch it with the start button. You can pause the simulation whenever you want and check if it is executing effectively with RTF visor. At the end you can save your code or load it to resume it whenever you want.

