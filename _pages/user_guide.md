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

## Linux Users:

1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

2. Pull the current distribution of Robotics Academy Docker Image **(currently version 4.5.11)**:

```bash
docker pull jderobot/robotics-academy:4.5.11
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

## Windows Users:

Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V

1. Install WSL2. Here's a link to the [tutorial](https://learn.microsoft.com/en-us/windows/wsl/install).

2. Install Docker Desktop. Docker made an app for Windows users to adapt the user experience. You can download it from this [link](https://www.docker.com/products/docker-desktop/)

3. Enable Docker Desktop WSL integration: In order to wsl2 to recognise Docker, you need to enable it. For that, go to Docker Desktop -> Settings -> Resources -> WSL integration. Click on the check box and the slider.

![WSL integration](/assets/images/user_guide/wsl-integration-docker)

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

2. On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

3. Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

4. The exercise can be used, and you may start programming your solution, after the alert.

