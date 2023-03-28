---
permalink: /user_guide/
title: "Installation Instructions"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

**Robotics-Academy** supports Linux (Ubuntu 18.04, 20.04 and other distributions), MacOS and Windows. The installation has been greatly simplified, as all the required dependencies are already pre-installed in the Robotics-Academy Docker Image. The users of this release should:

1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

2. Pull the current distribution of Robotics Academy Docker Image:

	```bash
  docker pull jderobot/robotics-academy:latest
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

### How to perform the exercises?
- Start a new docker container of the image and keep it running in the background:

	```bash
  docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

- The exercise can be used after the alert.

### Enable GPU Acceleration
ROS and Gazebo can be accelerated within RoboticsAcademy thanks to VirtualGL if a GPU is available.

### Linux


- **Intel:** For Linux machines and Intel GPUs, acceleration can be achieved by simply setting the ```--device``` argument when running the Docker container:
```bash
docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

- **NVIDIA:** For NVIDIA GPUs, acceleration can be achieved by [installing the nvidia-container-runtime package](https://docs.docker.com/config/containers/resource_constraints/#gpu), and then running the command above, but adding the ```--gpus all``` flag:
```bash
docker run --rm -it --gpus all --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
```

- **MULTIPLE GPUs:** If the PC has several GPUs, we can choose which one will be used by setting the ```DRI_NAME``` environment variable (e.g. ```card0``` or ```card1```)
```bash
docker run --rm -it --gpus all --device /dev/dri -e DRI_NAME=card1 -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
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
For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/).

