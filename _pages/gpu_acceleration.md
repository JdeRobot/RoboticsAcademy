---
permalink: /gpu_acceleration/
title: "GPU Acceleration Instructions"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC GPU Acceleration"
toc_icon: "cog"
---
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