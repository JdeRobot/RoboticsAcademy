### [Back to main README.][]

[Back to main README.]: ../README.md

# GPU Acceleration
ROS and Gazebo can be accelerated within RoboticsAcademy thanks to VirtualGL if a GPU is available.

## Linux

### Auto
If the PC has several GPUs, it will select them in order: NVIDIA, Intel, Only CPU.
```
docker run --rm -it $(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "") --device /dev/dri -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 --link academy_db jderobot/robotics-academy:latest
```


### Intel
For Linux machines and Intel GPUs, acceleration can be achieved by simply setting the ```--device``` argument when running the Docker container:
```
docker run --rm -it --device /dev/dri -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 --link academy_db jderobot/robotics-academy:latest
```

### NVIDIA
For NVIDIA GPUs, acceleration can be achieved by [installing the nvidia-container-runtime package](https://docs.docker.com/config/containers/resource_constraints/#gpu), and then running the command above, but adding the ```--gpus all``` flag:
```
docker run --rm -it --gpus all --device /dev/dri -p 7164:7164 -p 6080:6080 -p 1108:1108 -p 7163:7163 --link academy_db jderobot/robotics-academy:latest
```

## Windows
For Windows machines, acceleration can be achieved for NVIDIA GPUs if a valid CUDA installation is available. Useful docs for proper installation of WSL2 + CUDA + Docker Desktop:
- [WSL2 + CUDA](https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl)
- [WSL2 + Docker Desktop](https://docs.docker.com/desktop/features/wsl/)

Once these requirements are ready, you should be able to run Robotics Academy with GPU acceleration as follows:
```
docker run --rm -it --gpus all -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --device /dev/dri -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 --link academy_db jderobot/robotics-academy:latest
```